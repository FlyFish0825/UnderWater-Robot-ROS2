// ==============================================================================
// internal/underwater_devices/src/devices_node.cpp
// ==============================================================================
// devices 节点：
// - 订阅 gateway/frames，调用 adapters 解码 IMU、深度传感器、推进器状态帧；
// - 发布自定义状态话题 (ImuState, DepthState, MotorState) 与 ROS 标准 sensor_msgs 话题；
// - 驱动 IMU 姿态参考标定组件 (ImuCalibration)；
// - 对全部测量物理量维护独立的有效性标志与保活定时器，失效或超时后主动置无效并广播。
// ==============================================================================
#include "underwater_devices/adapters.hpp"
#include "underwater_devices/imu_calibration.hpp"
#include "underwater_mcu_gateway/protocol.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <map>
#include <optional>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <underwater_interfaces/msg/sensor_status.hpp>

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

namespace ui = underwater_interfaces::msg;
namespace adapt = underwater::devices;
namespace wire = underwater::wire;

class DevicesNode final : public rclcpp::Node {
public:
  DevicesNode() : Node("devices") {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.read_only = true;

    imu_frame_ = declare_parameter<std::string>("imu_frame_id", "imu_sensor", d);
    depth_frame_ = declare_parameter<std::string>("depth_frame_id", "pressure_sensor", d);
    timeout_ = std::chrono::milliseconds(declare_parameter<int>("measurement_timeout_ms", 500, d));

    if (timeout_.count() < 50 || timeout_.count() > 10000) {
      throw std::invalid_argument("measurement timeout out of range (allowed: 50..10000 ms)");
    }

    auto q = rclcpp::SensorDataQoS().keep_last(100);

    // 传感器自定义话题发布者
    imu_ = create_publisher<ui::ImuState>("sensors/imu/raw", q);
    imu_baro_ = create_publisher<ui::ImuState>("sensors/imu/barometer", q);
    depth_ = create_publisher<ui::DepthState>("sensors/depth/state", q);

    // 标准 sensor_msgs 话题发布者
    imu_ros_ = create_publisher<sensor_msgs::msg::Imu>("sensors/imu/data", q);
    imu_pressure_ = create_publisher<sensor_msgs::msg::FluidPressure>("sensors/imu/pressure", q);
    imu_temperature_ = create_publisher<sensor_msgs::msg::Temperature>("sensors/imu/temperature", q);
    pressure_ = create_publisher<sensor_msgs::msg::FluidPressure>("sensors/depth/pressure", q);
    temperature_ = create_publisher<sensor_msgs::msg::Temperature>("sensors/depth/temperature", q);

    motor_ = create_publisher<ui::MotorState>("actuators/thrusters/state", q);
    status_ = create_publisher<ui::SensorStatus>("sensors/status", 10);

    // 姿态参考标定组件
    calibration_ = std::make_unique<adapt::ImuCalibration>(*this, imu_frame_, timeout_);

    // 订阅网关原始协议帧
    sub_ = create_subscription<ui::GatewayFrame>(
        "gateway/frames",
        rclcpp::SensorDataQoS().keep_last(100),
        [this](ui::GatewayFrame::ConstSharedPtr f) { receive(*f); });

    // 100ms 周期检查各测量缓存是否超时
    watchdog_ = create_wall_timer(100ms, [this] { expire(); });
  }

private:
  void receive(const ui::GatewayFrame &f) {
    const auto age = (now() - rclcpp::Time(f.header.stamp)).nanoseconds();
    if (age > timeout_.count() * 1000000LL || age < -100000000LL) {
      return;  // 丢弃过旧帧，并容忍 100ms 以内的时钟提前容差
    }

    // 检测到串口重连：清空全部缓存并向计算图广播失效
    if (f.connection_id != connection_) {
      invalidate();
      connection_ = f.connection_id;
      sequences_.clear();
      motors_.clear();
      imu_cache_.clear();
      depth_cache_.reset();
      statuses_.clear();
    }

    // 0x5b 传感器管理帧：按 target 进行序号去重与回绕检测
    if (f.family == 0x5b && f.flags == 8) {
      auto it = sequences_.find(f.target);
      if (it != sequences_.end()) {
        const auto step = static_cast<std::uint32_t>(f.sequence - it->second);
        if (step == 0 || step >= 0x80000000u) {
          return;  // 重复帧或序号回绕倒退，丢弃
        }
      }
      sequences_[f.target] = f.sequence;
    }

    const auto t = Clock::now();

    // 1. 解码 IMU 报文
    ui::ImuState im;
    if (adapt::imu(f, im)) {
      im.header.frame_id = imu_frame_;
      imu_->publish(im);
      imu_cache_[im.group] = {im, t};
      calibration_->process(im);

      if (im.group == 0x80 && im.raw_valid) {
        sensor_msgs::msg::Imu m;
        m.header = im.header;
        m.orientation_covariance[0] = -1;  // 设备坐标系定义未知，不伪造 ROS 朝向
        m.angular_velocity.x = im.angular_velocity_rad_s[0];
        m.angular_velocity.y = im.angular_velocity_rad_s[1];
        m.angular_velocity.z = im.angular_velocity_rad_s[2];
        m.linear_acceleration.x = im.acceleration_m_s2[0];
        m.linear_acceleration.y = im.acceleration_m_s2[1];
        m.linear_acceleration.z = im.acceleration_m_s2[2];
        imu_ros_->publish(m);
      }

      if (im.group == 0x85) {
        imu_baro_->publish(im);
        if (im.baro_pressure_valid) {
          sensor_msgs::msg::FluidPressure p;
          p.header = im.header;
          p.fluid_pressure = im.baro_pressure_pa;
          imu_pressure_->publish(p);
        }
        if (im.baro_temperature_valid) {
          sensor_msgs::msg::Temperature p;
          p.header = im.header;
          p.temperature = im.baro_temperature_c;
          imu_temperature_->publish(p);
        }
      }
      return;
    }

    // 2. 解码深度传感器报文
    ui::DepthState de;
    if (adapt::depth(f, de)) {
      de.header.frame_id = depth_frame_;
      depth_->publish(de);
      depth_cache_ = std::make_pair(de, t);

      if (de.pressure_valid) {
        sensor_msgs::msg::FluidPressure p;
        p.header = de.header;
        p.fluid_pressure = de.pressure_pa;
        pressure_->publish(p);
      }
      if (de.temperature_valid) {
        sensor_msgs::msg::Temperature p;
        p.header = de.header;
        p.temperature = de.temperature_c;
        temperature_->publish(p);
      }
      return;
    }

    // 3. 解码电机反馈报文
    ui::MotorState mo;
    if (adapt::motor_feedback(f, mo)) {
      auto &entry = motors_[mo.node_id];
      entry.message = mo;
      entry.feedback = t;
      entry.message.heartbeat_valid = entry.heartbeat && (t - *entry.heartbeat < 2500ms);
      motor_->publish(entry.message);
      return;
    }

    // 4. 解码电机心跳报文
    if (adapt::motor_heartbeat(f)) {
      const auto id = static_cast<std::uint8_t>(f.can_id - 0x280);
      auto &entry = motors_[id];
      entry.heartbeat = t;

      auto &m = entry.message;
      m.connection_id = f.connection_id;
      m.node_id = id;
      m.online = true;
      m.heartbeat_valid = true;
      m.state = f.payload[1];
      m.current_calibration_done = (f.payload[2] != 0);
      m.voltage_limited = (f.payload[5] != 0);

      // 反馈若已过期，清空数值字段仅保留心跳状态
      if (!entry.feedback || (t - *entry.feedback > timeout_)) {
        clear_feedback(m);
        m.header = f.header;
      }
      motor_->publish(m);
      return;
    }

    // 5. 解码传感器状态自检帧 (command=0x83)
    if (f.family == 0x5b && f.flags == 8 && f.command == 0x83 && f.payload.size() == 20 &&
        (f.target == 1 || f.target == 2)) {
      ui::SensorStatus s;
      s.header = f.header;
      s.connection_id = f.connection_id;
      s.target = f.target;
      s.fresh = true;
      s.device_status = wire::u32(f.payload, 0);
      s.sample_sequence = wire::u32(f.payload, 4);

      const auto sample_age = wire::u32(f.payload, 8);
      s.sample_age_ms = (sample_age == 0xffffffffu) ? std::int64_t(-1) : static_cast<std::int64_t>(sample_age);
      s.online = (s.device_status & 1) && (s.sample_age_ms >= 0) && (s.sample_age_ms < timeout_.count());
      s.good_frames = wire::u32(f.payload, 12);
      s.errors = wire::u32(f.payload, 16);

      status_->publish(s);
      statuses_[s.target] = {s, t};
    }
  }

  // 清空电机反馈数值 (置 NaN 避免下游误用旧值)
  static void clear_feedback(ui::MotorState &m) {
    m.feedback_valid = false;
    m.speed_rpm = adapt::invalid();
    m.estimated_bus_current_a = adapt::invalid();
    m.bus_voltage_v = adapt::invalid();
    m.mcu_temperature_c = adapt::invalid();
    m.device_timestamp_valid = false;
  }

  void invalidate_imu(ui::ImuState &m) {
    m.fresh = false;
    m.raw_valid = false;
    m.quaternion_valid = false;
    m.euler_valid = false;
    m.baro_pressure_valid = false;
    m.baro_temperature_valid = false;
    m.baro_height_valid = false;
    m.baro_reference_valid = false;

    m.baro_pressure_pa = adapt::invalid();
    m.baro_temperature_c = adapt::invalid();
    m.baro_height_m = adapt::invalid();
    m.baro_reference_pa = adapt::invalid();

    m.acceleration_m_s2.fill(adapt::invalid());
    m.angular_velocity_rad_s.fill(adapt::invalid());
    m.magnetic_protocol_units.fill(adapt::invalid());
    m.quaternion_wxyz.fill(adapt::invalid());
    m.euler_rpy_rad.fill(adapt::invalid());

    imu_->publish(m);
    calibration_->process(m);
    if (m.group == 0x85) {
      imu_baro_->publish(m);
    }
  }

  void invalidate_depth(ui::DepthState &m) {
    m.fresh = false;
    m.raw_valid = false;
    m.pressure_valid = false;
    m.temperature_valid = false;
    m.depth_valid = false;
    m.zero_valid = false;

    m.pressure_pa = adapt::invalid();
    m.temperature_c = adapt::invalid();
    m.raw_depth_m = adapt::invalid();
    m.filtered_depth_m = adapt::invalid();
    m.surface_pressure_pa = adapt::invalid();
    m.d1 = 0;
    m.d2 = 0;

    depth_->publish(m);
  }

  void invalidate() {
    for (auto &kv : imu_cache_) {
      if (kv.second.first.fresh) {
        invalidate_imu(kv.second.first);
      }
    }
    if (depth_cache_ && depth_cache_->first.fresh) {
      invalidate_depth(depth_cache_->first);
    }
    for (auto &kv : motors_) {
      auto &m = kv.second.message;
      clear_feedback(m);
      m.online = false;
      m.heartbeat_valid = false;
      motor_->publish(m);
    }
    for (auto &kv : statuses_) {
      auto &s = kv.second.first;
      s.fresh = false;
      s.online = false;
      status_->publish(s);
    }
  }

  void expire() {
    const auto t = Clock::now();

    for (auto &kv : imu_cache_) {
      if (kv.second.first.fresh && (t - kv.second.second > timeout_)) {
        invalidate_imu(kv.second.first);
      }
    }

    if (depth_cache_ && depth_cache_->first.fresh && (t - depth_cache_->second > timeout_)) {
      invalidate_depth(depth_cache_->first);
    }

    for (auto &kv : motors_) {
      auto &e = kv.second;
      bool changed = false;

      if (e.message.feedback_valid && e.feedback && (t - *e.feedback > timeout_)) {
        clear_feedback(e.message);
        changed = true;
      }
      if (e.message.heartbeat_valid && e.heartbeat && (t - *e.heartbeat > 2500ms)) {
        e.message.heartbeat_valid = false;
        changed = true;
      }

      e.message.online = e.message.feedback_valid || e.message.heartbeat_valid;
      if (changed) {
        motor_->publish(e.message);
      }
    }

    for (auto &kv : statuses_) {
      if (kv.second.first.fresh && (t - kv.second.second > 2500ms)) {
        kv.second.first.fresh = false;
        kv.second.first.online = false;
        status_->publish(kv.second.first);
      }
    }
  }

  struct MotorCache {
    ui::MotorState message;
    std::optional<Clock::time_point> feedback;
    std::optional<Clock::time_point> heartbeat;
  };

  std::string imu_frame_;
  std::string depth_frame_;
  std::unique_ptr<adapt::ImuCalibration> calibration_;
  std::chrono::milliseconds timeout_{500};
  std::uint64_t connection_ = 0;

  std::map<std::uint8_t, std::uint32_t> sequences_;
  std::map<std::uint8_t, std::pair<ui::ImuState, Clock::time_point>> imu_cache_;
  std::optional<std::pair<ui::DepthState, Clock::time_point>> depth_cache_;
  std::map<std::uint8_t, std::pair<ui::SensorStatus, Clock::time_point>> statuses_;
  std::map<std::uint8_t, MotorCache> motors_;

  rclcpp::Publisher<ui::ImuState>::SharedPtr imu_;
  rclcpp::Publisher<ui::ImuState>::SharedPtr imu_baro_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr imu_pressure_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr imu_temperature_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_ros_;
  rclcpp::Publisher<ui::DepthState>::SharedPtr depth_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_;
  rclcpp::Publisher<ui::MotorState>::SharedPtr motor_;
  rclcpp::Publisher<ui::SensorStatus>::SharedPtr status_;
  rclcpp::Subscription<ui::GatewayFrame>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DevicesNode>());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("devices"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

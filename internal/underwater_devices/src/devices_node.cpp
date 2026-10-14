// ==============================================================================
// internal/underwater_devices/src/devices_node.cpp
// ==============================================================================
// devices 节点：
// - 订阅 gateway/frames，调用 adapters 解码 IMU、深度传感器、推进器状态帧；
// - 发布自定义状态话题 (ImuState, DepthState, MotorState) 与 ROS 标准 sensor_msgs 话题；
// - 驱动 IMU 姿态参考标定组件 (ImuCalibration)；
// - 对全部测量物理量维护独立的有效性标志与保活定时器，失效或超时后主动置无效并广播。
// ==============================================================================
// 初学者导读（underwater_devices / devices_node.cpp）
// 所属：设备层的核心 ROS 2 节点；可执行文件为 devices_node，ROS 节点名为 devices。
// 数据链位置：传感器 → STM32 → MCU 串口网关（underwater_mcu_gateway）→ gateway/frames → 【本节点】→
//   各状态话题 → 控制/推进器。本节点负责把二进制帧翻译成 ROS 消息。
// 交互：
//   订阅 gateway/frames (GatewayFrame)；
//   发布 sensors/imu/raw、sensors/imu/barometer、sensors/depth/state、
//     actuators/thrusters/state、sensors/status，以及标准话题 sensors/imu/data
//     (sensor_msgs/Imu)、sensors/imu/pressure、sensors/imu/temperature、
//     sensors/depth/pressure、sensors/depth/temperature；
//   ImuCalibration 子组件再发布 sensors/imu/calibrated 并提供标定服务。
// ROS 2 概念：Node 是能收发话题、调用服务的独立小程序，这里继承 rclcpp::Node；
//   create_publisher/create_subscription 分别创建发布者/订阅者；订阅回调每来一条消息
//   执行一次；QoS 决定可靠性与历史深度；Parameter 是节点配置项；create_wall_timer
//   创建周期定时器；spin 让节点开始处理事件，直到 shutdown。
// C++ 概念：类继承与私有成员封装；std::unique_ptr 是独占所有权的智能指针；lambda 是
//   匿名函数，常作为回调传入；std::optional 表示“可有可无”的值；std::map 是按键有序
//   的字典；std::pair 把两个值打包在一起。
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

// steady_clock 是单调时钟，不受系统时间被调整的影响，适合测量超时。
using Clock = std::chrono::steady_clock;
// 引入 chrono 字面量，之后可以直接写 100ms。
using namespace std::chrono_literals;

// 给长命名空间起短别名：ui::ImuState 即 underwater_interfaces::msg::ImuState。
namespace ui = underwater_interfaces::msg;
namespace adapt = underwater::devices;
namespace wire = underwater::wire;

// DevicesNode：本文件唯一的节点类；final 表示不允许再被继承。
class DevicesNode final : public rclcpp::Node {
public:
  // 构造函数：节点名为 "devices"；在这里声明参数、创建发布者/订阅者/定时器。
  DevicesNode() : Node("devices") {
    // ParameterDescriptor 用来描述参数（read_only=true 表示运行时不可修改）。
    rcl_interfaces::msg::ParameterDescriptor d;
    d.read_only = true;

    // declare_parameter<类型>(名字, 默认值, 描述)：声明并读取节点参数。
    imu_frame_ = declare_parameter<std::string>("imu_frame_id", "imu_sensor", d);
    depth_frame_ = declare_parameter<std::string>("depth_frame_id", "pressure_sensor", d);
    timeout_ = std::chrono::milliseconds(declare_parameter<int>("measurement_timeout_ms", 500, d));

    if (timeout_.count() < 50 || timeout_.count() > 10000) {
      throw std::invalid_argument("measurement timeout out of range (allowed: 50..10000 ms)");
    }

    // SensorDataQoS：传感器数据常用的 QoS（尽力而为、只保留最新若干条）。
    auto q = rclcpp::SensorDataQoS().keep_last(100);

    // 传感器自定义话题发布者
    imu_ = create_publisher<ui::ImuState>("sensors/imu/raw", q);
    imu_baro_ = create_publisher<ui::ImuState>("sensors/imu/barometer", q);
    depth_ = create_publisher<ui::DepthState>("sensors/depth/state", q);

    // 同时把数据转成 ROS 标准 sensor_msgs 消息，方便 RViz/rosbag 等通用工具使用。
    // 标准 sensor_msgs 话题发布者
    imu_ros_ = create_publisher<sensor_msgs::msg::Imu>("sensors/imu/data", q);
    imu_pressure_ = create_publisher<sensor_msgs::msg::FluidPressure>("sensors/imu/pressure", q);
    imu_temperature_ = create_publisher<sensor_msgs::msg::Temperature>("sensors/imu/temperature", q);
    pressure_ = create_publisher<sensor_msgs::msg::FluidPressure>("sensors/depth/pressure", q);
    temperature_ = create_publisher<sensor_msgs::msg::Temperature>("sensors/depth/temperature", q);

    motor_ = create_publisher<ui::MotorState>("actuators/thrusters/state", q);
    status_ = create_publisher<ui::SensorStatus>("sensors/status", 10);

    // 标定组件不是节点，只是普通对象，通过构造函数接收本节点引用。
    // 姿态参考标定组件
    calibration_ = std::make_unique<adapt::ImuCalibration>(*this, imu_frame_, timeout_);

    // create_subscription<消息类型>(话题名, QoS, 回调)：收到消息时调用 receive()。
    // 订阅网关原始协议帧
    sub_ = create_subscription<ui::GatewayFrame>(
        "gateway/frames",
        rclcpp::SensorDataQoS().keep_last(100),
        [this](ui::GatewayFrame::ConstSharedPtr f) { receive(*f); });

    // 看门狗定时器：即使没有新数据，也要周期检查是否超时并广播失效。
    // 100ms 周期检查各测量缓存是否超时
    watchdog_ = create_wall_timer(100ms, [this] { expire(); });
  }

private:
  // receive()：每收到一帧网关数据调用一次，按协议依次尝试各种解码。
  void receive(const ui::GatewayFrame &f) {
    // 用帧头时间与当前时间比较，丢弃太旧的帧；负的 age 容忍时钟小幅提前。
    const auto age = (now() - rclcpp::Time(f.header.stamp)).nanoseconds();
    if (age > timeout_.count() * 1000000LL || age < -100000000LL) {
      return;  // 丢弃过旧帧，并容忍 100ms 以内的时钟提前容差
    }

    // connection_id 变化表示网关重新打开了串口（MCU 重启 / USB 重新枚举 / 网关重启），旧缓存作废。
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

    // 序号去重：同一 target 的 sequence 必须严格递增，否则视为重复或乱序。
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

    // 依次尝试：IMU → 深度 → 电机反馈 → 心跳 → 状态自检；命中一个就 return。
    // 1. 解码 IMU 报文
    ui::ImuState im;
    // adapt::imu 返回 true 表示这确实是 IMU 帧。
    if (adapt::imu(f, im)) {
      im.header.frame_id = imu_frame_;
      imu_->publish(im);
      imu_cache_[im.group] = {im, t};
      calibration_->process(im);

      // 再发布一份标准 sensor_msgs/Imu（供通用工具）；orientation_covariance[0] = -1
      // 是 ROS 惯例，表示“本消息不提供朝向估计”。
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

    // 深度帧处理：发布自定义 DepthState，并另发标准压力/温度消息。
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

    // 电机反馈帧：按 node_id 缓存，并判断心跳是否仍在有效期（2500ms）。
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

    // 心跳帧：即使反馈缺失，也要更新电机的在线状态。
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

    // 传感器自检帧 (0x83)：设备健康统计，发布到 sensors/status。
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

  // clear_feedback()：把电机测量值置 NaN 并标记无效，防止下游误用旧值。
  // 清空电机反馈数值 (置 NaN 避免下游误用旧值)
  static void clear_feedback(ui::MotorState &m) {
    m.feedback_valid = false;
    m.speed_rpm = adapt::invalid();
    m.estimated_bus_current_a = adapt::invalid();
    m.bus_voltage_v = adapt::invalid();
    m.mcu_temperature_c = adapt::invalid();
    m.device_timestamp_valid = false;
  }

  // invalidate_imu()：把一帧 IMU 缓存标记失效并重新发布，通知下游“数据没了”。
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

  // invalidate_depth()：把深度缓存标记失效并重新发布。
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

  // invalidate()：串口重连时把所有缓存一次性作废并广播。
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

  // expire()：看门狗调用的周期性超时检查。
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

  // MotorCache：每个电机节点缓存自己的最新状态和两类时间戳。
  struct MotorCache {
    ui::MotorState message;
    std::optional<Clock::time_point> feedback;
    std::optional<Clock::time_point> heartbeat;
  };

  // ---- 成员变量：参数、缓存、发布者、订阅者、定时器 ----
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

// 程序入口：rclcpp::init 初始化，spin 进入事件循环，Ctrl+C 后 shutdown。
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

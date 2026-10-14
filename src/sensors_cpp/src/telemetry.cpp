// ==============================================================================
// src/sensors_cpp/src/telemetry.cpp
// ==============================================================================
// 传感器遥测与 CSV 数据记录实现：
// - 覆盖 depth, depth_publisher, imu_publisher, sensors_sub, sensors_sub_jilushuju；
// - 订阅原始与标定 IMU 数据及深度数据，以 10 Hz 周期记录 CSV 或格式化回显；
// - 单路数据超过 500ms 未更新主动标记无效并赋 NaN。
// ==============================================================================
// ------------------------------------------------------------------------------
// 【初学者导读】
// 所属：sensors_cpp 包，传感器遥测与 CSV 记录（telemetry.cpp）。
// 角色：覆盖 depth, depth_publisher, imu_publisher, sensors_sub, sensors_sub_jilushuju。
//       订阅内部状态话题，转成 msg_pkg 的对外话题，或按 10Hz 写 CSV / 打印。
// 交互：订阅 <ns>/sensors/imu/state、<ns>/sensors/imu/calibrated、<ns>/sensors/imu/calibration_status、
//       <ns>/sensors/depth/state；部分入口发布 /imu_data 与 /depth_temp。
// ROS 2 概念白话：
//   Subscription（订阅）＝从话题收消息，每来一条执行一次回调（这里是 lambda）。
//   QoS＝传输策略；SensorDataQoS() 适合传感器高频数据，transient_local() 表示保留最后一条给后来的订阅者。
//   spin_some＝处理当前积压的回调后立即返回，适合自己控制循环节奏的场合。
//   RCLCPP_INFO_THROTTLE＝限频打印日志，避免刷屏。
// C++ 概念白话：
//   std::array<double,3>＝长度固定的数组，栈上分配、无动态内存。
//   lambda＝匿名函数，[&] 捕获外部变量，常作为回调传入。
//   std::ofstream＝输出文件流，构造后像 std::cout 一样用 << 写文件。
//   返回空值/NaN＝用“非数字”表示该路数据当前无效。
// ------------------------------------------------------------------------------
#include "sensors_cpp/entry.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#include <unistd.h>

#include <msg_pkg/msg/depth_sensor.hpp>
#include <msg_pkg/msg/imu_data.hpp>
#include <rclcpp/rclcpp.hpp>
#include <underwater_interfaces/msg/depth_state.hpp>
#include <underwater_interfaces/msg/imu_calibration_status.hpp>
#include <underwater_interfaces/msg/imu_state.hpp>

namespace compatibility {

// 用 steady_clock（单调时钟）测时间间隔，不会因为系统对时而被拨动。
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
// 给 underwater_interfaces::msg 起短别名 ui，后面写 ui::ImuState 更简洁。
namespace ui = underwater_interfaces::msg;

// 角度换算：消息里欧拉角/角速度用弧度，而人习惯看度，这里预存 180/π。
static constexpr double degrees = 57.295779513082320876;  // 弧度转角度系数 (180 / π)

// telemetry()：按命令选择订阅哪些话题，并以 10Hz 输出或写 CSV，直到超时或 Ctrl+C。
int telemetry(const std::string &entry, const std::shared_ptr<rclcpp::Node> &node) {
  const auto ns = backend_namespace(node);
  const bool imu_entry = (entry == "imu_publisher");
  const bool depth_entry = (entry == "depth_publisher" || entry == "depth");
  const bool record = (entry == "sensors_sub_jilushuju");
// NaN＝“非数字”，用来表示某路数据当前无效，打印或写 CSV 时一眼可辨。
  const double nan = std::numeric_limits<double>::quiet_NaN();

  std::array<double, 3> acc{nan, nan, nan};
  std::array<double, 3> gyro{nan, nan, nan};
  std::array<double, 3> angle{nan, nan, nan};

  double depth = nan;
  double temp = nan;
  double pressure = nan;

  bool raw_valid = false;
  bool attitude_valid = false;
  bool depth_valid = false;
  bool temp_valid = false;
  bool depth_fresh = false;

  Clock::time_point raw_at{};
  Clock::time_point attitude_at{};
  Clock::time_point depth_at{};
  std::uint64_t connection = 0;

// 缓存最近一次 IMU / 深度消息的 header（时间戳+坐标系），对外转发时沿用。
  std_msgs::msg::Header imu_header;
  std_msgs::msg::Header depth_header;

  rclcpp::Publisher<msg_pkg::msg::ImuData>::SharedPtr imu_pub;
  rclcpp::Publisher<msg_pkg::msg::DepthSensor>::SharedPtr depth_pub;

// imu_publisher 入口：先检查 /imu_data 是否已有发布者，避免冲突，再创建发布者。
  if (imu_entry) {
    if (node->count_publishers("/imu_data")) {
      throw std::runtime_error("/imu_data already has a publisher; reuse the running imu_publisher");
    }
    imu_pub = node->create_publisher<msg_pkg::msg::ImuData>("/imu_data", 10);
  }

// depth / depth_publisher 入口：创建 /depth_temp 发布者（两者是二选一的替代入口）。
  if (depth_entry) {
    if (node->count_publishers("/depth_temp")) {
      throw std::runtime_error("/depth_temp already has a publisher; depth and depth_publisher are alternative entries");
    }
    depth_pub = node->create_publisher<msg_pkg::msg::DepthSensor>("/depth_temp", 10);
  }

// reset()：当连接换设备（connection_id 变化）时清空所有缓存，防止新旧数据混用。
  auto reset = [&](std::uint64_t id) {
    if (connection == id) {
      return;
    }
    connection = id;
    acc.fill(nan);
    gyro.fill(nan);
    angle.fill(nan);
    depth = temp = pressure = nan;
    raw_valid = attitude_valid = depth_valid = temp_valid = depth_fresh = false;
  };

// 按参数选择订阅“原始姿态”还是“标定后姿态”话题；话题名前缀是命名空间 ns。
  const auto imu_source = node->get_parameter("imu_source").as_string();
  const auto imu_topic = ns + (imu_source == "calibrated" ? "/sensors/imu/calibrated" : "/sensors/imu/state");
  std::string reference_saved_at;

// 订阅标定状态。QoS 用 transient_local()：晚加入的订阅者也能拿到最近一次状态（类似 latched）。
  auto reference_status = node->create_subscription<ui::ImuCalibrationStatus>(
      ns + "/sensors/imu/calibration_status",
      rclcpp::QoS(1).transient_local(),
      [&](ui::ImuCalibrationStatus::ConstSharedPtr m) {
        reference_saved_at = (imu_source == "calibrated" && m->calibrated) ? m->saved_at : "";
      });

  RCLCPP_INFO(node->get_logger(), "IMU source: %s (acceleration/gyro remain in sensor axes)", imu_topic.c_str());

// 订阅 IMU 状态：group 0x80 是原始量（加速度/角速度），0x81 是姿态量（欧拉角）。
// 角速度与欧拉角由弧度换算成度；数据无效时填 NaN。
  auto imu_sub = node->create_subscription<ui::ImuState>(
      imu_topic,
      rclcpp::SensorDataQoS(),
      [&](ui::ImuState::ConstSharedPtr m) {
        reset(m->connection_id);
        if (m->group == 0x80) {
          raw_valid = m->fresh && m->raw_valid;
          raw_at = Clock::now();
          imu_header = m->header;
          for (int i = 0; i < 3; ++i) {
            acc[i] = raw_valid ? m->acceleration_m_s2[i] : nan;
            gyro[i] = raw_valid ? m->angular_velocity_rad_s[i] * degrees : nan;
          }
        } else if (m->group == 0x81) {
          attitude_valid = m->fresh && m->euler_valid;
          attitude_at = Clock::now();
          imu_header = m->header;
          for (int i = 0; i < 3; ++i) {
            angle[i] = attitude_valid ? m->euler_rpy_rad[i] * degrees : nan;
          }
        }
      });

// 订阅深度状态：滤波后深度、温度、压力，并分别记录有效标志。
  auto depth_sub = node->create_subscription<ui::DepthState>(
      ns + "/sensors/depth/state",
      rclcpp::SensorDataQoS(),
      [&](ui::DepthState::ConstSharedPtr m) {
        reset(m->connection_id);
        depth_at = Clock::now();
        depth_header = m->header;
        depth_fresh = m->fresh;
        depth_valid = m->fresh && m->depth_valid;
        temp_valid = m->fresh && m->temperature_valid;
        depth = depth_valid ? m->filtered_depth_m : nan;
        temp = temp_valid ? m->temperature_c : nan;
        pressure = (m->fresh && m->pressure_valid) ? m->pressure_pa : nan;
      });

// 只有记录命令（sensors_sub_jilushuju）才打开 CSV；文件名含毫秒时间戳与进程号，避免重名。
  std::ofstream file;
  if (record) {
// output_dir 参数决定写盘目录（默认 DEFAULT_RECORD_DIR）；目录不存在则递归创建。
    const auto out = node->declare_parameter<std::string>("output_dir", DEFAULT_RECORD_DIR);
    std::filesystem::create_directories(out);
    const auto stamp = std::chrono::system_clock::now().time_since_epoch();
    const auto filename = std::filesystem::path(out) /
                          ("sensors_" +
                           std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(stamp).count()) +
                           "_" + std::to_string(getpid()) + ".csv");
    file.open(filename);
    if (!file) {
      throw std::runtime_error("Cannot create recording file");
    }

    file << "receipt_time_s,depth_m,temp_c,pressure_pa,acc_x,acc_y,acc_z,gyro_x_deg_s,gyro_y_deg_s,gyro_z_deg_s,"
         << "roll_deg,pitch_deg,yaw_deg,raw_valid,attitude_valid,depth_valid,imu_source,imu_reference_saved_at\n";
    RCLCPP_INFO(node->get_logger(), "Recording CSV at 10 Hz: %s", filename.c_str());
  }

// duration_s 限制运行时长：0 表示一直跑到 Ctrl+C。
  const auto start = Clock::now();
  const double duration = node->get_parameter("duration_s").as_double();
  if (!std::isfinite(duration) || duration < 0) {
    throw std::invalid_argument("duration_s must be nonnegative");
  }

  auto next = Clock::now();
  unsigned rows = 0;

// 主循环：spin 处理回调 -> 按 10Hz 节拍 -> 检查数据新鲜度 -> 发布/记录。
  while (running() &&
         (duration == 0 || std::chrono::duration<double>(Clock::now() - start).count() < duration)) {
    rclcpp::spin_some(node);

// 未到下一个 10Hz 节拍就小睡 5ms，避免忙等占满 CPU。
    if (Clock::now() < next) {
      std::this_thread::sleep_for(5ms);
      continue;
    }
    next = Clock::now() + 100ms;  // 10 Hz 周期

// 超过 500ms 没更新就判为失效：清空数值并置 NaN，防止把旧数据当成当前值。
    if (Clock::now() - raw_at > 500ms) {
      raw_valid = false;
      acc.fill(nan);
      gyro.fill(nan);
    }
    if (Clock::now() - attitude_at > 500ms) {
      attitude_valid = false;
      angle.fill(nan);
    }
    if (Clock::now() - depth_at > 500ms) {
      depth_valid = temp_valid = depth_fresh = false;
      depth = temp = pressure = nan;
    }

// 把缓存整理成 msg_pkg/ImuData 并发布（仅 imu_publisher 入口）。
    if (imu_pub) {
      msg_pkg::msg::ImuData m;
      m.header = imu_header;
      m.acc_x = acc[0];
      m.acc_y = acc[1];
      m.acc_z = acc[2];
      m.gyro_x = gyro[0];
      m.gyro_y = gyro[1];
      m.gyro_z = gyro[2];
      m.angle_x = angle[0];
      m.angle_y = angle[1];
      m.angle_z = angle[2];
      m.raw_valid = raw_valid;
      m.attitude_valid = attitude_valid;
      imu_pub->publish(m);
    }

// 把缓存整理成 msg_pkg/DepthSensor 并发布（仅 depth 类入口）。
    if (depth_pub) {
      msg_pkg::msg::DepthSensor m;
      m.header = depth_header;
      m.depth = depth;
      m.temp = temp;
      m.pressure_pa = pressure;
      m.fresh = depth_fresh;
      m.depth_valid = depth_valid;
      m.temperature_valid = temp_valid;
      depth_pub->publish(m);
    }

// 追加一行 CSV；setprecision(15) 保证浮点精度。每 10 行 flush 一次，让数据尽快落盘。
    if (file.is_open()) {
      file << std::setprecision(15) << node->now().seconds() << ',' << depth << ',' << temp << ',' << pressure;
      for (const auto &a : {acc, gyro, angle}) {
        for (double v : a) {
          file << ',' << v;
        }
      }
      file << ',' << raw_valid << ',' << attitude_valid << ',' << depth_valid << ','
           << imu_source << ',' << reference_saved_at << '\n';
      if (++rows % 10 == 0) {
        file.flush();
      }
      if (!file) {
        throw std::runtime_error("Recording write failed");
      }
    }

// 用 THROTTLE 限频打印：同一类日志最多每 500ms 一条，避免刷屏。
    if (imu_entry) {
      RCLCPP_INFO_THROTTLE(
          node->get_logger(),
          *node->get_clock(),
          500,
          "angle(deg): %.3f %.3f %.3f | acc(m/s2): %.3f %.3f %.3f | raw=%d attitude=%d",
          angle[0], angle[1], angle[2],
          acc[0], acc[1], acc[2],
          raw_valid, attitude_valid);
    } else if (depth_entry) {
      RCLCPP_INFO_THROTTLE(
          node->get_logger(),
          *node->get_clock(),
          500,
          "depth=%.4f m temp=%.2f C pressure=%.1f Pa valid=%d",
          depth, temp, pressure, depth_valid);
    } else {
      RCLCPP_INFO_THROTTLE(
          node->get_logger(),
          *node->get_clock(),
          500,
          "depth=%.4f m | roll/pitch/yaw=%.3f %.3f %.3f deg | raw=%d attitude=%d depth=%d",
          depth,
          angle[0], angle[1], angle[2],
          raw_valid, attitude_valid, depth_valid);
    }
  }

  if (file.is_open()) {
    file.flush();
    RCLCPP_INFO(node->get_logger(), "Recording closed, %u rows", rows);
  }
  return 0;
}

}  // namespace compatibility

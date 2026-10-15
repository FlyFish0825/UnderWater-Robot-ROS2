// ==============================================================================
// internal/underwater_devices/src/sensors/imu/imu_calibration.cpp
// ==============================================================================
// IMU 姿态参考标定的 ROS 服务与话题封装：
// - 装载与清理参考文件，提供 sensors/imu/calibrate 与 clear 服务；
// - 把原始 ImuState (0x81) 换算为相对基准的修正姿态，发布到 sensors/imu/calibrated；
// - 未标定时修正姿态话题依然发布，但姿态四元数与欧拉角字段置为 NaN。
// ==============================================================================
// 初学者导读（underwater_devices / imu_calibration.cpp）
// 所属：IMU 姿态参考标定的 ROS 封装实现，把纯计算（attitude_reference.*）挂到
//   ROS 服务与话题上。
// 谁调用谁：devices_node.cpp 构造 ImuCalibration，并对每帧 IMU 数据调用 process()。
// 交互：发布 sensors/imu/calibrated、sensors/imu/calibration_status；
//   提供 sensors/imu/calibrate (underwater_interfaces/srv/CalibrateImu) 与
//   sensors/imu/clear_calibration (std_srvs/srv/Trigger) 服务；
//   参数 calibration_file 指定持久化路径。
// ROS 2 概念：create_service 注册服务回调，回调里用 send_response 回复；
//   create_wall_timer 周期执行；node_.now() 取 ROS 时间；QoS 控制发布可靠性。
// C++ 概念：成员初始化列表 (: node_(node), ...)；lambda 捕获 [this]；
//   std::shared_ptr<rmw_request_id_t> 是服务请求标识，回复时必须带上它。
#include "underwater_devices/imu_calibration.hpp"

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace underwater::devices {

namespace ui = underwater_interfaces::msg;

namespace {
constexpr double degrees = 57.295779513082320876;  // 弧度转度

Vector3 degrees_rpy(const Quaternion &q) {
  auto result = quaternion_rpy(q);
  for (auto &v : result) {
    v *= degrees;
  }
  return result;
}

std::string saved_time() {
  const auto now = std::time(nullptr);
  std::tm t{};
  gmtime_r(&now, &t);  // 统一用 UTC
  std::ostringstream out;
  out << std::put_time(&t, "%Y-%m-%dT%H:%M:%SZ");
  return out.str();
}
}  // namespace

// 单调秒：基于 steady_clock，用 duration<double> 转成带小数的秒。
double ImuCalibration::monotonic_seconds() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

ImuCalibration::ImuCalibration(
    rclcpp::Node &node,
    const std::string &frame,
    std::chrono::milliseconds timeout)
    : node_(node),
      frame_(frame),
      timeout_s_(timeout.count() / 1000.0) {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;

  CaptureLimits limits;
  limits.raw_timeout_s = timeout_s_;  // 原始数据超时与测量超时保持一致
  limits.gyro_limit_rad_s = node_.declare_parameter<double>(
      "imu_calibration_max_gyro_rad_s", 0.05, descriptor);
  limits.spread_limit_deg = node_.declare_parameter<double>(
      "imu_calibration_max_spread_deg", 1.0, descriptor);
  limits.min_duration_s = node_.declare_parameter<double>(
      "imu_calibration_min_duration_s", 1.0, descriptor);
  capture_ = ReferenceCapture(limits);

  path_ = node_.declare_parameter<std::string>(
      "imu_calibration_file", "", descriptor);
  const char *home = std::getenv("HOME");
  if (path_.empty()) {
    // 未显式配置时按 ROS_HOME 或 ~/.ros/<命名空间>/imu_reference.txt 推导。
    const char *ros_home = std::getenv("ROS_HOME");
    if ((!ros_home || !*ros_home) && (!home || !*home)) {
      throw std::invalid_argument(
          "set HOME, ROS_HOME or an absolute imu_calibration_file");
    }
    const auto base = (ros_home && *ros_home)
                          ? std::filesystem::path(ros_home)
                          : std::filesystem::path(home) / ".ros";
    const std::string ns = node_.get_namespace();
    path_ = (base / (ns == "/" ? "underwater_robot" : ns.substr(1)) /
             "imu_reference.txt")
                .string();
  } else if (path_.rfind("~/", 0) == 0) {
    if (!home || !*home) {
      throw std::invalid_argument(
          "cannot expand imu_calibration_file without HOME");  // 需要 HOME 才能展开 ~
    }
    path_ = (std::filesystem::path(home) / path_.substr(2)).string();
  }
  if (!std::filesystem::path(path_).is_absolute()) {
    throw std::invalid_argument("imu_calibration_file must be absolute");
  }

  corrected_ = node_.create_publisher<ui::ImuState>(
      "sensors/imu/calibrated", rclcpp::SensorDataQoS().keep_last(20));
  status_ = node_.create_publisher<ui::ImuCalibrationStatus>(
      "sensors/imu/calibration_status",
      rclcpp::QoS(1).transient_local());  // 状态需向新订阅者补发

  try {
    active_ = load_attitude_reference(path_, frame_);
    detail_ = active_ ? "saved attitude reference loaded"
                      : "uncalibrated: explicitly capture a known stationary pose";
  } catch (const std::exception &e) {
    detail_ = "reference load failed: " + std::string(e.what());  // 加载失败不致命，原始数据仍可发布
    RCLCPP_ERROR(
        node_.get_logger(), "%s; raw data remain available", detail_.c_str());
  }

  service_ = node_.create_service<Service>(
      "sensors/imu/calibrate",
      [this](
          std::shared_ptr<rmw_request_id_t> header,
          Service::Request::SharedPtr request) {
        if (pending_) {
          reply(header, false, "another calibration is in progress");  // 同一时刻只允许一次采集
          return;
        }
        try {
          capture_.start(
              request->reference_rpy_deg,
              request->samples,
              request->timeout_s,
              monotonic_seconds());
        } catch (const std::exception &e) {
          reply(header, false, e.what());
          return;
        }
        pending_ = header;
        detail_ = capture_.detail();
        publish_status();
      });

  clear_service_ = node_.create_service<std_srvs::srv::Trigger>(
      "sensors/imu/clear_calibration",
      [this](
          std_srvs::srv::Trigger::Request::SharedPtr,
          std_srvs::srv::Trigger::Response::SharedPtr response) {
        if (pending_) {
          capture_.cancel("calibration cancelled by clear request");
          finish_request();
        }
        try {
          clear_attitude_reference(path_);
          active_.reset();
          detail_ = "reference cleared; calibrated attitude unavailable until a new capture";
          response->success = true;
          if (latest_attitude_) {
            publish_corrected(*latest_attitude_);  // 立即用 NaN 姿态刷新一次输出
          }
        } catch (const std::exception &e) {
          detail_ = "clear failed; reference preserved: " + std::string(e.what());
        }
        response->message = detail_;
        publish_status();
      });

  timer_ = node_.create_wall_timer(
      std::chrono::milliseconds(200), [this] {  // 200 ms 驱动采集超时与状态发布
        capture_.tick(monotonic_seconds());
        finish_request();
        publish_status();
      });

  RCLCPP_INFO(
      node_.get_logger(),
      "IMU attitude reference: %s; file=%s",
      detail_.c_str(),
      path_.c_str());
  publish_status();
}

void ImuCalibration::reply(
    const std::shared_ptr<rmw_request_id_t> &header,
    bool ok,
    const std::string &detail) {
  Service::Response response;
  response.success = ok;
  response.message = detail;
  response.samples_collected = capture_.count();
  response.calibration_file = path_;
  response.captured_rpy_deg.fill(std::numeric_limits<double>::quiet_NaN());  // 默认 NaN，仅成功且有基准时填实际值
  if (ok && active_) {
    response.captured_rpy_deg = degrees_rpy(active_->baseline);
  }
  try {
    service_->send_response(*header, response);
  } catch (const std::exception &e) {
    RCLCPP_WARN(
        node_.get_logger(), "IMU calibration response: %s", e.what());
  }
}

void ImuCalibration::finish_request() {
  if (!pending_ || capture_.busy()) {
    return;  // 仍在采集中则不回复
  }
  bool success = false;
  if (auto candidate = capture_.take_candidate()) {
    candidate->saved_at = saved_time();
    try {
      save_attitude_reference(path_, *candidate);
      active_ = *candidate;
      success = true;  // 写盘成功后才启用新基准
      detail_ = "stationary attitude reference saved and activated";
    } catch (const std::exception &e) {
      detail_ = "save failed; previous reference preserved: " + std::string(e.what());
    }
  } else {
    detail_ = capture_.detail();
  }
  reply(pending_, success, detail_);
  pending_.reset();
  RCLCPP_INFO(node_.get_logger(), "IMU calibration: %s", detail_.c_str());
  publish_status();
}

void ImuCalibration::process(const ui::ImuState &input) {
  const double now = monotonic_seconds();
  if (latest_attitude_ && latest_attitude_->connection_id != input.connection_id) {
    latest_attitude_.reset();  // 重连后旧姿态缓存作废
  }
  if (input.group == 0x80) {
    capture_.raw(
        input.angular_velocity_rad_s,
        input.fresh && input.raw_valid,
        input.connection_id,
        now);
  }
  if (input.group == 0x81) {
    latest_attitude_ = input;
    attitude_at_ = now;
    capture_.attitude(
        input.euler_rpy_rad,
        input.fresh && input.euler_valid,
        input.connection_id,
        input.header.frame_id,
        now);
  }
  finish_request();
  publish_corrected(input);
}

void ImuCalibration::publish_corrected(const ui::ImuState &input) {
  auto output = input;
  if (input.group == 0x81) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    output.quaternion_valid = output.euler_valid = false;  // 先假定无有效姿态，再按条件填充
    output.quaternion_wxyz.fill(nan);
    output.euler_rpy_rad.fill(nan);
    // 相对姿态计算：传感器坐标系的角速度/加速度与 frame_id 保持不变
    if (active_ && input.fresh && input.euler_valid &&
        monotonic_seconds() - attitude_at_ <= timeout_s_) {  // 姿态数据超时则不换算
      try {
        output.quaternion_wxyz = reference_attitude(*active_, input.euler_rpy_rad);
        output.euler_rpy_rad = quaternion_rpy(output.quaternion_wxyz);
        output.quaternion_valid = output.euler_valid = true;
      } catch (const std::exception &e) {
        detail_ = "invalid attitude: " + std::string(e.what());
      }
    }
  }
  corrected_->publish(output);
}

void ImuCalibration::publish_status() {
  ui::ImuCalibrationStatus status;
  status.header.stamp = node_.now();
  status.header.frame_id = frame_;
  status.calibrated = active_.has_value();
  status.collecting = capture_.busy();
  status.attitude_fresh = latest_attitude_ && latest_attitude_->fresh &&
                          latest_attitude_->euler_valid &&
                          monotonic_seconds() - attitude_at_ <= timeout_s_;
  status.samples_collected = capture_.count();
  status.samples_requested = capture_.requested();
  status.captured_rpy_deg.fill(std::numeric_limits<double>::quiet_NaN());  // 无基准时角度字段为 NaN
  status.reference_rpy_deg.fill(std::numeric_limits<double>::quiet_NaN());
  if (active_) {
    status.captured_rpy_deg = degrees_rpy(active_->baseline);
    status.reference_rpy_deg = active_->target_rpy_deg;
    status.saved_at = active_->saved_at;
  }
  status.calibration_file = path_;
  status.detail = capture_.busy() ? capture_.detail() : detail_;  // 采集中优先显示采集进度
  status_->publish(status);
}

}  // namespace underwater::devices

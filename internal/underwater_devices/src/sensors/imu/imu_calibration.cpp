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
#include <ctime>
#include <iomanip>
#include <sstream>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace underwater::devices {

// 单调秒：基于 steady_clock，用 duration<double> 转成带小数的秒。
double ImuCalibration::monotonic_seconds() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

// 构造函数：声明参数、装载旧标定、创建发布者与服务。
ImuCalibration::ImuCalibration(
    rclcpp::Node &node,
    const std::string &frame,
    std::chrono::milliseconds timeout)
    : node_(node),
      frame_(frame),
      timeout_s_(std::chrono::duration<double>(timeout).count()) {
  // ParameterDescriptor：read_only=true 表示该参数在运行期不可修改。
  rcl_interfaces::msg::ParameterDescriptor d;
  d.read_only = true;

  path_ = node_.declare_parameter<std::string>(
      "calibration_file",
      "/var/lib/underwater_robot/imu_reference.txt",
      d);

  // 尝试加载已有标定文件；load 失败会抛异常，这里捕获并给出告警。
  // 尝试从持久化文件装载已有基准
  try {
    active_ = load_attitude_reference(path_, frame_);
    if (active_) {
      detail_ = "active attitude reference loaded from " + path_;
      RCLCPP_INFO(node_.get_logger(), "%s", detail_.c_str());
    } else {
      detail_ = "no attitude reference file found; uncalibrated";
    }
  } catch (const std::exception &e) {
    detail_ = "failed to load reference: " + std::string(e.what());
    RCLCPP_WARN(node_.get_logger(), "%s", detail_.c_str());
  }

  corrected_ = node_.create_publisher<underwater_interfaces::msg::ImuState>(
      "sensors/imu/calibrated",
      rclcpp::SensorDataQoS().keep_last(100));

  status_ = node_.create_publisher<underwater_interfaces::msg::ImuCalibrationStatus>(
      "sensors/imu/calibration_status",
      rclcpp::QoS(10));

  // 标定服务回调：收到请求后启动采集，并把请求句柄暂存，等采集完成再回复。
  // 标定触发服务
  service_ = node_.create_service<Service>(
      "sensors/imu/calibrate",
      [this](
          const std::shared_ptr<rmw_request_id_t> header,
          const std::shared_ptr<Service::Request> request) {
        if (capture_.busy()) {
          reply(header, false, "calibration already in progress");
          return;
        }

        try {
          const Vector3 target{
              request->target_roll_deg,
              request->target_pitch_deg,
              request->target_yaw_deg};
          capture_.start(target, request->samples, request->timeout_s, monotonic_seconds());
          pending_ = header;
          detail_ = "collecting stationary attitude samples";
        } catch (const std::exception &e) {
          reply(header, false, e.what());
        }
      });

  // 清除标定服务：取消进行中的采集并删除文件。
  // 清除标定服务
  clear_service_ = node_.create_service<std_srvs::srv::Trigger>(
      "sensors/imu/clear_calibration",
      [this](
          const std::shared_ptr<rmw_request_id_t> header,
          const std::shared_ptr<std_srvs::srv::Trigger::Request>) {
        if (capture_.busy()) {
          capture_.cancel("cleared by service call");
          finish_request();
        }
        try {
          clear_attitude_reference(path_);
          active_.reset();
          detail_ = "calibration cleared; uncalibrated";
          std_srvs::srv::Trigger::Response res;
          res.success = true;
          res.message = detail_;
          clear_service_->send_response(*header, res);
        } catch (const std::exception &e) {
          std_srvs::srv::Trigger::Response res;
          res.success = false;
          res.message = e.what();
          clear_service_->send_response(*header, res);
        }
        publish_status();
      });

  // 定时器：驱动采集超时检查、发现采集完成并回复请求、周期发布状态。
  // 100ms 周期状态广播定时器
  timer_ = node_.create_wall_timer(
      std::chrono::milliseconds(100),
      [this] {
        capture_.tick(monotonic_seconds());
        if (!capture_.busy() && pending_) {
          finish_request();
        }
        publish_status();
      });
}

// reply()：统一构造响应并发送给挂起的请求。
void ImuCalibration::reply(
    const std::shared_ptr<rmw_request_id_t> &header,
    bool ok,
    const std::string &msg) {
  Service::Response res;
  res.success = ok;
  res.message = msg;
  service_->send_response(*header, res);
}

// finish_request()：采集结束时决定成功/失败；成功则原子保存后再启用新基准。
void ImuCalibration::finish_request() {
  if (!pending_) {
    return;
  }
  const auto header = pending_;
  pending_.reset();

  auto candidate = capture_.take_candidate();
  if (!candidate) {
    reply(header, false, capture_.detail());
    detail_ = capture_.detail();
    return;
  }

  // 生成 UTC ISO-8601 时间戳，记录标定时刻。
  // 生成 UTC ISO-8601 时间戳
  const auto now = std::chrono::system_clock::now();
  const auto time_t_now = std::chrono::system_clock::to_time_t(now);
  std::ostringstream ss;
  ss << std::put_time(std::gmtime(&time_t_now), "%Y-%m-%dT%H:%M:%SZ");
  candidate->saved_at = ss.str();

  // 必须先保存成功，active_ 才允许指向新基准（见头文件约定）。
  // 持久化保存新标定基准
  try {
    save_attitude_reference(path_, *candidate);
    active_ = candidate;
    detail_ = "new attitude reference calibrated and saved to " + path_;
    reply(header, true, detail_);
  } catch (const std::exception &e) {
    detail_ = "failed to save reference: " + std::string(e.what());
    reply(header, false, detail_);
  }
}

// process()：每帧 ImuState 调用；0x80 喂原始角速度，0x81 喂姿态并发布修正姿态。
void ImuCalibration::process(const underwater_interfaces::msg::ImuState &input) {
  const double now = monotonic_seconds();

  if (input.group == 0x80) {
    capture_.raw(
        {input.angular_velocity_rad_s[0], input.angular_velocity_rad_s[1], input.angular_velocity_rad_s[2]},
        input.raw_valid,
        input.connection_id,
        now);
  } else if (input.group == 0x81) {
    capture_.attitude(
        {input.euler_rpy_rad[0], input.euler_rpy_rad[1], input.euler_rpy_rad[2]},
        input.euler_valid,
        input.connection_id,
        frame_,
        now);
    publish_corrected(input);
  }
}

// publish_corrected()：把原始姿态换算成相对基准姿态后发布；未标定时字段置 NaN。
void ImuCalibration::publish_corrected(const underwater_interfaces::msg::ImuState &input) {
  auto out = input;

  // 只有 active_ 存在且姿态有效才做换算，否则明确置为无效。
  if (active_ && input.euler_valid) {
    try {
      const auto q = reference_attitude(
          *active_,
          {input.euler_rpy_rad[0], input.euler_rpy_rad[1], input.euler_rpy_rad[2]});
      const auto rpy = quaternion_rpy(q);

      out.quaternion_valid = true;
      out.euler_valid = true;
      out.quaternion_wxyz = q;
      out.euler_rpy_rad = rpy;
    } catch (...) {
      out.quaternion_valid = false;
      out.euler_valid = false;
      out.quaternion_wxyz.fill(invalid());
      out.euler_rpy_rad.fill(invalid());
    }
  } else {
    // 未标定时，姿态字段明确赋 NaN
    out.quaternion_valid = false;
    out.euler_valid = false;
    out.quaternion_wxyz.fill(invalid());
    out.euler_rpy_rad.fill(invalid());
  }

  corrected_->publish(out);
}

// publish_status()：把当前标定状态打包发布，供 GUI/诊断使用。
void ImuCalibration::publish_status() {
  underwater_interfaces::msg::ImuCalibrationStatus status;
  status.header.stamp = node_.now();
  status.header.frame_id = frame_;
  status.calibrated = active_.has_value();
  status.capturing = capture_.busy();
  status.samples_collected = capture_.count();
  status.samples_requested = capture_.requested();

  if (active_) {
    status.baseline_quaternion = active_->baseline;
    status.target_rpy_deg = active_->target_rpy_deg;
    status.saved_at = active_->saved_at;
  }
  status.detail = detail_;
  status_->publish(status);
}

}  // namespace underwater::devices

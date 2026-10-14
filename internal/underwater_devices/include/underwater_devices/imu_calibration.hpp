// IMU 姿态参考标定组件：负责 sensors/imu/calibrate 服务、参考文件的装载与清理，
// 并把原始 ImuState 换算成“相对标定基准”的修正姿态发布到 sensors/imu/calibrated。
// 与线协议解码解耦，只消费已解析的 ImuState，不直接接触串口数据。
// 初学者导读（underwater_devices / imu_calibration.hpp）
// 所属：IMU 姿态参考标定的 ROS 封装层。它把“原始姿态 → 相对基准姿态”的换算
//   打包发布到 sensors/imu/calibrated，并提供标定/清除服务。
// 谁调用谁：devices_node.cpp 创建 ImuCalibration，并在每帧 IMU 数据到来时调用 process()；
//   本类的服务回调由 ROS executor 触发。
// 交互：发布 sensors/imu/calibrated 与 sensors/imu/calibration_status；
//   提供 sensors/imu/calibrate (CalibrateImu) 与 clear_calibration (Trigger) 服务；
//   读取参数 calibration_file（标定文件路径）。
// ROS 2 概念：Service 是一问一答的远程调用，用 send_response 回复；Parameter 是节点可
//   配置项，用 declare_parameter 声明；Publisher 负责发布话题消息。
// C++ 概念：std::shared_ptr 是共享所有权的智能指针（ROS 接口对象常用它）；lambda 里的
//   [this] 捕获当前对象；std::chrono 表示时间长度/时刻。
#pragma once
#include "underwater_devices/attitude_reference.hpp"
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <underwater_interfaces/msg/imu_state.hpp>
#include <underwater_interfaces/msg/imu_calibration_status.hpp>
#include <underwater_interfaces/srv/calibrate_imu.hpp>

namespace underwater::devices {
// Downstream attitude reference, separate from wire decoding and raw sensor topics.
class ImuCalibration {
public:
  ImuCalibration(rclcpp::Node &node,const std::string &frame,std::chrono::milliseconds timeout);  // timeout 为姿态数据最大陈旧时间
  void process(const underwater_interfaces::msg::ImuState &input);  // 每帧调用：喂采集器并发布修正姿态
private:
  using Service=underwater_interfaces::srv::CalibrateImu;
  static double monotonic_seconds();  // steady_clock 单调秒
  void finish_request();  // 采集结束或取消时回复挂起的服务请求
  void reply(const std::shared_ptr<rmw_request_id_t> &header,bool ok,const std::string &detail);
  void publish_status();
  void publish_corrected(const underwater_interfaces::msg::ImuState &input);  // 仅对 0x81 姿态帧做换算
  rclcpp::Node &node_;
  std::string frame_,path_,detail_;
  double timeout_s_;
  ReferenceCapture capture_;
  std::optional<AttitudeReference> active_;  // 当前生效基准；未标定时为 nullopt
  std::optional<underwater_interfaces::msg::ImuState> latest_attitude_;  // 最近一帧 0x81 姿态缓存
  double attitude_at_=0;
  rclcpp::Publisher<underwater_interfaces::msg::ImuState>::SharedPtr corrected_;
  rclcpp::Publisher<underwater_interfaces::msg::ImuCalibrationStatus>::SharedPtr status_;
  rclcpp::Service<Service>::SharedPtr service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
  std::shared_ptr<rmw_request_id_t> pending_;  // 正在等待采集结果的服务请求
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace underwater::devices

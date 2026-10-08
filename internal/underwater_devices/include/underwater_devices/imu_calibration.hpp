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
  ImuCalibration(rclcpp::Node &node,const std::string &frame,std::chrono::milliseconds timeout);
  void process(const underwater_interfaces::msg::ImuState &input);
private:
  using Service=underwater_interfaces::srv::CalibrateImu;
  static double monotonic_seconds();
  void finish_request();
  void reply(const std::shared_ptr<rmw_request_id_t> &header,bool ok,const std::string &detail);
  void publish_status();
  void publish_corrected(const underwater_interfaces::msg::ImuState &input);
  rclcpp::Node &node_;
  std::string frame_,path_,detail_;
  double timeout_s_;
  ReferenceCapture capture_;
  std::optional<AttitudeReference> active_;
  std::optional<underwater_interfaces::msg::ImuState> latest_attitude_;
  double attitude_at_=0;
  rclcpp::Publisher<underwater_interfaces::msg::ImuState>::SharedPtr corrected_;
  rclcpp::Publisher<underwater_interfaces::msg::ImuCalibrationStatus>::SharedPtr status_;
  rclcpp::Service<Service>::SharedPtr service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
  std::shared_ptr<rmw_request_id_t> pending_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace underwater::devices

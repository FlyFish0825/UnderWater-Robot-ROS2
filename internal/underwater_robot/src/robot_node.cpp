#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <stdexcept>

#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace underwater_robot
{

// Software heartbeat only. Hardware interfaces are added in separate modules.
class RobotNode final : public rclcpp::Node
{
public:
  RobotNode()
  : Node("robot_node")
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Software status publication period in milliseconds.";
    descriptor.read_only = true;
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = 1;
    range.to_value = 3600000;
    range.step = 1;
    descriptor.integer_range.push_back(range);

    const auto period_ms =
      declare_parameter<std::int64_t>("status_period_ms", 1000, descriptor);
    if (period_ms < 1 || period_ms > 3600000) {
      throw std::invalid_argument("status_period_ms must be in [1, 3600000]");
    }

    status_publisher_ = create_publisher<std_msgs::msg::String>("status", 10);
    status_timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      [this]() {
        std_msgs::msg::String message;
        message.data = "node_running";
        status_publisher_->publish(message);
      });

    RCLCPP_INFO(
      get_logger(), "Software heartbeat started; period=%ld ms; hardware state is on gateway/status.",
      static_cast<long>(period_ms));
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace underwater_robot

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<underwater_robot::RobotNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("robot_node"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}

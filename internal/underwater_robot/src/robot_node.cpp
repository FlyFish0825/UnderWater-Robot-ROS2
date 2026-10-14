// robot_node.cpp —— 机器人主节点的“软件心跳”占位实现（节点名 robot_node，话题 status，队列 10）。
// 本节点不接触硬件、不涉及 TCP 端口；硬件状态由 underwater_mcu_gateway 发布在 gateway/status。
// 参数 status_period_ms（只读，范围 [1,3600000]，默认 1000 毫秒）：心跳发布周期，改动需重启节点。
//
// ── 初学者导读 ────────────────────────────────────────────────────────────
// robot_node 是整个软件栈的“软件心跳”示例节点：它不碰硬件、不开端口，只周期性地
// 往 <命名空间>/status 发一条字符串 "node_running"，用来证明“这个进程还活着”。
// 硬件是否在线要看 underwater_mcu_gateway 发布的 gateway/status，二者不要混淆。
// ROS 2 概念：
//   - Node：继承 rclcpp::Node 的 RobotNode；create_publisher 用来发消息；
//   - Parameter：declare_parameter 声明参数，这里用 IntegerRange 限定取值范围；
//   - Timer：create_wall_timer 按墙上时间周期性触发回调（回调是 lambda）；
//   - spin：main 里 rclcpp::spin 让节点一直处理回调，直到 Ctrl+C。
// C++ 概念：final；智能指针 SharedPtr；lambda [this] 捕获；异常 try/catch。
// ─────────────────────────────────────────────────────────────────────────
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
// 仅做软件心跳：周期性发布 "node_running"，用于确认节点存活。
// RobotNode：只有“发布心跳”一个职责的最小节点。
class RobotNode final : public rclcpp::Node
{
public:
  RobotNode()
  : Node("robot_node")
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Software status publication period in milliseconds.";
    descriptor.read_only = true;  // 只读参数，运行期不允许动态修改
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = 1;
    range.to_value = 3600000;  // 上限 1 小时，避免过大的定时间隔
    range.step = 1;
    descriptor.integer_range.push_back(range);

    // declare_parameter<类型>(名字, 默认值, 描述符)：声明并读取参数，可被 --ros-args -p 覆盖。
    const auto period_ms =
      declare_parameter<std::int64_t>("status_period_ms", 1000, descriptor);  // 默认 1000 毫秒
    if (period_ms < 1 || period_ms > 3600000) {
      throw std::invalid_argument("status_period_ms must be in [1, 3600000]");
    }

    // create_publisher<消息类型>(话题名, 队列深度)：创建发布者。
    status_publisher_ = create_publisher<std_msgs::msg::String>("status", 10);  // 话题 <ns>/status，队列 10
    // create_wall_timer(周期, 回调)：到点就执行一次回调，常用于周期任务。
    status_timer_ = create_wall_timer(  // 墙钟定时器：按 period_ms 触发心跳
      std::chrono::milliseconds(period_ms),
      [this]() {
        std_msgs::msg::String message;
        message.data = "node_running";  // 固定载荷，仅表示进程存活
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

// 进程入口：单线程 spin；参数非法或运行异常时记录错误并返回退出码 1。
// 进程入口：rclcpp::init 初始化 -> rclcpp::spin 运行节点 -> shutdown 收尾。
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

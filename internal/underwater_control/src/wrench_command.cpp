// ==============================================================================
// internal/underwater_control/src/wrench_command.cpp
// ==============================================================================
// 命令行 wrench 调试发送工具：
// 按固定 20 Hz 频率向 control/wrench_request 持续发布期望 wrench。
// 参数顺序：Fx, Fy, Fz (N), Mx, My, Mz (N·m)；
// 发布结束后补发 5 帧零力矩；即使提前中断，推力分配器的输入看门狗也会在超时后自动停机。
// ==============================================================================
// ------------------------------------------------------------------------------
// 【初学者导读】本文件是一个命令行的“调试发送器”，节点名 "wrench_command"。
// 用途：手动给推力分配节点发一个期望 wrench，观察电机转速是否符合预期。
// 与谁交互：发布话题 control/wrench_request (geometry_msgs/WrenchStamped)。
// ROS 2 概念白话：
//   Node＝独立小程序；Publisher＝发布消息的出口；
//   QoS(1)＝只保留最新一条；spin_some＝不阻塞地处理一下待办回调；
//   参数 Parameter＝可通过 --ros-args -p 覆盖的配置（frame_id、duration_s）。
// C++ 概念：
//   std::stod＝字符串转 double；std::array＝定长数组；
//   lambda send＝把“发一帧”的代码封装成可复用的小函数。
// 安全：结束时补发 5 帧零力矩；即使被 Ctrl+C 中断，分配节点的输入看门狗也会超时停机。
// ------------------------------------------------------------------------------
#include <array>
// 标准库：定长数组、时钟、数学、输入输出、异常、字符串。
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

// ROS 2 客户端库与消息类型。
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

// main：解析命令行参数 -> 建节点 -> 等分配器订阅 -> 按 20Hz 发布 -> 收尾发零。
// 返回值：0 成功，1 运行异常，2 参数个数不对。
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // 去除 ROS 专用参数后，必须保留 6 个 wrench 浮点数值
  const auto args = rclcpp::remove_ros_arguments(argc, argv);
  if (args.size() != 7) {
    std::cerr << "Usage: wrench_command Fx Fy Fz Mx My Mz [--ros-args ...]\n";
    rclcpp::shutdown();
    return 2;
  }

  std::array<double, 6> values{};
  try {
    for (int i = 0; i < 6; ++i) {
      std::size_t used = 0;
      values[i] = std::stod(args[i + 1], &used);
      if (used != args[i + 1].size() || !std::isfinite(values[i]) || std::abs(values[i]) > 1e6) {
        throw std::invalid_argument("invalid wrench value: must be finite and within 1e6");
      }
    }

// 创建节点并读取两个参数：坐标系名 frame_id 与持续时间 duration_s。
    auto node = std::make_shared<rclcpp::Node>("wrench_command");
    const auto frame = node->declare_parameter<std::string>("frame_id", "thruster_center");
    const double duration = node->declare_parameter<double>("duration_s", 5.0);

    if (!std::isfinite(duration) || duration <= 0 || duration > 60) {
      throw std::invalid_argument("duration_s must be within (0, 60] seconds");
    }

// 创建发布者，话题与分配节点订阅的保持一致，QoS 深度 1。
    auto pub = node->create_publisher<geometry_msgs::msg::WrenchStamped>(
        "control/wrench_request", rclcpp::QoS(1));

    // 等待分配器节点完成订阅 (最多等待 3 秒)
    const auto wait_end = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (rclcpp::ok() && pub->get_subscription_count() == 0 && std::chrono::steady_clock::now() < wait_end) {
      rclcpp::spin_some(node);
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }

    if (pub->get_subscription_count() == 0) {
      throw std::runtime_error("No allocator subscribed to control/wrench_request");
    }

// send lambda：把当前 wrench 值（或全零）打包成一帧 WrenchStamped 发出去。
// zero=true 时只填时间戳与坐标系，六维数值保持默认 0，用于安全停车。
    auto send = [&](bool zero) {
      geometry_msgs::msg::WrenchStamped w;
      w.header.stamp = node->now();
      w.header.frame_id = frame;
      if (!zero) {
        w.wrench.force.x = values[0];
        w.wrench.force.y = values[1];
        w.wrench.force.z = values[2];
        w.wrench.torque.x = values[3];
        w.wrench.torque.y = values[4];
        w.wrench.torque.z = values[5];
      }
      pub->publish(w);
    };

    // 20Hz 循环持续发布期望 wrench
// 主循环：按 duration_s 指定的时长持续发布期望 wrench。
    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < duration) {
      send(false);
      rclcpp::spin_some(node);
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }

    // 结束阶段连续下发 5 帧全零安全停机指令
    for (int i = 0; i < 5 && rclcpp::ok(); ++i) {
      send(true);
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "Wrench stream finished; zero sent. Input watchdog also stops on interruption.\n";
    node.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << "\n";
    rclcpp::shutdown();
    return 1;
  }
}

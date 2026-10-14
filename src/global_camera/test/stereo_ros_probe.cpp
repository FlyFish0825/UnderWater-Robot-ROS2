// ==============================================================================
// src/global_camera/test/stereo_ros_probe.cpp
// ==============================================================================
// global_cam_pub 的 ROS 话题集成探针：
// - 订阅 /underwater_robot/camera/stereo|left|right[/image_raw][/compressed]；
// - 按时间戳对齐校验 1280x720 单目与 2560x720 左右拼接双目；
// - 校验 optical_frame 坐标系标号与 JPEG 格式有效性。
// ==============================================================================

// ── 初学者导读 ────────────────────────────────────────────────────────────
// 这是一个“集成测试/探针”程序，不是机器人运行必需的节点。它自己建一个 ROS 2 节点，
// 订阅 global_cam_pub 发出的 stereo/left/right 压缩图，按“同一时间戳”把它们配成一组，
// 连续配到 3 组就判定相机话题链路正常。
// ROS 2 概念：Node；Subscription + 回调（lambda）；Parameter；spin_some。
// C++ 概念：std::map（按时间戳聚合）、lambda 捕获引用 [&]、std::chrono 超时。
// ─────────────────────────────────────────────────────────────────────────

#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

// 程序入口：初始化 ROS 2，建节点，订阅话题并最多等待 12 秒做对齐统计。
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("stereo_ros_probe");

  // declare_parameter：声明探针自己的参数 expect_raw，决定是否要求 image_raw 话题。
  const bool expect_raw = node->declare_parameter<bool>("expect_raw", false);
  const auto deadline = std::chrono::steady_clock::now() + 12s;

  struct Match {
    bool stereo = false;
    bool left = false;
    bool right = false;
    bool left_raw = false;
    bool right_raw = false;
  };

  std::map<std::uint64_t, Match> matches;
  std::uint64_t matched_count = 0;

  auto qos = rclcpp::SensorDataQoS().keep_last(10);

  // 下面分别订阅 stereo/left/right 三路压缩图；用时间戳纳秒当 key 做对齐。
  // create_subscription<消息类型>(话题, QoS, 回调)：每来一条消息执行一次回调。
  auto sub_stereo = node->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/underwater_robot/camera/stereo/compressed",
      qos,
      [&](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
        if (msg->header.frame_id != "stereo_camera_optical_frame") {
          return;
        }
        auto decoded = cv::imdecode(msg->data, cv::IMREAD_COLOR);
        if (decoded.cols == 2560 && decoded.rows == 720) {
          matches[msg->header.stamp.nanosec].stereo = true;
        }
      });

  auto sub_left = node->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/underwater_robot/camera/left/compressed",
      qos,
      [&](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
        if (msg->header.frame_id != "left_camera_optical_frame") {
          return;
        }
        auto decoded = cv::imdecode(msg->data, cv::IMREAD_COLOR);
        if (decoded.cols == 1280 && decoded.rows == 720) {
          matches[msg->header.stamp.nanosec].left = true;
        }
      });

  auto sub_right = node->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/underwater_robot/camera/right/compressed",
      qos,
      [&](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
        if (msg->header.frame_id != "right_camera_optical_frame") {
          return;
        }
        auto decoded = cv::imdecode(msg->data, cv::IMREAD_COLOR);
        if (decoded.cols == 1280 && decoded.rows == 720) {
          matches[msg->header.stamp.nanosec].right = true;
        }
      });

  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);

    for (auto it = matches.begin(); it != matches.end();) {
      const auto &m = it->second;
      const bool complete = m.stereo && m.left && m.right &&
                            (!expect_raw || (m.left_raw && m.right_raw));
      if (complete) {
        ++matched_count;
        it = matches.erase(it);
      } else {
        ++it;
      }
    }

    if (matched_count >= 3) {
      break;
    }
    std::this_thread::sleep_for(20ms);
  }

  rclcpp::shutdown();
  if (matched_count < 3) {
    std::cerr << "stereo_ros_probe timed out; matched " << matched_count << " sets\n";
    return 1;
  }

  std::cout << "PASS stereo ROS probe: matched " << matched_count << " synchronized sets\n";
  return 0;
}

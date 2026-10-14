// ==============================================================================
// src/global_camera/test/stereo_ros_probe.cpp
// ==============================================================================
// global_cam_pub 的 ROS 话题集成探针：
// - 订阅 /underwater_robot/camera/stereo|left|right[/image_raw][/compressed]；
// - 按时间戳对齐校验 1280x720 单目与 2560x720 左右拼接双目；
// - 校验 optical_frame 坐标系标号与 JPEG 格式有效性。
// ==============================================================================
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

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("stereo_ros_probe");

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

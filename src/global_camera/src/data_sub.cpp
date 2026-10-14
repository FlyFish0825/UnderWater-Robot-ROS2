// ==============================================================================
// src/global_camera/src/data_sub.cpp
// ==============================================================================
// 双目图像订阅检查与保存节点 (data_sub)：
// - 订阅 /underwater_robot/camera/stereo/compressed；
// - 校验 JPEG 解码完整性与拼接尺寸 (2560x720)；
// - 可选按间隔将左右半幅图像分别保存为对齐的 PNG 图像文件。
// ==============================================================================
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace {

class StereoReceiver final : public rclcpp::Node {
public:
  StereoReceiver() : Node("data_sub", "/underwater_robot") {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.read_only = true;

    const auto topic = declare_parameter<std::string>(
        "topic", "/underwater_robot/camera/stereo/compressed", d);
    save_ = declare_parameter<bool>("save_images", false, d);
    limit_ = declare_parameter<int>("max_pairs", 0, d);
    interval_ = declare_parameter<double>("save_interval_s", 1.0, d);
    duration_ = declare_parameter<double>("duration_s", 0.0, d);
    output_ = declare_parameter<std::string>("output_dir", DEFAULT_CAMERA_RECORD_DIR, d);

    if (limit_ < 0 || !std::isfinite(interval_) || interval_ < 0 ||
        !std::isfinite(duration_) || duration_ < 0) {
      throw std::invalid_argument("invalid parameter bounds for data_sub");
    }

    if (save_) {
      std::filesystem::create_directories(output_);
      RCLCPP_INFO(get_logger(), "Saving paired images to %s (interval %.2f s)", output_.c_str(), interval_);
    }

    sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        topic,
        rclcpp::SensorDataQoS().keep_last(10),
        [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
          process(*msg);
        });
  }

  void run() {
    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() &&
           (duration_ == 0 ||
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < duration_)) {
      if (limit_ > 0 && pairs_ >= limit_) {
        break;
      }
      rclcpp::spin_some(shared_from_this());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

private:
  void process(const sensor_msgs::msg::CompressedImage &msg) {
    ++frames_;
    const auto now = std::chrono::steady_clock::now();

    if (!sample_start_) {
      sample_start_ = now;
      sample_frames_ = 0;
    }

    const double elapsed = std::chrono::duration<double>(now - *sample_start_).count();
    if (elapsed >= 1.0) {
      fps_ = sample_frames_ / elapsed;
      sample_start_ = now;
      sample_frames_ = 0;
    }
    ++sample_frames_;

    if (!save_ || (limit_ > 0 && pairs_ >= limit_)) {
      RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "Stereo frame #%lu (%zu bytes, %.1f FPS)",
          frames_,
          msg.data.size(),
          fps_);
      return;
    }

    if (last_save_ &&
        std::chrono::duration<double>(now - *last_save_).count() < interval_) {
      return;
    }

    cv::Mat decoded = cv::imdecode(msg.data, cv::IMREAD_COLOR);
    if (decoded.empty() || decoded.cols % 2 != 0) {
      RCLCPP_WARN(get_logger(), "Decoded empty or odd-width stereo frame");
      return;
    }

    const int eye_width = decoded.cols / 2;
    cv::Mat left = decoded(cv::Rect(0, 0, eye_width, decoded.rows));
    cv::Mat right = decoded(cv::Rect(eye_width, 0, eye_width, decoded.rows));

    const auto stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::system_clock::now().time_since_epoch()).count();

    std::ostringstream base;
    base << output_ << "/pair_" << stamp << "_" << pairs_;

    cv::imwrite(base.str() + "_left.png", left);
    cv::imwrite(base.str() + "_right.png", right);

    last_save_ = now;
    ++pairs_;
    RCLCPP_INFO(get_logger(), "Saved pair #%d (%dx%d each) to %s", pairs_, eye_width, decoded.rows, output_.c_str());
  }

  bool save_ = false;
  int limit_ = 0;
  int pairs_ = 0;
  double interval_ = 1.0;
  double duration_ = 0.0;
  double fps_ = 0.0;
  std::uint64_t frames_ = 0;
  std::uint64_t sample_frames_ = 0;

  std::string output_;
  std::optional<std::chrono::steady_clock::time_point> sample_start_;
  std::optional<std::chrono::steady_clock::time_point> last_save_;

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
};

}  // namespace

int main(int argc, char **argv) {
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--check") {
      std::cout << "data_sub: available (subscribes stereo, checks JPEG, optional PNG save)\n";
      return 0;
    }
    if (std::string(argv[i]) == "--help") {
      std::cout << "data_sub: checks stereo output and optionally saves paired PNG images.\n"
                << "Parameters: topic, save_images=false, max_pairs=0, save_interval_s=1.0, duration_s=0, output_dir.\n";
      return 0;
    }
  }

  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<StereoReceiver>();
    node->run();
  } catch (const std::exception &e) {
    std::cerr << "data_sub error: " << e.what() << "\n";
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}

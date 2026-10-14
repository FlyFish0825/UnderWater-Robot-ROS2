// ==============================================================================
// src/global_camera/src/data_sub.cpp
// ==============================================================================
// 双目图像订阅检查与保存节点 (data_sub)：
// - 订阅 /underwater_robot/camera/stereo/compressed；
// - 校验 JPEG 解码完整性与拼接尺寸 (2560x720)；
// - 可选按间隔将左右半幅图像分别保存为对齐的 PNG 图像文件。
// ==============================================================================

// ── 初学者导读 ────────────────────────────────────────────────────────────
// 这是 global_camera 里的“检查/保存”入口（可执行文件 data_sub）。它不驱动相机，
// 而是订阅另一路话题 /underwater_robot/camera/stereo/compressed（双目拼接后的 JPEG），
// 自己解码确认图像完整，并可选把左、右两半分别存成 PNG，用来做离线核对与实验记录。
// 会出现的 ROS 2 概念：
//   - Node：继承 rclcpp::Node 的 StereoReceiver 就是一个节点；构造时传入节点名 "data_sub"
//     与命名空间 "/underwater_robot"，所以它的话题都落在该命名空间下。
//   - Parameter（参数）：declare_parameter 声明可在命令行用 --ros-args -p 覆盖的配置项。
//   - Subscription（订阅）：create_subscription<消息类型>(话题, QoS, 回调)；
//     每收到一条消息就自动执行一次回调（这里用 lambda）。
//   - QoS：这里用 SensorDataQoS().keep_last(10)，表示“传感器数据”策略，只缓存最近 10 条。
//   - spin_some：处理一次当前已到达的回调，然后立刻返回。
// 会出现的 C++ 概念：
//   - final：禁止其它类再继承 StereoReceiver。
//   - lambda：[this](msg){ process(*msg); } 捕获当前对象，把消息交给成员函数处理。
//   - std::optional：可能“还没有值”的变量（如上次保存时刻），用 has_value/解引用判断。
//   - std::chrono：时间库；steady_clock 是单调时钟（适合计时），system_clock 是墙上时钟。
// ─────────────────────────────────────────────────────────────────────────

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

// StereoReceiver：双目接收节点。构造时声明参数、建立订阅；run() 驱动事件循环。
// 核心工作流：收到压缩图 → 统计帧率 → 按需解码 → 左右切分 → 存 PNG。
class StereoReceiver final : public rclcpp::Node {
public:
  StereoReceiver() : Node("data_sub", "/underwater_robot") {
    // ParameterDescriptor：描述一个参数的属性；read_only=true 表示运行期不可被外部改动。
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

    // 订阅压缩图像话题；回调里调用 process()。SensorDataQoS 适合高频传感器流。
    sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        topic,
        rclcpp::SensorDataQoS().keep_last(10),
        [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
          process(*msg);
        });
  }

  // 主循环：在 duration_s 时间内反复 spin_some（处理订阅回调）并小睡 10ms；
  // 达到 max_pairs 或超时、或收到 Ctrl+C（rclcpp::ok() 变 false）就退出。
  void run() {
    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() &&
           (duration_ == 0 ||
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < duration_)) {
      if (limit_ > 0 && pairs_ >= limit_) {
        break;
      }
      // spin_some：只处理“此刻已排队”的回调，随后立刻返回，适合自己掌控节奏的循环。
      rclcpp::spin_some(shared_from_this());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

private:
  // 每收到一帧就调用：先算 FPS，再决定是否按间隔保存左右目 PNG。
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

    // 把 JPEG 字节解码成 BGR 图像矩阵；解码失败会得到空 Mat。
    cv::Mat decoded = cv::imdecode(msg.data, cv::IMREAD_COLOR);
    if (decoded.empty() || decoded.cols % 2 != 0) {
      RCLCPP_WARN(get_logger(), "Decoded empty or odd-width stereo frame");
      return;
    }

    // 拼接图左右各占一半；用 cv::Rect 取“感兴趣区域”得到左目/右目子图。
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

// 程序入口。先处理 --check / --help（不接触硬件，方便脚本探测）；
// 否则 rclcpp::init 初始化 ROS 2、创建节点并运行，最后 rclcpp::shutdown。
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

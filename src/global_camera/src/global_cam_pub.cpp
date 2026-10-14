// ==============================================================================
// src/global_camera/src/global_cam_pub.cpp
// ==============================================================================
// 双目相机独立采集与发布节点 (global_cam_pub)：
// - 独占打开 V4L2 设备 -> 采集左右拼接 MJPG 画面 -> 按需 180° 旋转矫正；
// - 编码 JPEG 并在 TCP 9001 端口以 UWSC 帧格式推流；
// - 发布 ROS 压缩与原始图像话题以及状态 JSON (camera/status)；
// - 具备设备断开自动重连与健康守护机制。
// ==============================================================================

// ── 初学者导读 ────────────────────────────────────────────────────────────
// global_cam_pub 是“双目相机采集”节点：自己打开 V4L2 摄像头（默认 /dev/video0），
// 拿到左右拼接的 MJPG 画面，整理成两类输出：
//   1) ROS 图像话题：camera/stereo/compressed（拼接）、camera/left|right/compressed，
//      以及可选的 camera/left|right/image_raw；另发 camera/status（JSON 文本状态）。
//   2) TCP 9001：通过 StreamServer 把拼接 JPEG 推给上位机 GUI（信号流里的“旁路”）。
// 它与 MCU 串口、控制权完全独立，所以相机出问题不会影响电机控制。
// 结构：run() 是主线程（限速、发布、状态），capture_loop() 是采集线程（只留最新帧），
// 两者用 mutex + shared_ptr 交换“最新一帧”。
// ROS 2 概念：Node；Publisher；Parameter（declare_parameter）；QoS（SensorDataQoS、
// transient_local）；这里用自己写的循环 + rclcpp::ok() 控制退出。
// C++ 概念：std::thread 线程；std::atomic；std::mutex + lock_guard；std::unique_ptr；
// shared_ptr；lambda；override；std::chrono 限速。
// ─────────────────────────────────────────────────────────────────────────

#include "global_camera/stereo_transform.hpp"
#include "global_camera/stream_server.hpp"
#include "global_camera/wire.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

namespace {

using Clock = std::chrono::steady_clock;
using Compressed = sensor_msgs::msg::CompressedImage;
using Image = sensor_msgs::msg::Image;

// 把字符串转义后包成 JSON 字符串（只保留可打印 ASCII），用于 camera/status。
std::string json_string(const std::string &value) {
  std::ostringstream s;
  s << '"';
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') {
      s << '\\' << c;
    } else if (c >= 32 && c <= 126) {
      s << c;
    }
  }
  s << '"';
  return s.str();
}

// 独占锁定摄像头设备文件描述符，防止多进程冲突抢占 V4L2
// RAII 小工具：析构时自动释放文件锁，避免忘记 unlock/close。
class CameraLock {
public:
  ~CameraLock() { release(); }

  bool acquire(const std::string &path) {
    release();
    fd_ = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
    if (fd_ < 0) {
      return false;
    }
    if (flock(fd_, LOCK_EX | LOCK_NB) != 0) {
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    return true;
  }

  void release() {
    if (fd_ >= 0) {
      flock(fd_, LOCK_UN);
      ::close(fd_);
      fd_ = -1;
    }
  }

private:
  int fd_ = -1;
};

// StereoCamera：双目采集主节点。final 表示它不允许再被继承。
// 成员大致三块：ROS 发布者、采集线程共享状态、可选的 TCP 推流服务。
class StereoCamera final : public rclcpp::Node {
public:
  // 构造函数：声明全部参数并做范围校验，创建发布者、可选启动 TCP 推流服务。
  StereoCamera() : Node("global_cam_pub", "/underwater_robot") {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.read_only = true;

    device_ = declare_parameter<std::string>("device", "/dev/video0", d);
    width_ = declare_parameter<int>("width", 2560, d);
    height_ = declare_parameter<int>("height", 720, d);
    fps_ = declare_parameter<double>("fps", 30.0, d);
    capture_fps_ = declare_parameter<double>("capture_fps", 60.0, d);
    quality_ = declare_parameter<int>("jpeg_quality", 80, d);
    rotate_ = declare_parameter<bool>("rotate_180", true, d);
    port_ = declare_parameter<int>("tcp_port", 9001, d);
    raw_ = declare_parameter<bool>("publish_raw", false, d);
    reconnect_ = declare_parameter<double>("reconnect_interval_s", 2.0, d);
    duration_ = declare_parameter<double>("duration_s", 0.0, d);
    const auto bind = declare_parameter<std::string>("tcp_bind_address", "0.0.0.0", d);

    if (width_ < 2 || width_ % 2 != 0 || width_ > 8192 ||
        height_ < 2 || height_ > 4096 ||
        fps_ <= 0 || fps_ > 120 ||
        capture_fps_ <= 0 || capture_fps_ > 120 ||
        quality_ < 1 || quality_ > 100 ||
        port_ < 0 || port_ > 65535 ||
        reconnect_ <= 0 || duration_ < 0) {
      throw std::invalid_argument("parameter bounds check failed for global_cam_pub");
    }

    auto qos = rclcpp::SensorDataQoS().keep_last(2);
    stereo_ = create_publisher<Compressed>("camera/stereo/compressed", qos);
    left_ = create_publisher<Compressed>("camera/left/compressed", qos);
    right_ = create_publisher<Compressed>("camera/right/compressed", qos);

    if (raw_) {
      left_raw_ = create_publisher<Image>("camera/left/image_raw", qos);
      right_raw_ = create_publisher<Image>("camera/right/image_raw", qos);
    }

    // transient_local：晚加入的订阅者也能收到最近一次状态（类似“锁存”）。
    status_ = create_publisher<std_msgs::msg::String>(
        "camera/status",
        rclcpp::QoS(1).transient_local());

    if (port_) {
      // make_unique：创建 StreamServer（内部会开监听端口并启动推流线程）。
      server_ = std::make_unique<global_camera::StreamServer>(bind, static_cast<std::uint16_t>(port_));
      port_ = server_->port();
    }

    publish_status();
  }

  // 析构：通知采集线程停止并 join，确保线程安全退出。
  ~StereoCamera() override {
    stopping_ = true;
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
  }

  // 主线程：按 fps_ 限速循环，取最新帧、旋转矫正、切左右目、发布 ROS 话题并推流。
  // 还负责检测采集“代际(epoch)”变化：一旦相机重连/掉线，清空推流并刷新状态。
  void run() {
    capture_thread_ = std::thread(&StereoCamera::capture_loop, this);
    const auto start = Clock::now();
    const auto period = std::chrono::duration<double>(1.0 / fps_);
    auto next = Clock::now();
    auto fps_start = Clock::now();
    std::uint64_t last_seq = 0;
    std::uint64_t last_epoch = 0;

    while (rclcpp::ok() && !stopping_ &&
           (duration_ == 0 || std::chrono::duration<double>(Clock::now() - start).count() < duration_)) {
      std::this_thread::sleep_until(next);
      next += std::chrono::duration_cast<Clock::duration>(period);

      std::shared_ptr<const CapturedFrame> frame;
      bool source_on = false;
      bool fatal = false;
      std::uint64_t epoch = 0;
      std::string cap_detail;

      {
        std::lock_guard<std::mutex> guard(capture_mutex_);
        frame = latest_capture_;
        source_on = source_online_;
        fatal = capture_fatal_;
        epoch = capture_epoch_;
        cap_detail = capture_detail_;
      }

      if (fatal) {
        throw std::runtime_error(cap_detail);
      }

      if (epoch != last_epoch) {
        last_epoch = epoch;
        last_seq = 0;
        online_ = false;
        detail_ = cap_detail;
        if (server_) {
          server_->clear();
        }
        publish_status();
        continue;
      }

      if (!source_on || !frame || frame->sequence == last_seq) {
        continue;
      }
      last_seq = frame->sequence;

      try {
        cv::Mat work = frame->image.clone();
        global_camera::orient_stereo(work, rotate_);

        const int eye_w = work.cols / 2;
        cv::Mat l = work(cv::Rect(0, 0, eye_w, work.rows));
        cv::Mat r = work(cv::Rect(eye_w, 0, eye_w, work.rows));

        const auto stamp = frame->stamp;
        auto stereo_msg = compress(work, stamp, "stereo_camera_optical_frame");
        stereo_->publish(stereo_msg);
        ++frames_;
        ++sample_frames_;

        const double elapsed = std::chrono::duration<double>(Clock::now() - fps_start).count();
        if (elapsed >= 1.0) {
          measured_fps_ = sample_frames_ / elapsed;
          fps_start = Clock::now();
          sample_frames_ = 0;
          publish_status();
        }

        if (server_) {
          const auto ns = static_cast<std::uint64_t>(stamp.nanoseconds());
          server_->publish(frames_, ns, work.cols, work.rows, stereo_msg.data);
        }

        // 按需编码单目图像，节约 CPU
        // get_subscription_count()：只有真的有人在订阅时才编码单目，省 CPU。
        if (left_->get_subscription_count()) {
          left_->publish(compress(l, stamp, "left_camera_optical_frame"));
        }
        if (right_->get_subscription_count()) {
          right_->publish(compress(r, stamp, "right_camera_optical_frame"));
        }
        if (left_raw_ && left_raw_->get_subscription_count()) {
          left_raw_->publish(image(l, stamp, "left_camera_optical_frame"));
        }
        if (right_raw_ && right_raw_->get_subscription_count()) {
          right_raw_->publish(image(r, stamp, "right_camera_optical_frame"));
        }

        if (!online_) {
          online_ = true;
          detail_ = "streaming";
          publish_status();
        }
      } catch (const cv::Exception &e) {
        throw std::runtime_error(e.what());
      }
    }

    stopping_ = true;
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    online_ = false;
    detail_ = "stopped";
    if (server_) {
      server_->clear();
    }
    publish_status();
  }

private:
  // 采集线程与主线程之间传递的一帧：图像 + ROS 时间戳 + 递增序号。
  struct CapturedFrame {
    cv::Mat image;
    rclcpp::Time stamp;
    std::uint64_t sequence;
  };

  // 采集线程：打开/重开 V4L2 设备并持续 read()，只把“最新一帧”放入 latest_capture_。
  // 打不开或读失败时记录原因、递增 epoch 并等待 reconnect_ 秒后重试。
  void capture_loop() {
    auto retry = Clock::now();
    std::uint64_t sequence = 0;

    try {
      while (rclcpp::ok() && !stopping_) {
        if (!capture_.isOpened()) {
          if (Clock::now() < retry) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
          }
          retry = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(reconnect_));

          if (!lock_.acquire(device_)) {
            capture_failure("camera device missing");
            continue;
          }
          if (!capture_.open(device_, cv::CAP_V4L2)) {
            capture_failure("V4L2 open failed");
            continue;
          }

          // 设置 V4L2 采集参数：MJPG 格式、分辨率、帧率、只缓存 2 帧。
          capture_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
          capture_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
          capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
          capture_.set(cv::CAP_PROP_FPS, capture_fps_);
          capture_.set(cv::CAP_PROP_BUFFERSIZE, 2);

          RCLCPP_INFO(
              get_logger(),
              "V4L2 negotiated %.0fx%.0f, camera FPS %.1f; latest-frame capture worker",
              capture_.get(cv::CAP_PROP_FRAME_WIDTH),
              capture_.get(cv::CAP_PROP_FRAME_HEIGHT),
              capture_.get(cv::CAP_PROP_FPS));
        }

        try {
          cv::Mat image_mat;
          if (!capture_.read(image_mat) || image_mat.empty()) {
            capture_failure("camera frame read failed");
            continue;
          }
          if (image_mat.cols != width_ || image_mat.rows != height_ || image_mat.type() != CV_8UC3) {
            capture_failure("unexpected frame dimensions/type: " + std::to_string(image_mat.cols) + "x" + std::to_string(image_mat.rows));
            continue;
          }

          auto frame = std::make_shared<CapturedFrame>(CapturedFrame{std::move(image_mat), now(), ++sequence});
          std::lock_guard<std::mutex> guard(capture_mutex_);
          latest_capture_ = std::move(frame);
          source_online_ = true;
          capture_detail_ = "streaming";
        } catch (const cv::Exception &e) {
          capture_failure(e.what());
        }
      }
    } catch (const std::exception &e) {
      capture_failure(e.what());
      std::lock_guard<std::mutex> guard(capture_mutex_);
      capture_fatal_ = true;
    }
    capture_.release();
  }

  // 统一的失败处理：释放相机、清空最新帧、置离线、递增 epoch，并让推流断开。
  void capture_failure(const std::string &reason) {
    capture_.release();
    {
      std::lock_guard<std::mutex> guard(capture_mutex_);
      latest_capture_.reset();
      source_online_ = false;
      capture_detail_ = reason;
      ++capture_epoch_;
    }
    if (server_) {
      server_->clear();
    }
    RCLCPP_WARN(get_logger(), "%s; retrying camera", reason.c_str());
  }

  // 把 OpenCV 图像编码成 ROS CompressedImage（JPEG）；quality_ 来自参数。
  Compressed compress(const cv::Mat &frame, const rclcpp::Time &stamp, const std::string &id) {
    Compressed m;
    m.header.stamp = stamp;
    m.header.frame_id = id;
    m.format = "bgr8; jpeg compressed bgr8";
    if (!cv::imencode(".jpg", frame, m.data, {cv::IMWRITE_JPEG_QUALITY, quality_})) {
      throw std::runtime_error("JPEG encoding failed");
    }
    return m;
  }

  // 把 OpenCV 图像转成 ROS 原始 Image（bgr8），逐行 memcpy 保证 step 紧凑。
  Image image(const cv::Mat &frame, const rclcpp::Time &stamp, const std::string &id) {
    Image m;
    m.header.stamp = stamp;
    m.header.frame_id = id;
    m.height = frame.rows;
    m.width = frame.cols;
    m.encoding = "bgr8";
    m.is_bigendian = 0;
    m.step = frame.cols * 3;
    m.data.resize(m.step * m.height);
    for (int row = 0; row < frame.rows; ++row) {
      std::memcpy(m.data.data() + row * m.step, frame.ptr(row), m.step);
    }
    return m;
  }

  // 发布 camera/status：一段 JSON 文本，含在线状态、分辨率、FPS、TCP 端口与客户端数。
  void publish_status() {
    if (!rclcpp::ok()) {
      return;
    }
    std::ostringstream s;
    s << "{\"online\":" << (online_ ? "true" : "false")
      << ",\"device\":" << json_string(device_)
      << ",\"width\":" << (online_ ? width_ : 0)
      << ",\"height\":" << (online_ ? height_ : 0)
      << ",\"eye_width\":" << (online_ ? width_ / 2 : 0)
      << ",\"fps\":" << std::fixed << std::setprecision(2) << measured_fps_
      << ",\"sequence\":" << frames_
      << ",\"tcp_port\":" << port_
      << ",\"clients\":" << (server_ ? server_->clients() : 0)
      << ",\"tcp_frames_sent\":" << (server_ ? server_->frames_sent() : 0)
      << ",\"rotation_degrees\":" << (rotate_ ? 180 : 0)
      << ",\"calibrated\":false,\"detail\":" << json_string(detail_) << '}';

    std_msgs::msg::String message;
    message.data = s.str();
    status_->publish(message);
  }

  std::string device_;
  std::string detail_ = "starting";
  int width_ = 0;
  int height_ = 0;
  int quality_ = 0;
  int port_ = 0;

  double fps_ = 0.0;
  double capture_fps_ = 0.0;
  double reconnect_ = 0.0;
  double duration_ = 0.0;
  double measured_fps_ = 0.0;

  bool raw_ = false;
  bool rotate_ = true;
  bool online_ = false;

  std::uint64_t frames_ = 0;
  std::uint64_t sample_frames_ = 0;

  CameraLock lock_;
  cv::VideoCapture capture_;
  std::atomic<bool> stopping_{false};
  std::thread capture_thread_;
  std::mutex capture_mutex_;
  std::shared_ptr<const CapturedFrame> latest_capture_;

  std::string capture_detail_ = "starting";
  bool source_online_ = false;
  bool capture_fatal_ = false;
  std::uint64_t capture_epoch_ = 0;

  std::unique_ptr<global_camera::StreamServer> server_;

  rclcpp::Publisher<Compressed>::SharedPtr stereo_;
  rclcpp::Publisher<Compressed>::SharedPtr left_;
  rclcpp::Publisher<Compressed>::SharedPtr right_;
  rclcpp::Publisher<Image>::SharedPtr left_raw_;
  rclcpp::Publisher<Image>::SharedPtr right_raw_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
};

}  // namespace

// 程序入口：--check/--help 不碰硬件；否则初始化 ROS 2，创建节点并 run()。
int main(int argc, char **argv) {
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--check") {
      std::cout << "global_cam_pub: available (C++ stereo capture + ROS images + UWSC TCP)\n";
      return 0;
    }
    if (std::string(argv[i]) == "--help") {
      std::cout << "global_cam_pub: /dev/video0 MJPG 2560x720, output target 30 FPS; left/right halves; TCP 9001.\n"
                << "ROS parameters: device, width, height, fps (output), capture_fps (V4L2), jpeg_quality, rotate_180=true (whole frame), tcp_bind_address, tcp_port (0 disables), publish_raw, reconnect_interval_s, duration_s (0 unlimited).\n";
      return 0;
    }
  }

  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<StereoCamera>();
    node->run();
  } catch (const std::exception &e) {
    std::cerr << "global_cam_pub: " << e.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}

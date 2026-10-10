// global_cam_pub.cpp —— 双目相机独立采集与发布节点（节点名 global_cam_pub）。
// 职责：独占打开 V4L2 设备 -> 采集整幅左右拼接 MJPG 图 -> 按需旋转 180° -> 编码 JPEG，
//       发布 ROS 话题并在 TCP 9001 以 UWSC 帧格式转发（tcp_port=0 可关闭 TCP）。
// 话题（命名空间 /underwater_robot）：camera/stereo/compressed、camera/left|right/image_raw[/compressed]、
//       camera/status（transient_local 状态 JSON）。
// 参数默认：device=/dev/video0，width=2560 height=720（左右各 1280x720），fps=30（输出）、
//       capture_fps=60（V4L2 采集）、jpeg_quality=80、rotate_180=true、publish_raw=false、
//       reconnect_interval_s=2.0、duration_s=0（不限时）。本入口不启动 MCU 后台。
#include "global_camera/stream_server.hpp"
#include "global_camera/wire.hpp"
#include "global_camera/stereo_transform.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <sys/file.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <atomic>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
namespace {
using Clock=std::chrono::steady_clock;
using Compressed=sensor_msgs::msg::CompressedImage;
using Image=sensor_msgs::msg::Image;
std::string json_string(const std::string &value) {  // 转义为 JSON 字符串字面量
  std::ostringstream s;s<<'"';
  for(unsigned char c:value) {
    if(c=='"' || c=='\\')s<<'\\'<<c;
    else if(c<32)s<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<static_cast<unsigned>(c)<<std::dec;
    else s<<c;
  }
  s<<'"';return s.str();
}
class CameraLock {  // 基于 flock 的相机设备独占锁，防止两个 global_cam_pub 抢同一台相机
public:
  ~CameraLock(){if(fd_>=0)close(fd_);}
  // 锁文件按设备 rdev 生成（/tmp/underwater-stereo-<rdev>.lock），非阻塞加锁失败即抛异常。
  bool acquire(const std::string &device) {
    if(fd_>=0)return true;
    struct stat info{};
    if(stat(device.c_str(),&info)<0)return false;
    if(!S_ISCHR(info.st_mode))throw std::runtime_error("camera device is not a character device");
    const auto path="/tmp/underwater-stereo-"+std::to_string(info.st_rdev)+".lock";
    fd_=open(path.c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    if(fd_<0)throw std::runtime_error("camera ownership lock: "+std::string(strerror(errno)));
    if(flock(fd_,LOCK_EX|LOCK_NB)<0) {
      close(fd_);fd_=-1;throw std::runtime_error("another global_cam_pub already owns "+device);
    }
    return true;
  }
private:
  int fd_=-1;
};
class StereoCamera final:public rclcpp::Node {
public:
  StereoCamera():Node("global_cam_pub","/underwater_robot") {
    rcl_interfaces::msg::ParameterDescriptor d;d.read_only=true;  // 全部参数只读，运行时不可改
    device_=declare_parameter<std::string>("device","/dev/video0",d);
    width_=declare_parameter<int>("width",2560,d);height_=declare_parameter<int>("height",720,d);  // 拼接图整幅尺寸，左右各占一半
    fps_=declare_parameter<double>("fps",30.0,d);        // ROS 输出/编码帧率
    capture_fps_=declare_parameter<double>("capture_fps",60.0,d);  // V4L2 采集帧率（>=fps_）
    quality_=declare_parameter<int>("jpeg_quality",80,d);  // JPEG 质量 1-100
    raw_=declare_parameter<bool>("publish_raw",false,d);   // 是否额外发布左/右原始 BGR8 图
    rotate_=declare_parameter<bool>("rotate_180",true,d);  // 相机倒装，整幅旋转 180°
    reconnect_=declare_parameter<double>("reconnect_interval_s",2.0,d);  // 断流重连间隔（0.2-60s）
    const auto address=declare_parameter<std::string>("tcp_bind_address","0.0.0.0",d);
    port_=declare_parameter<int>("tcp_port",9001,d);       // 双目图像端口；0 表示禁用 TCP
    duration_=declare_parameter<double>("duration_s",0.0,d);  // 运行时长上限，0 为不限
    if(device_.empty() || width_<2 || width_%2 || width_>8192 || height_<1 || height_>4096 ||
       !std::isfinite(fps_) || fps_<1 || fps_>60 || !std::isfinite(capture_fps_) ||
       capture_fps_<fps_ || capture_fps_>60 || quality_<1 || quality_>100 ||
       !std::isfinite(reconnect_) || reconnect_<0.2 || reconnect_>60 ||
       port_<0 || port_>65535 || !std::isfinite(duration_) || duration_<0)
      throw std::invalid_argument("invalid device/size/fps/quality/reconnect/port/duration parameters");
    auto q=rclcpp::SensorDataQoS().keep_last(1);  // SensorData QoS，队列 1：只关心最新帧
    stereo_=create_publisher<Compressed>("camera/stereo/compressed",q);   // 左右拼接整幅 JPEG
    left_=create_publisher<Compressed>("camera/left/image_raw/compressed",q);
    right_=create_publisher<Compressed>("camera/right/image_raw/compressed",q);
    if(raw_) {
      left_raw_=create_publisher<Image>("camera/left/image_raw",q);
      right_raw_=create_publisher<Image>("camera/right/image_raw",q);
    }
    status_=create_publisher<std_msgs::msg::String>("camera/status",rclcpp::QoS(1).transient_local());  // 状态 JSON，晚加入者也能收到
    if(port_)server_=std::make_unique<global_camera::StreamServer>(address,static_cast<std::uint16_t>(port_));
    RCLCPP_INFO(get_logger(),"Side-by-side stereo: %s, requested %dx%d MJPG %.1f FPS; TCP %s:%d",
      device_.c_str(),width_,height_,fps_,address.c_str(),port_);
    publish_status();
  }
  ~StereoCamera() override {
    stopping_=true;
    if(capture_thread_.joinable())capture_thread_.join();
  }
  void run() {  // 主循环：限速转发最新帧，1 秒刷新一次实测帧率与状态
    capture_thread_=std::thread([this]{capture_loop();});  // 采集与发布解耦，避免网络阻塞采集
    const auto started=Clock::now();auto next=started;auto status_at=started;
    std::uint64_t processed=0,epoch=0;
    while(rclcpp::ok() && (duration_==0 || std::chrono::duration<double>(Clock::now()-started).count()<duration_)) {
      rclcpp::spin_some(shared_from_this());
      if(server_ && !server_->healthy())throw std::runtime_error("camera TCP worker failed");
      std::shared_ptr<const CapturedFrame> captured;
      bool source_online,fatal;
      std::uint64_t source_epoch;
      {
        std::lock_guard<std::mutex> guard(capture_mutex_);
        captured=latest_capture_;source_online=source_online_;fatal=capture_fatal_;
        source_epoch=capture_epoch_;detail_=capture_detail_;
      }
      if(fatal)throw std::runtime_error(detail_);
      if(source_epoch!=epoch) {epoch=source_epoch;online_=false;if(server_)server_->clear();publish_status();}  // 采集世代变化：断开旧画面客户端
      if(!source_online)online_=false;
      const auto t=Clock::now();
      if(t-status_at>=std::chrono::seconds(1)) {
        measured_fps_=static_cast<double>(frames_-sample_frames_)/std::chrono::duration<double>(t-status_at).count();  // 滑动窗口实测输出帧率
        sample_frames_=frames_;status_at=t;publish_status();
      }
      if(!source_online || !captured || captured->sequence==processed) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));continue;  // 无新帧，短睡眠避免忙等
      }
      if(Clock::now()<next){std::this_thread::sleep_for(std::chrono::milliseconds(2));continue;}  // 输出限速到 fps_
      const auto period=std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0/fps_));
      next+=period;
      if(next<Clock::now()-period)next=Clock::now();  // 长时间落后时重置节拍，避免追赶补发
      try {
        processed=captured->sequence;
        cv::Mat frame=captured->image;
        global_camera::orient_stereo(frame,rotate_);  // 倒装时整幅旋转 180°（左右半幅互换）
        const auto stamp=captured->stamp;
        const auto l=frame(cv::Rect(0,0,width_/2,height_));      // 左目 = 左半幅
        const auto r=frame(cv::Rect(width_/2,0,width_/2,height_));  // 右目 = 右半幅
        auto jpeg=compress(frame,stamp,"stereo_camera_optical_frame");  // 整幅 JPEG，与 TCP 转发内容一致
        {
          std::lock_guard<std::mutex> guard(capture_mutex_);
          if(!source_online_ || source_epoch!=capture_epoch_)continue;  // 编码期间掉线则丢弃该帧
        }
        const auto sequence=++frames_;
        if(server_)server_->publish(sequence,static_cast<std::uint64_t>(stamp.nanoseconds()),width_,height_,jpeg.data);
        stereo_->publish(jpeg);
        if(left_->get_subscription_count())left_->publish(compress(l,stamp,"left_camera_optical_frame"));  // 有订阅者才编码，省 CPU
        if(right_->get_subscription_count())right_->publish(compress(r,stamp,"right_camera_optical_frame"));
        if(left_raw_ && left_raw_->get_subscription_count())left_raw_->publish(image(l,stamp,"left_camera_optical_frame"));
        if(right_raw_ && right_raw_->get_subscription_count())right_raw_->publish(image(r,stamp,"right_camera_optical_frame"));
        if(!online_) { online_=true;detail_="streaming";publish_status(); }
      } catch(const cv::Exception &e) { throw std::runtime_error(e.what()); }
    }
    stopping_=true;if(capture_thread_.joinable())capture_thread_.join();
    online_=false;detail_="stopped";if(server_)server_->clear();publish_status();
  }
private:
  struct CapturedFrame { cv::Mat image; rclcpp::Time stamp; std::uint64_t sequence; };
  void capture_loop() {  // 采集线程：V4L2 打开 MJPG，只保留最新一帧
    auto retry=Clock::now();std::uint64_t sequence=0;
    try {
      while(rclcpp::ok() && !stopping_) {
        if(!capture_.isOpened()) {
          if(Clock::now()<retry){std::this_thread::sleep_for(std::chrono::milliseconds(10));continue;}  // 重连退避
          retry=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(reconnect_));
          if(!lock_.acquire(device_)) { capture_failure("camera device missing");continue; }
          if(!capture_.open(device_,cv::CAP_V4L2)) { capture_failure("V4L2 open failed");continue; }
          capture_.set(cv::CAP_PROP_FOURCC,cv::VideoWriter::fourcc('M','J','P','G'));  // 设备侧 MJPG 解码，降低 USB 带宽
          capture_.set(cv::CAP_PROP_FRAME_WIDTH,width_);capture_.set(cv::CAP_PROP_FRAME_HEIGHT,height_);
          capture_.set(cv::CAP_PROP_FPS,capture_fps_);capture_.set(cv::CAP_PROP_BUFFERSIZE,2);  // 驱动缓冲仅 2 帧，抑制延迟累积
          RCLCPP_INFO(get_logger(),"V4L2 negotiated %.0fx%.0f, camera FPS %.1f; latest-frame capture worker",
            capture_.get(cv::CAP_PROP_FRAME_WIDTH),capture_.get(cv::CAP_PROP_FRAME_HEIGHT),capture_.get(cv::CAP_PROP_FPS));
        }
        try {
          cv::Mat image;
          if(!capture_.read(image) || image.empty()) { capture_failure("camera frame read failed");continue; }
          if(image.cols!=width_ || image.rows!=height_ || image.type()!=CV_8UC3) {
            capture_failure("unexpected frame dimensions/type: "+std::to_string(image.cols)+"x"+std::to_string(image.rows));continue;  // 尺寸/像素格式不符即重连
          }
          auto frame=std::make_shared<CapturedFrame>(CapturedFrame{std::move(image),now(),++sequence});
          std::lock_guard<std::mutex> guard(capture_mutex_);
          latest_capture_=std::move(frame);source_online_=true;capture_detail_="streaming";  // 覆盖旧帧，不排队
        } catch(const cv::Exception &e) {capture_failure(e.what());}
      }
    } catch(const std::exception &e) {
      capture_failure(e.what());std::lock_guard<std::mutex> guard(capture_mutex_);capture_fatal_=true;
    }
    capture_.release();
  }
  void capture_failure(const std::string &reason) {  // 采集失败：释放设备、清空待发帧、递增世代
    capture_.release();
    {std::lock_guard<std::mutex> guard(capture_mutex_);
      latest_capture_.reset();source_online_=false;capture_detail_=reason;++capture_epoch_;}
    if(server_)server_->clear();  // 立即断开 TCP 客户端，避免向前端推送陈旧画面
    RCLCPP_WARN(get_logger(),"%s; retrying camera",reason.c_str());
  }
  Compressed compress(const cv::Mat &frame,const rclcpp::Time &stamp,const std::string &id) {  // BGR8 -> JPEG 编码
    Compressed m;m.header.stamp=stamp;m.header.frame_id=id;m.format="bgr8; jpeg compressed bgr8";  // 与 image_transport 约定一致
    if(!cv::imencode(".jpg",frame,m.data,{cv::IMWRITE_JPEG_QUALITY,quality_}))
      throw std::runtime_error("JPEG encoding failed");
    return m;
  }
  Image image(const cv::Mat &frame,const rclcpp::Time &stamp,const std::string &id) {  // 逐行拷贝为 BGR8 原始图（step=cols*3）
    Image m;m.header.stamp=stamp;m.header.frame_id=id;m.height=frame.rows;m.width=frame.cols;
    m.encoding="bgr8";m.is_bigendian=0;m.step=frame.cols*3;m.data.resize(m.step*m.height);
    for(int row=0;row<frame.rows;++row)std::memcpy(m.data.data()+row*m.step,frame.ptr(row),m.step);
    return m;
  }
  void publish_status() {  // 发布 camera/status 状态 JSON（在线、分辨率、实测帧率、TCP 客户端数等）
    if(!rclcpp::ok())return;
    std::ostringstream s;s<<"{\"online\":"<<(online_?"true":"false")
      <<",\"device\":"<<json_string(device_)<<",\"width\":"<<(online_?width_:0)
      <<",\"height\":"<<(online_?height_:0)<<",\"eye_width\":"<<(online_?width_/2:0)  // 单目宽度
      <<",\"fps\":"<<std::fixed<<std::setprecision(2)<<measured_fps_
      <<",\"sequence\":"<<frames_<<",\"tcp_port\":"<<port_
      <<",\"clients\":"<<(server_?server_->clients():0)
      <<",\"tcp_frames_sent\":"<<(server_?server_->frames_sent():0)
      <<",\"rotation_degrees\":"<<(rotate_?180:0)
      <<",\"calibrated\":false,\"detail\":"<<json_string(detail_)<<'}';  // 双目尚未标定
    std_msgs::msg::String message;message.data=s.str();status_->publish(message);
  }
  std::string device_,detail_="starting";
  int width_=0,height_=0,quality_=0,port_=0;
  double fps_=0,capture_fps_=0,reconnect_=0,duration_=0,measured_fps_=0;
  bool raw_=false,rotate_=true,online_=false;
  std::uint64_t frames_=0,sample_frames_=0;
  CameraLock lock_;
  cv::VideoCapture capture_;
  std::atomic<bool> stopping_{false};
  std::thread capture_thread_;
  std::mutex capture_mutex_;
  std::shared_ptr<const CapturedFrame> latest_capture_;  // 最新一帧，采集线程覆盖写
  std::string capture_detail_="starting";
  bool source_online_=false,capture_fatal_=false;
  std::uint64_t capture_epoch_=0;                        // 采集世代号，掉线/重连时递增
  std::unique_ptr<global_camera::StreamServer> server_;  // tcp_port=0 时为 nullptr
  rclcpp::Publisher<Compressed>::SharedPtr stereo_,left_,right_;
  rclcpp::Publisher<Image>::SharedPtr left_raw_,right_raw_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
};
}
int main(int argc,char **argv) {
  for(int i=1;i<argc;++i) {  // 诊断入口：仅 --check/--help，不打开相机、不启动 MCU 后台
    if(std::string(argv[i])=="--check") { std::cout<<"global_cam_pub: available (C++ stereo capture + ROS images + UWSC TCP)\n";return 0; }
    if(std::string(argv[i])=="--help") {
      std::cout<<"global_cam_pub: /dev/video0 MJPG 2560x720, output target 30 FPS; left/right halves; TCP 9001.\n"
        <<"ROS parameters: device, width, height, fps (output), capture_fps (V4L2), jpeg_quality, rotate_180=true (whole frame), tcp_bind_address, tcp_port (0 disables), publish_raw, reconnect_interval_s, duration_s (0 unlimited).\n";return 0;
    }
  }
  rclcpp::init(argc,argv);int result=0;
  try { auto node=std::make_shared<StereoCamera>();node->run(); }
  catch(const std::exception &e) { std::cerr<<"global_cam_pub: "<<e.what()<<'\n';result=1; }
  rclcpp::shutdown();return result;
}

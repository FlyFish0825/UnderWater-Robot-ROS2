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
std::string json_string(const std::string &value) {
  std::ostringstream s;s<<'"';
  for(unsigned char c:value) {
    if(c=='"' || c=='\\')s<<'\\'<<c;
    else if(c<32)s<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<static_cast<unsigned>(c)<<std::dec;
    else s<<c;
  }
  s<<'"';return s.str();
}
class CameraLock {
public:
  ~CameraLock(){if(fd_>=0)close(fd_);}
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
    rcl_interfaces::msg::ParameterDescriptor d;d.read_only=true;
    device_=declare_parameter<std::string>("device","/dev/video0",d);
    width_=declare_parameter<int>("width",2560,d);height_=declare_parameter<int>("height",720,d);
    fps_=declare_parameter<double>("fps",30.0,d);
    capture_fps_=declare_parameter<double>("capture_fps",60.0,d);
    quality_=declare_parameter<int>("jpeg_quality",80,d);
    raw_=declare_parameter<bool>("publish_raw",false,d);
    rotate_=declare_parameter<bool>("rotate_180",true,d);
    reconnect_=declare_parameter<double>("reconnect_interval_s",2.0,d);
    const auto address=declare_parameter<std::string>("tcp_bind_address","0.0.0.0",d);
    port_=declare_parameter<int>("tcp_port",9001,d);
    duration_=declare_parameter<double>("duration_s",0.0,d);
    if(device_.empty() || width_<2 || width_%2 || width_>8192 || height_<1 || height_>4096 ||
       !std::isfinite(fps_) || fps_<1 || fps_>60 || !std::isfinite(capture_fps_) ||
       capture_fps_<fps_ || capture_fps_>60 || quality_<1 || quality_>100 ||
       !std::isfinite(reconnect_) || reconnect_<0.2 || reconnect_>60 ||
       port_<0 || port_>65535 || !std::isfinite(duration_) || duration_<0)
      throw std::invalid_argument("invalid device/size/fps/quality/reconnect/port/duration parameters");
    auto q=rclcpp::SensorDataQoS().keep_last(1);
    stereo_=create_publisher<Compressed>("camera/stereo/compressed",q);
    left_=create_publisher<Compressed>("camera/left/image_raw/compressed",q);
    right_=create_publisher<Compressed>("camera/right/image_raw/compressed",q);
    if(raw_) {
      left_raw_=create_publisher<Image>("camera/left/image_raw",q);
      right_raw_=create_publisher<Image>("camera/right/image_raw",q);
    }
    status_=create_publisher<std_msgs::msg::String>("camera/status",rclcpp::QoS(1).transient_local());
    if(port_)server_=std::make_unique<global_camera::StreamServer>(address,static_cast<std::uint16_t>(port_));
    RCLCPP_INFO(get_logger(),"Side-by-side stereo: %s, requested %dx%d MJPG %.1f FPS; TCP %s:%d",
      device_.c_str(),width_,height_,fps_,address.c_str(),port_);
    publish_status();
  }
  ~StereoCamera() override {
    stopping_=true;
    if(capture_thread_.joinable())capture_thread_.join();
  }
  void run() {
    capture_thread_=std::thread([this]{capture_loop();});
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
      if(source_epoch!=epoch) {epoch=source_epoch;online_=false;if(server_)server_->clear();publish_status();}
      if(!source_online)online_=false;
      const auto t=Clock::now();
      if(t-status_at>=std::chrono::seconds(1)) {
        measured_fps_=static_cast<double>(frames_-sample_frames_)/std::chrono::duration<double>(t-status_at).count();
        sample_frames_=frames_;status_at=t;publish_status();
      }
      if(!source_online || !captured || captured->sequence==processed) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));continue;
      }
      if(Clock::now()<next){std::this_thread::sleep_for(std::chrono::milliseconds(2));continue;}
      const auto period=std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0/fps_));
      next+=period;
      if(next<Clock::now()-period)next=Clock::now();
      try {
        processed=captured->sequence;
        cv::Mat frame=captured->image;
        global_camera::orient_stereo(frame,rotate_);
        const auto stamp=captured->stamp;
        const auto l=frame(cv::Rect(0,0,width_/2,height_));
        const auto r=frame(cv::Rect(width_/2,0,width_/2,height_));
        auto jpeg=compress(frame,stamp,"stereo_camera_optical_frame");
        {
          std::lock_guard<std::mutex> guard(capture_mutex_);
          if(!source_online_ || source_epoch!=capture_epoch_)continue;
        }
        const auto sequence=++frames_;
        if(server_)server_->publish(sequence,static_cast<std::uint64_t>(stamp.nanoseconds()),width_,height_,jpeg.data);
        stereo_->publish(jpeg);
        if(left_->get_subscription_count())left_->publish(compress(l,stamp,"left_camera_optical_frame"));
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
  void capture_loop() {
    auto retry=Clock::now();std::uint64_t sequence=0;
    try {
      while(rclcpp::ok() && !stopping_) {
        if(!capture_.isOpened()) {
          if(Clock::now()<retry){std::this_thread::sleep_for(std::chrono::milliseconds(10));continue;}
          retry=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(reconnect_));
          if(!lock_.acquire(device_)) { capture_failure("camera device missing");continue; }
          if(!capture_.open(device_,cv::CAP_V4L2)) { capture_failure("V4L2 open failed");continue; }
          capture_.set(cv::CAP_PROP_FOURCC,cv::VideoWriter::fourcc('M','J','P','G'));
          capture_.set(cv::CAP_PROP_FRAME_WIDTH,width_);capture_.set(cv::CAP_PROP_FRAME_HEIGHT,height_);
          capture_.set(cv::CAP_PROP_FPS,capture_fps_);capture_.set(cv::CAP_PROP_BUFFERSIZE,2);
          RCLCPP_INFO(get_logger(),"V4L2 negotiated %.0fx%.0f, camera FPS %.1f; latest-frame capture worker",
            capture_.get(cv::CAP_PROP_FRAME_WIDTH),capture_.get(cv::CAP_PROP_FRAME_HEIGHT),capture_.get(cv::CAP_PROP_FPS));
        }
        try {
          cv::Mat image;
          if(!capture_.read(image) || image.empty()) { capture_failure("camera frame read failed");continue; }
          if(image.cols!=width_ || image.rows!=height_ || image.type()!=CV_8UC3) {
            capture_failure("unexpected frame dimensions/type: "+std::to_string(image.cols)+"x"+std::to_string(image.rows));continue;
          }
          auto frame=std::make_shared<CapturedFrame>(CapturedFrame{std::move(image),now(),++sequence});
          std::lock_guard<std::mutex> guard(capture_mutex_);
          latest_capture_=std::move(frame);source_online_=true;capture_detail_="streaming";
        } catch(const cv::Exception &e) {capture_failure(e.what());}
      }
    } catch(const std::exception &e) {
      capture_failure(e.what());std::lock_guard<std::mutex> guard(capture_mutex_);capture_fatal_=true;
    }
    capture_.release();
  }
  void capture_failure(const std::string &reason) {
    capture_.release();
    {std::lock_guard<std::mutex> guard(capture_mutex_);
      latest_capture_.reset();source_online_=false;capture_detail_=reason;++capture_epoch_;}
    if(server_)server_->clear();
    RCLCPP_WARN(get_logger(),"%s; retrying camera",reason.c_str());
  }
  Compressed compress(const cv::Mat &frame,const rclcpp::Time &stamp,const std::string &id) {
    Compressed m;m.header.stamp=stamp;m.header.frame_id=id;m.format="bgr8; jpeg compressed bgr8";
    if(!cv::imencode(".jpg",frame,m.data,{cv::IMWRITE_JPEG_QUALITY,quality_}))
      throw std::runtime_error("JPEG encoding failed");
    return m;
  }
  Image image(const cv::Mat &frame,const rclcpp::Time &stamp,const std::string &id) {
    Image m;m.header.stamp=stamp;m.header.frame_id=id;m.height=frame.rows;m.width=frame.cols;
    m.encoding="bgr8";m.is_bigendian=0;m.step=frame.cols*3;m.data.resize(m.step*m.height);
    for(int row=0;row<frame.rows;++row)std::memcpy(m.data.data()+row*m.step,frame.ptr(row),m.step);
    return m;
  }
  void publish_status() {
    if(!rclcpp::ok())return;
    std::ostringstream s;s<<"{\"online\":"<<(online_?"true":"false")
      <<",\"device\":"<<json_string(device_)<<",\"width\":"<<(online_?width_:0)
      <<",\"height\":"<<(online_?height_:0)<<",\"eye_width\":"<<(online_?width_/2:0)
      <<",\"fps\":"<<std::fixed<<std::setprecision(2)<<measured_fps_
      <<",\"sequence\":"<<frames_<<",\"tcp_port\":"<<port_
      <<",\"clients\":"<<(server_?server_->clients():0)
      <<",\"tcp_frames_sent\":"<<(server_?server_->frames_sent():0)
      <<",\"rotation_degrees\":"<<(rotate_?180:0)
      <<",\"calibrated\":false,\"detail\":"<<json_string(detail_)<<'}';
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
  std::shared_ptr<const CapturedFrame> latest_capture_;
  std::string capture_detail_="starting";
  bool source_online_=false,capture_fatal_=false;
  std::uint64_t capture_epoch_=0;
  std::unique_ptr<global_camera::StreamServer> server_;
  rclcpp::Publisher<Compressed>::SharedPtr stereo_,left_,right_;
  rclcpp::Publisher<Image>::SharedPtr left_raw_,right_raw_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
};
}
int main(int argc,char **argv) {
  for(int i=1;i<argc;++i) {
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

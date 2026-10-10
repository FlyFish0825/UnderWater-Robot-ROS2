// data_sub.cpp —— 双目图像订阅检查节点（节点名 data_sub），用于验证 9001 转发链路的 ROS 侧输出。
// 职责：订阅 /underwater_robot/camera/stereo/compressed，校验 JPEG 与拼接尺寸；可选按间隔把左右半幅
//       分别保存为 PNG 成对文件。本节点不打开相机，也不启动 MCU 后台。
// 参数默认：topic=/underwater_robot/camera/stereo/compressed、save_images=false、
//       max_pairs=0（不限）、save_interval_s=1.0、duration_s=0（不限时）、output_dir=录制根目录。
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>
namespace {
class StereoReceiver final:public rclcpp::Node {
public:
  StereoReceiver():Node("data_sub","/underwater_robot") {
    rcl_interfaces::msg::ParameterDescriptor d;d.read_only=true;  // 参数只读
    const auto topic=declare_parameter<std::string>("topic","/underwater_robot/camera/stereo/compressed",d);
    save_=declare_parameter<bool>("save_images",false,d);       // 是否落盘保存左右 PNG
    limit_=declare_parameter<int>("max_pairs",0,d);             // 保存对数上限，0 为不限
    interval_=declare_parameter<double>("save_interval_s",1.0,d);  // 相邻两次保存的最小间隔（秒）
    duration_=declare_parameter<double>("duration_s",0.0,d);    // 运行时长上限，0 为不限
    output_=declare_parameter<std::string>("output_dir",DEFAULT_CAMERA_RECORD_DIR,d);
    if(limit_<0 || !std::isfinite(interval_) || interval_<0 || !std::isfinite(duration_) || duration_<0)
      throw std::invalid_argument("invalid max_pairs/save_interval_s/duration_s");
    if(save_) {
      // Each invocation gets a session directory; earlier image pairs are never overwritten.
      const auto stamp=std::chrono::system_clock::now().time_since_epoch();
      output_/="session_"+std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(stamp).count());  // 每次运行独立 session_<ns> 目录
      std::filesystem::create_directories(output_/"left");std::filesystem::create_directories(output_/"right");
      RCLCPP_INFO(get_logger(),"Stereo recording directory: %s",output_.c_str());
    }
    subscription_=create_subscription<sensor_msgs::msg::CompressedImage>(topic,rclcpp::SensorDataQoS().keep_last(1),  // 队列 1，只处理最新帧
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr m){receive(*m);});
    started_=std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(),"Receiving %s; save_images=%s",topic.c_str(),save_?"true":"false");
  }
  bool done() const {  // 退出条件：失败 / 已达保存对数上限 / 超过运行时长
    return failed_ || (save_ && limit_>0 && saved_>=static_cast<unsigned>(limit_)) ||
      (duration_>0 && std::chrono::duration<double>(std::chrono::steady_clock::now()-started_).count()>=duration_);
  }
  bool failed() const { return failed_; }
private:
  void receive(const sensor_msgs::msg::CompressedImage &message) {  // 校验并可选保存一帧拼接图
    try {
      if(message.data.empty() || message.data.size()>8*1024*1024)throw std::runtime_error("invalid JPEG size");  // 与 wire.hpp 上限一致
      const auto image=cv::imdecode(message.data,cv::IMREAD_COLOR);
      if(image.empty() || image.cols<2 || image.cols%2 || image.cols>8192 || image.rows>4096)
        throw std::runtime_error("invalid side-by-side JPEG");  // 宽度必须为偶数，左右各一半
      ++received_;
      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,"received=%llu stereo=%dx%d each=%dx%d stamp=%d.%09u",  // 1 秒最多打印一次
        static_cast<unsigned long long>(received_),image.cols,image.rows,image.cols/2,image.rows,
        message.header.stamp.sec,message.header.stamp.nanosec);
      const auto t=std::chrono::steady_clock::now();
      if(!save_ || (saved_ && std::chrono::duration<double>(t-last_save_).count()<interval_))return;  // 限频保存
      const auto name=std::to_string(message.header.stamp.sec)+"_"+std::to_string(message.header.stamp.nanosec)+"_"+std::to_string(saved_)+".png";  // 文件名含时间戳与序号
      const auto left=output_/"left"/name,right=output_/"right"/name;
      const auto left_image=image(cv::Rect(0,0,image.cols/2,image.rows));           // 左半幅
      const auto right_image=image(cv::Rect(image.cols/2,0,image.cols/2,image.rows));  // 右半幅
      // Cleanup an incomplete pair on a write failure.
      try {
        if(!cv::imwrite(left.string(),left_image) || !cv::imwrite(right.string(),right_image))
          throw std::runtime_error("PNG pair write failed");
      } catch(...) {
        std::error_code ignored;std::filesystem::remove(left,ignored);std::filesystem::remove(right,ignored);throw;  // 成对失败则两文件都删除
      }
      ++saved_;last_save_=t;
      RCLCPP_INFO(get_logger(),"Saved stereo pair %u: %s",saved_,name.c_str());
    } catch(const std::exception &e) { failed_=true;RCLCPP_ERROR(get_logger(),"%s",e.what()); }
  }
  bool save_=false,failed_=false;
  int limit_=0;unsigned saved_=0;std::uint64_t received_=0;
  double interval_=0,duration_=0;
  std::filesystem::path output_;
  std::chrono::steady_clock::time_point started_,last_save_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
};
}
int main(int argc,char **argv) {
  for(int i=1;i<argc;++i) {  // 诊断入口：--check/--help 仅打印说明
    if(std::string(argv[i])=="--check") { std::cout<<"data_sub: available (stereo JPEG verification and optional PNG pair recording)\n";return 0; }
    if(std::string(argv[i])=="--help") {
      std::cout<<"data_sub subscribes to /underwater_robot/camera/stereo/compressed. It never opens the camera.\n"
        <<"ROS parameters: topic, save_images=false, output_dir="<<DEFAULT_CAMERA_RECORD_DIR
        <<", max_pairs=0 (unlimited), save_interval_s=1.0, duration_s=0 (unlimited).\n";return 0;
    }
  }
  rclcpp::init(argc,argv);int result=0;
  try {
    auto node=std::make_shared<StereoReceiver>();
    while(rclcpp::ok() && !node->done()) { rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
    if(node->failed())result=1;
  } catch(const std::exception &e) { std::cerr<<"data_sub: "<<e.what()<<'\n';result=1; }
  rclcpp::shutdown();return result;
}

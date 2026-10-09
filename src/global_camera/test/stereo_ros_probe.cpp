#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <iostream>
#include <map>
#include <thread>
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("stereo_ros_probe");
  const bool raw=node->declare_parameter<bool>("expect_raw",false);
  std::map<std::uint64_t,unsigned> frames;
  bool failed=false;
  unsigned matched=0;
  auto check=[&](const std_msgs::msg::Header &h,unsigned bit) {
    const auto key=static_cast<std::uint64_t>(h.stamp.sec)*1000000000ULL+h.stamp.nanosec;
    auto &bits=frames[key];const auto before=bits;bits|=bit;
    const unsigned expected=raw?31:7;
    if(bits==expected && before!=expected)++matched;
    if(frames.size()>100)frames.erase(frames.begin());
  };
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr> compressed;
  for(unsigned i=0;i<3;++i) {
    const std::string topic=i==0?"stereo/compressed":i==1?"left/image_raw/compressed":"right/image_raw/compressed";
    compressed.push_back(node->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/underwater_robot/camera/"+topic,rclcpp::SensorDataQoS().keep_last(1),
      [&,i](sensor_msgs::msg::CompressedImage::ConstSharedPtr m) {
        auto image=cv::imdecode(m->data,cv::IMREAD_COLOR);
        if(image.rows!=720 || image.cols!=(i==0?2560:1280)) {failed=true;return;}
        const std::string id=i==0?"stereo_camera_optical_frame":i==1?"left_camera_optical_frame":"right_camera_optical_frame";
        if(m->header.frame_id!=id){failed=true;return;}
        check(m->header,1U<<i);
      }));
  }
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> images;
  if(raw)for(unsigned i=0;i<2;++i) {
    const std::string eye=i==0?"left":"right";
    images.push_back(node->create_subscription<sensor_msgs::msg::Image>(
      "/underwater_robot/camera/"+eye+"/image_raw",rclcpp::SensorDataQoS().keep_last(1),
      [&,i](sensor_msgs::msg::Image::ConstSharedPtr m) {
        if(m->width!=1280 || m->height!=720 || m->encoding!="bgr8" || m->step!=3840 || m->data.size()!=2764800){failed=true;return;}
        check(m->header,1U<<(3+i));
      }));
  }
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(12);
  while(rclcpp::ok() && !failed && matched<3 && std::chrono::steady_clock::now()<deadline) {
    rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::cout<<(matched>=3&&!failed?"PASS":"FAIL")<<" stereo ROS: matched="<<matched<<" raw="<<raw<<"\n";
  node.reset();rclcpp::shutdown();return matched>=3&&!failed?0:1;
}

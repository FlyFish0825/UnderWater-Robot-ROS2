// 命令行 wrench 演示工具：按固定频率向 control/wrench_request 发布一次 wrench。
// 参数顺序 Fx Fy Fz(N) Mx My Mz(N·m)，坐标系由 frame_id 参数给出（默认 thruster_center）。
// 发布结束后补发 5 帧零力矩；即使被中断，分配器的输入看门狗也会在超时后停机。
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  const auto args=rclcpp::remove_ros_arguments(argc,argv);  // 去掉 --ros-args 后只留六个数值
  if(args.size()!=7) {
    std::cerr<<"Usage: wrench_command Fx Fy Fz Mx My Mz [--ros-args ...]\n"; rclcpp::shutdown(); return 2;
  }
  std::array<double,6> values{};
  try {
    for(int i=0;i<6;++i) { std::size_t used=0; values[i]=std::stod(args[i+1],&used);
      if(used!=args[i+1].size() || !std::isfinite(values[i]) || std::abs(values[i])>1e6) throw std::invalid_argument("invalid wrench");  // 必须整串可解析且有限
    }
    auto node=std::make_shared<rclcpp::Node>("wrench_command");
    const auto frame=node->declare_parameter<std::string>("frame_id","thruster_center");
    const double duration=node->declare_parameter<double>("duration_s",5.0);
    if(!std::isfinite(duration)||duration<=0||duration>60) throw std::invalid_argument("duration_s must be 0..60");  // 最长 60 s
    auto pub=node->create_publisher<geometry_msgs::msg::WrenchStamped>("control/wrench_request",rclcpp::QoS(1));
    const auto wait_end=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    while(rclcpp::ok() && pub->get_subscription_count()==0 && std::chrono::steady_clock::now()<wait_end) {
      rclcpp::spin_some(node); rclcpp::sleep_for(std::chrono::milliseconds(20));  // 最多等 3 s 让分配器订阅上来
    }
    if(pub->get_subscription_count()==0) throw std::runtime_error("No allocator subscribed");
    auto send=[&](bool zero) {
      geometry_msgs::msg::WrenchStamped w; w.header.stamp=node->now(); w.header.frame_id=frame;
      if(!zero) { w.wrench.force.x=values[0]; w.wrench.force.y=values[1]; w.wrench.force.z=values[2];
        w.wrench.torque.x=values[3]; w.wrench.torque.y=values[4]; w.wrench.torque.z=values[5]; }
      pub->publish(w);
    };
    const auto start=std::chrono::steady_clock::now();
    while(rclcpp::ok() && std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<duration) {
      send(false); rclcpp::spin_some(node); rclcpp::sleep_for(std::chrono::milliseconds(50));  // 20 Hz 持续发布
    }
    for(int i=0;i<5 && rclcpp::ok();++i) { send(true); rclcpp::sleep_for(std::chrono::milliseconds(50)); }  // 收尾补发零指令
    std::cout<<"Wrench stream finished; zero sent. Input watchdog also stops on interruption.\n";
    node.reset(); rclcpp::shutdown(); return 0;
  } catch(const std::exception & e) { std::cerr<<e.what()<<"\n"; rclcpp::shutdown(); return 1; }
}

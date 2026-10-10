// sensors_cpp 兼容入口的统一运行时：信号处理、ROS 参数声明与入口分派。
// 由 src/main.cpp 生成的全部入口共用（入口名经编译宏 COMPAT_ENTRY 注入），
// 如 ros2 run sensors_cpp depth / sensors_sub / sensors_sub_jilushuju / manual_control。
// 约定：--help 打印用法返回 0；--check 对未实现入口返回 69；未实现入口直接返回 69。
#include "sensors_cpp/entry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <iostream>
#include <set>
#include <stdexcept>
#include <csignal>
namespace compatibility {
// SIGINT/SIGTERM 中断标志，供 running() 和各入口主循环共同读取。
static volatile std::sig_atomic_t interrupted=0;
static void interrupt(int) { interrupted=1; }
bool running() { return rclcpp::ok() && !interrupted; }
// 仅保留命令名、尚未实现的旧入口（相机/控制器/轨迹）；运行它们不作任何动作而返回 69。
static const std::set<std::string> missing{
  "camera_display","camera_save","camera_ros","camera_parallel_ros",
  "auto_control","auto_control_new","semi_control","semi_control_fc",
  "semi_control_fcadrc","global_cam_pub","data_sub"};
// 主流程：--help/--check 快路径 → rclcpp::init → 声明参数 → BackendLease → 分派 manual/telemetry。
int run(const std::string &entry, int argc, char **argv) {
  for(int i=1;i<argc;++i) if(std::string(argv[i])=="--help" || std::string(argv[i])=="--check") {
    const bool unavailable=missing.count(entry);
    std::cout << entry << ": " << (unavailable?"entry registered; implementation unavailable":"available") << "\n";
    if(std::string(argv[i])=="--help") {
      std::cout << "C++ compatibility entry. ROS parameters: robot_namespace=/underwater_robot, auto_start=true, duration_s=0.\n";
      std::cout << "Telemetry: imu_source=raw (default) or calibrated; calibrated selects the saved attitude reference.\n"; // imu_source 仅遥测入口使用
      if(entry=="manual_control") std::cout << "W/S Fx, A/D yaw, I/J Fz, T/F Fy, Y/G roll, U/H pitch. Values are levels: 0=stop, 1=start, 100=max. Q or Ctrl+C stops and exits. preview=true disables enable requests.\n"; // preview 默认 false，true 时只观察不 enable
      if(entry=="sensors_sub_jilushuju") std::cout << "CSV is saved under " << DEFAULT_RECORD_DIR << "; override output_dir.\n"; // 记录目录来自编译期 DEFAULT_RECORD_DIR
      return 0;
    }
    return unavailable?69:0;
  }
  if(missing.count(entry)) {
    std::cerr << entry << ": command name is retained, but this camera/controller/trajectory feature has not been implemented. No backend or actuator is started. See README.\n";
    return 69;
  }
  rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None); // 自行处理信号，不用 rclcpp 默认 handler
  std::signal(SIGINT,interrupt); std::signal(SIGTERM,interrupt);
  int result=0;
  try {
    auto node=std::make_shared<rclcpp::Node>(entry);
    node->declare_parameter<std::string>("robot_namespace","/underwater_robot"); // 机器人命名空间，默认 /underwater_robot
    node->declare_parameter<bool>("auto_start",true); // false 时不自动准备共享后台，只复用已有后台
    node->declare_parameter<std::string>("serial_port","/dev/serial/by-id/usb-STMicroelectronics_Lamost_UnderWater_Robot_ComPort_3588346E3134-if00"); // 后台启动时使用的串口设备
    node->declare_parameter<double>("duration_s",0.0); // 运行秒数，0=不限时直到收到中断信号
    const auto imu_source=node->declare_parameter<std::string>("imu_source","raw"); // 遥测姿态来源，默认 raw
    if(imu_source!="raw" && imu_source!="calibrated") throw std::invalid_argument("imu_source must be raw or calibrated");
    if(entry=="manual_control"||entry=="forces_pub")node->declare_parameter<bool>("preview",false);
    if(entry=="time_read_test") {
      std::cout << "ROS time: " << node->now().seconds() << "\n";
    } else {
      BackendLease backend(node); // 进入作用域即检测/复用/启动共享后台，离开作用域释放租约
      if(running()) result=(entry=="manual_control"||entry=="forces_pub")?manual(entry,node):telemetry(entry,node); // 控制类入口走 manual，其余走 telemetry
    }
    node.reset();
  } catch(const std::exception &e) {
    std::cerr << entry << ": " << e.what() << "\n"; result=1; // 参数非法或后台准备失败统一返回 1
  }
  if(rclcpp::ok()) rclcpp::shutdown();
  return result;
}
}

#include "sensors_cpp/entry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <iostream>
#include <set>
#include <stdexcept>
#include <csignal>
namespace compatibility {
static volatile std::sig_atomic_t interrupted=0;
static void interrupt(int) { interrupted=1; }
bool running() { return rclcpp::ok() && !interrupted; }
static const std::set<std::string> missing{
  "camera_display","camera_save","camera_ros","camera_parallel_ros",
  "auto_control","auto_control_new","semi_control","semi_control_fc",
  "semi_control_fcadrc","global_cam_pub","data_sub"};
int run(const std::string &entry, int argc, char **argv) {
  for(int i=1;i<argc;++i) if(std::string(argv[i])=="--help" || std::string(argv[i])=="--check") {
    const bool unavailable=missing.count(entry);
    std::cout << entry << ": " << (unavailable?"entry registered; implementation unavailable":"available") << "\n";
    if(std::string(argv[i])=="--help") {
      std::cout << "C++ compatibility entry. ROS parameters: robot_namespace=/underwater_robot, auto_start=true, duration_s=0.\n";
      std::cout << "Telemetry: imu_source=raw (default) or calibrated; calibrated selects the saved attitude reference.\n";
      if(entry=="manual_control") std::cout << "W/S Fx, A/D yaw, I/J Fz, T/F Fy, Y/G roll, U/H pitch. Values are levels: 0=stop, 1=start, 100=max. Q or Ctrl+C stops and exits. preview=true disables enable requests.\n";
      if(entry=="sensors_sub_jilushuju") std::cout << "CSV is saved under " << DEFAULT_RECORD_DIR << "; override output_dir.\n";
      return 0;
    }
    return unavailable?69:0;
  }
  if(missing.count(entry)) {
    std::cerr << entry << ": command name is retained, but this camera/controller/trajectory feature has not been implemented. No backend or actuator is started. See README.\n";
    return 69;
  }
  rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT,interrupt); std::signal(SIGTERM,interrupt);
  int result=0;
  try {
    auto node=std::make_shared<rclcpp::Node>(entry);
    node->declare_parameter<std::string>("robot_namespace","/underwater_robot");
    node->declare_parameter<bool>("auto_start",true);
    node->declare_parameter<std::string>("serial_port","/dev/serial/by-id/usb-STMicroelectronics_Lamost_UnderWater_Robot_ComPort_3588346E3134-if00");
    node->declare_parameter<double>("duration_s",0.0);
    const auto imu_source=node->declare_parameter<std::string>("imu_source","raw");
    if(imu_source!="raw" && imu_source!="calibrated") throw std::invalid_argument("imu_source must be raw or calibrated");
    if(entry=="manual_control"||entry=="forces_pub")node->declare_parameter<bool>("preview",false);
    if(entry=="time_read_test") {
      std::cout << "ROS time: " << node->now().seconds() << "\n";
    } else {
      BackendLease backend(node);
      if(running()) result=(entry=="manual_control"||entry=="forces_pub")?manual(entry,node):telemetry(entry,node);
    }
    node.reset();
  } catch(const std::exception &e) {
    std::cerr << entry << ": " << e.what() << "\n"; result=1;
  }
  if(rclcpp::ok()) rclcpp::shutdown();
  return result;
}
}

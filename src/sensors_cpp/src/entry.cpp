// ==============================================================================
// src/sensors_cpp/src/entry.cpp
// ==============================================================================
// sensors_cpp 兼容入口统一运行时：
// - 信号处理与优雅退出 (running)
// - ROS 参数声明与通用参数绑定
// - 命令行选项快速解析 (--help, --check) 与入口分派
// ==============================================================================
// ------------------------------------------------------------------------------
// 【初学者导读】
// 所属：sensors_cpp 的兼容入口运行时（entry.cpp）。
// 角色：实现头文件声明的 run()/running()：信号处理、ROS 2 初始化、参数声明、命令分派。
// 与谁交互：调用 backend.cpp 的 BackendLease、manual.cpp 的 manual()、telemetry.cpp 的 telemetry()。
// ROS 2 概念白话：
//   rclcpp::init / shutdown＝打开和关闭 ROS 2 通信系统，每个进程只需一次。
//   Node（节点）＝能收发话题、调用服务的独立小程序。
//   Parameter（参数）＝节点配置项，命令行 -p 名:=值 设置，用 declare_parameter 登记默认值。
//   信号（SIGINT/SIGTERM）＝系统发给进程的“停止”通知，Ctrl+C 触发 SIGINT。
// C++ 概念白话：
//   std::set＝有序去重集合，这里用来快速判断“命令名是否在未实现列表里”。
//   shared_ptr<Node>＝共享所有权的智能指针，最后一个引用消失时自动释放节点。
//   RAII＝作用域对象离开时自动执行析构，BackendLease 就靠它在退出时释放后台租约。
// ------------------------------------------------------------------------------
#include "sensors_cpp/entry.hpp"

#include <algorithm>
#include <csignal>
#include <iostream>
#include <set>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

// compatibility 命名空间：本文件所有实现都放在这里，与头文件声明保持一致。
namespace compatibility {

// 全局中断标志：信号处理函数里只应改这种“原子标志”，不能做复杂操作。
// volatile 让编译器每次都真正读内存；sig_atomic_t 保证读写不会被信号打断成半截。
// SIGINT / SIGTERM 中断标志，供 running() 和各业务主循环原子读取
static volatile std::sig_atomic_t interrupted = 0;

// interrupt()：收到 Ctrl+C(SIGINT) 或终止(SIGTERM) 时被系统调用，只把标志置 1。
static void interrupt(int) {
  interrupted = 1;
}

// running()：后台就绪且未被打断时返回 true，供各业务 while 循环判断是否继续。
bool running() {
  return rclcpp::ok() && !interrupted;
}

// 仅保留命令名、尚未实现的旧入口 (相机/控制器/轨迹)；调用它们返回 69 (EX_UNAVAILABLE)
// missing 集合：列出“命令名保留但尚未实现”的旧入口（相机/控制器/轨迹相关）。
// run() 遇到它们返回 69（EX_UNAVAILABLE），提醒用户该功能尚未实现。
static const std::set<std::string> missing{
    "camera_display",
    "camera_save",
    "camera_ros",
    "camera_parallel_ros",
    "auto_control",
    "auto_control_new",
    "semi_control",
    "semi_control_fc",
    "semi_control_fcadrc",
    "global_cam_pub",
    "data_sub"};

// 主流程：--help / --check 快路径 -> rclcpp::init -> 参数声明 -> BackendLease -> 业务分派
// run() 主流程：--help/--check 快路径 -> rclcpp::init -> 声明参数 -> 建 BackendLease -> 业务分派。
int run(const std::string &entry, int argc, char **argv) {
// 先扫描命令行里的 --help / --check：无需初始化 ROS 2 就能打印信息并返回，速度快。
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "--check") {
// 若命令在 missing 里，--check 返回 69，--help 则打印可用性说明后返回 0。
      const bool unavailable = (missing.count(entry) != 0);
      std::cout << entry << ": "
                << (unavailable ? "entry registered; implementation unavailable" : "available")
                << "\n";

      if (std::string(argv[i]) == "--help") {
        std::cout << "C++ compatibility entry. ROS parameters: robot_namespace=/underwater_robot, auto_start=true, duration_s=0.\n";
        std::cout << "Telemetry: imu_source=raw (default) or calibrated; calibrated selects the saved attitude reference.\n";
        if (entry == "manual_control") {
          std::cout << "W/S Fx, A/D yaw, I/J Fz, T/F Fy, Y/G roll, U/H pitch. Values are levels: 0=stop, 1=start, 100=max. Q or Ctrl+C stops and exits. preview=true disables enable requests.\n";
        }
        if (entry == "sensors_sub_jilushuju") {
          std::cout << "CSV is saved under " << DEFAULT_RECORD_DIR << "; override output_dir.\n";
        }
        return 0;
      }
      return unavailable ? 69 : 0;
    }
  }

  if (missing.count(entry)) {
    std::cerr << entry
              << ": command name is retained, but this camera/controller/trajectory feature has not been implemented. "
              << "No backend or actuator is started. See README.\n";
    return 69;
  }

  // 自行处理退出信号，不用 rclcpp 默认 handler
// 初始化 ROS 2：解析 ROS 参数、建立 DDS 通信。这里自行处理信号，故关闭默认 handler。
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, interrupt);
  std::signal(SIGTERM, interrupt);

  int result = 0;
  try {
// 创建节点：节点名就是命令名，方便在 ros2 node list 里区分。最后 node.reset() 释放它。
    auto node = std::make_shared<rclcpp::Node>(entry);

    // 声明通用运行参数
// 声明参数：若命令行未提供就采用默认值。declare_parameter 返回参数当前值。
    node->declare_parameter<std::string>("robot_namespace", "/underwater_robot");
    node->declare_parameter<bool>("auto_start", true);
    node->declare_parameter<std::string>(
        "serial_port",
        "/dev/serial/by-id/usb-STMicroelectronics_Lamost_UnderWater_Robot_ComPort_3588346E3134-if00");
    node->declare_parameter<double>("duration_s", 0.0);

// 选择 IMU 数据来源：raw=原始姿态，calibrated=标定后姿态；出现其它值视为非法并抛异常。
    const auto imu_source = node->declare_parameter<std::string>("imu_source", "raw");
    if (imu_source != "raw" && imu_source != "calibrated") {
      throw std::invalid_argument("imu_source must be raw or calibrated");
    }

// preview 参数只在手动控制类入口声明：true 时只观察推力分配，不申请底层使能。
    if (entry == "manual_control" || entry == "forces_pub") {
      node->declare_parameter<bool>("preview", false);
    }

    if (entry == "time_read_test") {
      std::cout << "ROS time: " << node->now().seconds() << "\n";
    } else {
      // 进入作用域即按需复用/拉起后台，离开作用域自动释放租约
// 这就是 RAII：进入作用域按需复用/拉起共享后台，离开作用域自动释放租约。
      BackendLease backend(node);
// 只有后台就绪且未被打断才进入具体业务；否则直接走收尾清理。
      if (running()) {
        result = (entry == "manual_control" || entry == "forces_pub")
                     ? manual(entry, node)
                     : telemetry(entry, node);
      }
    }
    node.reset();
  } catch (const std::exception &e) {
    std::cerr << entry << ": " << e.what() << "\n";
    result = 1;
  }

// 收尾：如果 ROS 2 还开着就关闭它，释放通信资源，然后返回业务退出码。
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}

}  // namespace compatibility

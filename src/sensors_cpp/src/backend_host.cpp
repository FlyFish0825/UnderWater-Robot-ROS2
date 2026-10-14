// ==============================================================================
// src/sensors_cpp/src/backend_host.cpp
// ==============================================================================
// backend_host 可执行程序入口：
// 由兼容入口进程在需要时自动 fork + detach 拉起，一般无需手工直接运行。
// 用法：backend_host <runtime_dir> <namespace> <serial_port>
// 返回码：64 参数个数不对；0 表示已有其它宿主持有后台 (单例守护)；1 为启动异常。
// ==============================================================================
// ------------------------------------------------------------------------------
// 【初学者导读】
// 所属：sensors_cpp 包的可执行程序 backend_host。
// 角色：兼容入口（depth、manual_control 等）在需要时 fork + detach 拉起它；它再负责
//       启动 ros2 launch underwater_bringup bringup.launch.xml，并守护其整个生命周期。
// 为何存在：多个入口共用同一个后台硬件会话，避免重复打开串口、重复启动节点。
// ROS 2 概念：launch 文件＝一次性启动多个节点的脚本；命名空间/参数通过命令行传给 launch。
// C++ 概念：execl / fork＝类 Unix 的进程创建与程序替换；argv[1..3] 是命令行参数。
// ------------------------------------------------------------------------------
#include "sensors_cpp/entry.hpp"
#include <iostream>

// main：校验参数个数后把工作交给 compatibility::backend_host()。
// 参数：<runtime_dir> <namespace> <serial_port>。
int main(int argc, char **argv) {
  if (argc != 4) {
    return 64;  // 必须精确传入 3 个参数
  }
  try {
    return compatibility::backend_host(argv[1], argv[2], argv[3]);
  } catch (const std::exception &e) {
    std::cerr << "backend_host error: " << e.what() << "\n";
    return 1;
  }
}

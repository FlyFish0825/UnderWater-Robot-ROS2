// ==============================================================================
// src/manual_control/src/main.cpp
// ==============================================================================
// manual_control 兼容入口源：
// 提供命令名 manual_control, forces_pub, time_read_test；实际底层实现由 sensors_cpp 承载。
// ==============================================================================
// ------------------------------------------------------------------------------
// 【初学者导读】
// 所属：manual_control 包（历史命令名的兼容外壳）。
// 角色：只提供命令名 manual_control, forces_pub, time_read_test；真正的逻辑全部由 sensors_cpp 的 compatibility::run() 承载。
// 这样旧脚本里 ros2 run manual_control ... 不用改，底层实现却已统一到 C++。
// ROS 2 概念：一个可执行文件就是一个命令，内部通常对应一个或多个 Node（此处由 run() 创建）。
// ------------------------------------------------------------------------------
#include "sensors_cpp/entry.hpp"

// main：转调 compatibility::run()，COMPAT_ENTRY 在编译时被注入为命令名。
int main(int argc, char **argv) {
  return compatibility::run(COMPAT_ENTRY, argc, argv);
}

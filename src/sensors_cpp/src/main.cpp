// ==============================================================================
// src/sensors_cpp/src/main.cpp
// ==============================================================================
// sensors_cpp 所有兼容入口共用的 main 目标源文件：
// COMPAT_ENTRY 宏由 CMake 根据可执行目标名动态注入。
// 对应命令包括：depth, sensors_sub, sensors_sub_jilushuju, manual_control 等。
// ==============================================================================
// ------------------------------------------------------------------------------
// 【初学者导读】
// 所属：sensors_cpp 包，所有兼容入口（depth、sensors_sub、manual_control 等）共用的 main。
// 角色：main 本身什么都不做，只把命令名 COMPAT_ENTRY 和命令行交给 compatibility::run()。
// 关键点：COMPAT_ENTRY 由 CMake 在编译时按可执行目标名注入，因此同一个源文件能生成多个命令。
// ROS 2 概念：这里的入口最终会创建 Node、发布/订阅话题、调用服务，细节见 entry.cpp。
// ------------------------------------------------------------------------------
#include "sensors_cpp/entry.hpp"

// main：直接转调统一运行入口 run()，返回值就是进程退出码。
int main(int argc, char **argv) {
// COMPAT_ENTRY 是编译期注入的命令名字符串（宏），决定 run() 走哪条业务分支。
  return compatibility::run(COMPAT_ENTRY, argc, argv);
}

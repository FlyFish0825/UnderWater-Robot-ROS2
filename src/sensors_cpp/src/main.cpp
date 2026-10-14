// ==============================================================================
// src/sensors_cpp/src/main.cpp
// ==============================================================================
// sensors_cpp 所有兼容入口共用的 main 目标源文件：
// COMPAT_ENTRY 宏由 CMake 根据可执行目标名动态注入。
// 对应命令包括：depth, sensors_sub, sensors_sub_jilushuju, manual_control 等。
// ==============================================================================
#include "sensors_cpp/entry.hpp"

int main(int argc, char **argv) {
  return compatibility::run(COMPAT_ENTRY, argc, argv);
}

// ==============================================================================
// src/manual_control/src/main.cpp
// ==============================================================================
// manual_control 兼容入口源：
// 提供命令名 manual_control, forces_pub, time_read_test；实际底层实现由 sensors_cpp 承载。
// ==============================================================================
#include "sensors_cpp/entry.hpp"

int main(int argc, char **argv) {
  return compatibility::run(COMPAT_ENTRY, argc, argv);
}

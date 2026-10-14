// ==============================================================================
// src/depth_py/src/main.cpp
// ==============================================================================
// depth_py 兼容入口源：
// 提供命令名 depth_publisher；实际底层实现由 sensors_cpp 的 compat_runtime 承载。
// ==============================================================================
#include "sensors_cpp/entry.hpp"

int main(int argc, char **argv) {
  return compatibility::run(COMPAT_ENTRY, argc, argv);
}

// ==============================================================================
// src/sensors_cpp/src/backend_host.cpp
// ==============================================================================
// backend_host 可执行程序入口：
// 由兼容入口进程在需要时自动 fork + detach 拉起，一般无需手工直接运行。
// 用法：backend_host <runtime_dir> <namespace> <serial_port>
// 返回码：64 参数个数不对；0 表示已有其它宿主持有后台 (单例守护)；1 为启动异常。
// ==============================================================================
#include "sensors_cpp/entry.hpp"
#include <iostream>

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

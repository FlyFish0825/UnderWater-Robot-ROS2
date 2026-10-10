// backend_host 可执行程序入口：由入口进程在需要时 fork+detach 拉起，一般不手工运行。
// 用法：backend_host <runtime_dir> <namespace> <serial_port>（命令名 sensors_cpp/backend_host）。
// 返回码：64 参数个数不对；0 表示已有其它宿主持有后台（单例）；1 为启动异常。
#include "sensors_cpp/entry.hpp"
#include <iostream>
int main(int argc,char **argv) {
  if(argc!=4) return 64; // 必须正好 3 个参数
  try { return compatibility::backend_host(argv[1],argv[2],argv[3]); }
  catch(const std::exception &e) { std::cerr << e.what() << "\n"; return 1; }
}

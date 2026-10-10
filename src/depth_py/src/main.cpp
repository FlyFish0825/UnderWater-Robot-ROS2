// depth_py 兼容入口源（唯一源文件，被本包所有可执行目标复用）。
// 本包只提供命令名 depth_publisher；实际实现全部在 sensors_cpp 的 compat_runtime 中，
// 由 CMake 注入的 COMPAT_ENTRY 宏在运行期决定分派到哪个后端。不涉及 TCP 端口。
#include "sensors_cpp/entry.hpp"
// 薄入口：解析 COMPAT_ENTRY 指定的后端，并按其返回值作为进程退出码。
int main(int argc, char **argv) { return compatibility::run(COMPAT_ENTRY, argc, argv); }

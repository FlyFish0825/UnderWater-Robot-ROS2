// sensors_cpp 所有兼容入口共用的 main：COMPAT_ENTRY 由 CMake 按可执行目标名注入。
// 对应命令如 ros2 run sensors_cpp depth / sensors_sub / sensors_sub_jilushuju / manual_control。
#include "sensors_cpp/entry.hpp"
int main(int argc, char **argv) { return compatibility::run(COMPAT_ENTRY, argc, argv); }

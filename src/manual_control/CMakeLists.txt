# manual_control：以 C++ 保留原 fc_sensors_ros2 的手动控制类命令名。
# 产物：manual_control（手动操控入口）、forces_pub（发布 Forces 指令）、
#       time_read_test（时间读取自测），运行 `ros2 run manual_control <入口名>`。
# 无独立实现，链接 sensors_cpp::compat_runtime，用 COMPAT_ENTRY 宏分派。
cmake_minimum_required(VERSION 3.16)
project(manual_control)
find_package(ament_cmake REQUIRED)  # ROS 2 构建基础
find_package(sensors_cpp REQUIRED)  # 提供 compat_runtime 共享库
# 逐个生成兼容入口：入口名即 ros2 run 子命令名
foreach(entry manual_control forces_pub time_read_test)
  add_executable(${entry} src/main.cpp)  # 统一薄入口源
  target_compile_definitions(${entry} PRIVATE COMPAT_ENTRY="${entry}")  # 注入入口名，运行期据此分派
  target_link_libraries(${entry} sensors_cpp::compat_runtime)  # 链接共享后端
  install(TARGETS ${entry} DESTINATION lib/${PROJECT_NAME})  # 装入 lib/manual_control 供 ros2 run 调用
endforeach()
ament_package()

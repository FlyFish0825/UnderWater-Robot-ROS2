# imu_py：以 C++ 保留原 fc_sensors_ros2 的 Python 式命令名 imu_publisher。
# 产物：可执行文件 imu_publisher，运行 `ros2 run imu_py imu_publisher`。
# 无独立实现，仅链接 sensors_cpp 的 compat_runtime，用 COMPAT_ENTRY 宏分派到共享后端。
cmake_minimum_required(VERSION 3.16)
project(imu_py)
find_package(ament_cmake REQUIRED)  # ROS 2 构建基础
find_package(sensors_cpp REQUIRED)  # 提供 compat_runtime 共享库
# 逐个生成兼容入口：入口名即 ros2 run 子命令名
foreach(entry imu_publisher)
  add_executable(${entry} src/main.cpp)  # 统一薄入口源
  target_compile_definitions(${entry} PRIVATE COMPAT_ENTRY="${entry}")  # 注入入口名，运行期据此分派
  target_link_libraries(${entry} sensors_cpp::compat_runtime)  # 链接共享后端
  install(TARGETS ${entry} DESTINATION lib/${PROJECT_NAME})  # 装入 lib/imu_py 才能被 ros2 run 找到
endforeach()
ament_package()

# underwater_bringup：系统级启动包，只提供 launch/config，不含常规节点。
# 产物：share/underwater_bringup 下的 launch/ 与 config/（robot、manual、tcp、monitor 等入口）。
# 特殊需求：依赖查找与探针可执行文件全部位于 BUILD_TESTING 内，仅测试构建时编译安装。
cmake_minimum_required(VERSION 3.8)
project(underwater_bringup)
find_package(ament_cmake REQUIRED)  # ROS 2 构建基础
# 安装启动文件与参数配置，供 ros2 launch underwater_bringup <xxx>.launch.xml 使用
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
# 集成探针：仅 -DBUILD_TESTING=ON 时编译，不参与正常运行
if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME remote_manual_launch_contract
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/remote_manual_launch_test.py)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(underwater_interfaces REQUIRED)
  # 探针 1：telemetry_probe —— 校验传感器遥测话题
  add_executable(telemetry_probe ../../tests/integration/telemetry_probe.cpp)
  target_compile_features(telemetry_probe PUBLIC cxx_std_17)
  ament_target_dependencies(telemetry_probe rclcpp sensor_msgs underwater_interfaces)
  install(TARGETS telemetry_probe DESTINATION lib/${PROJECT_NAME})
  find_package(geometry_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  # 探针 2：imu_calibration_probe —— 校验 IMU 标定服务
  add_executable(imu_calibration_probe ../../tests/integration/imu_calibration_probe.cpp)
  target_compile_features(imu_calibration_probe PUBLIC cxx_std_17)
  ament_target_dependencies(imu_calibration_probe rclcpp std_srvs underwater_interfaces)
  install(TARGETS imu_calibration_probe DESTINATION lib/${PROJECT_NAME})
  find_package(underwater_mcu_gateway REQUIRED)
  # 探针 3：serial_control_probe —— 校验串口控制链路（复用网关协议库）
  add_executable(serial_control_probe ../../tests/integration/serial_control_probe.cpp)
  target_compile_features(serial_control_probe PUBLIC cxx_std_17)
  target_link_libraries(serial_control_probe underwater_mcu_gateway::gateway_protocol util)
  ament_target_dependencies(serial_control_probe rclcpp geometry_msgs std_srvs underwater_interfaces)
  install(TARGETS serial_control_probe DESTINATION lib/${PROJECT_NAME})
endif()
ament_package()  # 注册 ament 包（须放最后）

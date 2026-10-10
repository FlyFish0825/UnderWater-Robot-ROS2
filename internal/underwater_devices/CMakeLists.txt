# underwater_devices：设备层，把网关原始帧解码为 IMU/深度/电机状态并做 IMU 标定。
# 产出库：device_decoders（解码器）、attitude_reference（姿态参考）；
# 产出可执行文件：devices_node，运行 `ros2 run underwater_devices devices_node`。
# 特殊需求：C++17 + 严格告警，复用 underwater_mcu_gateway::gateway_protocol；
#           BUILD_TESTING 时注册 device_validity 与 imu_attitude_reference 测试。
cmake_minimum_required(VERSION 3.8)
project(underwater_devices)
find_package(ament_cmake REQUIRED)  # ROS 2 构建基础
find_package(rclcpp REQUIRED)  # 节点运行时
find_package(rcl_interfaces REQUIRED)  # 参数/日志标准接口
find_package(sensor_msgs REQUIRED)  # 发布标准 IMU/图像消息
find_package(std_srvs REQUIRED)  # 标准服务类型
find_package(underwater_interfaces REQUIRED)  # 内部状态消息定义
find_package(underwater_mcu_gateway REQUIRED)  # 复用网关协议库
# 库 1：device_decoders —— IMU/压力深度/推进器原始数据解码
add_library(device_decoders src/sensors/imu/imu_decoder.cpp src/sensors/pressure_depth/depth_decoder.cpp src/actuators/thrusters/motor_decoder.cpp)
target_compile_features(device_decoders PUBLIC cxx_std_17)  # 要求 C++17
target_include_directories(device_decoders PUBLIC include)  # 导出 public 头目录
target_link_libraries(device_decoders underwater_mcu_gateway::gateway_protocol)  # 复用网关协议库
ament_target_dependencies(device_decoders underwater_interfaces underwater_mcu_gateway)
target_compile_options(device_decoders PRIVATE -Wall -Wextra -Wpedantic)  # 严格告警
# 库 2：attitude_reference —— IMU 姿态参考基准（含文件读写实现）
add_library(attitude_reference src/sensors/imu/attitude_reference.cpp src/sensors/imu/attitude_reference_file.cpp)
target_compile_features(attitude_reference PUBLIC cxx_std_17)
target_include_directories(attitude_reference PUBLIC include)
target_compile_options(attitude_reference PRIVATE -Wall -Wextra -Wpedantic)
# 主可执行文件：devices_node —— 订阅网关数据，发布解码后的传感器状态并处理 IMU 标定
add_executable(devices_node src/devices_node.cpp src/sensors/imu/imu_calibration.cpp)
target_link_libraries(devices_node device_decoders attitude_reference)
ament_target_dependencies(devices_node rclcpp rcl_interfaces sensor_msgs std_srvs underwater_interfaces)
install(TARGETS devices_node DESTINATION lib/${PROJECT_NAME})  # 装入 lib/underwater_devices 供 ros2 run 调用
# 测试代码：仅 -DBUILD_TESTING=ON 时编译注册
if(BUILD_TESTING)
  enable_testing()
  add_executable(device_test test/device_test.cpp)  # 测试 1：解码数据有效性
  target_link_libraries(device_test device_decoders)
  ament_target_dependencies(device_test underwater_interfaces)
  add_test(NAME device_validity COMMAND device_test)
  add_executable(attitude_reference_test test/attitude_reference_test.cpp)  # 测试 2：IMU 姿态参考
  target_link_libraries(attitude_reference_test attitude_reference)
  add_test(NAME imu_attitude_reference COMMAND attitude_reference_test)
endif()
ament_package()  # 注册 ament 包（须放最后）

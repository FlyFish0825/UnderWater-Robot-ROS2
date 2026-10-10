# underwater_mcu_gateway：MCU 串口网关，负责串口协议编解码与 TCP 桥接。
# 产出库：gateway_protocol、tcp_serial_bridge、control_ownership、tcp_control_server；
# 产出可执行文件：mcu_gateway_node，运行 `ros2 run underwater_mcu_gateway mcu_gateway_node`。
# 特殊需求：C++17 + 严格告警；导出 gateway_protocolTargets 与 include/；
#           BUILD_TESTING 时注册 protocol/motor/depth_zero/tcp/ownership 五个契约测试。
cmake_minimum_required(VERSION 3.8)
project(underwater_mcu_gateway)
find_package(ament_cmake REQUIRED)  # ROS 2 构建基础
find_package(rclcpp REQUIRED)  # 节点运行时
find_package(rcl_interfaces REQUIRED)  # 参数事件/日志等标准接口
find_package(std_srvs REQUIRED)  # 标准服务类型
find_package(underwater_interfaces REQUIRED)  # 内部消息与服务定义
# 库 1：gateway_protocol —— 串口流解析、编解码与电机/深度命令构造（对外导出）
add_library(gateway_protocol src/protocol/stream_parser.cpp src/protocol/codec.cpp src/commands/motor_commands.cpp src/commands/depth_zero.cpp)
target_compile_features(gateway_protocol PUBLIC cxx_std_17)  # 要求 C++17
target_include_directories(gateway_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  # 构建期/安装期头文件导出
target_compile_options(gateway_protocol PRIVATE -Wall -Wextra -Wpedantic)  # 严格告警
# 库 2：tcp_serial_bridge —— 串口与 TCP 双向桥接（对应 9000 原始串口端口）
add_library(tcp_serial_bridge src/transport/tcp_serial_bridge.cpp)
target_compile_features(tcp_serial_bridge PUBLIC cxx_std_17)
target_include_directories(tcp_serial_bridge PUBLIC include)
target_compile_options(tcp_serial_bridge PRIVATE -Wall -Wextra -Wpedantic)
# 库 3：control_ownership —— 控制权仲裁，保证同一时刻只有一个串口持有者
add_library(control_ownership src/control_ownership.cpp)
target_compile_features(control_ownership PUBLIC cxx_std_17)
target_include_directories(control_ownership PUBLIC include)
target_compile_options(control_ownership PRIVATE -Wall -Wextra -Wpedantic)
# 库 4：tcp_control_server —— 控制权 TCP 服务端（对应 9002 控制权端口）
add_library(tcp_control_server src/transport/tcp_control_server.cpp)
target_compile_features(tcp_control_server PUBLIC cxx_std_17)
target_include_directories(tcp_control_server PUBLIC include)
target_link_libraries(tcp_control_server control_ownership)
target_compile_options(tcp_control_server PRIVATE -Wall -Wextra -Wpedantic)
# 主可执行文件：mcu_gateway_node —— 串口 IO + 上述库组装为 ROS 2 网关节点
add_executable(mcu_gateway_node src/nodes/mcu_gateway_node.cpp src/transport/serial_io.cpp)
target_link_libraries(mcu_gateway_node gateway_protocol tcp_serial_bridge tcp_control_server control_ownership)
ament_target_dependencies(mcu_gateway_node rclcpp rcl_interfaces std_srvs underwater_interfaces)
target_compile_options(mcu_gateway_node PRIVATE -Wall -Wextra -Wpedantic)
# 安装：协议库进 lib 并导出，节点进 lib/${PROJECT_NAME}，头文件进 include
install(TARGETS gateway_protocol EXPORT gateway_protocolTargets ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS mcu_gateway_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(gateway_protocolTargets HAS_LIBRARY_TARGET)  # 供 underwater_devices 等复用
ament_export_include_directories(include)
# 测试代码：仅 -DBUILD_TESTING=ON 时编译并注册 5 个契约测试
if(BUILD_TESTING)
  enable_testing()
  add_executable(protocol_test test/protocol_test.cpp)  # 测试 1：串口流解析协议
  target_link_libraries(protocol_test gateway_protocol)
  add_test(NAME protocol_contract COMMAND protocol_test)
  add_executable(motor_command_test test/motor_command_test.cpp)  # 测试 2：电机命令编码
  target_link_libraries(motor_command_test gateway_protocol)
  add_test(NAME motor_command_contract COMMAND motor_command_test)
  add_executable(depth_zero_test test/depth_zero_test.cpp)  # 测试 3：深度置零命令
  target_link_libraries(depth_zero_test gateway_protocol)
  add_test(NAME depth_zero_contract COMMAND depth_zero_test)
  add_executable(tcp_serial_bridge_test test/tcp_serial_bridge_test.cpp)  # 测试 4：串口/TCP 桥接
  target_link_libraries(tcp_serial_bridge_test tcp_serial_bridge)
  add_test(NAME tcp_serial_bridge_contract COMMAND tcp_serial_bridge_test)
  add_executable(control_ownership_test test/control_ownership_test.cpp)  # 测试 5：控制权互斥
  target_link_libraries(control_ownership_test control_ownership tcp_control_server tcp_serial_bridge)
  add_test(NAME control_ownership_contract COMMAND control_ownership_test)
endif()
ament_package()  # 注册 ament 包（须放最后）

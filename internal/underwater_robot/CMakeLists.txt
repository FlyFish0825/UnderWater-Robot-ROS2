# underwater_robot：机器人本体基础 bringup 包。
# 产出：可执行文件 robot_node，运行 `ros2 run underwater_robot robot_node`。
# 特殊需求：C++17；仅 GNU/Clang 追加 -Wall -Wextra -Wpedantic；
#           install 同时安装 launch/ 与 config/ 到 share/underwater_robot。
cmake_minimum_required(VERSION 3.8)
project(underwater_robot)

find_package(ament_cmake REQUIRED)  # ROS 2 构建基础
find_package(rclcpp REQUIRED)  # 节点运行时
find_package(rcl_interfaces REQUIRED)  # 参数事件/日志标准接口
find_package(std_msgs REQUIRED)  # 标准消息类型

# 主可执行文件：robot_node —— 本体节点
add_executable(robot_node src/robot_node.cpp)
target_compile_features(robot_node PUBLIC cxx_std_17)  # 要求 C++17
# 仅对 GNU/Clang 追加严格告警，避免其它编译器报未知选项
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(robot_node PRIVATE -Wall -Wextra -Wpedantic)
endif()
ament_target_dependencies(robot_node rclcpp rcl_interfaces std_msgs)

install(TARGETS robot_node DESTINATION lib/${PROJECT_NAME})  # 装入 lib/underwater_robot 供 ros2 run 调用
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})  # 安装启动文件与配置

ament_package()  # 注册 ament 包（须放最后）

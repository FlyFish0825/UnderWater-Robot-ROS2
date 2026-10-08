cmake_minimum_required(VERSION 3.8)
project(underwater_robot)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(robot_node src/robot_node.cpp)
target_compile_features(robot_node PUBLIC cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(robot_node PRIVATE -Wall -Wextra -Wpedantic)
endif()
ament_target_dependencies(robot_node rclcpp rcl_interfaces std_msgs)

install(TARGETS robot_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()

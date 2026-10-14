# underwater_control：六轴推力分配与控制权/手动指令管理包。
# 产出：库 rov_allocation（依赖第三方 ThrustAllocator 与 Eigen/OSQP）、
#       可执行文件 control_node、remote_manual_node 与 wrench_command，运行 `ros2 run underwater_control <名>`。
# 特殊需求：C++17 + 严格告警；用 add_subdirectory 引入 third_party/ThrustAllocator_API_CurrentROV，
#           并强制关闭其示例与 FetchContent 联网；install 附带 config/。
cmake_minimum_required(VERSION 3.22)
project(underwater_control LANGUAGES C CXX)

option(REMOTE_MANUAL_CORE_ONLY "Build offline portable manual protocol tests without ROS or hardware" OFF)
add_library(remote_manual_core STATIC src/remote/remote_manual.cpp)
target_compile_features(remote_manual_core PUBLIC cxx_std_17)
target_include_directories(remote_manual_core PUBLIC include)
target_compile_options(remote_manual_core PRIVATE -Wall -Wextra -Wpedantic)
if(REMOTE_MANUAL_CORE_ONLY)
  enable_testing()
  add_executable(remote_manual_test test/remote_manual_test.cpp)
  target_link_libraries(remote_manual_test remote_manual_core)
  add_test(NAME remote_manual_contract COMMAND remote_manual_test)
  return()
endif()

find_package(ament_cmake REQUIRED)  # ROS 2 构建基础
find_package(rclcpp REQUIRED)  # 节点运行时
find_package(rcl_interfaces REQUIRED)  # 参数/日志标准接口
find_package(geometry_msgs REQUIRED)  # 力/矩等控制指令消息
find_package(std_srvs REQUIRED)  # 标准服务类型
find_package(underwater_interfaces REQUIRED)  # 内部消息与服务定义

# 第三方推力分配库构建开关：不编译其示例，并禁止构建期联网拉取依赖
set(THRUST_ALLOCATOR_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
set(FETCHCONTENT_FULLY_DISCONNECTED ON CACHE BOOL "" FORCE)
# 以子目录引入第三方推力分配库（内部使用 Eigen/OSQP），EXCLUDE_FROM_ALL 避免多余目标
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/ThrustAllocator_API_CurrentROV"
  "${CMAKE_CURRENT_BINARY_DIR}/vendor" EXCLUDE_FROM_ALL)

# 库：rov_allocation —— 推力分配求解与线性 RPM 换算
add_library(rov_allocation src/allocation/allocator.cpp src/allocation/linear_rpm.cpp)
target_compile_features(rov_allocation PUBLIC cxx_std_17)
target_include_directories(rov_allocation PUBLIC include)
target_link_libraries(rov_allocation PUBLIC ThrustAllocator::thrust_allocator)  # 链接第三方分配器
target_compile_options(rov_allocation PRIVATE -Wall -Wextra -Wpedantic)

# 入口 1：control_node —— 控制权/手动指令主节点
add_executable(control_node src/authority/control_node.cpp)
target_link_libraries(control_node rov_allocation)
ament_target_dependencies(control_node rclcpp rcl_interfaces geometry_msgs std_srvs underwater_interfaces)

# 入口 2：remote_manual_node —— 远程上位机/手柄业务手动控制节点（TCP 9003）
add_library(remote_manual_tcp STATIC src/remote/remote_tcp.cpp)
target_link_libraries(remote_manual_tcp PUBLIC remote_manual_core)
target_compile_options(remote_manual_tcp PRIVATE -Wall -Wextra -Wpedantic)

add_executable(remote_manual_node src/remote/remote_manual_node.cpp)
target_link_libraries(remote_manual_node remote_manual_tcp)
target_compile_options(remote_manual_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(remote_manual_node rclcpp rcl_interfaces geometry_msgs std_srvs underwater_interfaces)

# 入口 3：wrench_command —— 直接下发六轴力/矩指令（调试用）
add_executable(wrench_command src/wrench_command.cpp)
target_compile_features(wrench_command PUBLIC cxx_std_17)
ament_target_dependencies(wrench_command rclcpp geometry_msgs)

# 安装可执行文件到 lib/underwater_control，配置目录到 share/underwater_control
install(TARGETS control_node wrench_command remote_manual_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# 测试代码：仅 -DBUILD_TESTING=ON 时编译
if(BUILD_TESTING)
  enable_testing()
  add_executable(remote_manual_test test/remote_manual_test.cpp)
  target_link_libraries(remote_manual_test remote_manual_core)
  add_test(NAME remote_manual_contract COMMAND remote_manual_test)
  add_executable(remote_tcp_test test/remote_tcp_test.cpp)
  target_link_libraries(remote_tcp_test remote_manual_tcp)
  add_test(NAME remote_tcp_contract COMMAND remote_tcp_test)
  add_executable(allocation_test test/allocation_test.cpp)  # 推力分配契约测试
  target_link_libraries(allocation_test rov_allocation)
  add_test(NAME thrust_allocation_contract COMMAND allocation_test)
  # Offline examples use the same allocator; never start ROS or open serial ports.
  add_executable(allocation_examples test/allocation_examples.cpp)  # 离线示例：不启动 ROS、不打开串口
  target_link_libraries(allocation_examples rov_allocation)
endif()

ament_package()  # 注册 ament 包（须放最后）

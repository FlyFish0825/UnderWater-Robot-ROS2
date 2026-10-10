# sensors_cpp：兼容层核心包，用 C++ 复刻原 fc_sensors_ros2 的 14 个命令名。
# 产出：共享库 compat_runtime（导出 include/ 头与 compatTargets）、C++ 宿主 backend_host，
#       以及 depth / sensors_sub / sensors_sub_jilushuju / camera_* / auto_control* /
#       semi_control* 入口可执行文件，运行方式 `ros2 run sensors_cpp <入口名>`。
# 特殊需求：C++17 + 严格告警；BUILD_TESTING 时另编 compat_probe 集成探针。
cmake_minimum_required(VERSION 3.16)
project(sensors_cpp)
find_package(ament_cmake REQUIRED)  # ROS 2 构建基础
# 一次性查找全部 ROS 依赖（rclcpp、消息与服务接口等）
foreach(dep rclcpp ament_index_cpp std_srvs geometry_msgs underwater_interfaces msg_pkg)
  find_package(${dep} REQUIRED)
endforeach()
# 共享后端库：所有兼容入口共用同一套运行逻辑
add_library(compat_runtime SHARED src/entry.cpp src/backend.cpp src/telemetry.cpp src/manual.cpp)
get_filename_component(ROBOT_WORKSPACE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
target_compile_definitions(compat_runtime PRIVATE DEFAULT_RECORD_DIR="${ROBOT_WORKSPACE_ROOT}/Recodes")  # 默认录制目录 <工作区>/Recodes
target_compile_features(compat_runtime PUBLIC cxx_std_17)  # 对下游要求 C++17
target_compile_options(compat_runtime PRIVATE -Wall -Wextra -Wpedantic)  # 严格告警
target_include_directories(compat_runtime PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)  # 构建期/安装期头文件导出
ament_target_dependencies(compat_runtime rclcpp ament_index_cpp std_srvs geometry_msgs underwater_interfaces msg_pkg)  # 后端库的 ROS 接口依赖
# 独立 C++ 宿主进程：不经过兼容入口名，直接驱动共享后端
add_executable(backend_host src/backend_host.cpp)
target_link_libraries(backend_host compat_runtime)
install(TARGETS backend_host DESTINATION lib/${PROJECT_NAME})
# 生成 12 个兼容入口（对应原 Python 命令名）：
#   depth → 深度/压力；sensors_sub → 传感器订阅显示；sensors_sub_jilushuju → 数据记录(记录数据)；
#   camera_display/camera_save/camera_ros/camera_parallel_ros → 相机显示、存图与 ROS 图像；
#   auto_control/auto_control_new → 自动控制；semi_control/semi_control_fc/semi_control_fcadrc → 半自动控制。
foreach(entry depth sensors_sub sensors_sub_jilushuju camera_display camera_save camera_ros camera_parallel_ros auto_control auto_control_new semi_control semi_control_fc semi_control_fcadrc)
  add_executable(${entry} src/main.cpp)  # 所有入口共用同一份 main.cpp
  target_compile_definitions(${entry} PRIVATE COMPAT_ENTRY="${entry}")  # 入口名宏，运行期分派到对应逻辑
  target_link_libraries(${entry} compat_runtime)  # 链接共享后端
  install(TARGETS ${entry} DESTINATION lib/${PROJECT_NAME})  # 装入 lib/sensors_cpp 供 ros2 run 调用
endforeach()
install(TARGETS compat_runtime EXPORT compatTargets LIBRARY DESTINATION lib)  # 导出共享库并登记到 compatTargets
install(DIRECTORY include/ DESTINATION include)  # 安装对外头文件
ament_export_targets(compatTargets HAS_LIBRARY_TARGET)  # 供下游 find_package(sensors_cpp) 使用
ament_export_dependencies(rclcpp ament_index_cpp std_srvs geometry_msgs underwater_interfaces msg_pkg)  # 依赖透传给下游
# 测试代码：仅 -DBUILD_TESTING=ON（colcon test 默认）时编译
if(BUILD_TESTING)
  add_executable(compat_probe ../../tests/integration/compat_probe.cpp)  # 兼容性集成探针
  target_link_libraries(compat_probe compat_runtime util)
  install(TARGETS compat_probe DESTINATION lib/${PROJECT_NAME})
endif()
ament_package()  # 注册 ament 包（须放最后）

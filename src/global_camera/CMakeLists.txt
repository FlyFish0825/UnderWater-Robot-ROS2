# global_camera：双目（side-by-side）相机采集与 JPEG TCP 转发包。
# 产出：库 stereo_stream（TCP 推流）、可执行文件 global_cam_pub（发布 ROS 图像）与
#       data_sub（订阅图像并落盘/显示）；运行 `ros2 run global_camera <global_cam_pub|data_sub>`。
# 特殊需求：OpenCV(core/imgproc/imgcodecs/videoio)、Threads、C++17；
#           install 附带 launch/ 与 config/；BUILD_TESTING 时注册 3 个测试目标。
cmake_minimum_required(VERSION 3.16)
project(global_camera)
find_package(ament_cmake REQUIRED)  # ROS 2 构建基础
find_package(rclcpp REQUIRED)  # ROS 2 C++ 客户端库
find_package(sensor_msgs REQUIRED)  # 图像消息 Image/CameraInfo
find_package(std_msgs REQUIRED)  # 标准消息类型
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio)  # 采集/编码依赖
find_package(Threads REQUIRED)  # 推流线程
# 推流库：封装 TCP 图像服务端，可被可执行文件与测试复用
add_library(stereo_stream src/stream_server.cpp)
target_compile_features(stereo_stream PUBLIC cxx_std_17)  # 要求 C++17
target_include_directories(stereo_stream PUBLIC include)  # 导出 public 头目录
target_link_libraries(stereo_stream PUBLIC Threads::Threads)  # 推流线程
target_compile_options(stereo_stream PRIVATE -Wall -Wextra -Wpedantic)  # 严格告警
# 入口 1：global_cam_pub —— 采集双目图像，发布 sensor_msgs 并推流到 TCP
add_executable(global_cam_pub src/global_cam_pub.cpp)
target_include_directories(global_cam_pub PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(global_cam_pub stereo_stream ${OpenCV_LIBS})
ament_target_dependencies(global_cam_pub rclcpp sensor_msgs std_msgs)
# 入口 2：data_sub —— 订阅相机话题，供显示/保存与 TCP 数据分发
add_executable(data_sub src/data_sub.cpp)
target_compile_features(data_sub PRIVATE cxx_std_17)
target_include_directories(data_sub PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(data_sub ${OpenCV_LIBS})
ament_target_dependencies(data_sub rclcpp sensor_msgs)
get_filename_component(WORKSPACE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
target_compile_definitions(data_sub PRIVATE DEFAULT_CAMERA_RECORD_DIR="${WORKSPACE_ROOT}/Recodes/stereo")  # 默认双目录像目录 <工作区>/Recodes/stereo
foreach(target global_cam_pub data_sub)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)  # 两个入口统一严格告警
endforeach()
# 安装可执行文件到 lib/global_camera，launch/config 到 share/global_camera
install(TARGETS global_cam_pub data_sub DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
# 测试代码：仅 -DBUILD_TESTING=ON 时编译注册
if(BUILD_TESTING)
  enable_testing()
  add_executable(stream_test test/stream_test.cpp)  # 测试 1：TCP 推流协议契约
  target_link_libraries(stream_test stereo_stream)
  add_test(NAME stereo_tcp_contract COMMAND stream_test)
  set_tests_properties(stereo_tcp_contract PROPERTIES TIMEOUT 20)  # 超时上限 20s
  add_executable(stereo_transform_test test/stereo_transform_test.cpp)  # 测试 2：双目图像朝向变换
  target_compile_features(stereo_transform_test PRIVATE cxx_std_17)
  target_include_directories(stereo_transform_test PRIVATE include ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(stereo_transform_test ${OpenCV_LIBS})
  add_test(NAME stereo_orientation COMMAND stereo_transform_test)
  add_executable(stereo_ros_probe test/stereo_ros_probe.cpp)  # 测试 3：ROS 图像话题探针（不注册 ctest）
  target_compile_features(stereo_ros_probe PRIVATE cxx_std_17)
  target_include_directories(stereo_ros_probe PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(stereo_ros_probe ${OpenCV_LIBS})
  ament_target_dependencies(stereo_ros_probe rclcpp sensor_msgs)
endif()
ament_package()  # 注册 ament 包（须放最后）

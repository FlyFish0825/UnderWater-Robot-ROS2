# underwater_description：机器人 URDF 描述包，纯资源包，无库与可执行文件。
# 产物：share/underwater_description/urdf/，安装后供 robot_state_publisher 等读取。
cmake_minimum_required(VERSION 3.8)
project(underwater_description)
find_package(ament_cmake REQUIRED)  # ROS 2 构建基础
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})  # 安装 URDF 到 share 目录
ament_package()  # 注册 ament 包（须放最后）

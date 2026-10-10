# msg_pkg：兼容层自定义消息包（ImuData/DepthSensor/CamPose/Globalcam/Forces）。
# 纯接口包：只用 rosidl 生成消息，不产出可执行文件或库，无第三方依赖。
cmake_minimum_required(VERSION 3.8)
project(msg_pkg)
find_package(ament_cmake REQUIRED)  # ROS 2 构建基础
find_package(std_msgs REQUIRED)  # 消息中复用标准类型
find_package(rosidl_default_generators REQUIRED)  # 消息代码生成器
# 声明对外提供的 5 个 .msg 接口，DEPENDENCIES 指定生成时引用的依赖包
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ImuData.msg" "msg/DepthSensor.msg" "msg/CamPose.msg" "msg/Globalcam.msg" "msg/Forces.msg"
  DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)  # 下游包运行时依赖
ament_package()  # 注册 ament 包（须放最后）

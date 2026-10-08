cmake_minimum_required(VERSION 3.8)
project(msg_pkg)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ImuData.msg" "msg/DepthSensor.msg" "msg/CamPose.msg" "msg/Globalcam.msg" "msg/Forces.msg"
  DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()

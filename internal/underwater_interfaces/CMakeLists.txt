cmake_minimum_required(VERSION 3.8)
project(underwater_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GatewayFrame.msg"
  "msg/RawSerial.msg"
  "msg/GatewayStatus.msg"
  "msg/ImuState.msg"
  "msg/ImuCalibrationStatus.msg"
  "srv/CalibrateImu.srv"
  "msg/DepthState.msg"
  "msg/SensorStatus.msg"
  "msg/MotorState.msg"
  "msg/MotorSetpoint.msg"
  "msg/AllocationStatus.msg"
  DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()

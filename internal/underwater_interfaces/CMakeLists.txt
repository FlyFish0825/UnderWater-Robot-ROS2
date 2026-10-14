# underwater_interfaces：内部消息与服务接口包（网关帧、IMU/深度/电机状态、控制权等）。
# 纯接口包：由 rosidl 生成 11 个 .msg 与 2 个 .srv，不产出可执行文件或库。
# 特殊需求：无第三方依赖，仅 std_msgs；下游靠 rosidl_default_runtime 运行。
cmake_minimum_required(VERSION 3.8)
project(underwater_interfaces)
find_package(ament_cmake REQUIRED)  # ROS 2 构建基础
find_package(rosidl_default_generators REQUIRED)  # 消息/服务代码生成器
find_package(std_msgs REQUIRED)  # 消息字段复用标准类型
# 声明全部对外接口：msg/GatewayFrame、RawSerial、GatewayStatus、ImuState、
# ImuCalibrationStatus、DepthState、SensorStatus、MotorState、MotorSetpoint、AllocationStatus，
# srv/CalibrateImu、ControlOwnership
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GatewayFrame.msg"
  "msg/RawSerial.msg"
  "msg/GatewayStatus.msg"
  "msg/ImuState.msg"
  "msg/ImuCalibrationStatus.msg"
  "srv/CalibrateImu.srv"
  "srv/ControlOwnership.srv"
  "srv/ValidateManualLease.srv"
  "msg/DepthState.msg"
  "msg/SensorStatus.msg"
  "msg/MotorState.msg"
  "msg/MotorSetpoint.msg"
  "msg/AllocationStatus.msg"
  DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)  # 下游包运行时依赖
ament_package()  # 注册 ament 包（须放最后）

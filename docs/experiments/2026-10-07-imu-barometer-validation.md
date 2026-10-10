# IMU 气压接入验证（2026-10-07）

## 已完成

- ROS ImuState 增加四个 baro 数值和独立有效位，0x85/133 与原始/姿态分组独立。
- 增加 sensors/imu/barometer、sensors/imu/fluid_pressure、sensors/imu/temperature。
- 数值缺失/失效为 NaN，原始/姿态消息不携带伪造气压。
- IMU 气压与 MS5837 压力、水面参考和水深保持分离。
- H750 sensor_service 新增0x85转发；原生驱动已经有数据解码，不改驱动/UART/校准配置。
- 文档同步了字段、单位、时序、能力位、未烧录状态和查看方式。

## 软件验证

ROS 7个包编译通过；5个CTest测试全部通过：
- protocol_contract：1292项。
- motor_command_contract：25项。
- depth_zero_contract：47项。
- device_validity：25项（含新气压字段、有效位、NaN、格式和目标检查）。
- thrust_allocation_contract：226项。

ROS合成集成通过：持续40次raw更新期间，
只注入1次气压数据，标准气压/温度各收到1次，之后气压仍独立过期。
没有把后续raw数据当作气压刷新。

固件 imu_sensor_host_test、imu_baro_stream_test 均通过。
新增测试使用字面量浮点字节布局，覆盖负高度、重复序号、未入队重试、
32位时间戳回绕、非有限数据有效位清除及真正的AA5B编码/CRC解码。
Debug/Release均构建通过，Flash分别118912/60468字节，均小于128KiB。

## Nano 部署与实机边界

ROS节点已重启为新消息结构，背景启动PID记录在 log/imu-barometer-live/launch.pid。
实机group=129能够输出姿态，新baro字段为false/NaN；这是非气压分组的正确输出。
MS5837水面参考压力仍为101453 Pa，未重新归零；电机控制armed=false。

当前现场H750旧固件没有0x85转发，采样窗口未收到0x85。
Windows/Nano均未检测到ST-Link/WCH/CMSIS-DAP烧录器，因此没有烧录新固件。
**尚未证明真实气压值、单位量级、频率及稳定性，不能把合成验证当作气压实测。**

固件工程基线：机器人中控分支 f690594；只修改传感器转发与相关文档/测试，未提交。
Release产物：
- ELF SHA-256：16d33f2e74c199987c21e33e2705793b53960793c0dda3fe7c76b7b518edffbf
- BIN SHA-256：7daa70dda8149256d2ea3adffe845ba6ee5e7998010ee13ebd0fac6662a97ef1

产物位于 F:/file/BaiduSyncdisk/Project/CAN_To_Uart/CAN_To_Uart/build/Release/。
烧录后需重新验证 IMU 气压组和深度数据。深度计上电默认使用固定空气参考 P0=100965 Pa；只有实测显示有偏差时才在空气/水面调用 zero 服务修正当前运行基准。

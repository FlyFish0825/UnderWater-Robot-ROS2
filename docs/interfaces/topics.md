# ROS 接口

默认前缀为 /underwater_robot。自定义消息见 internal/underwater_interfaces/msg。

| 相对话题 | 类型 | 语义 |
|---|---|---|
| status | std_msgs/String | C++ 基础节点的软件心跳 node_running |
| gateway/frames | underwater_interfaces/GatewayFrame | CRC/结构校验后的 AA55/58/59/5B，设备适配器内部入口 |
| gateway/raw_rx | underwater_interfaces/RawSerial | 可选原始 RX 块，默认不发布；包含启动积压和不完整帧 |
| gateway/status | underwater_interfaces/GatewayStatus | 连接、接收/错误统计、心跳年龄、五个队列占用率，约 1 Hz |
| sensors/imu/state | underwater_interfaces/ImuState | 原始0x80、姿态0x81、气压0x85分组发布；各组独立有效 |
| sensors/imu/calibrated | underwater_interfaces/ImuState | 新增：0x81 为已保存基准的相对姿态；其他分组保持原始传感器数据；无基准时姿态无效 |
| sensors/imu/calibration_status | underwater_interfaces/ImuCalibrationStatus | 新增：基准、采样进度、新鲜度、保存路径及失败原因；可靠、transient local |
| sensors/imu/barometer | underwater_interfaces/ImuState | 仅0x85及其失效通知，含气压/温度/高度/参考气压 |
| sensors/imu/fluid_pressure | sensor_msgs/FluidPressure | 仅有效IMU气压，Pa；不是MS5837压力 |
| sensors/imu/temperature | sensor_msgs/Temperature | 仅有效IMU气压计温度，°C |
| sensors/imu/data_raw | sensor_msgs/Imu | 加速度 m/s²、角速度 rad/s；姿态不可用，零协方差表示协方差未知 |
| sensors/depth/state | underwater_interfaces/DepthState | 原始 ADC、压力、温度、深度、零点和型号有效位 |
| sensors/pressure/fluid_pressure | sensor_msgs/FluidPressure | 仅有效压力，单位 Pa；未收到有效测量时不发布 |
| sensors/pressure/temperature | sensor_msgs/Temperature | 仅有效温度，单位摄氏度 |
| sensors/status | underwater_interfaces/SensorStatus | TARGET 1/2 状态、采样年龄、成功和错误累计计数 |
| actuators/thrusters/state | underwater_interfaces/MotorState | 节点 1..8 的普通反馈和心跳状态 |
| control/wrench_request | geometry_msgs/WrenchStamped | base_link 六轴模型输入；时间戳必须新鲜，单发布者 |
| control/manual_request | geometry_msgs/WrenchStamped | 键盘六轴档位 -100..100；0 停机，1 档起步；与 wrench_request 合计只能一个发布者 |
| control/allocation | underwater_interfaces/AllocationStatus | QP 推力、建议 RPM、死区后的预测六轴量和启停状态 |
| actuators/thrusters/setpoint | underwater_interfaces/MotorSetpoint | 绑定当前串口连接与 enable 会话的受限内部命令 |

所有 header.stamp 均为接收时刻（失效通知保留最后接收时刻），不是绝对采样时刻。
消费方必须检查 fresh/valid/online，不得把历史状态位当作新鲜测量；同时对标准话题设置超时。
IMU 磁场保持协议单位，不标记 Tesla；电机电流是固件估算母线电流，温度是 MCU 内部温度。
IMU 气压计字段、有效位、帧布局和固件要求见 [imu-barometer.md](imu-barometer.md)。
气压高度不用于替代水深，两个传感器的温度和参考压力也各自独立。
MotorState 中 heartbeat_valid 与 feedback_valid 独立，只有后者为真时四个数值字段有效。
模型未确认时深度计仍可能上报有效 D1/D2，但压力/温度/水深不因此有效。
ROS 输入帧话题是可信机器人内部接口，不是鉴权命令服务；部署到外部网络前需明确网络边界。

`control/enable`（std_srvs/SetBool）为显式启停服务，需 manual.launch.xml。
`sensors/imu/calibrate`（underwater_interfaces/CalibrateImu）采集已知静止基准，保存成功后才返回成功；
`sensors/imu/clear_calibration`（std_srvs/Trigger）清除当前基准。
这两项只修改主机上的姿态参考，不向 MCU 发送标定或电机命令。
数学约定、文件位置和 `imu_source` 选择见 [IMU 校正教程](../usage/05-imu-calibration.md)。
`sensors/depth/zero`（std_srvs/Trigger）把当前空气/水面处记录为深度零点，自动处理必要的 02BA 配置；
只有读回零点并收到新的有效深度测量后才 success=true。详见部署说明 depth-zero.md。
MotorSetpoint 由分配节点生成，外部物理力请求使用 wrench_request，键盘档位使用 manual_request；
两种输入不能同时占用控制权，不要绕过控制入口直接发布电机向量。
gateway/status 的 tx_frames/tx_bytes 只表示完整写入串口，不代表 CAN 应答或电机已经转动。

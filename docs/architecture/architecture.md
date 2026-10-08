# UnderWater-Robot-ROS2 架构与实施范围

项目名与 Nano 工作空间保持 `UnderWater-Robot-ROS2`。采用用户提供的内部分类结构，
没有引入参考方案中的工作空间名称。分类目录没有 package.xml，只有实际可编译模块才创建 ROS 包。

## 内部实现的七个包

以下包已移到 internal/，包名和调试入口保留。src/ 的六个入口包为 msg_pkg、imu_py、
depth_py、sensors_cpp、manual_control、global_camera；目前合计 13 个实际 ROS 包。
入口可用性、共享后台和数据适配见 [README](../../README.md)。

| 分类 | 包 | 当前职责 |
|---|---|---|
| core | underwater_interfaces | 九种消息：串口原始块、校验后帧、网关状态、IMU、深度、传感器状态、电机反馈、电机目标、分配状态 |
| hardware | underwater_mcu_gateway | C++17 Linux USB CDC 读取、独占打开、重连、分帧、CRC、时间戳和统计 |
| hardware | underwater_devices | C++17 IMU/深度/推进器反馈解码、SI 换算、有效位、去重、过期失效 |
| control | underwater_control | 用户库的六轴 QP 分配、临时比例 RPM、显式启停、单来源和输入超时 |
| robot | underwater_description | 仅 base_link 的 URDF 起点；不虚构安装外参、惯量或机器人尺寸 |
| robot | underwater_bringup | XML 整机/监测启动、实例参数、记录话题表、集成验证入口 |
| robot | underwater_robot | 保留前一版 C++ 软件心跳节点，兼容原基础启动命令 |

## 数据路径

USB CDC → SerialIo → Parser → GatewayFrame → devices_node → 设备话题

- 只有 mcu_gateway_node 打开共享 MCU 串口。
- transport 处理文件描述符和重连；protocol 是不依赖 ROS/Qt 的 C++ 库。
- 网关节点把完整帧路由为内部消息；具体传感器/电机 payload 的语义由设备适配器处理。
- 原始字节可选发布到 gateway/raw_rx，和解析器取得同一份读取副本；默认关闭以减少流量。
- 默认相对话题名，由启动文件统一加 underwater_robot 命名空间。
- 设备数据采用 SensorDataQoS，深度有限；状态消息使用可靠 QoS。
- header.stamp 表示 Nano 收到该帧的 ROS 时间，不是 MCU 采样时间；设备原始微秒时间另存。
- 首次打开丢弃 1 秒结构化输出以排空旧积压；随后拒绝落后当前设备时间高水位超过 500 ms 的帧。
  这是有限回绕/连续流假设下的旧帧过滤，不等于已完成时钟同步或绝对延迟标定。
- 重连重置解析/时间基准，connection_id 区分连接；测量默认 500 ms 过期，心跳/状态 2.5 s 过期。
- IMU 原始、姿态、气压三组独立失效；原始数据不会延长气压有效期，心跳不会延长电机测量有效期。
- 无效数值用 NaN 和 false 有效标志表达。标准 ROS 传感器话题只在对应测量有效时发布。

## 源码范围与后续功能

src 按原项目的六个业务入口包组织，internal 保存上述七个底层包。
未实现功能的空目录和 .gitkeep 已移除，后续功能集中记录在
[README 的后续规划](../../README.md#后续规划)，开始实施时再创建所需目录。
当前不新增闭环控制、相机、其他设备驱动、通信桥、估计/SLAM、任务管理或仿真模块。
原 underwater_* 包名、调试入口和原生消息保持不变。兼容入口新增自动后台管理、
msg_pkg 外部消息及原命令名；未实现功能的入口返回明确错误，不运行空算法。
项目自身的协议、串口、传感器及电机反馈模块均为 C++；启动配置采用 XML/YAML。

## 当前执行边界

SerialIo 在电机发送或深度命令启用时使用 O_RDWR，两者都关闭时使用 O_RDONLY。
深度命令默认可用，但只在明确调用 sensors/depth/zero 时查询/配置 02BA 并记录水面参考。
manual.launch.xml 显式 motor_tx:=true 时才允许电机命令，
只允许已校验的 0x100 电机速度/运行向量，启动先停机，每次 enable 必须建立零输出会话，
输入/命令超时停止，重连后必须重新 enable。没有通用参数修改、IMU硬件校准、照明或 Bootloader 入口。
主机停机依赖链路仍可写，不能替代 MCU 端通信看门狗。详情见动力分配使用说明。

IMU 世界参考系与安装姿态尚未确认。因此设备原始四元数/欧拉角保存在自定义消息；
标准 sensor_msgs/Imu 仅提供原始加速度和角速度，orientation_covariance[0]=-1。
未发布设备到 base_link 的 TF，也未启动虚构的定位/控制算法。

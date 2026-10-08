# 首版架构验收记录（2026-10-07）

环境：Nano 实际为 Jetson Orin NX，Ubuntu 22.04.5 / ROS 2 Humble / aarch64。
路径：/home/lamost/UnderWater-Robot-ROS2。Windows 工作区保存同一份源码。

## 构建

六个 ament_cmake 包全部通过 RelWithDebInfo 构建。
协议、串口、设备适配器和集成探针为 C++17；工程源码内没有 Python 文件。
构建产物中 ROS 工具链自动生成的消息绑定不属于手写 Python 模块。

## 离线验证

- protocol_contract：通过，1292 项检查。
- device_validity：通过，14 项检查。
- 覆盖分片、单比特破坏、CRC、混流、合法嵌套帧头、60 KB 噪声、半帧超时、
  g/SI 换算、状态有效位、NaN、普通电机反馈量纲与心跳 CRC。
- synthetic integration：通过，实际 ROS 设备节点接收内部合成输入，
  IMU、深度及电机反馈均出现有效/无效结果和超时失效通知。
  独立 ROS_DOMAIN_ID=189，gateway_enabled=false，没有打开串口。
- 这些检查不是电机控制、传感器标定或算法精度验收。

## 实机接收链路

约 2026-10-07 21:38:42–21:38:52（UTC+08:00），ROS_DOMAIN_ID=188，仅本机通信。
使用 tools/validate.sh live，C++ 集成探针约运行 10 秒。

| 项目 | 结果 |
|---|---:|
| IMU 原始组 | 212 |
| IMU 姿态组 | 424 |
| 标准 sensor_msgs/Imu | 212 |
| 深度计组 | 213 |
| 网关状态消息 | 9 |
| 电机普通反馈 | 0 |
| 网关统计 RX 字节（末次状态时刻） | 57084 |
| CRC 错误 / 结构错误 | 0 / 0 |
| 启动积压或过期过滤帧数 | 349 |
| 末次心跳年龄 | 350 ms |
| 无效值处理错误 | 0 |

数量是探针收到的消息数，包含发现/启动等待，不等同于采样频率或无丢包保证。

深度计末次状态 0x503：ONLINE、RAW_VALID、PROM_VALID、CONFIG_UNKNOWN。
D1=6320631，D2=7821145；pressure_valid=false、depth_valid=false。
对应物理量保留为 NaN，没有自动确认型号或归零。

本轮没有新的电机反馈，不能据此确认电机在线。
网关无串口发送 API，以 O_RDONLY 打开；没有下发运行、校准、配置、归零或 Bootloader 命令。
全部测试节点正常停止，fuser 确认串口已释放，没有开机自启。

## 尚未验证或实现

- USB 物理插拔的实机重连；串口异常处理和重连机制已实现，但本轮未主动插拔。
- 绝对采样时钟同步、IMU 世界参考系、设备安装外参、传感器标定精度。
- 电机当前在线状态、发送/控制/控制权链路、推进器分配。
- 相机、照明、网络桥、自动控制、研究算法、任务状态机和仿真。
- URDF 只有 base_link，不包含虚构尺寸或安装关系。

详细运行日志留在 Nano 的 log/integration-synthetic、log/integration-live，
离线日志在对应 build/<package>/Testing/Temporary 下，均为忽略的构建/测试产物。

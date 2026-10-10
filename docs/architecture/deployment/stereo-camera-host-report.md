# Nano 双目相机上位机接入报告

日期：2026-10-09。供 Underwater_GUI 相机页面修改时使用。

**当前部署说明（2026-10-10 复核）：** 相机转发已改为随 ROS launch 生命周期运行，旧开机服务已禁用。后续现场链路实收约 5.7–5.8 FPS，不能用下面早期 30.087 FPS 的记录代表当前性能；见[后续现场记录](../../experiments/2026-10-09-tcp-repair.md)和[操作手册](../../usage/09-forwarding-handbook.md)。当前测试后台使用 `camera_enabled=false`，9001 未开启，本次文档复核没有重测相机。

## 早期部署与测量记录（后续部署已替代）

早期记录曾在现场 Nano 编译部署并启用 `underwater-camera.service`，当时运行并设置开机启动；该方式现已取消。
Windows 真实 TCP 接收 300 帧并解码成功，最终本机接收实测 30.087 FPS，平均每帧 158,762 字节，
按 30 FPS 约 4.76 MB/s（38.10 Mbit/s，仅 JPEG 负载）。这是一段现场测量，画面复杂度与负载会改变结果。
两客户端接收、重连、服务重启恢复、ROS 左右压缩图与可选原始图、三对图片保存均已验证。
物理拔插相机和整机重启尚未实测；不以软件重启测试代替这两项硬件验证。

## 实现范围与连接

Nano `192.168.20.70` 上的 `global_camera/global_cam_pub` 独占 `/dev/video0`，
采集 Cherry Dual Camera 的 MJPG 左右拼接帧。按参考脚本 `camera_capture.py` 的约定：
默认完整尺寸 2560×720，左半为左目，右半为右目，各 1280×720。
用户要求默认左右并排显示。
输出目标为 30 FPS，JPEG 质量 80。设备端请求 60 FPS，采集解码和输出编码并行，
各阶段仅保留最新帧；30 FPS 是输出目标，实际交付性能以 Windows 实收和解码的现场测量为准。
整台相机倒置安装，Nano 默认先将完整拼接帧旋转 180°，再拆分左右目。
输出左目来自原始帧右半，输出右目来自原始帧左半；两目都同时转正。
转发 JPEG 已转正，**上位机不要再旋转或再次交换左右目**。
`rotate_180` 启动参数默认 true，可显式设 false 保留原始方向。

Windows 作为 TCP 客户端连接 `192.168.20.70:9001`。连接后不发请求，服务端直接推帧。
画面未标定、未矫正，没有视觉定位和深度结果。请不要把图像端口接入 BootloaderCommunicationService
的 MCU 字节解析器，也不要把 CamPose/Globalcam 的消息定义解释为已实现位姿。

| 用途 | 端口 | 协议 |
|---|---|---|
| 既有 MCU 原始串口转发 | 9000 | 原 AA55/AA58/AA5B 等串口字节流 |
| 新双目图像转发 | 9001 | UWSC v1，40 字节头 + 拼接 JPEG |

两个连接分别管理。图像节点不启动 MCU、分配器，不改变串口转发配置；图像客户端也不能发送控制命令。
服务为局域网无鉴权接收接口；部署边界为现有可信机器人网络。最多 4 个同时接收的客户端。

## UWSC v1 帧格式

所有整数使用**网络字节序（大端）**，头固定 40 字节，没有 C/C++ 结构体填充。

| 偏移 | 字节数 | 字段 | v1 定义 |
|---:|---:|---|---|
| 0 | 4 | magic | ASCII `UWSC`，十六进制 `55 57 53 43` |
| 4 | 2 | version | 1 |
| 6 | 2 | flags | 1，bit0 表示左右拼接，其他位为 0 |
| 8 | 8 | sequence | uint64，成功生成的输出帧编号，从 1 开始；同进程重连继续，进程重启重置 |
| 16 | 8 | receipt_ns | uint64，Nano ROS 时钟接收时刻，单位纳秒；不是硬件曝光时间 |
| 24 | 4 | width | uint32，完整拼接图宽；默认 2560，必须为偶数 |
| 28 | 4 | height | uint32，完整拼接图高；默认 720 |
| 32 | 4 | jpeg_size | uint32，后续 JPEG 字节数，不含头 |
| 36 | 4 | reserved | 0 |
| 40 | jpeg_size | jpeg | 一张完整彩色 JPEG，左右目均包含其中 |

解析限制：`0 < jpeg_size <= 8,388,608`；`2 <= width <= 8192` 且为偶数；
`1 <= height <= 4096`；version=1、flags=1、reserved=0。解码后图像尺寸必须等于头中尺寸。
不接受未知版本、异常长度或不匹配尺寸；异常时关闭图像连接并清空缓存，然后按重连策略重试。
协议没有校验和，依赖 TCP 完整传输与 JPEG 解码校验。

示例：sequence=1、receipt_ns=2、width=2560、height=720、jpeg_size=9 的头为：

```text
55 57 53 43 00 01 00 01
00 00 00 00 00 00 00 01
00 00 00 00 00 00 00 02
00 00 0A 00 00 00 02 D0
00 00 00 09 00 00 00 00
```

这只是帧头格式示例，9 字节不代表一张真实可解码 JPEG。

## Qt 接收要求

建议在 Service 层新增独立 `StereoCameraService`，使用自己的 `QTcpSocket`；
相机页面只消费已解码的图像、连接状态和帧信息，保持 `UI → Service → Transport` 边界。
TCP 连接建议设置 `socket.setProxy(QNetworkProxy::NoProxy)`，与既有 Nano 连接行为一致。

TCP 会出现半包和粘包，`readyRead` 不是一帧：

1. 将收到字节追加到专用接收缓存；缓存和单次读量均设置上限。
2. 不满 40 字节时等待；满 40 字节后验证 magic、版本、flags、长度、尺寸、reserved。
3. 使用 `qFromBigEndian<quint16/quint32/quint64>` 按各字段偏移读取；不要 reinterpret_cast 成结构体。
4. 未收齐 `40 + jpeg_size` 字节时等待；收齐后取 JPEG，移除当前帧，继续处理后续完整帧。
5. JPEG 解码在工作线程进行，先用 `QImageReader::size()` 验证 JPEG 内部尺寸与头一致，
   再使用 `QImage::fromData(jpeg, "JPEG")`，并再次校验解码尺寸。
6. 左图 `image.copy(0, 0, width/2, height)`；右图 `image.copy(width/2, 0, width/2, height)`。
   两张图源于同一张拼接 JPEG，不要用两个独立视频连接匹配左右帧。
7. UI 默认并排、保持比例显示。连续收到多帧时只提交最新完整帧，避免 UI 队列积压。

建议解析缓存上限为 `40 + 8 MiB`，分段从 socket 读取并边读边解析；不要先无界 readAll 再校验。
Qt socket 读缓存也设置有限上限，JPEG 解码队列深度设为 1。

## 断线、无图与状态

连接状态与图像新鲜度分别管理。TCP 已连上但还没收到有效图像，显示“等待相机画面”。
以 Windows 单调时钟记录最后一张有效帧的本地接收时刻；默认 2 秒没有新图，显示“画面超时”。
不要直接用 Nano receipt_ns 与 Windows 系统时间相减，两台机器可能未同步。

断开或错误时清除接收缓存、待解码帧、连接内的 sequence 状态，清空或明显标记最后画面已失效。
可按 1、2、5 秒间隔重连，上限 5 秒；成功收到新帧后恢复正常。
sequence 允许跳号，服务端会丢弃积压旧帧；新连接不要求从 1 开始。

Nano 相机读取失败时清空服务端缓存并断开既有图像连接，然后重开相机。
慢客户端发送单帧超过 2 秒也会被断开。服务端不发送 HTTP 响应，不提供请求应答和远程保存命令；
向图像端口发送字节会使该连接关闭。

ROS 状态话题 `/underwater_robot/camera/status` 为 std_msgs/String，内容是 JSON：
`online`、`device`、`width`、`height`、`eye_width`、`fps`、`sequence`、`tcp_port`、
`clients`、`tcp_frames_sent`、`rotation_degrees`、`calibrated`、`detail`。约 1 Hz；calibrated 当前固定 false。
TCP 仅传帧头与 JPEG，上述 JSON 不混入 9001 字节流；Qt 无需部署 ROS 即可看图。

## 运行与上位机验收

当前日常在 Nano 前台启动完整后台（只有在未运行后台时执行）：

```bash
cd /home/lamost/UnderWater-Robot-ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch underwater_bringup robot.launch.xml
```

仅调试相机时可选择 `ros2 launch global_camera stereo.launch.xml`，但不得与已有采集节点重复启动。历史服务模板为 `deploy/underwater-camera.service.example`，当前机器人不启用该服务。早期硬件验证见
[2026-10-09 双目验证记录](../../experiments/2026-10-09-stereo-camera-validation.md)。

修改相机参数后，在现场测试结束时正常停止原 launch，再以所需配置启动；不要恢复旧开机服务。`data_sub` 等订阅者可同时运行。启动流程见[相机教程](../../usage/07-stereo-camera.md)。

上位机应覆盖：真实画面、半包、连续多帧、长度越界、JPEG 损坏、节点重启、断线重连、
连接但无帧、界面切换退出、慢解码和 9000/9001 同时连接。
检查相机页面连接、断线、关闭时不会改变电机页面或原串口连接。
本报告只定义接入契约，本次不修改 Underwater_GUI。

# 2026-10-09 Nano 双目相机部署与验证

[双目教程](../usage/07-stereo-camera.md) · [上位机接入报告](../deployment/stereo-camera-host-report.md)

## 环境与范围

- Nano：`lamost@192.168.20.70`，aarch64，ROS 2 Humble，OpenCV 4.8.0。
- Nano 项目：`/home/lamost/UnderWater-Robot-ROS2`；现场目录不含 `.git`。
- 本地分支：`codex/nano-tcp-stream`；仅部署 `src/global_camera` 与相机服务模板。
- USB 相机：`1bcf:2d4f Cherry Dual Camera`，驱动 `uvcvideo`。
- `/dev/video0` 有 capture 能力；`/dev/video1` 不作为第二目使用。
- 参考用户 `Camera-Calibration/camera_capture.py`：单设备读拼接帧，中线拆分。
- 按用户确认的整台相机倒置安装，默认完整拼接帧旋转 180° 后再拆分，输出左右半与源左右半交换。
- 没有启动 MCU 串口、推进器控制或串口转发服务；本次只部署相机模块。

## 备份

部署前源包备份：
`/home/lamost/underwater-before-global-camera-20261009-191233/global-camera-original.tar.gz`。
SHA-256：`76638b53d7fafb899cd4b48ff3c089fdca1b749730fd30f70a8f9ec09aa1fcde`。

## 相机模式与性能

使用 V4L2 格式枚举核实 MJPG 支持 2560×720、10/15/20/25/30/60 FPS。
相同尺寸的 YUYV 仅枚举 2 FPS，因此采用 MJPG。

最终请求设备 60 FPS，输出节奏目标 30 FPS，使用两个 V4L2 缓冲。
采集解码与旋转/JPEG 编码并行，两者之间仅保留最新帧；串行处理的实测速率约 20 FPS，
通过并行处理达到最终 30 FPS。不以枚举速率或请求值作为实测速率。

| 最终 Windows TCP 验证 | 结果 |
|---|---|
| 连续完整帧 | 300 帧，sequence 1–300，严格递增，无缺帧 |
| 拼接图 | 2560×720，彩色 JPEG 全部解码成功 |
| 左右图 | 各 1280×720，同一源帧拆分 |
| 纳秒接收时间 | 严格递增 |
| 按 Windows 单调时钟实测接收 | 30.0865365265 FPS |
| 按源接收时间戳实测输出 | 30.1257502953 FPS |
| 平均 JPEG 大小 | 158,761.99 字节 |
| 估计 JPEG 负载 | 按 30 FPS 约 4.76 MB/s，38.10 Mbit/s |
| 两个客户端 | 收到 sequence 300/301，均解码成功 |
| 重新连接 | 正常收到有效帧 |

画面已人工查看，左右图正向、并排。未改动亮度、镜像、标定或矫正。
Windows 原始证据保存在本地 `artifacts/stereo-camera-20261009/`：
`stereo.jpg`、`left.png`、`right.png`、`tcp-validation.json`、`service-restart.json`。
JPEG 为有损压缩；拆分后保存 PNG 不恢复已损失的细节。

## 软件和 ROS 验证

- Nano 构建：`global_camera` 单包 RelWithDebInfo 编译通过。
- 两个 CTest 通过，0 失败：
  - TCP：大端帧头、长度约束、分段读取、只发最新帧、双客户端、离线清缓存、重连、慢客户端超时。
  - 旋转：完整帧旋转 180°、左右半交换、关闭旋转时不变、非法尺寸拒绝。
- `stereo_ros_probe` 在最终真实节点上通过：3 组时间戳一致的拼接 JPEG、左 JPEG、右 JPEG。
  先前也验证了可选左 BGR8、右 BGR8 的尺寸、frame_id 和数据长度；正式服务默认 publish_raw=false。
- `data_sub` 保存 3 对转正后的左右 PNG，保存会话：
  `/home/lamost/UnderWater-Robot-ROS2/Recodes/stereo/session_1791545995473616837/`。
- 第二个采集进程设置 TCP 禁用后仍被设备锁拒绝，退出码 1，未夺取正在运行的相机。
- 两个入口 `--check` 返回 0，检查本身不打开相机。

## 实际部署

已安装 `/etc/systemd/system/underwater-camera.service`，配置为 User=lamost，
启动 `ros2 launch global_camera stereo.launch.xml`，异常退出自动重启，已设置开机启动。
默认 TCP 监听 `0.0.0.0:9001`，上位机连接 `192.168.20.70:9001`。

真实服务重启检查：重启前 sequence=312；原连接收到 EOF，重连收到新进程 sequence=1。
验证的是相机服务重启与 TCP 恢复，没有重启 Nano 整机。

## 尚未验证或实现

没有实测物理拔插相机、整机重启后的启动时序、长时间运行和上位机 Qt 界面。
没有加载双目标定、发布 CameraInfo/TF、双目矫正、深度或视觉位姿。
这些边界已在上位机报告中注明；上位机应按约定处理超时、无图和断线。

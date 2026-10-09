# 双目相机采集、转发和记录

[教程索引](README.md) · [上位机接入报告](../architecture/deployment/stereo-camera-host-report.md)

## 画面来源

`global_camera` 为 C++17 ROS 2 包。`global_cam_pub` 使用 OpenCV 的 V4L2 后端，
读取 `/dev/video0` 的 MJPG 拼接画面；默认 2560×720、输出目标 30 FPS、JPEG 质量 80。
`capture_fps` 默认请求设备以 60 FPS 采集，`fps` 控制输出目标；实际速率以状态和接收实测为准。
采集解码与旋转编码在两个线程并行进行，两者之间只保留最新一帧，不累积旧画面。
按用户提供的 `Camera-Calibration/camera_capture.py` 约定，左半为左目，右半为右目，
各 1280×720。实际帧尺寸不符合配置时会报告离线并重试，避免静默采用错误裁剪尺寸。
整台相机倒置安装，默认 `rotate_180:=true`：先将完整拼接帧旋转 180°，再中线拆分。
ROS 图像、TCP JPEG 和记录图片均已转正。输出左目来自原始帧右半，输出右目来自原始帧左半。
`rotate_180:=false` 可保留相机原方向，上位机默认不要重复旋转。

只有采集节点打开相机。`data_sub` 和 TCP 上位机只接收图像，均不会打开相机或 MCU 串口。
已创建的图像帧是未标定的原始视图，JPEG 是有损压缩；没有加载内外参、发布 CameraInfo、
矫正、深度重建或发布 CamPose/Globalcam 位姿。

## 编译和启动

```bash
cd /home/lamost/UnderWater-Robot-ROS2
source /opt/ros/humble/setup.bash
colcon build --packages-select global_camera --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ros2 launch global_camera stereo.launch.xml
```

也可直接运行 `ros2 run global_camera global_cam_pub`。直接运行和 launch 的默认命名空间均为
`/underwater_robot`；launch 提供 `namespace`、`device`、`tcp_port` 参数。
参数只能在启动时配置，不支持运行中更改。配置模板为 `src/global_camera/config/stereo.yaml`。

```bash
ros2 run global_camera global_cam_pub --ros-args -p fps:=30.0 -p jpeg_quality:=80
ros2 launch global_camera stereo.launch.xml tcp_port:=9001
```

`tcp_port:=0` 禁用 TCP，仍发布 ROS 图像。`duration_s` 默认 0 为持续运行，可用于有界采集。
相机读取失败时清空待发送帧、断开图像客户端并重开设备；默认重试间隔 2 秒。
设备级进程锁阻止另一个 global_cam_pub 重复采集同一相机。

## 图像与状态话题

| 默认话题 | 类型 | 说明 |
|---|---|---|
| `/underwater_robot/camera/stereo/compressed` | sensor_msgs/CompressedImage | 左右拼接 JPEG，同一帧；TCP 使用同一份 JPEG |
| `/underwater_robot/camera/left/image_raw/compressed` | sensor_msgs/CompressedImage | 左半 JPEG，有订阅者时编码 |
| `/underwater_robot/camera/right/image_raw/compressed` | sensor_msgs/CompressedImage | 右半 JPEG，有订阅者时编码 |
| `/underwater_robot/camera/left/image_raw` | sensor_msgs/Image | 可选 BGR8，`publish_raw:=true` 时创建 |
| `/underwater_robot/camera/right/image_raw` | sensor_msgs/Image | 可选 BGR8，`publish_raw:=true` 时创建 |
| `/underwater_robot/camera/status` | std_msgs/String | JSON 状态，约 1 Hz，可靠、transient local |

图像 QoS 为 best effort、volatile、depth=1。左右图与拼接图使用同一次采集接收时间，
不是相机硬件曝光时间戳。`frame_id` 分别为 `left_camera_optical_frame`、
`right_camera_optical_frame`、`stereo_camera_optical_frame`，目前没有发布对应 TF。

```bash
ros2 topic echo /underwater_robot/camera/status std_msgs/msg/String
ros2 run global_camera data_sub
```

## 保存左右图片

```bash
ros2 run global_camera data_sub --ros-args -p save_images:=true -p max_pairs:=10 -p save_interval_s:=1.0
```

默认输出 `/home/lamost/UnderWater-Robot-ROS2/Recodes/stereo/session_<时间>/left` 和 `right`。
同一对图片同名，文件名含源图像接收时间和保存编号。每次运行创建独立会话目录。
`max_pairs:=0` 持续记录；`duration_s` 限定总运行时间；`output_dir` 修改父目录。
订阅端从拼接 JPEG 解码后拆分保存 PNG，因此保存的 PNG 仍包含 JPEG 的压缩损失。
这与原参考脚本从采集帧直接保存 PNG 的质量路径不同，标定采集需要评估压缩对角点的影响。
`data_sub` 默认只检查尺寸和打印接收信息，不写图片、不显示窗口。

## 上位机转发

Nano 监听 `0.0.0.0:9001`，Windows 主动连接 `192.168.20.70:9001`。
这是 UWSC v1 二进制 TCP 图像流，不是 HTTP/MJPEG、RTSP 或原串口协议。
每帧为 40 字节网络字节序头加一张左右拼接 JPEG；完整接口、Qt 解析要求和状态策略见
[上位机接入报告](../architecture/deployment/stereo-camera-host-report.md)。

服务最多允许 4 个客户端，仅保留最新完整帧；慢客户端单帧发送超过 2 秒会断开。
TCP 9000 原始串口与图像 9001 是独立连接，图像节点不会启动或更改串口服务。
相机节点退出后，图像端口关闭。Ctrl+C 可结束前台节点。
可选服务模板为 `deploy/underwater-camera.service.example`，模板本身不会启用开机启动。
当前 Nano 已按用户要求取消相机与串口转发的开机自启。
运行 `ros2 launch underwater_bringup robot.launch.xml` 会同时启动网关、两条转发与相机；
停止 launch 后两个端口一起关闭。兼容入口自动准备的后台也使用同一配置。
后台已运行时只启动订阅端，避免再次启动相机采集。
完整固定地址与启动流程见[局域网与 TCP 恢复](08-lan-recovery.md)。

## 检查

```bash
ros2 run global_camera global_cam_pub --check
ros2 run global_camera data_sub --check
colcon test --packages-select global_camera --event-handlers console_direct+
colcon test-result --test-result-base build/global_camera --verbose
```

`--check` 验证实现入口存在，返回 0，不打开相机；实时相机是否在线应读取 status。
原 `sensors_cpp camera_display/camera_save/camera_ros/camera_parallel_ros` 仍是未实现入口。
网络连接失败时先检查采集节点是否运行、9001 是否监听及双方网络连通性。

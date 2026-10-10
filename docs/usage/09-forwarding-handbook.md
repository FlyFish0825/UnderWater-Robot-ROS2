# 09 机器人串口与相机转发操作手册

[项目 README](../../README.md) · [教程总目录](README.md) · [串口详细教程](06-nano-tcp.md) · [相机详细教程](07-stereo-camera.md)

本手册用于现有机器人与 Windows 上位机的日常操作，工作空间为 `/home/lamost/UnderWater-Robot-ROS2`。命令对应当前 `main` 主分支；旧 `v1.0.0` 标签不包含当前相机转发、控制权及后续传感器修复。每次启动沿用设备配置，不自动校准深度或 IMU。

## 目录

- [地址与三个端口](#ports)
- [开机、登录与加载环境](#login)
- [启动与停止后台](#backend)
- [上位机连接串口转发](#serial)
- [上位机连接相机转发](#camera)
- [GUI 与 Nano 切换控制权](#ownership)
- [查看 IMU 与深度计](#sensors)
- [相机帧率与网络判断](#fps)
- [故障速查与只读检查](#checks)
- [版本、同步与验证记录](#version)

<a id="ports"></a>
## 地址与三个端口

机器人是服务器，Windows 上位机主动连接，不需要在 Windows 安装 ROS。

| 用途 | 地址 | 操作入口 |
|---|---|---|
| MCU 原始串口转发 | `192.168.20.70:9000` | 上位机“设置”→ `Nano TCP` |
| 双目相机转发 | `192.168.20.70:9001` | 上位机“视觉”→“机器人双目” |
| 串口控制权管理 | `192.168.20.70:9002` | 上位机自动连接管理端口；Nano 使用 ROS 服务 |

9000 的数据保持 MCU 原串口格式；9001 使用 UWSC v1 拼接 JPEG。两者的内容和解析器不同。SSH 的 22 端口可达，仅说明能登录机器人，不能证明转发已经启动。

固定地址配置随开机恢复；转发随 ROS 后台运行，**开机不自动启动转发**。只有网关打开 MCU 串口，只有相机采集节点打开相机。固定 IP 的维护见[局域网教程](08-lan-recovery.md)。

<a id="login"></a>
## 开机、登录与加载环境

连接机器人电源和局域网，在 Windows 终端登录：

```powershell
ssh -t lamost@192.168.20.70
```

随后在 Nano 终端执行；每开一个新终端都重新加载一次：

```bash
cd /home/lamost/UnderWater-Robot-ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

启动前先查看是否已有后台，以及设备是否存在：

```bash
pgrep -af 'ros2 launch|mcu_gateway_node|global_cam_pub'
ls -l /dev/serial/by-id/ /dev/video* 2>/dev/null
fuser -v /dev/ttyACM0 /dev/video0
```

若已有本项目后台，直接连接或查看数据。不要再启动第二个网关。默认 MCU 设备使用带标识的 by-id 路径；实际路径不同，按[首次启动教程](00-nano-first-start.md)指定 `serial_port`。WCH-Link 接在控制板上时 IMU 不能正常使用；验收 IMU 前拔下下载器，保持探头连接。

<a id="backend"></a>
## 启动与停止后台

### 方式 A：明确启动完整机器人后台

在已加载环境的 Nano 终端执行，并保持这个终端运行：

```bash
ros2 launch underwater_bringup robot.launch.xml
```

默认启动串口 9000、控制权 9002 和相机 9001；Nano ROS 电机发送默认关闭。相机缺失时相机节点重试，串口缺失时网关等待重新连接；端口监听不等于已收到真实数据。

后台已运行时，在其他终端只启动订阅端，例如：

```bash
ros2 run sensors_cpp sensors_sub
```

### 方式 B：沿用熟悉的入口自动准备后台

没有后台时，也可只运行：

```bash
ros2 run sensors_cpp sensors_sub
```

它会准备或复用共享后台，自动准备时也默认开启两条转发和控制权端口。最后一个兼容查看、记录或控制入口退出后，它创建的后台自动关闭。方式 A 手动启动的后台由原启动终端负责关闭，订阅端退出不会关闭它。两种方式选一种即可；普通 `ros2 topic` 查询不会启动后台。

### 停止与只测传感器

结束控制时，先在原控制入口正常停止；Nano 键盘入口按 `Q`。随后停止记录、相机预览和串口连接。方式 A 最后在启动后台的终端按 `Ctrl+C`；方式 B 退出全部兼容入口。各节点退出后相应转发端口关闭。

仅测试传感器时，可以在**启动新后台之前**选择：

```bash
ros2 launch underwater_bringup robot.launch.xml camera_enabled:=false
```

这时只有 9000/9002，相机 9001 未开启。运行中的后台不能通过另开一个 launch 修改参数；需要切换配置时，由启动者正常结束原后台再启动。上位机“断开”或“停止预览”只关闭对应客户端连接，不关闭 Nano 后台。

<a id="serial"></a>
## 上位机连接串口转发

1. 打开 Windows 上位机 `build/gui/rov_ui.exe`，进入“设置”。
2. 连接方式选择 `Nano TCP`，地址填 `192.168.20.70`，端口填 `9000`。
3. 只看数据时保持“上位机控制”未勾选，点击“连接”。
4. 收到心跳后查看 IMU、深度计和设备反馈；缺测仍显示无效，不能用心跳代替传感器有效性检查。

9000 一次只接纳一个客户端。其他 GUI 或串口验收探针已连接时，先让它正常断开；相机连接独立。系统 HTTP 代理不会用于本项目局域网 socket。详细的重连、运行库和验证说明见[串口转发教程](06-nano-tcp.md)。

<a id="camera"></a>
## 上位机连接相机转发

1. 确认完整后台已启动，或仅在未运行相机采集节点时使用独立相机入口。
2. 上位机进入“视觉”，来源选择“机器人双目”。
3. 地址填 `192.168.20.70`，端口填 `9001`，点击“启动预览”。
4. 查看左右目、最后帧及实时帧率；“保存当前帧”在 Windows 保存拼接图。
5. 点击“停止”结束预览；串口连接继续独立工作。

拼接尺寸为 2560×720，左右各 1280×720。相机倒置安装，Nano 已将完整拼接画面旋转 180° 后拆分输出，GUI 不再重复旋转。当前为未标定、未矫正画面；不会自动产生深度重建或位姿结果。

仅调试相机且没有其他相机采集节点时：

```bash
ros2 launch global_camera stereo.launch.xml
```

此入口仅启动相机，不启动 MCU 串口网关。详细参数、图片记录和协议见[相机转发教程](07-stereo-camera.md)。

<a id="ownership"></a>
## GUI 与 Nano 切换控制权

初始由 Nano 持有串口控制权，GUI 连接后只读。只有一个所有者可写串口；相机查看不需要串口控制权。

| 操作 | 结果 |
|---|---|
| GUI 勾选“上位机控制”或申请控制权 | 普通申请；Nano 占用时提示错误并要求明确接管 |
| GUI 点击“接管控制权” | 转移到 GUI；9000 重新连接并验证本次令牌后允许原始字节发送 |
| GUI 点击“释放控制权”或取消选项 | 撤销 GUI 写权限并归还 Nano |
| Nano 普通申请 | GUI 占用时失败，保持原所有者 |
| Nano 明确接管 | 转移到 Nano，撤销 GUI 令牌和旧串口写会话 |
| Nano 释放 | 进入无人持有状态，任一方可重新申请 |

在已加载 ROS 环境的 Nano 终端使用：

```bash
# 普通申请；被 GUI 占用时 success=false
ros2 service call /underwater_robot/gateway/control underwater_interfaces/srv/ControlOwnership "{action: 0}"

# 明确接管
ros2 service call /underwater_robot/gateway/control underwater_interfaces/srv/ControlOwnership "{action: 1}"

# Nano 释放
ros2 service call /underwater_robot/gateway/control underwater_interfaces/srv/ControlOwnership "{action: 2}"
```

回复中 `owner=0/1/2` 分别表示无人、Nano、GUI，须同时看 `success`。GUI 接管后通过 Nano 发送原串口字节，令牌握手由 Nano 移除，不给应用数据添加新封装。GUI 断线或管理心跳超时会撤销权限并归还 Nano；重连不会自动恢复旧控制命令。

接管/断线路径会尝试发送安全停止事务。Nano ROS 电机发送另受 `motor_tx` 与启用门禁保护，持有控制权不等于电机已启用；GUI 写权限不靠 `tcp_allow_tx` 开启，该旧参数已弃用。Nano 实际手动控制的启动配置见[手动控制教程](03-manual-control.md)，不要同时运行两个控制来源。

<a id="sensors"></a>
## 查看 IMU 与深度计

在已有后台上查看：

```bash
ros2 run sensors_cpp sensors_sub
ros2 topic echo /underwater_robot/sensors/status --filter 'm.target == 2' --once
ros2 topic echo /underwater_robot/sensors/depth/state --qos-reliability best_effort --once
```

第一条持续运行；要执行后两条，另开一个已加载环境的终端。深度应检查 `fresh`、压力/温度/深度有效位、`good_frames` 和 `errors`。原始 D1/D2 是 ADC 读数，不是水深。

设备固定为唯一的 MS5837-02BA，不需要确认、选择或写入型号。MCU 上电使用固定空气参考 **P0=100965 Pa**；正常启动和重连不执行归零。空气中的毫米级正负读数可能随压力变化，不能因此每天重新校准。只有现场确需修正时再使用[显式水面归零](02-sensors-recording.md#zero)。IMU 气压高度与水深独立。

2026-10-10 修复固件已烧录并实测：深度约 25 Hz、累计 29788 个有效样本、设备错误 0；拔下 WCH-Link 后 IMU 原始、姿态和气压同时有效。这是当次现场记录，当前情况以实时话题为准。

<a id="fps"></a>
## 相机帧率与网络判断

GUI 的“标称帧率 30 FPS”是输出目标，“实时帧率”是 Windows 成功解码的接收速率。查看 Nano 实际输出：

```bash
ros2 topic echo /underwater_robot/camera/status std_msgs/msg/String --once
```

`fps` 为节点实际输出速率，`tcp_frames_sent` 为服务累计发送帧数；它们不能代替 Windows 的实收帧率。`capture_fps` 默认请求设备 60 FPS，`fps` 默认限制输出目标为 30 FPS，设备是否接受请求仍需实测。

2026-10-09 同一现场链路测得 Nano 输出约 29.96–29.98 FPS，Windows 实收约 **5.7–5.8 FPS**，JPEG 平均约 198–200 KB。单帧解码约 13 ms，读取约 161 ms；调整缓存未消除差异。详见[现场性能记录](../experiments/2026-10-09-tcp-repair.md)。这不是当前每次连接都会得到的固定速率，也不能据此断定只由网络造成。

按每帧约 200 KB 估算，30 FPS 的 JPEG 负载约为 `200000 × 30 × 8 = 48 Mbit/s`，还需协议开销和吞吐余量。网卡显示的 100 Mbit/s 或 1 Gbit/s 是协商链路速率，不是端到端有效吞吐；判断能否实收 30 FPS 要同时看节点输出、实际 JPEG 大小、TCP 读取和 Windows 解码。当前实机记录尚未证明上位机实收 30 FPS。

<a id="checks"></a>
## 故障速查与只读检查

| 现象 | 先检查 |
|---|---|
| SSH 能连，9000/9002 连不上 | ROS 后台是否已启动，网关监听及 MCU USB 是否正常 |
| 串口能连，相机超时 | 是否用了 `camera_enabled:=false`，相机是否枚举、节点是否运行及 9001 是否监听 |
| 相机能连但没图 | 看相机 status 的在线状态；连接建立不等于收到有效 JPEG |
| 有心跳但没有 IMU | WCH-Link 是否已拔，IMU 接线/供电与实时有效位 |
| 有心跳但没有深度 | 探头是否接好，查看 target=2 的周期状态、样本年龄和错误码；先排查，勿先校零 |
| GUI 能看不能发 | 检查控制权；占用时明确点击“接管控制权” |
| Nano 申请失败，owner=2 | GUI 正占用；需要切换时执行 action 1 |
| 第二个串口客户端断开 | 9000 单客户端限制，先断开原客户端 |
| 预览只有约 6 FPS | 按上一节比较输出与实收；30 FPS 标签并非实测值 |

以下检查只读，不发送电机或校准命令：

```bash
ss -lntp | grep -E ':(9000|9001|9002)( |$)' || true
fuser -v /dev/ttyACM0 /dev/video0
ros2 topic echo /underwater_robot/gateway/status --once
ros2 topic echo /underwater_robot/sensors/status --filter 'm.target == 2' --once
```

Windows 终端可检查端口是否可达；不在已有串口客户端连接时额外探测 9000：

```powershell
Test-NetConnection 192.168.20.70 -Port 9000
Test-NetConnection 192.168.20.70 -Port 9001
Test-NetConnection 192.168.20.70 -Port 9002
```

TCP 探测会短暂建立连接，`TcpTestSucceeded=True` 只证明端口可达。完整的双路接收、JPEG 解码和重连验收见[局域网验收](08-lan-recovery.md#同时接收与重连验收)；人工硬件探针须在现场控制暂歇时执行。

<a id="version"></a>
## 版本、同步与验证记录

本机源码维护位置为 `G:/360MoveData/Users/Administered/Documents/ChatGPT/UnderWater-Robot-ROS2`，Nano 正式部署为 `/home/lamost/UnderWater-Robot-ROS2`。2026-10-10 已为机器人现有目录建立 Git 管理，本地、GitHub 与 Nano 统一为 `main`，原开发分支已合入主分支。更新前检查 `git status -sb`、`git branch --show-current` 和 `git rev-parse HEAD`，保留原配置与记录备份；网络正常时使用 `git pull --ff-only origin main`。Nano 无法下载 GitHub 时，可从本地通过局域网传入已校验的 Git bundle 与对应文件，不强制覆盖现场修改。代码变化后在 Nano 编译，重新加载 `install/setup.bash`，在结束当前测试后再切换后台。仅文档变化无需重新编译或重启。

- [传感器修复、GUI/Nano 控制权实机验收](../experiments/2026-10-10-sensor-fix-control-acceptance.md)
- [固定地址、双路转发与帧率实测](../experiments/2026-10-09-tcp-repair.md)
- [维护与故障排查](04-maintenance.md)

2026-10-10 编写本手册时，现场继续运行用户要求保留的传感器后台，`camera_enabled=false`、9000/9002 已监听、9001 未开启；本次编写手册没有重新启动后台或重测相机。该现场状态为记录快照，日常使用按上面的启动步骤选择配置。

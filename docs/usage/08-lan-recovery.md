# 固定局域网地址与两条 TCP 转发恢复

[教程总目录](README.md) · [原始串口](06-nano-tcp.md) · [双目相机](07-stereo-camera.md)

## 地址与端口

机器人提供服务器，电脑主动连接。目标地址沿用 `192.168.20.70`；
串口原始数据使用 TCP `9000`，双目拼接 JPEG 使用 TCP `9001`，独占控制权管理使用 TCP `9002`。
相机格式为 UWSC v1，不能用串口解析器读取相机端口。

2026-10-09 已在真实机器人上持久保存 `192.168.20.70/24`，绑定网卡
`enP8p1s0` / `74:25:54:90:98:84`，自动连接优先级 100，网关与 DNS 为 `192.168.20.1`。
已在机器人重新启动后用同一地址建立新的 SSH 连接并核对实际静态配置。

## 随 ROS 2 启动两条转发

按用户要求，开机只恢复固定网络地址，**不自动启动转发进程**。
启动机器人 ROS 2 后台后，原始串口 9000、控制权 9002 和相机 9001 一起启动；停止对应后台节点后端口关闭。显式设置 `camera_enabled:=false` 时不启动相机和 9001。日常操作见[转发手册](09-forwarding-handbook.md)。
现场相机系统服务和旧用户串口服务已停止并禁用，本轮临时串口系统服务已移除。

推荐启动：

```bash
cd /home/lamost/UnderWater-Robot-ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch underwater_bringup robot.launch.xml
```

`base.launch.xml`、`manual.launch.xml`、`monitor.launch.xml` 和 `tcp.launch.xml`
默认也启动两条转发。兼容传感器/记录/手动入口自动准备的后台使用 `manual.launch.xml`，
因此沿用 `ros2 run sensors_cpp sensors_sub` 等常用入口时，也自动准备两个转发。
`ros2 topic` 查询等普通 ROS 2 CLI 命令不会创建后台。

网关仍只有一个串口所有者。Nano 初始持有控制权，GUI 初始只读，明确接管后可发送原始串口数据；启用 TCP 不自动开启 Nano ROS 电机发送。控制权操作见[串口教程](06-nano-tcp.md#duplex)。
相机由独立节点采集，缺失时重试，不要求停止其他节点。
需要关闭全部转发或隔离测试时，在支持这些参数的 `base.launch.xml` 等入口显式传 `tcp_port:=0 tcp_control_port:=0 camera_enabled:=false`；仅测试传感器时，`robot.launch.xml camera_enabled:=false` 保留 9000/9002。
改变设备或图像端口可传 `camera_device:=/dev/video0 camera_tcp_port:=9001`。
相机节点单独调试仍可使用 `ros2 launch global_camera stereo.launch.xml`，
但应先停止已有采集后台，避免重复占用相机/端口。

```bash
systemctl is-enabled underwater-camera.service
systemctl --user is-enabled underwater-tcp.service
ss -ltnp | grep -E ':(9000|9001|9002)( |$)'
fuser -v /dev/ttyACM0 /dev/video0
```

如果串口不存在，网关继续等待设备，客户端连接会被关闭；相机缺失时相机节点重试。
端口监听只证明进程已启动，仍需下面的数据验收。
只读原始串口目前只允许一个客户端，验收前应断开已有 GUI 的串口连接。

<a id="ssh-可达但-tcp-端口拒绝"></a>
## SSH 可达，但 TCP 端口拒绝

SSH 的 22 端口可连接，只证明 Nano 的网络地址可达；它不表示 ROS 串口网关或相机转发已经启动。
机器人后台未运行时，9000、9001、9002 都不会监听。通过 SSH 在 Nano 检查：

```bash
systemctl --user is-active underwater-robot-session
ss -lntp | grep -E ':(9000|9001|9002)( |$)' || true
pgrep -af 'mcu_gateway_node|global_cam_pub'
ls -l /dev/serial/by-id/ /dev/ttyACM* 2>/dev/null
fuser -v /dev/ttyACM0
```

若后台未运行，确认没有另一个终端或兼容入口已经持有串口后，再按上面的推荐命令启动一次 `robot.launch.xml`。
如果 `/dev/serial/by-id/` 中没有 STM32 CDC 设备，先恢复控制板 USB 连接和上电状态；不要靠重复启动 ROS 网关来修复缺失的串口设备。
9000/9002 已监听但客户端仍失败时，再核对监听地址是否为 `0.0.0.0`、Nano 实际地址和 Windows 防火墙规则。
9000 可连接却无心跳时，检查串口设备与 `GatewayStatus`；9001 单独失败则检查相机节点与 `/dev/video0`。

## 固定 IPv4，并保留远程回退

先检查机器人的实际以太网接口、当前连接配置、网关和 DNS：

```bash
ip -br address
ip route
nmcli device status
nmcli -f GENERAL.DEVICE,GENERAL.HWADDR,GENERAL.CONNECTION,IP4.ADDRESS,IP4.GATEWAY,IP4.DNS device show
```

确认网卡由 NetworkManager 管理，地址、掩码和网关适合实际局域网。
本机器人已使用 `192.168.20.70/24` 与 `192.168.20.1`；其他现场应使用实际网段。
网络管理员可将 `.70` 从动态地址池排除，或者按机器人网卡 MAC 保留，
避免 DHCP 把同一地址分配给别的设备。此次没有访问路由器后台，ARP 冲突检测只能检查当时在线设备。

NetworkManager 的 `ipv4.method=manual` 把地址写入持久连接配置，
配合 `connection.autoconnect=yes` 在启动时应用。依据
[NetworkManager 配置文档](https://www.networkmanager.dev/docs/api/1.34/nm-settings-nmcli.html)。
这里不使用一次性的 `ip address add`。

[固定地址脚本](../../tools/robot-static-ip.sh)支持 stage、confirm、status、rollback：

1. 复制当前以太网连接，绑定实际网卡 MAC 和接口，保留原连接。
2. 检查目标地址冲突，创建独立于 SSH 会话的 180 秒回退定时任务。
3. 激活静态连接；此时新配置尚未自动用于开机。
4. 从电脑用新地址建立新的 SSH 连接，然后确认，取消回退并启用静态配置的开机连接。
5. 若不能连上或没有确认，定时任务恢复原连接；确认与回退使用文件锁串行执行。

以下使用已核对的机器人网卡，其他设备需替换接口、地址和 DNS。

```bash
sudo bash tools/robot-static-ip.sh stage enP8p1s0 192.168.20.70/24 192.168.20.1 192.168.20.1
```

在**新建立的 SSH 连接**中执行：

```bash
sudo bash /var/lib/underwater-network/robot-static-ip.sh confirm
```

查看实际配置：

```bash
sudo bash tools/robot-static-ip.sh status
nmcli connection show --active
```

若尚未确认，也可以手动恢复：

```bash
sudo bash /var/lib/underwater-network/robot-static-ip.sh rollback
```

脚本要求 Bash、NetworkManager、systemd、flock、iproute2，地址变化时还要求 iputils-arping。
确认后保留原连接配置，关闭其自动连接；IPv6 保留原配置。
脚本的本地模拟测试不代替真实 NetworkManager、远程回退和整机重启测试。

## 同时接收与重连验收

电脑运行 [只读验收客户端](../../tools/test_tcp_forwarding.py)，需要 Python 和 Pillow。
该文件仅为测试工具，机器人运行时转发模块仍为 C++。

```text
python tools/test_tcp_forwarding.py --host 192.168.20.70 --duration 10 --output artifacts/tcp-repair/lan-acceptance.json
```

测试同时连接 9000/9001，然后关闭并重连两条连接：

- 串口：必须收到 CRC 正确的 AA58 心跳，记录各协议族 CRC 正确帧计数和字节量。
- 相机：逐帧校验 UWSC 头、尺寸、长度、序号和源时间戳，再实际解码 JPEG，记录尺寸和接收 FPS。
- 两条链路都不发送命令。失败报告保留具体异常，不把仅能建立连接当作测试通过。

再停止并重新启动 ROS 2 后台，重跑验收，检查新会话恢复；确认地址后重启整机，
用同一 `.70` 地址建立全新 SSH 连接，确认两个端口尚未监听，然后启动 ROS 2 并再次验收。
最终要同时证明：静态连接在开机后生效、转发跟随 ROS 2 生命周期、真实相机帧和真实 MCU 心跳均持续到达。
伪服务器、协议单元测试和服务 active 状态不能替代这三个现场条件。

## 本地软件检查

```text
bash -n tools/robot-static-ip.sh
bash tools/test_robot_static_ip.sh
python tools/test_tcp_forwarding_client.py
```

模拟测试验证配置阶段、确认、回退、地址冲突拒绝、错误子网拒绝及未激活静态配置不能确认。
客户端测试验证已知协议报文、半包、CRC 错误、JPEG 错误、非法头、同时接收和重连，
使用本机伪服务器，不代表机器人在线。

# 06 Nano 原始串口 TCP 服务

[教程总目录](README.md) · 上一篇：[IMU 基准姿态校正](05-imu-calibration.md) · [项目 README](../../README.md)

## 本页索引

- [连接方式](#topology)
- [Nano 启动](#server)
- [上位机连接](#client)
- [双向模式与串口归属](#duplex)
- [用户服务](#service)
- [重连与故障排查](#recovery)
- [验证范围](#validation)

<a id="topology"></a>
## 连接方式

```text
MCU USB CDC ↔ Nano 现有 mcu_gateway ↔ 原始字节 TCP ↔ 上位机 TcpTransport
                    ↓                                  ↓
              原有 ROS 解析                       原有协议/数据/界面
```

Nano 是服务器，上位机是客户端。只配置 Nano 固定地址和端口，不记录客户端电脑的 IP；
电脑 IP 变化后重新连接即可。当前现场 Nano 为 `192.168.20.70`，示例端口 `9000`。
Windows 上位机不需要安装 ROS。

只在现有网关包中增加一个 TCP 字节模块，不增加第二个串口读取进程，也不增加 ROS 包。
网络中直接传输原始串口字节，不加 JSON、长度头或 ROS 封装，不依赖 IMU/深度计在线。
AA55、AA58、AA5B 等协议仍由两端原有协议层处理，TCP 分片和粘包不改变帧语义。

<a id="server"></a>
## Nano 启动

先正常停止已有网关，确认没有其他进程占用串口：

```bash
fuser -v /dev/ttyACM0
```

使用包含 TCP 模块的新源码编译：

```bash
cd /home/lamost/UnderWater-Robot-ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --packages-select underwater_mcu_gateway underwater_bringup \
  --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

已有工作空间可按上述选择包编译；新机器第一次需要先按 [安装教程](01-installation.md) 完整构建依赖。

只启用原始串口服务器、禁止所有主机命令的最小启动命令：

```bash
ros2 run underwater_mcu_gateway mcu_gateway_node --ros-args \
  -r __ns:=/underwater_robot \
  -p tcp_port:=9000 \
  -p tcp_bind_address:=0.0.0.0 \
  -p tcp_allow_tx:=false \
  -p enable_depth_commands:=false \
  -p enable_motor_tx:=false
```

串口默认使用现有 MCU by-id 路径；需要时加 `-p port:=/dev/ttyACM0`。
串口不存在时服务会继续等待，连接恢复后上位机重连。没有传感器不影响转发网关心跳和 CAN 数据。

若还需要 ROS 设备解析节点，可改用下列启动入口，二者不要同时运行：

```bash
ros2 launch underwater_bringup tcp.launch.xml \
  tcp_port:=9000 tcp_allow_tx:=false serial_port:=/dev/ttyACM0
```

此 launch 沿用原网关配置，深度归零服务仍是独立、显式调用的能力；默认不允许电机发送。

| 参数 | 默认值与用途 |
|---|---|
| `tcp_port` | 网关节点默认 0，关闭网络；TCP launch 默认 9000 |
| `tcp_bind_address` | `0.0.0.0`；可以改成 Nano 的具体局域网 IPv4 地址 |
| `tcp_allow_tx` | false；true 时允许客户端原始字节写入串口 |

这是可信局域网内的明文字节服务，不包含公网身份认证或加密，不应通过路由器映射到公网。

<a id="client"></a>
## 上位机连接

使用已包含 `TcpTransport` 的 Underwater_GUI：

1. 打开“设置”页的连接栏。
2. 将连接方式从 `USB CDC` 改为 `Nano TCP`。
3. 填入 Nano 地址 `192.168.20.70`，端口 `9000`。
4. 首次测试保持“允许发送”未勾选，点击“连接”。
5. 等待显示 TCP 已连接及下位机心跳。

地址、端口和连接方式会保存；发送权限不会保存为下次自动开启。
首次启动或重新打开程序后点击连接即可；网络意外断开时当前连接请求会自动重试。
点击“断开”会取消自动重连。切回 USB 模式恢复原有 USB 扫描和连接行为。

连接到 Nano 时不再扫描并抢占本机 USB 设备。TCP 走局域网直连，不继承系统 HTTP 代理。
解析、Dashboard、Motor Debug、传感器和科研记录继续使用同一条现有服务链。

<a id="duplex"></a>
## 双向模式与串口归属

只读模式已经满足查看原始串口数据。需要发送配置、控制或升级命令时：

1. 正常停止只读服务。
2. Nano 以 `tcp_allow_tx=true` 启动。
3. 上位机重新连接前勾选“允许发送”。

```bash
ros2 launch underwater_bringup tcp.launch.xml \
  tcp_port:=9000 tcp_allow_tx:=true serial_port:=/dev/ttyACM0
```

双向模式将串口命令归客户端独占：禁止同时设置 `enable_motor_tx=true`，
并禁用本地深度归零命令写入，避免两个控制来源交错。原始接收和 ROS 遥测解析仍继续。
上位机与 Nano 两端都必须允许发送；上位机单独勾选不会改变 Nano 配置。

允许远端写入时，实际控制/升级使用上位机原有操作流程。Nano 不重新解释这些原始请求为 ROS 控制请求，
也不把现有 ROS `control/enable` 门禁当作远端字节流的门禁。
不要同时运行 ROS 手动控制和上位机控制。

双向模式在串口打开、网络断开和正常退出时，会尽力发送明确的全节点停止命令。
网络故障检测和主机停机仍依赖进程及通信链路，不能替代 MCU 看门狗和独立急停。
重连不恢复旧命令，升级中断应重新发起，不能把 TCP 重连当作断点续传。

<a id="service"></a>
## 用户服务

[用户服务模板](../../deploy/underwater-tcp.service.example) 仅启动原始网关，默认只读，且关闭本地深度/电机写入。
先结束手动网关，再安装并启动：

```bash
mkdir -p ~/.config/systemd/user
cp deploy/underwater-tcp.service.example ~/.config/systemd/user/underwater-tcp.service
systemctl --user daemon-reload
systemctl --user start underwater-tcp.service
systemctl --user status underwater-tcp.service
```

模板假定项目在 `~/UnderWater-Robot-ROS2`。已有同名服务时先核对并保留原配置，不盲目覆盖。

```bash
systemctl --user stop underwater-tcp.service
systemctl --user restart underwater-tcp.service
journalctl --user -u underwater-tcp.service -n 50 --no-pager
```

`start` 只启动当前用户服务，不等于开机自启。需要随用户会话启动时再显式执行：

```bash
systemctl --user enable underwater-tcp.service
```

无登录会话的开机常驻还涉及用户 linger 配置，应根据实际部署策略设置。
服务默认域/命名空间中的网关可以被其他 ROS 入口发现，但该最小服务不自动启动设备解析和分配器。

<a id="recovery"></a>
## 重连与故障排查

- 单个服务只允许一个 TCP 客户端；第二个连接会被关闭，不抢占原客户端。
- 接收缓存上限 256 KiB，慢客户端会断开，避免无限占用内存或堵住串口处理。
- USB 丢失会断开网络会话，重新枚举后由上位机重连，避免不同串口会话混合。
- 上位机连接超时 3 秒，连续 5 秒未收到数据会重连，重试间隔从 1 秒增长到最多 5 秒。
- Nano 使用 TCP keepalive 和发送超时；断线检测不是即时物理断电保证。
- 两端断线都清空待发缓存；上位机同时清理半帧、时间戳展开器、配置等待和固件流传输状态。

| 现象 | 检查 |
|---|---|
| 连接被拒绝 | Nano 服务是否启动、地址和端口是否正确 |
| 连上立即断开 | 串口是否可用/稳定，是否已有另一客户端 |
| 心跳正常但没有传感器/电机数据 | 现场设备是否实际连接并上报；转发不能生成缺失测量 |
| 能接收但不能发送 | 两端发送开关是否都已开启，服务是否仍是只读 |
| 频繁重连 | 网络是否稳定、MCU 是否持续上报、客户端是否处理过慢 |
| ROS 控制启用失败 | 当前是否为远端独占模式；不要同时控制 |
| GUI 提示缺 Qt5Network.dll | 运行上位机的 `tools/deploy-tcp-runtime.ps1` |

Nano 检查监听、占用和数据状态：

```bash
ss -lntp | grep ':9000'
fuser -v /dev/ttyACM0
ros2 topic echo /underwater_robot/gateway/status --once
```

<a id="validation"></a>
## 验证范围

服务器单元测试：

```bash
ctest --test-dir build/underwater_mcu_gateway --output-on-failure
```

覆盖字节一致性、分片、单客户端、只读拒绝、断线、慢客户端和会话清理。
上位机测试包括 TCP transport/service、原协议回归和连接栏交互。
跨机器验证先使用 Nano 伪串口回环，再对实际 USB 网关做只读测试；均不需要真实传感器在线。
测试中收到网关心跳只证明网关链路可用，不是电机动作或传感器有效测量证明。
本轮结果、恢复点与部署状态见 [2026-10-08 验证记录](../experiments/2026-10-08-tcp-validation.md)。

[返回教程总目录](README.md) · 上一篇：[05 IMU 基准姿态校正](05-imu-calibration.md)

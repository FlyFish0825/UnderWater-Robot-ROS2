# 06 串口 TCP 转发与控制权服务

[教程总目录](README.md) · 上一篇：[IMU 基准姿态校正](05-imu-calibration.md) · [项目 README](../../README.md)

> **说明**：板载计算平台为 **NVIDIA Jetson Orin NX 16GB Super**。文中提及的“Nano”及网络协议标识（如 TCP 9002 控制权状态 `STATE NANO`）为板载端/机载端服务标识。

日常开机、GUI 连接与控制权切换见[串口与相机转发操作手册](09-forwarding-handbook.md)。本页为原始串口和 9002 控制权的详细说明。

功能分支新增的 `GUI_MANUAL` 不等于本篇原始模式 `GUI`：前者由 GUI 操作、Nano 执行，9000 只读，9003 接收业务输入；后者允许 GUI 独占原始写入并阻止 ROS 电机输出。第一页六自由度操控应阅读[第 10 篇](10-remote-manual-control.md)，不要套用原始调试模式。

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
MCU USB CDC ↔ Nano mcu_gateway ↔ TCP 9000 原始串口 ↔ 上位机 TcpTransport
                    ↕
             TCP 9002 控制权管理
                    ↕
          Nano ROS 服务 / 上位机设置
```

Nano 是服务器，上位机是客户端。只配置 Nano 固定地址和端口，不记录客户端电脑的 IP；
电脑 IP 变化后重新连接即可。Nano 目标地址为 `192.168.20.70`，示例端口 `9000`。
地址是否已经持久固定必须核对机器人的实际网络配置；本次连接检查与固定地址流程见
[固定局域网地址与两条转发恢复](08-lan-recovery.md)。
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
colcon build
source install/setup.bash
```

上述命令全量编译所有项目包；新机器按 [安装教程](01-installation.md) 准备依赖。

启用原始串口服务器、默认禁止 Nano ROS 电机发送的最小启动命令（GUI 初始只读，显式接管后可发送）：

```bash
ros2 run underwater_mcu_gateway mcu_gateway_node --ros-args \
  -r __ns:=/underwater_robot \
  -p tcp_port:=9000 \
  -p tcp_control_port:=9002 \
  -p tcp_bind_address:=0.0.0.0 \
  -p enable_motor_tx:=false
```

串口默认使用现有 MCU by-id 路径；需要时加 `-p port:=/dev/ttyACM0`。
串口不存在时服务会继续等待，连接恢复后上位机重连。没有传感器不影响转发网关心跳和 CAN 数据。

若还需要 ROS 设备解析节点，可改用下列启动入口，二者不要同时运行：

```bash
ros2 launch underwater_bringup tcp.launch.xml \
  tcp_port:=9000 tcp_control_port:=9002 serial_port:=/dev/ttyACM0
```

此 launch 沿用原网关配置，深度归零服务仍是独立、显式调用的能力；默认不允许电机发送。

| 参数 | 默认值与用途 |
|---|---|
| `tcp_port` | 网关节点默认 0，关闭网络；TCP launch 默认 9000 |
| `tcp_bind_address` | `0.0.0.0`；可以改成 Nano 的具体局域网 IPv4 地址 |
| `tcp_control_port` | 9002；控制权申请、接管、释放与存活确认端口；设为 0 可关闭 |
| `tcp_allow_tx` | 兼容旧命令的弃用参数，不授予串口控制权；应使用所有权管理操作 |

这是可信局域网内的明文字节服务，不包含公网身份认证或加密，不应通过路由器映射到公网。

<a id="client"></a>
## 上位机连接

使用已包含原始字节传输和控制权客户端的 Underwater_GUI：

1. 打开“设置”页的连接栏。
2. 将连接方式从 `USB CDC` 改为 `Nano TCP`。
3. 填入 Nano 地址 `192.168.20.70`，端口 `9000`。
4. 初次只看数据时保持“上位机控制”未勾选，点击“连接”。
5. 等待显示 TCP 已连接及下位机心跳。

地址、端口和连接方式会保存；控制权在 GUI 退出或网络断线时释放，下次连接默认只读。
首次启动或重新打开程序后点击连接即可；网络意外断开时当前连接请求会自动重试。
点击“断开”会取消自动重连。切回 USB 模式恢复原有 USB 扫描和连接行为。

连接到 Nano 时不再扫描并抢占本机 USB 设备。TCP 走局域网直连，不继承系统 HTTP 代理。
解析、Dashboard、Motor Debug、传感器和科研记录继续使用同一条现有服务链。

<a id="duplex"></a>
## 双向模式与串口归属

查看数据时只需连接 9000，Nano 保留控制权，GUI 只收不发。要在 GUI 中发送配置、
控制或升级命令，在“设置”中申请控制权。若 Nano 正持有控制权，GUI 会报告占用并
提示使用“接管控制权”。接管后，GUI 使用已认证的 9000 会话发送原始串口字节。
控制权占用包含 ROS 电机指令、深度归零、CAN 配置、固件升级及所有直接串口字节，
因此不会出现 Nano ROS 控制与 GUI 下行字节同时写串口的情况。

Nano 上使用 ROS 服务申请、接管和释放控制权：

```bash
ros2 service call /underwater_robot/gateway/control \
  underwater_interfaces/srv/ControlOwnership "{action: 0}"   # 申请（有人占用时失败）
ros2 service call /underwater_robot/gateway/control \
  underwater_interfaces/srv/ControlOwnership "{action: 1}"   # 明确接管
ros2 service call /underwater_robot/gateway/control \
  underwater_interfaces/srv/ControlOwnership "{action: 2}"   # Nano 释放后由任何一方重新申请
```

回复必须同时看 `success` 与 `owner`：owner 0=无人持有、1=Nano、2=GUI。普通申请被占用时 `success=false`，原所有者保持；只有 action 1 明确接管才转移控制权。GUI 在设置页点击“释放控制权”会归还 Nano；Nano action 2 则释放到无人持有。

Nano 的 ROS 电机发送仍由 `enable_motor_tx` 参数单独保护，默认关闭；只有在明确需要
Nano ROS 控制时，才在启动配置中设置 `motor_tx:=true`。该参数不会给远程 GUI 下发权限。
GUI 的串口控制权通过 9002 申请，9000 在握手期间校验本次会话令牌；握手头由 Nano
移除，之后的串口数据保持原始字节格式。GUI 取消选项后控制权返回 Nano；Nano 执行
action 2 时进入无人持有状态，后续可由任一方重新申请。

允许远端写入时，实际控制/升级使用上位机原有操作流程。Nano 不重新解释这些原始请求为 ROS 控制请求，
也不把现有 ROS `control/enable` 门禁当作远端字节流的门禁。
不要同时运行 ROS 手动控制和上位机控制。

任一侧显式接管时，Nano 会先写入全节点停止事务，再切换唯一所有者。GUI 控制会话
断开、管理心跳超时或控制权被 Nano 接管时，会撤销 GUI 令牌、关闭对应原始串口会话，
并将控制权返回 Nano、写入停止事务。停止事务属于尽力执行，不能替代 MCU 看门狗与急停。
GUI 必须重新获取控制权才会继续发送；重连不恢复旧命令，升级中断必须重新发起。

<a id="service"></a>
## 用户服务

当前机器人采用[随 ROS 2 同启两条转发](08-lan-recovery.md#随-ros-2-启动两条转发)，
不设置开机转发服务。以下只保留可选用户服务的历史操作；与 ROS 2 后台不可同时占用串口。

[用户服务模板](../../deploy/underwater-tcp.service.example) 仅启动原始网关，默认由 Nano 持有控制权且关闭 ROS 电机发送；GUI 初始只读，明确接管后可发送。深度命令另受控制权检查，不会自动校零。
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

- 原始串口端口一次只接纳一个客户端；第二个客户端被关闭，不抢占现有会话。
- Nano 默认拥有控制权；普通 `CLAIM` 不会抢占。只有明确的 `TAKEOVER` 动作会转移所有者。
- GUI 需持续向 9002 发送令牌心跳；心跳断开后最迟 3 秒撤销令牌并关闭写入会话。
- 接收缓存上限 256 KiB，慢客户端会断开，避免无限占用内存或堵住串口处理。
- USB 丢失会断开网络会话，重新枚举后由上位机重连，避免不同串口会话混合。
- 上位机连接超时 3 秒，连续 5 秒未收到数据会重连，重试间隔从 1 秒增长到最多 5 秒。
- Nano 使用 TCP keepalive 和发送超时；断线检测不是即时物理断电保证。
- 两端断线都清空待发缓存；上位机同时清理半帧、时间戳展开器、配置等待和固件流传输状态。

| 现象 | 检查 |
|---|---|
| SSH 22 能连，但 TCP 9000/9002 被拒绝 | SSH 只证明网络地址可达；检查 ROS 后台是否运行、网关是否成功启动并监听 9000/9002，见[分层诊断](08-lan-recovery.md#ssh-可达但-tcp-端口拒绝) |
| TCP 9001 被拒绝 | 检查机器人后台是否启动相机节点；9001 与串口网关独立 |
| 连上立即断开 | 串口是否可用/稳定，是否已有另一客户端 |
| 心跳正常但没有传感器/电机数据 | 现场设备是否实际连接并上报；转发不能生成缺失测量 |
| 能接收但不能发送 | 检查 `ros2 topic echo /underwater_robot/gateway/status --once` 的 `control_owner`；GUI 使用“接管控制权”，Nano 使用服务 action 1 |
| 频繁重连 | 网络是否稳定、MCU 是否持续上报、客户端是否处理过慢 |
| ROS 控制启用失败 | 当前是否为远端独占模式；不要同时控制 |
| GUI 提示缺 Qt5Network.dll | 运行上位机的 `tools/deploy-tcp-runtime.ps1` |

Nano 检查监听、进程、串口和数据状态：

```bash
ss -lntp | grep ':9000'
ss -lntp | grep ':9002'
pgrep -af 'mcu_gateway_node|global_cam_pub'
ls -l /dev/serial/by-id/ /dev/ttyACM* 2>/dev/null
fuser -v /dev/ttyACM0
ros2 topic echo /underwater_robot/gateway/status --once
```

<a id="validation"></a>
## 验证范围

服务器单元测试：

```bash
ctest --test-dir build/underwater_mcu_gateway --output-on-failure
```

覆盖字节一致性、分片、只读拒绝、会话令牌、独占所有权、占用拒绝、明确接管、Nano/GUI
释放、客户端断线和心跳超时。上位机测试覆盖控制界面、管理连接及原有协议解析。

2026-10-10 已用真实机器人与 GUI 正式通信类验收：普通 GUI 申请被 Nano 拒绝；GUI 明确接管后发送原始 20 字节深度 GET_STATUS 并收到 MCU 匹配回复；GUI 释放成功；Nano 普通申请在 GUI 占用时被拒绝，明确接管成功并撤销 GUI 写权限；Nano 释放到 NONE 后重新申请回 NANO。测试未发送电机运行或校零命令，详见[实机验收](../experiments/2026-10-10-sensor-fix-control-acceptance.md)。
上位机测试包括 TCP transport/service、原协议回归和连接栏交互。
跨机器验证先使用 Nano 伪串口回环，再对实际 USB 网关做只读测试；均不需要真实传感器在线。
测试中收到网关心跳只证明网关链路可用，不是电机动作或传感器有效测量证明。
本轮结果、恢复点与部署状态见 [2026-10-08 验证记录](../experiments/2026-10-08-tcp-validation.md)。
控制权仲裁、固定型号/保留 P0 的最新实现与测试范围见 [2026-10-10 控制权交接记录](../experiments/2026-10-10-control-ownership.md)。

[返回教程总目录](README.md) · 上一篇：[05 IMU 基准姿态校正](05-imu-calibration.md)

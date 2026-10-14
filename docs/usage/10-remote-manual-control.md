# 10 上位机业务级手动控制（TCP 9003）

[教程总目录](README.md) · [项目 README](../../README.md) · [验证与整理记录](../experiments/2026-10-12-remote-manual-verification.md)

适用版本：ROS2 与 GUI 的 `codex/dashboard-controller-input` 功能分支，TCP 9003 v2。
尚未合并的功能不能仅靠获取 `main` 获得；原 v1 客户端需同步升级。本文命令在 Nano/Linux 的 Bash 执行，Windows 不需要安装 ROS 来使用 GUI。

本接口新增于 `underwater_control`，显式启动，保留原终端键位、方向和分配数学。
2026-10-11 修正操作权与执行者混淆：9002 新增 GUI_MANUAL 业务模式，原 GUI 原始模式继续保留；
分配器和 MotorGate 增加权限/会话失效门禁。数据路径：

`GUI → TCP 9003 → remote_manual_node → control/manual_request → control_node → 网关 → MCU`

六轴输入为 **-100..100 的整数档位**，不是 N / N·m，也不是直接 RPM；每条请求最多一轴非零。
Fx/Fy/Fz/K/M/N 直接沿用现有 ROS2 手动方向，帧为 `thruster_center`，
由原分配器解释 `manual_axis_signs`，9003 不重复转换方向，不自行分配推进器。
GUI 键盘/手柄的来源切换、Y/A 调档、方向释放清零属于 GUI 输入逻辑；本节点只接受完整当前档位。

## 显式启动与控制权

2026-10-11 已在 Nano 的独立源码副本完成 Linux / ROS2 Humble 完整编译，
协议状态机、原推进器分配和启动配置的 CTest 均通过。实机运动尚未验收。
不要对已有占用串口的后台叠加启动另一份网关；以下启动方式只允许选择一种。

```bash
cd /home/lamost/UnderWater-Robot-ROS2
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch underwater_bringup remote_manual.launch.xml \
  motor_tx:=false tcp_allow_tx:=false \
  tcp_bind_address:=192.168.20.70 remote_bind_address:=192.168.20.70
```

上述地址是当前 Nano 的局域网地址，换机器时必须替换；不要改成公网监听来解决连接问题。
默认 `motor_tx=false`，因此能连 9003、看到状态，但启用会被拒绝。
具备现场独立急停、方向检查和安全条件后，启动者才可明确设置 `motor_tx:=true`。
远程 ENABLE 不修改这个参数，不请求 9002 GUI 接管，不自动抢占或释放控制权。
必须持有同一 GUI 会话的 `control_owner=GUI_MANUAL` 权限；若已接管 GUI 原始模式，
先停用旧路径并明确切换业务模式。相机、9000、9002 的原启动参数继续透传，
包括 `tcp_bind_address` 和 `tcp_allow_tx`；`tcp_allow_tx=false` 不是替代 9002 控制权管理的全局电机禁写开关。

该入口已经包含串口网关、设备解析、原分配器、相机和 9003 适配节点，
不需要额外启动 `tcp.launch.xml` 或终端键盘程序。9000 是串口数据，9001 是视频，
9002 仲裁业务操控和原始串口两种互斥权限，9003 是第一页六自由度的业务输入。
在上位机设置中连接 Nano 的 `192.168.20.70:9000`，第一页业务服务会连接同一地址的 9003。
在设置中选择“六自由度操控（经 Nano）”，点击“接管控制权”后再到第一页启用。
此时 GUI 持有操作权，Nano 继续计算与执行；业务模式的 9000 只读。
电机调试/升级需停用、等待确认、切到原始模式再明确接管。原始模式阻止 ROS 电机输出。

只检查连接时不要点击接管或启用、不要发送 INPUT 或调用启停服务。
保持电机发送关闭时，启用开关不能成功打开属于预期发送门禁，不代表 GUI 没有操作权。
实际操控必须同时满足：GUI 已取得业务操作权、启动时明确允许电机发送、串口/状态新鲜、启用握手完成且无其他输入源。

### 后台运行方式与日常查看

前面的 launch 是前台运行，正常退出使用 Ctrl+C。历史部署曾用临时用户服务
`rov-remote-manual.service`，它不是开机自启；只有已经部署该服务的机器才适用以下查看命令。
源码不负责自动安装或启动这个临时服务，机器重启后不能据此假定 9000～9003 正在监听。

```bash
systemctl --user status rov-remote-manual.service
journalctl --user -u rov-remote-manual.service --no-pager -n 30
```

SSH 退出不会自动停止已有用户服务。不要在服务运行时又执行 launch 命令；
需切回终端运行时，先正常停止已知服务：

```bash
systemctl --user stop rov-remote-manual.service
```

启动/正常停止仍可能写入原网关的保护性停止帧，即使 `motor_tx=false`。
这不是请求电机旋转，也不能证明物理停机。
历史部署与本轮离线提交检查分开记录在[验证记录](../experiments/2026-10-12-remote-manual-verification.md)。
那里描述的是有日期的核验快照，不是当前设备状态；本轮整理没有连接 Nano。

若已有完整 manual 后台，只启动业务适配节点，不再启动第二份后台：

```bash
ros2 run underwater_control remote_manual_node --ros-args \
  -r __ns:=/underwater_robot -p bind_address:=192.168.20.70 -p port:=9003
```

地址须换成本机实际 IPv4。`bind_address` / `port` 只读；launch 对应
`remote_bind_address` / `remote_manual_port`，默认 `0.0.0.0:9003`。
此协议使用同一 9002 会话私有令牌授权，但没有传输加密，不是公网身份认证。
仅用于可信隔离控制网；通过防火墙限制控制机，勿开放公网。调试时可绑定 `127.0.0.1`。
已授权控制连接不能被第二客户端替换；未授权连接可被替换，首次授权最多等待 3 秒，失败即断开。
授权续验失败/超时也断开并停用，旧/异会话令牌不能刷新授权。

监听或已连接但未 ENABLE 时不创建手动输入发布者，也不调用禁用服务，原终端工具可独立使用。
ENABLE 先拒绝现存请求发布者/已 armed 的分配器，再创建临时发布者，经过 1 s DDS 发现等待重新检查，
只有独占且下位机门禁满足才调用 `control/enable`。启用过程中和运行中仍检测其他发布者。
停用完成后释放发布者；原分配器的单来源检查仍是独立防线。

### 上位机操作顺序

1. 在设置页连接 Nano TCP 9000；确认管理服务 9002 和业务服务 9003 可达。`tcp_control_port:=9002` 是端口号，`:=0` 才是关闭，不是布尔开关。
2. 选择“六自由度操控（经 Nano）”，显式接管；应显示 `GUI_MANUAL` 或“GUI 持有权限，Nano 执行”。已经持有权限时申请/接管按钮变灰是正常的，释放按钮仍可用。
3. 现场具备运行条件且启动参数允许发送后，进入第一页，显式开启“启用控制”。授权成功不等于已启用；等下位机确认后才使用 GUI 已配置的键盘/手柄输入。
4. 停止操作先归零并关闭启用，等待停用确认；切换到电机调试前再明确切换原始模式和接管。原终端键盘程序应独立使用，不与 GUI 同时发布。
5. 断线、接管变化或后台重启后不续跑旧目标；重新连接、接管、显式启用。不能用重发旧 INPUT 或绕过界面来恢复。

| 现象 | 优先核查 |
|---|---|
| 管理按钮全部变灰 | Nano TCP/9002 是否连接；USB CDC 连接不是 Nano 网络控制服务 |
| 已接管但第一页启用失败 | 是否 GUI_MANUAL、motor_tx 是否允许、串口是否在线、是否存在另一输入发布者；查看拒绝原因 |
| 能看遥测但没有业务状态 | 普通 manual/tcp 后台没有 9003；需统一 remote_manual 入口或只补适配节点，勿再起网关 |
| 显示协议不匹配 | 两端是否同为 v2、安装目录是否已完整重编译并重新加载 |
| 切模式/重连后开关自动关闭 | 旧租约或会话失效，需要重新显式启用，不自动恢复运动 |

## 共同协议 v2（两端必须同步升级）

ASCII 命令行，以 LF 结束（也接收 CRLF）。每行最多 1024 字节正文，接收和输出缓冲各最多 4096 字节。
每 20 ms 轮询最多接收 4096 字节、32 条命令；达到字节预算或超出命令预算按洪泛断开并停用。
输出未排空达 500 ms 或排队越界同样停用。所有序号是严格递增的正 uint64 十进制整数，
每次 TCP 连接重新开始；不会通过浮点数解析会话号。

服务端连接后：

```text
HELLO ROV_MANUAL 2
```

客户端：

```text
AUTH seq leaseToken
ENABLE seq leaseToken
DISABLE seq
INPUT seq session Fx Fy Fz K M N
```

服务端：

```text
RESULT seq ok session detailHex
STATE counter session state armed inputFresh gatewayReady solverOk lastInputSeq Fx Fy Fz K M N rpm1 rpm2 rpm3 rpm4 rpm5 rpm6 rpm7 rpm8 detailHex
```

STATE 共 **24 个 token**（下标 0..23），counter 在每条 TCP 连接从 1 递增，通常 10 Hz；
state 为 DISABLED / ENABLING / ENABLED / STOPPING / FAULT。布尔是 0/1，
detailHex 为 UTF-8 原因的十六进制或 `-`，长度受限。lastInputSeq 表示最新被业务接口接受的输入序号，
六轴字段是接受的档位，RPM 是分配器建议值，**不是实际测得电机转速或执行确认**。

`leaseToken` 是 9002 返回的 32 个小写十六进制字符组成的私有令牌（表示 128 bit），不写日志或公开状态。
AUTH 只验证和保留连接，成功 RESULT 的 session 可为 0，不创建运动发布者或调用启用。
9003 经 `gateway/validate_manual_lease` 校验令牌及权限代次，每 100 ms 续验，
400 ms 未确认即失效；网关独立的 500 ms 授权过期会停止并退休旧电机会话。
9002 的 3 秒会话心跳独立有效，9003 续验不能替代它。模式切换/接管旋转令牌，旧授权失效。
v1 的无令牌 ENABLE 被拒绝，不能兼容绕过授权。

ENABLE 的成功 RESULT **只发一次且带真实非零 session**：服务成功后还须等新 allocation session armed，
与 gateway motor_session_id 一致、motor_rearm_required=false、零输出握手已完成。
等待期间只报 STATE ENABLING，不发送 `RESULT seq 1 0 ...`；失败/取消返回 RESULT ok=0。
首个 INPUT 尚未到达时 inputFresh 可以为 0，不以此阻塞 ENABLED，避免循环等待。
启用状态表示下位机软件门禁确认，不能作为电机已旋转/物理停机的证明。
成功 INPUT 不逐包回 RESULT；错误 INPUT 返回拒绝并停用；DISABLE 的 RESULT 表示停机已申请，
STATE STOPPING → DISABLED 才表示服务与下位机状态已确认解除启用。

GUI 应持续以新序号发送当前档位（建议 50 ms），档位不变也发送；重复序号不被视为心跳。
只有新 INPUT 被接收时才加本机 ROS 时间戳并发布，定时器不刷新旧缓存命令。
即使六轴为零，已 ENABLE 后也要持续 INPUT；500 ms 没有新 INPUT 请求停用，必须重新显式启用。

## 失败与恢复

- 非整数、越界、多轴、重复/倒序序号、旧 session、提前 INPUT：拒绝并清零停用。
- TCP 断开、超时/背压、授权/网关心跳过期、控制模式或会话代次变化、串口 connection_id 改变、求解失败或分配器 disarm：清零停用。
- allocation 状态超过 500 ms 或 gateway 超过 2.5 s 失效；同时检查状态本身的 ROS 时间戳，不能以排队旧消息保活。
- ENABLE 总等待最多 2.5 s；断线/取消之后到达的启用成功回调必须再发禁用。
  每次补偿停用有内部代号，旧禁用应答不能覆盖晚到的启用成功。
- 停机未确认/启用应答丢失时保持 STOPPING 和临时发布者，不允许重启旧目标或另一远程会话。
  同时依赖既有输入超时与 MotorGate；服务恢复后确认解除启用。永久丢失应答需现场确认安全后恢复后台，不能强行自动放行。
- 网络物理断开/进程强杀时无法保证停机命令送达，必须保留 MCU 通信看门狗、独立急停或断电手段。

## 离线验证

Windows 无 ROS 环境时只构建协议、行缓冲、背压缓冲与会话核心，不启动 ROS 节点或网络监听：

```powershell
cmake -S internal/underwater_control -B build/test/remote-manual-core -G Ninja -DREMOTE_MANUAL_CORE_ONLY=ON
cmake --build build/test/remote-manual-core
ctest --test-dir build/test/remote-manual-core --output-on-failure
```

需已激活可用的 C++17 / Ninja 工具链；本轮使用现有锁定 MinGW 8.1、CMake 3.28.6。
Linux 完整构建后的同名测试也纳入 `ctest --test-dir build/underwater_control`。
`remote_tcp_contract` 仅监听 127.0.0.1，验证未授权连接时限/替换与已授权连接保护。
`test/remote_manual_ros_test.py <remote_manual_node路径>` 用独立 localhost DDS 域及模拟状态/服务
测试真正适配节点的授权、禁止发送门禁、零帧确认和权限代次变化；不启动串口网关、不接触硬件。
本地通过的纯 C++ 测试不代表 POSIX 网络、ROS 服务/DDS集成、Jetson 完整构建或实际电机验证已完成。

Linux 无硬件回归还包括 `ctest --test-dir build/underwater_bringup --output-on-failure`。
其中 `remote_manual_launch_contract` 只解析 XML，检查两级启动入口保留禁止发送的默认值
并透传网络参数；不会启动 ROS 节点、打开串口或调用控制服务。

无 ROS 时也可用 Python 直接运行 XML 检查：

```text
python -B internal/underwater_bringup/test/remote_manual_launch_test.py
```

`remote_manual_ros_test.py` 必须在具备 ROS/DDS 的 Linux 环境运行，不可把 Windows 上的语法检查当成该集成测试通过。

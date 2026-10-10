# 2026-10-10 Nano 串口启动修复验证

[使用教程索引](../usage/README.md) · [从开机启动](../usage/00-nano-first-start.md)

## 现场故障

通过 SSH 检查 `lamost@192.168.20.70`。ROS 工作空间为 `/home/lamost/UnderWater-Robot-ROS2`，正在运行的入口为 `manual.launch.xml motor_tx:=true`。

- MCU 正常枚举为 `0483:5740`，`/dev/ttyACM0` 与项目默认 by-id 路径均存在。
- `lamost` 已属于 `dialout`，设备权限为 `root:dialout 0660`。
- 网关状态为 `connected: false`、`rx_bytes: 0`、`tx_bytes: 0`，错误为 `serial TX deadline exceeded`，没有持有串口的文件描述符。
- 暂停旧后台后，单独以 raw 模式打开串口，两个 3 秒接收窗口分别收到 146,073 和 137,791 字节。串口打开与接收正常，故障发生在启动停止帧发送阶段。
- 当时串口保留了终端默认输入转换和回显设置。原 `SerialIo::close()` 恢复打开前的 termios，而 MCU 退出后仍会持续上报；恢复 ECHO/ICANON 会让二进制数据进入终端处理及自动回写路径。

原始故障最早如何形成 USB 写入阻塞没有完整抓包，因此不将所有 USB 写超时都归因于回显。本次确认并修复的是关闭后恢复终端设置，以及继承软件流控这两个可复现问题；现场链路恢复后完成了真实重复启动检查。

## 修复范围

1. `SerialIo::close()` 释放独占和文件描述符，保留 raw/no-echo，不再恢复 ECHO/ICANON。
2. 打开配置显式清除 IXOFF/IXANY；配合 `cfmakeraw()` 清除 IXON，避免二进制数据受软件流控影响。
3. 启动日志和状态区分 `open/configure` 与 `startup stop TX`，失败日志每 5 秒最多输出一次。
4. 新增 C++ 伪终端回归测试，验证二进制接收、默认只读 TX 门禁、关闭无回显、连续重开及软件流控清除。

保留 by-id 路径、单一串口所有者和现有启动参数。未修改 USB 固件、串口访问权限或写超时阈值。启动时仍按原逻辑发送停止帧。

Nano 修改前四个文件备份在 `artifacts/serial-startup-20261010/backup/`。部署时对节点和 CMake 做局部替换，保留设备上其他源代码。

## 验证结果

- 原实现伪终端回归失败：`software flow control still enabled`；单独检查关闭行为也失败：`close restored terminal echo/canonical mode`。
- Nano 上单独编译 `underwater_mcu_gateway` 成功，6 项 CTest 全部通过。
- `robot.launch.xml` 启动：收到 192,396 字节、5,227 有效帧，CRC 错误 0；退出后网关消失、串口释放。
- `manual.launch.xml motor_tx:=false` 启动：收到 192,342 字节、5,226 有效帧，CRC 错误 0；退出后串口释放。
- `manual.launch.xml motor_tx:=true` 启动：收到 193,062 字节、5,255 有效帧，CRC 错误 0。控制分配器按现有逻辑周期发送停止设点；采样结果 `armed: false`、`run_mask: 0`、八个 RPM 均为 0，网关 `motor_command_active: false`。退出正常。
- 最终普通后台持续接收 2,713,971 字节、73,002 有效帧，CRC/结构错误均为 0，`connected: true`、`tx_enabled: false`。正常后台的 TX 统计为 76 字节、2 帧启动停止事务。
- Windows 端 TCP 9000 并发接收 10 秒：456,200 字节，AA55 8,031 帧、AA5B 4,221 帧、AA58 10 帧，CRC 错误 0；断开后重连接收通过。
- 同时检查 TCP 9001：54 张有效 2560×720 JPEG，重连通过；实收约 5.37 FPS，此检查只证明相机转发未受修复影响。

LAN 客户端报告 `commands_sent: 0`。本次只验证串口连接、停止事务、真实遥测、重复启停和客户端重连，没有验证推进器转动，也没有执行整机重启或 USB 拔插测试。

## 最终状态与使用

验证结束后已关闭测试后台，串口无人占用，9000/9001/9002 无监听。串口保留 115200 8N1、raw/no-echo、无软硬件流控。

在 Nano 终端加载环境后使用原入口：

```bash
cd /home/lamost/UnderWater-Robot-ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch underwater_bringup robot.launch.xml
```

手动后台使用 `ros2 launch underwater_bringup manual.launch.xml motor_tx:=true`。两个后台二选一；先在已有后台的终端按 Ctrl+C，再切换入口。

部署与验证日志在 Nano 的 `artifacts/serial-startup-20261010/`。Windows 同名目录保存接收报告和下载的关键状态/测试日志。本次串口修复、回归测试及相关文档已在 Nano 本地提交，未推送到 GitHub。

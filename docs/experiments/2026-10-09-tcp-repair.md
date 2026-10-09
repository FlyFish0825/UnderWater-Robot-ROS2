# 2026-10-09 两条 TCP 转发与固定地址验证

[部署与验收教程](../usage/08-lan-recovery.md) · [此前串口验证](2026-10-08-tcp-validation.md)

## 最终行为

按用户要求，启动机器人 ROS 2 后台时开启全部转发，停止后台时关闭；开机不启动转发。
原始串口为 `192.168.20.70:9000`，双目 JPEG 为 `192.168.20.70:9001`。

## 现场故障与修复

机器人上线后，串口转发未运行，相机节点在等待缺失的 USB 相机。
MCU 显示 `0483:df11` DFU 模式，没有 `/dev/ttyACM0`。
用户接入相机并复位后，系统识别 `1bcf:2d4f` Cherry Dual Camera 和
`0483:5740` Virtual COM Port，正常串口出现。没有刷写 MCU 或发送电机命令。

原 base launch 默认 tcp_port=0，且没有相机 launch，常用 ROS 2 后台并不提供两条转发。
已改为串口 TCP 9000、相机 TCP 9001；robot、manual、monitor、tcp 入口统一提供相机与端口参数。
兼容传感器入口自动准备的 manual 后台获得同样行为。
隔离测试显式设置 tcp_port=0、camera_enabled=false，避免默认启动真实相机。
相机系统服务和旧用户串口服务已停止并禁用，本轮临时串口系统服务已移除。

## 固定地址与真实重启

| 项目 | 核对结果 |
|---|---|
| 接口/MAC | enP8p1s0 / 74:25:54:90:98:84 |
| 活跃连接 UUID | 79fb967d-b700-4e84-8bfb-b6b4b5c97f1f |
| IPv4 方法/地址 | manual / 192.168.20.70/24 |
| 自动连接/优先级 | yes / 100 |
| 网关/DNS | 192.168.20.1 |
| 原配置 | 保留，关闭 autoconnect |

原地址已是 manual，但优先级 -999，未绑定 MAC、未配置网关与 DNS；此次加固了持久配置。
固定地址脚本已在机器人执行并通过新 SSH 连接确认。
启动标识从 `5af07c11-3bc0-4d66-be24-b5cec3f6198f` 变为
`52b5400c-8fc4-4100-ab92-87086a090f14`，证明整机确实重新启动。
重启后用同一 .70 地址建立全新 SSH 连接，活跃连接仍为上述静态配置。
启动 ROS 2 前没有 9000/9001 监听；启动后两个端口同时出现。
没有访问路由器后台，不宣称 DHCP 地址池排除或保留已设置。

## 实际数据与生命周期测试

| 验证 | 结果 |
|---|---|
| 30 秒真实串口 | 888,104 字节；AA55 24,161 帧、AA5B 3,041 帧、AA58 30 帧；CRC 错误 0 |
| 同时真实相机 | 172 张 JPEG 全部解码；2560×720，左右各 1280×720；序号和源时间递增 |
| 双端口重连 | 两条均通过 |
| 停止/重新启动 robot launch | 两端口同时关闭/重新出现，真实数据验收通过 |
| sensors_cpp/sensors_sub 常用入口 | 自动启动两条转发；6 秒收到 176,242 字节、6 心跳、35 张有效 JPEG；重连通过 |
| 最后兼容入口退出 | 自动后台结束，两个端口关闭 |
| 网关实际状态 | connected=true、tx_enabled=false、tx_bytes=0、tx_frames=0，CRC/结构错误 0 |

报告在忽略目录 artifacts/tcp-repair/：lan-both-real.json、lan-tcp-metrics.json、
lan-send-autotune.json、lan-compat-entry.json。初始失败和中间检查记录保留。

## 性能限制

机器人相机节点输出约 29.96–29.98 FPS，当前 Windows 实际接收约 **5.7–5.8 FPS**。
平均 JPEG 约 198–200 KB；单帧解码约 13 ms，读取约 161 ms。
网络往返约 52 ms，TCP delivery_rate 约 8.1 Mbit/s；双方网卡协商为 1 Gbit/s。
协商速率不代表实测有效吞吐。

相机连接关闭 Nagle，移除固定 64 KiB 发送缓存限制；保留操作系统 TCP 缓存调节，
用户空间仍保留当前/最新帧，部分帧超过 2 秒会断开。
这些调整及客户端接收缓存增至 1 MiB 的对照没有将现场接收提高到 30 FPS。
不宣称上位机接收达到 30 FPS，也不把往返延迟断言为唯一原因。
功能转发、解码、并发及重连通过，较低接收帧率是本次明确保留的性能限制。

## 软件检查与恢复点

- 机器人 underwater_bringup、global_camera 构建通过。
- 机器人网关 4 个 CTest、相机 2 个 CTest 通过。
- Windows C++ protocol_test 的 1,292 项检查通过。
- 接收客户端 6 组测试通过：分片、CRC 错误、错误帧头、JPEG 错误、并发与重连。
- 固定地址模拟测试通过；现场静态配置激活/确认通过。未故意断开 SSH 来实测回退定时器。
- 修改后的隔离 synthetic 集成通过，只启动 devices_node，没有相机或网关进程。

源码备份：/home/lamost/tcp-repair-before-20261009/launch-source.tar.gz，
SHA-256 为 5c5cb3cf6cad5ead06108ba6b1f01984d02ecceb02aada93617f75bcc5f82f1e。
服务备份：/var/backups/underwater-tcp-20261009-191915/。
保留本轮开始时已有的相机源码、文档移动及 ZIP 等未提交工作。

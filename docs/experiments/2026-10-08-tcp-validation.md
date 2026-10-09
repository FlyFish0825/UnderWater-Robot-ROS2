# 2026-10-08 Nano TCP 与 Windows 上位机验证

[使用教程](../usage/06-nano-tcp.md) · [项目索引](../../README.md#文档索引)

## 范围与恢复点

Nano 固定地址提供原始串口 TCP 服务，Windows 上位机主动连接。同一局域网，不要求上位机固定 IP。
服务器复用原网关串口所有权；上位机复用原协议、数据和 UI 链路。

修改 GUI 前，先提交原有 34 个文件的工作，恢复点为：
`7c79bba50249c4361feb59cb5a1dcaadd5735f75`，中文说明“存档：保存 TCP 接入前的传感器、协议与固件页工作”。
随后检查 `git status --porcelain` 为空，再创建 `codex/nano-tcp-transport` 开发。
ROS 项目此前 IMU 工作保存为 `66c60d9`，TCP 改动位于 `codex/nano-tcp-stream`。

## 测试结果

| 检查 | 结果 |
|---|---|
| Nano 网关与启动包构建 | 隔离目录和生产目录均构建通过 |
| Nano 协议/电机命令/深度归零/TCP CTest | 4 个通过 |
| TCP 模块细项 | 26 项通过，含客户端源地址从 127.0.0.1 改为 127.0.0.2 后重连 |
| Windows TCP transport/service | 25 项检查通过 |
| Windows 相关 CTest | 先完成 8 个回归，随后新增电机断线会话测试；9 类测试均通过，最后针对 4 类重新回归 |
| Windows 正式主程序 | 锁定 Qt 5.15.19 / MinGW 8.1.0 构建通过，补充部署 Qt5Network.dll |
| LAN 双向伪串口 | Windows → Nano TCP → 伪串口 → 原 TCP → Windows，LINKTEST 报文返回一致 |
| LAN 合成流 | 12 秒收到 1522 字节、60 个心跳和 1 个回环 CAN 帧 |
| 实际 USB 网关只读 | 12 秒收到 1260 字节、12 个有效网关心跳；未收到 CAN 电机反馈帧 |
| 实际 GUI 连接栏 | 连到 Nano，收到 2 个真实心跳，检查连接/断开、只读和 1000 宽度布局 |
| 正式服务复核 | 从生产 install 路径运行，串口单一所有者，监听 9000，正式服务 GUI 验证通过 |

局域网联调发现系统 HTTP 代理影响非 loopback 连接，修正为每个 TCP socket 使用 NoProxy 后验证通过；
没有修改 Windows 全局代理。截图使用 Windows Qt 平台验证字体渲染；offscreen 测试仅证明控件交互与布局约束。
最终审查还补齐了断线后的电机运行锁存/延迟速度定时器清理，以及单节点下载、读取和数据窗口取消。
新增 `rov_motor_control_session_test` 验证断线清理不发送遗留速度请求，必须重新进行用户操作。

## 部署状态

本次安装并启动 `~/.config/systemd/user/underwater-tcp.service`，`ActiveState=active`、`SubState=running`。
未执行 enable，`UnitFileState=disabled`，不声称已经配置开机自启。
默认只读，`tcp_allow_tx=false`、`enable_motor_tx=false`、`enable_depth_commands=false`。
真实网关验证状态中串口 `tx_bytes=0`、`tx_frames=0`。

生产源码变更前核对了相关原文件与基线一致，原文件副本保存在
`/tmp/underwater-tcp-validation-20261008/production-before/`。
只部署 TCP 模块相关的网关、启动文件和服务模板，没有顺带替换尚未部署的 IMU 校正实现。

## 证据和限制

ROS 本地日志副本位于忽略目录 `artifacts/tcp-bridge/`；GUI 测试、局域网探针和截图位于
`build-test/nano-tcp/`，未写入主程序 `build/` 的测试目录。

真实设备只进行接收验证，没有发送电机运行、传感器配置或固件升级命令。
双向收发在伪串口上验证，不能由此宣称实际电机控制、固件下载或传感器测量已验证。
TCP、串口和主机停止帧不替代设备端看门狗及独立急停。

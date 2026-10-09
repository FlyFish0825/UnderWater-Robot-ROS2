# 2026-10-10 TCP 控制权交接与传感器保留策略

[Nano TCP 操作教程](../usage/06-nano-tcp.md)

## 控制权规则

MCU UART 仍由唯一的 mcu_gateway_node 进程打开。Nano 默认持有控制权；其 ROS 电机
setpoint 和深度归零请求仅在 Nano 是当前所有者时可写。enable_motor_tx 仍是 Nano
ROS 电机命令的独立启动保护，默认关闭。

TCP 9002 管理 CLAIM GUI、TAKEOVER GUI、RELEASE GUI 和带会话令牌的 PING。
普通申请遇到 Nano 或另一 GUI 占用会返回 BUSY，不会抢占；只有显式 TAKEOVER
会切换所有者。GUI 控制连接断开或 3 秒内没有心跳时，令牌失效，控制权回到 Nano。
Nano 通过 /underwater_robot/gateway/control 服务执行 CLAIM/TAKEOVER/RELEASE。

9000 仍输出裸串口字节。上位机取得令牌后连接 9000 并先发送一次 OWNER <token>；
网关验证并移除这一握手行，再把之后的数据逐字节转给 MCU UART。未持有有效令牌时，
9000 仍允许只读接收，但不接受串口写入。所有权变化、控制客户端断开或心跳超时都会
关闭旧写入会话，切换时尽力发送全节点停止事务并清除 Nano 的命令重使能状态。

## 深度计与 IMU

上位机只读遥测可用 MCU 周期 AA5B STATUS 的设备采样年龄建立时钟基准；不再要求先
发送 GET_STATUS，因此 9000 只读连接也能显示 IMU 原始值、姿态和深度计 D1/D2。

深度计物理型号固定为 MS5837-02BA。上位机没有型号选择/确认/写入；连接和 TCP 重连
不写型号、不发送 ZERO_DEPTH、不覆盖已有 P0。旧的启动型号配置与自动水面归零策略已
移除。读取配置是只读操作，原“采集水面零点”按钮是唯一的校准动作。若 MCU 本身报告
pressure/depth 无效，上位机显示 -- 并继续显示有效的 D1/D2，不造一个新基准。

## 自动化验证

2026-10-10 在隔离临时构建目录完成的结果：

- Nano underwater_interfaces 与 underwater_mcu_gateway 构建通过。
- 网关 CTest 5/5 通过：协议 1,292 项、电机事务 25 项、深度零点 47 项、TCP 原始桥 43 项、控制权仲裁 30 项。
- 上位机 Debug 构建通过，完整 CTest 21/21 通过；其中只读传感器时钟锚定与深度基准重连测试通过。
- 所有控制发送测试使用 localhost 合成服务；未将新版本部署到机器人，未发送实际电机控制帧，也未清除或重写当前设备的 P0。

机器人的现有 ROS 2 后台仍是唯一串口所有者。启用新 TCP 9002 管理服务需要重建并重启该后台；
重启会断开当前 GUI 会话，因此实机控制权接管和真实串口数据联调尚未完成。

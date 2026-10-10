# 03 手动控制

[教程总目录](README.md) · 上一篇：[传感器与数据记录](02-sensors-recording.md) · 下一篇：[维护与故障排查](04-maintenance.md)

## 本页索引

- [控制含义与准备](#preparation)
- [第一次操作先做隔离预览](#preview)
- [按键、档位与保持行为](#keyboard)
- [实际键盘控制](#live-control)
- [停机与恢复](#stop)
- [手动管理后台](#backend)
- [建议目标与实际反馈](#feedback)
- [实验激励与物理力接口](#excitation)

<a id="preparation"></a>
## 控制含义与准备

每个终端先加载环境：

```bash
cd /home/lamost/UnderWater-Robot-ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

键盘发送六轴手动档位，经分配器得到八台推进器目标转速。力矩以八推进器中心 `thruster_center` 为基准，
手动符号按 Fx、Fy、Fz、Mx、My、Mz 的 `[-1,1,1,-1,1,-1]` 和实际布局解释，
不直接等同于现场观察到的“前进/上浮”。方向、接线和反馈应按真实硬件逐项验证。

真实控制会下发电机命令。主机停机依赖 USB/CAN 通信链路，不能替代独立急停/断电措施。
实际带动力测试需具备独立停机手段，并按电机与推进器的允许工作条件进行。
发布前的合成和伪串口测试不代表实机试转、推力曲线或转动方向已经验证。

<a id="preview"></a>
## 第一次操作先做隔离预览

选择未被其他程序占用的 ROS 域，示例使用 186。确认下面的临时路径不存在：

```bash
test ! -e /tmp/underwater-preview-no-such-port && echo "预览路径不存在，可以继续"
```

仅在检查成功后继续。程序将使用不存在的串口，并自动创建监测后台：

```bash
ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1 \
ros2 run manual_control manual_control \
  --ros-args \
  -p preview:=true \
  -p serial_port:=/tmp/underwater-preview-no-such-port
```

按一次 `W`，观察 `Manual levels` 和 `Proposed RPM`。没有真实网关连接时 `gateway_ready=false` 是预期现象，
仍可观察模型分配结果。用 `Q` 或 `Ctrl+C` 退出。

另一个已加载环境的终端使用相同域查看完整结果：

```bash
ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1 \
ros2 topic echo /underwater_robot/control/allocation
```

`preview:=true` 只表示入口不主动调用启用服务；它仍会发布请求，不是全局禁止电机发送的开关。
因此预览要使用上述隔离环境，或明确确认已有后台 `motor_tx=false`。不要接到已经启用的真实控制后台上。

<a id="keyboard"></a>
## 按键、档位与保持行为

| 按键 | 操作 |
|---|---|
| `W` / `S` | `Fx` 增加 / 减少 1 档 |
| `T` / `F` | `Fy` 增加 / 减少 1 档 |
| `I` / `J` | `Fz` 增加 / 减少 1 档 |
| `Y` / `G` | 横滚 `K` 增加 / 减少 1 档 |
| `U` / `H` | 俯仰 `M` 增加 / 减少 1 档 |
| `A` / `D` | 偏航 `N` 增加 / 减少 1 档 |
| `Q` | 请求停机并退出 |
| `Ctrl+C` | 中断，执行退出停机流程 |

按键大小写均可，不需要回车。每个轴在 `-100～100` 之间，档位不是 N 或 N·m。
这是累加保持控制，松开按键不会自动归零：

```text
W → Fx=1
W → Fx=2
S → Fx=1
S → Fx=0
```

长按可能通过系统键盘重复连续加档，初次操作应单次点按。
组合输入时其他轴仍会保持原值；只把一个轴减到零不会清除其他轴。
数字 `0～7` 和半自动 P/O 等模式尚未实现，不会发送猜测的 MCU 模式字节。

默认手动映射：

- 六轴全零时目标全零。
- 默认布局六轴单独 ±1 都让四个必要通道以 ±1200 RPM 起步。
- 最大档位对应最强通道约 6000 RPM。
- 必要通道保持同一比例并越过最低转速；不会把各路分别强制抬速。组合不能保持比例时返回失败并停机。
- 单独 `Fx=1` 为 `[1200,-1200,1200,-1200,0,0,0,0] RPM`。
- 单独 `Fz=1` 为 `[0,0,0,0,1200,1200,-1200,-1200] RPM`，四台竖直推进器均运行。
- 12 个单轴正负一档的模型与伪串口结果见 [中心基准验证](../experiments/2026-10-09-thruster-center-startup.md)。

以上是建议目标，不能据此认定实测转速或实际推力。

<a id="live-control"></a>
## 实际键盘控制

结束隔离预览，在真实机器人使用的 ROS 域中操作。检查 `ROS_DOMAIN_ID`，不要把测试域误当实际域。
满足现场条件后运行：

```bash
ros2 run manual_control manual_control
```

需要在新建后台时指定串口：

```bash
ros2 run manual_control manual_control \
  --ros-args -p serial_port:=/dev/ttyACM0
```

流程如下：

1. 入口准备或复用后台，自动控制后台请求允许电机发送。
2. 分配器初始未启用；网关建立连接并准备停止握手。
3. 首次方向键触发启用申请。
4. 启用成功后持续发布当前档位。

网关需要已连接、稳定等待结束、心跳新鲜，并允许电机发送。失败时入口打印原因，清零请求；
排除问题后再按方向键申请新的控制会话。
自动监测后台转为控制后台时可能发生短暂重启，传感器可能短暂无效后恢复。
外部手动启动的监测后台不会被自动切换，见 [手动管理后台](#backend)。

首次实机操作可以单轴、单档开始：点按 `W` 一次并观察有效反馈，点按 `S` 一次回零，然后 `Q` 退出。
具体方向须由实机验证，不应根据按键名称直接假定。

键盘入口需要交互式终端。SSH 可先登录：

```bash
ssh -t lamost@192.168.20.70
```

此地址是当前部署示例，其他网络使用实际地址。登录后再加载环境并运行键盘程序。
不要将键盘入口放进没有终端输入的后台作业。

<a id="stop"></a>
## 停机与恢复

正常结束按 `Q` 或 `Ctrl+C`。入口发送零请求，实际控制模式尝试调用禁用服务，然后退出。
另一个终端也可主动禁用：

```bash
ros2 service call /underwater_robot/control/enable \
  std_srvs/srv/SetBool "{data: false}"
```

这是经通信链路发送的停机请求。若链路失效或无法确认停机，应使用现场独立急停/断电措施。
松开按键、关闭 SSH 窗口或直接强杀进程都不应作为正常停机步骤。

正常流程中，输入超时、串口重连、网关失效、多个请求来源等会取消启用。
排除故障后需要新的方向键操作申请会话，不应期待旧请求自动恢复输出。
只能运行一个请求来源，不能同时运行键盘入口、`forces_pub` 和 `wrench_command`。

<a id="backend"></a>
## 手动管理后台

自动入口适合日常使用；需要固定后台参数时可自己管理启动和关闭。
开始前正常结束原有入口/后台并确认没有另一份网关占用串口，不要直接叠加实例。

终端 A 启动监测分配后台：

```bash
ros2 launch underwater_bringup manual.launch.xml \
  motor_tx:=false \
  serial_port:=/dev/ttyACM0
```

终端 B 接入它预览：

```bash
ros2 run manual_control manual_control \
  --ros-args -p auto_start:=false -p preview:=true
```

转为真实控制前，结束预览和监测后台。满足现场条件后，终端 A 启动：

```bash
ros2 launch underwater_bringup manual.launch.xml \
  motor_tx:=true \
  serial_port:=/dev/ttyACM0
```

终端 B：

```bash
ros2 run manual_control manual_control --ros-args -p auto_start:=false
```

`motor_tx=true` 是网关允许发送，控制器的显式启用是另一个门禁。键盘入口负责在首个方向键申请启用。
`auto_start=false` 不会替你准备缺失的后台；外部 `motor_tx=false` 实例必须由启动者正常结束并以所需模式重启。

只需传感器后台时可使用 `underwater_bringup` 的 `robot.launch.xml`，但它不启动分配器。
`underwater_robot` 包自己的 `robot.launch.xml` 仅为软件心跳入口，两者不要混淆。

<a id="feedback"></a>
## 建议目标与实际反馈

保持后台运行，另一个终端查看分配状态：

```bash
ros2 topic echo /underwater_robot/control/allocation
```

重点字段为 `armed`、`input_fresh`、`gateway_ready`、`solver_ok`、`proposed_motor_rpm` 和 `detail`。

查看设备反馈：

```bash
ros2 topic echo /underwater_robot/actuators/thrusters/state \
  --qos-reliability best_effort
```

| 字段 | 含义 |
|---|---|
| `node_id` | 电机节点编号 |
| `online` | 在线状态 |
| `feedback_valid` | 数值反馈是否有效 |
| `heartbeat_valid` | 心跳有效，与数值反馈分别判断 |
| `speed_rpm` | 反馈转速 |
| `estimated_bus_current_a` | 固件估算母线电流 |
| `bus_voltage_v` | 母线电压 |
| `mcu_temperature_c` | MCU 内部温度 |

建议转速有值仅证明求解产生结果。网关 `tx_frames` 增长仅证明完整写入串口；
实际运行仍要结合有效反馈和现场测量。抓包分析见 [CAN 指令分析](../architecture/deployment/can-command-analysis.md)。

<a id="excitation"></a>
## 实验激励与物理力接口

`forces_pub` 使用原物理力模型输入，和键盘档位含义不同。先在独立预览域、确认临时串口路径不存在时预览：

```bash
ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1 \
ros2 run manual_control forces_pub \
  --ros-args \
  -p preview:=true \
  -p serial_port:=/tmp/underwater-preview-no-such-port \
  -p duration_s:=10.0
```

运行前先结束同域内的键盘请求来源。该入口按内置时间序列改变 `Fx`；默认持续 100 秒，
`duration_s=0` 对这个入口也表示使用默认 100 秒，然后请求停机。
不带 `preview=true` 时会主动申请启用，不等待按键，因此不能当作普通传感器查看命令。

物理力接口保留临时线性推力/RPM 模型。小幅请求可能全部落入转速死区，
请求存在但建议 RPM 为零不一定是通信故障。
`/forces`、`requested_wrench`、`modeled_rpm_wrench` 都不能作为实测推力。

原生 `wrench_command` 使用 `Fx Fy Fz Mx My Mz` 输入；其预览、启用和使用边界见
[动力分配说明](../architecture/deployment/thrust-allocation.md)。不要绕过分配节点直接发布内部电机设定。

[返回教程总目录](README.md) · 上一篇：[02 传感器与数据记录](02-sensors-recording.md) · 下一篇：[04 维护与故障排查](04-maintenance.md)

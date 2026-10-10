# Nano 从开机到运行 ROS 2

[项目 README](../../README.md) · [教程总目录](README.md) · 下一篇：[安装与编译](01-installation.md)

本文假设 Jetson Nano 已安装 Ubuntu 22.04、ROS 2 Humble，项目已部署并至少编译过一次，工作空间位于 `/home/lamost/UnderWater-Robot-ROS2`。按“只查看传感器”或“手动控制”选择一种启动方式。两种方式都启动串口网关，不能同时启动，以免争用 MCU 串口。

## 1. 打开终端并加载工作空间

Nano 开机进入 Ubuntu 后，打开终端。若从另一台电脑操作，可先 SSH 登录：

```bash
ssh -t lamost@192.168.20.70
```

在 Nano 终端中进入项目并加载 ROS 2 与工作空间环境：

```bash
cd /home/lamost/UnderWater-Robot-ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

每打开一个新终端，都要重新执行上面三条命令。若 `install/setup.bash` 不存在，先按[安装与编译](01-installation.md)完成依赖和编译。

确认 MCU 串口已接好并被系统识别：

```bash
ls -l /dev/serial/by-id/
```

项目默认使用带设备标识的 `/dev/serial/by-id/usb-STMicroelectronics_Lamost_UnderWater_Robot_ComPort_3588346E3134-if00`。若实际列出的名称不同，需要将启动命令中的 `serial_port` 改成实际路径。启动前可以用下面的命令查看是否已有程序占用串口：

```bash
fuser -v /dev/ttyACM0
```

## 2. 只查看 IMU 和深度计

在终端 A 启动传感器后台，并保持终端运行：

```bash
ros2 launch underwater_bringup robot.launch.xml
```

在终端 B 加载工作空间环境后，启动汇总查看：

```bash
ros2 run sensors_cpp sensors_sub
```

这个入口持续显示 IMU 和深度计数据。也可分别查看并发布兼容话题：

```bash
ros2 run imu_py imu_publisher
```

```bash
ros2 run depth_py depth_publisher
```

分别查看时，IMU 和深度入口各运行一份即可；`depth_publisher` 与 `sensors_cpp depth` 二选一。内部原始状态话题可在另一个终端查看：

```bash
ros2 topic echo /underwater_robot/sensors/imu/state --qos-reliability best_effort
```

```bash
ros2 topic echo /underwater_robot/sensors/depth/state --qos-reliability best_effort
```

深度状态应关注 `pressure_valid`、`temperature_valid`、`depth_valid`、`zero_valid` 和 `filtered_depth_m`。`d1`、`d2` 是传感器原始 ADC 读数，不是深度值。系统使用固定的 MS5837-02BA，不需要型号确认；固件上电采用固定空气参考 `P0=100965 Pa`，只按设备有效位读取压力和水深。只有现场确需修正时，才在空气/水面显式调用：

```bash
ros2 service call /underwater_robot/sensors/depth/zero std_srvs/srv/Trigger "{}"
```

等待服务返回 `success: true`，再查看深度状态。现场修正保存在 MCU RAM，MCU 复位后恢复固定空气参考；机器人启动或重连不会自动采集压力。

## 3. 手动控制推进器

手动控制后台本身会启动 MCU 网关、设备解析和动力分配器。若终端 A 正在运行 `robot.launch.xml`，先在启动它的终端按 `Ctrl+C` 正常退出；不要在旧后台仍占串口时再启动控制后台。

### 先做不发送电机命令的预览

终端 A：

```bash
ros2 launch underwater_bringup manual.launch.xml motor_tx:=false
```

终端 B：

```bash
ros2 run manual_control manual_control --ros-args -p auto_start:=false -p preview:=true
```

预览可以观察分配建议，不会请求启用电机。按 `Q` 退出键盘入口，再在终端 A 按 `Ctrl+C` 关闭后台。

### 实际手动操作

确认现场有独立急停或断电手段，且推进器周围安全后，终端 A 启动允许电机命令发送的后台：

```bash
ros2 launch underwater_bringup manual.launch.xml motor_tx:=true
```

保持终端 A 运行，在终端 B 加载环境后启动键盘入口：

```bash
ros2 run manual_control manual_control --ros-args -p auto_start:=false
```

按键含义如下。方向键会累计档位，松开按键不会自动归零；初次操作应单次轻按并观察反馈。

| 按键 | 作用 |
|---|---|
| `W` / `S` | `Fx` 加一档 / 减一档 |
| `T` / `F` | `Fy` 加一档 / 减一档 |
| `I` / `J` | `Fz` 加一档 / 减一档 |
| `Y` / `G` | 横滚档位加 / 减 |
| `U` / `H` | 俯仰档位加 / 减 |
| `A` / `D` | 偏航档位加 / 减 |
| `Q` | 请求停机并退出 |
| `Ctrl+C` | 执行退出停机流程 |

首个方向键会请求启用。正常结束时先在键盘入口按 `Q`，再在终端 A 按 `Ctrl+C`。主机停机依赖通信链路；若链路故障或无法确认电机停止，使用现场独立急停/断电手段。

### 单个电机说明

当前 ROS 手动控制是六轴输入再分配到推进器，没有只启动某一个电机的入口。单轴操作通常会分配给四台推进器；例如 `Fx=1` 的建议转速为 `[1200,-1200,1200,-1200,0,0,0,0] RPM`。这只是目标建议，不能作为实际转速反馈，也不能用于单台电机点动。

查看推进器反馈：

```bash
ros2 topic echo /underwater_robot/actuators/thrusters/state --qos-reliability best_effort
```

## 4. 排查“Enable rejected”

如果键盘入口提示 `Gateway must be connected, settled, heartbeating and enable_motor_tx=true`，在另一个已加载环境的终端查看网关状态：

```bash
ros2 topic echo /underwater_robot/gateway/status --once
```

确认网关已连接、稳定等待已结束、心跳新鲜且 `tx_enabled: true`。`manual_control` 使用 `auto_start:=false` 时只连接现有后台，不会替你把监测模式切成可发送模式。若 `tx_enabled` 为 false，按正常顺序关闭旧后台，再以 `manual.launch.xml motor_tx:=true` 启动；不要叠加启动另一个串口网关。

话题、有效位和数据记录详见[传感器与数据记录](02-sensors-recording.md)，手动控制细节详见[手动控制](03-manual-control.md)。

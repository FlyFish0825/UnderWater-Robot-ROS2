# 02 传感器与数据记录

[教程总目录](README.md) · 上一篇：[安装与编译](01-installation.md) · 下一篇：[手动控制](03-manual-control.md)

## 本页索引

- [终端与串口检查](#serial)
- [后台如何启动和复用](#backend)
- [查看 IMU 和深度](#telemetry)
- [数据有效性与网关状态](#validity)
- [水面深度归零](#zero)
- [CSV 记录](#csv)
- [ROS bag 录制与隔离回放](#bag)

<a id="serial"></a>
## 终端与串口检查

每个终端先执行：

```bash
cd /home/lamost/UnderWater-Robot-ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

检查 USB 串口和权限：

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyACM*
groups
```

当前配置默认路径为：

```text
/dev/serial/by-id/usb-STMicroelectronics_Lamost_UnderWater_Robot_ComPort_3588346E3134-if00
```

换 MCU 后以实际枚举路径为准。默认串口名包含设备标识，不保证另一块板使用同一路径。
用户组应包含 `dialout`；若没有，执行以下命令后退出并重新登录：

```bash
sudo usermod -aG dialout "$USER"
```

检查已有占用和节点：

```bash
fuser -v /dev/ttyACM0
ros2 node list
```

常见后台节点为 `/underwater_robot/mcu_gateway`、`/underwater_robot/devices`；
带分配器的后台还包含 `/underwater_robot/thrust_allocator`。
一条物理串口只能由一个网关管理。不要对已运行的机器人重复启动网关。

<a id="backend"></a>
## 后台如何启动和复用

兼容入口默认 `auto_start=true`：检测到网关就复用，未检测到时准备共享后台。
新建传感器、记录或预览后台使用 `motor_tx=false`；深度归零仍可通过独立服务显式请求。
复用已有控制后台时，传感器入口不会把该后台切换回监测模式。

同一用户、ROS 域和机器人命名空间下共享自动后台。最后一个兼容入口退出后关闭其创建的后台；
单独运行的 `ros2 topic echo` 不会为自动后台保留入口租约。需要查看话题时保持至少一个兼容入口运行。
外部手动启动的后台不会被这些入口自动关闭。

| ROS 参数 | 含义 |
|---|---|
| `robot_namespace` | 内部话题前缀，默认 `/underwater_robot` |
| `auto_start` | 默认 `true`；`false` 时只连接已有话题和服务 |
| `serial_port` | 新建自动后台时使用的串口；不改变已存在后台的配置 |
| `duration_s` | 传感器/记录入口默认 0，表示持续运行；必须非负 |
| `output_dir` | CSV 记录入口的保存目录 |
| `imu_source` | `raw`（默认）或 `calibrated`；校正源需先显式采集基准 |

改变 `robot_namespace` 不会改变兼容输出 `/imu_data`、`/depth_temp` 和 `/forces` 的绝对名称；
多机器人部署还需规划这些兼容话题的重映射。

<a id="telemetry"></a>
## 查看 IMU 和深度

单独查看 IMU，并发布 `/imu_data`：

```bash
ros2 run imu_py imu_publisher
```

查看深度、温度、压力，并发布 `/depth_temp`：

```bash
ros2 run depth_py depth_publisher
```

另一条等价深度入口为：

```bash
ros2 run sensors_cpp depth
```

这两个深度入口二选一，不能重复发布 `/depth_temp`。IMU 兼容发布入口也只运行一份。

一个终端汇总查看 IMU 和深度：

```bash
ros2 run sensors_cpp sensors_sub
```

汇总入口直接订阅内部状态，不需要预先运行 `imu_publisher` 和 `depth_publisher`。
用 `Ctrl+C` 退出；只查看 10 秒可以执行：

```bash
ros2 run sensors_cpp sensors_sub --ros-args -p duration_s:=10.0
```

新建后台时指定实际串口：

```bash
ros2 run sensors_cpp sensors_sub --ros-args -p serial_port:=/dev/ttyACM0
```

| 数据 | 兼容入口/CSV 单位 | 内部标准消息 |
|---|---|---|
| 深度 | m | 深度状态字段为 m |
| 温度 | °C | 温度消息为 °C |
| 压力 | Pa | 压力消息为 Pa |
| 加速度 | m/s² | `sensor_msgs/Imu` 为 m/s² |
| 角速度 | °/s | `sensor_msgs/Imu` 为 rad/s |
| 姿态欧拉角 | ° | 内部 `euler_rpy_rad` 为 rad |

命令名称兼容不代表旧消息 ABI 兼容；消息定义见 [msg_pkg](../../src/msg_pkg/msg)。
平放、水平和朝向的初始姿态校正见 [第 5 篇](05-imu-calibration.md)。
该功能保留原始数据，不需要 IMU 位置偏移，并支持主机保存与重启加载。

<a id="validity"></a>
## 数据有效性与网关状态

`NaN` 表示缺少可用测量。刚启动、数据过期、固件未提供有效压力或零点时都可能出现，不能当作 0。深度计型号固定为 MS5837-02BA，无需确认或写入型号。
原始 IMU 和姿态分组独立：加速度有效不代表姿态也有效。
深度计原始 ADC 有值，也不代表压力、温度和水深已完成有效换算。

保持后台运行，另一个终端检查：

```bash
ros2 topic echo /underwater_robot/sensors/depth/state \
  --qos-reliability best_effort
```

重点查看 `fresh`、`pressure_valid`、`temperature_valid`、`depth_valid`、`zero_valid`、
`filtered_depth_m` 和 `surface_pressure_pa`。压力和深度按设备上报有效位判断，不要求型号确认状态位。

网关状态：

```bash
ros2 topic echo /underwater_robot/gateway/status --once
```

| 字段 | 检查目的 |
|---|---|
| `connected` | 串口是否连接 |
| `settling` | 是否还在连接后的稳定等待阶段 |
| `rx_bytes`、`valid_frames` | 是否持续收到数据与有效协议帧 |
| `crc_errors`、`shape_errors` | 是否持续增加校验或结构错误 |
| `heartbeat_age_ms` | 心跳是否新鲜，负值表示尚无心跳 |
| `tx_enabled` | 网关是否允许电机命令发送 |
| `motor_command_active` | 当前电机命令状态 |

查看话题及发布频率：

```bash
ros2 topic list
ros2 topic hz /underwater_robot/sensors/imu/state
```

终端话题工具用 `Ctrl+C` 退出。字段和有效位的完整说明见 [ROS 接口](../interfaces/topics.md)。

<a id="zero"></a>
## 水面深度归零

终端 A 保持传感器入口运行。把当前项目使用的 MS5837-02BA 探头放在空气或水面参考位置，等待压力稳定。
终端 B 加载环境后调用：

```bash
ros2 service call /underwater_robot/sensors/depth/zero \
  std_srvs/srv/Trigger "{}"
```

成功应返回 `success: true`。服务只等待新的有效压力，发送一次 `ZERO_DEPTH`，读取设备实际 P0，
再等待零点后的有效深度；不会读取、确认或写入型号，也不会在启动或重连时自动归零。

随后检查：

```bash
ros2 topic echo /underwater_robot/sensors/depth/state --once
```

检查深度和零点有效，再观察是否接近零。此入口只在操作者明确调用时采集当前空气/水面压力。

- 机器人现有深度基准优先沿用设备上报的 P0；上位机不会用固定大气压覆盖它。当前固件把 P0 保存在 MCU RAM，掉电后若设备报告零点无效，再由操作者决定是否手动重采。
- 启动和断线重连不会自动归零。
- 水下归零会把当时位置定义为新的参考面。
- 此服务不会启用电机，也不是完整传感器标定。
- `enable_depth_commands` 与电机发送开关独立，必须允许此服务的受限写入。

详细机制见 [深度归零](../deployment/depth-zero.md)。

<a id="csv"></a>
## CSV 记录

开始 10 Hz 记录：

```bash
ros2 run sensors_cpp sensors_sub_jilushuju
```

终端打印实际文件名，格式为 `sensors_毫秒时间戳_进程号.csv`。
默认目录在编译时根据源码根目录确定：当前部署为 `/home/lamost/UnderWater-Robot-ROS2/Recodes/`。
不是运行命令时所在的任意当前目录。

指定目录并记录 60 秒：

```bash
ros2 run sensors_cpp sensors_sub_jilushuju \
  --ros-args \
  -p output_dir:=/home/lamost/robot_data/csv \
  -p duration_s:=60.0
```

按 `Ctrl+C` 正常结束会刷新并关闭文件；终端打印记录行数。

```bash
ls -lh /home/lamost/robot_data/csv/
```

| CSV 字段 | 含义 |
|---|---|
| `receipt_time_s` | 记录行生成时的 ROS 时间，秒 |
| `depth_m`、`temp_c`、`pressure_pa` | 深度、温度、压力 |
| `acc_x`、`acc_y`、`acc_z` | 加速度，m/s² |
| `gyro_x_deg_s`、`gyro_y_deg_s`、`gyro_z_deg_s` | 角速度，°/s |
| `roll_deg`、`pitch_deg`、`yaw_deg` | 欧拉角，° |
| `raw_valid`、`attitude_valid`、`depth_valid` | 对应分组有效状态 |
| `imu_source`、`imu_reference_saved_at` | 新增末尾列：姿态数据源及校正基准保存时间；原始源的保存时间为空 |

记录器定时保存当时缓存的各组数据，一行不代表所有传感器严格同步采样。
缺测字段保留 NaN；没有相机假数据。记录器本身不要求其他兼容发布入口先启动。

<a id="bag"></a>
## ROS bag 录制与隔离回放

保持传感器后台运行，在另一个终端录制遥测和状态：

```bash
mkdir -p /home/lamost/robot_data/bags
ros2 bag record \
  -o "/home/lamost/robot_data/bags/run_$(date +%Y%m%d_%H%M%S)" \
  /underwater_robot/gateway/status \
  /underwater_robot/sensors/imu/state \
  /underwater_robot/sensors/imu/data_raw \
  /underwater_robot/sensors/depth/state \
  /underwater_robot/sensors/status \
  /underwater_robot/actuators/thrusters/state \
  /underwater_robot/control/allocation
```

`Ctrl+C` 结束。将下面的路径替换成生成的实际录包目录：

```bash
ros2 bag info /home/lamost/robot_data/bags/run_实际时间
```

离线回放选用未被其他测试占用的 ROS 域，下面使用 187。
终端 A：

```bash
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 \
ros2 bag play /home/lamost/robot_data/bags/run_实际时间
```

终端 B 加载同一工作空间，使用相同域，且关闭自动后台：

```bash
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 \
ros2 run sensors_cpp sensors_sub --ros-args -p auto_start:=false
```

示例只录制遥测和状态。不要将控制请求、内部电机设定或原始网关帧回放到真实机器人所在 ROS 域。
回放时不需要启动物理串口网关。

参考：[ROS 官方录制与回放教程](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)。

[返回教程总目录](README.md) · 上一篇：[01 安装与编译](01-installation.md) · 下一篇：[03 手动控制](03-manual-control.md)

# 05 IMU 基准姿态校正

[教程总目录](README.md) · 上一篇：[维护与故障排查](04-maintenance.md) · [项目 README](../../README.md)

本功能为 `v1.0.0` 之后新增的开发功能，原 `v1.0.0` 标签不包含以下接口。
更新到包含此功能的源码并完整编译后使用。以下操作只在 ROS 主机保存基准，不向 MCU 发送硬件标定命令。

## 本页索引

- [功能与适用范围](#scope)
- [编译与准备](#prepare)
- [平放和朝向归零](#zero)
- [记录已知非零姿态](#known-pose)
- [显示与 CSV 记录](#display)
- [保存、重启加载和清除](#persistence)
- [状态与采样要求](#status)
- [数学约定与边界](#math)
- [验证与排障](#verification)

<a id="scope"></a>
## 功能与适用范围

把机器人固定在已知的平放、水平及朝向状态，采集一段稳定姿态作为基准。
后续得到相对于该基准的姿态，并可把基准标记成用户指定的 roll/pitch/yaw。

| 数据或操作 | 行为 |
|---|---|
| 原始 `sensors/imu/state` | 保留设备解码后的原始数据 |
| 新增 `sensors/imu/calibrated` | 姿态组输出基准校正结果；加速度、角速度、磁场、气压保持原数据 |
| 校正采样 | 显式服务请求后开始，检查角速度、姿态波动和采样时长 |
| 保存 | 保存原始基准、目标姿态、采样数、传感器 frame 和 UTC 时间 |
| 重启 | 从同一配置文件加载；不自动重新采集 |
| 清除 | 显式删除当前基准；校正姿态恢复无效，原始数据继续可用 |

本功能不需要 IMU 相对机器人中心的 XYZ，不做位置偏移的加速度补偿。
它也不校准陀螺仪零偏、磁场或传感器硬件，不估计质心，不去除重力。
显示归零与机器人中心处的真实加速度是不同问题。

<a id="prepare"></a>
## 编译与准备

本次新增 ROS 消息和服务，并修改了设备处理与兼容入口。结束旧实例后完整编译并在新终端加载新环境：

```bash
cd /home/lamost/UnderWater-Robot-ROS2
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

只编辑 Windows 源码不会自动更新 Jetson 原部署。先将对应源码部署到准备使用的工作空间，再编译。
不要把旧构建和新接口混用，也不要另起网关抢占串口。

终端 A 启动或复用后台，查看原始姿态：

```bash
ros2 run sensors_cpp sensors_sub
```

保持该入口运行。确认 IMU 的 `raw` 和 `attitude` 有效，机器人处于静止且电机已停止的状态。
如果复用的是旧版本后台，需要由其启动者正常关闭并启动新版本，才能出现新服务。

终端 B 加载同一工作空间后检查接口：

```bash
ros2 interface show underwater_interfaces/srv/CalibrateImu
ros2 service list
```

应看到 `/underwater_robot/sensors/imu/calibrate` 和 `/underwater_robot/sensors/imu/clear_calibration`。
使用自定义机器人命名空间时，替换命令中的 `/underwater_robot`。

<a id="zero"></a>
## 平放和朝向归零

1. 把机器人摆到期望的水平基准姿态。
2. 将机器人朝向希望记作零度的方向，并固定，避免晃动。
3. 保持终端 A 的原始传感器入口运行。
4. 在终端 B 发起采样，并保持机器人静止直到服务返回。

```bash
ros2 service call /underwater_robot/sensors/imu/calibrate \
  underwater_interfaces/srv/CalibrateImu \
  "{reference_rpy_deg: [0.0, 0.0, 0.0], samples: 100, timeout_s: 15.0}"
```

成功响应中包含：

```text
success: true
message: stationary attitude reference saved and activated
captured_rpy_deg: [...]       # 采集到的原始基准角，度
samples_collected: 100
calibration_file: ...        # 本机实际保存路径
```

只有采样通过且文件写入成功才返回 `success=true`。服务等待采样时，节点继续接收传感器数据，不阻塞网关。
机器人仍在同一姿态时，新校正姿态应接近 `[0,0,0]`，受噪声影响不保证每帧恰好为零。
此时原始话题中的角度仍是原始读数，便于检查校正前后的区别。

默认请求字段也可以用 `{}`，但上面的完整写法更便于记录实验条件。

<a id="known-pose"></a>
## 记录已知非零姿态

`reference_rpy_deg` 的顺序是 `[roll, pitch, yaw]`，全部为度。
例如已把机器人放平，并希望当前朝向记为 90°：

```bash
ros2 service call /underwater_robot/sensors/imu/calibrate \
  underwater_interfaces/srv/CalibrateImu \
  "{reference_rpy_deg: [0.0, 0.0, 90.0], samples: 100, timeout_s: 15.0}"
```

当前姿态便被标记为 `[0,0,90]`。这只是使用用户给定的朝向标签，不会自动确认磁北、真北或 ENU。
如果已准确摆到其他参考姿态，也可填入对应角度。roll/yaw 范围为 ±180°，pitch 为 ±90°，禁止 NaN/Inf。

单次已知姿态并不能同时识别未知的传感器世界参考和完整机械安装外参。
若 IMU 的轴与机器人轴不平行，后续机体 roll/pitch/yaw 对应关系仍需单轴转动验证；
这里得到的是按下面数学定义计算的基准相对姿态，不应直接当作已经标定完毕的机体坐标系。

<a id="display"></a>
## 显示与 CSV 记录

兼容入口默认 `imu_source=raw`，保持原有行为。查看校正结果时显式选择：

```bash
ros2 run sensors_cpp sensors_sub --ros-args -p imu_source:=calibrated
```

`imu_publisher` 也支持选择校正姿态，并向 `/imu_data` 发布：

```bash
ros2 run imu_py imu_publisher --ros-args -p imu_source:=calibrated
```

不要同时运行两份 `imu_publisher` 去发布同一个 `/imu_data`。
原始与校正的对照可以同时使用两个 `sensors_sub` 查看入口，或直接 echo 两个内部话题。
加速度和角速度仍是传感器轴数据，不能因为姿态已归零就当作机器人中心的测量。

直接查看完整校正消息：

```bash
ros2 topic echo /underwater_robot/sensors/imu/calibrated \
  --qos-reliability best_effort
```

该话题仍按 0x80 原始组、0x81 姿态组和 0x85 气压组分别发布。
检查姿态时关注 `group=129`、`fresh`、`euler_valid`、`quaternion_valid`。
没有有效基准时，校正姿态为 NaN、有效位为 false；不会偷偷用原始角度冒充校正结果。

记录校正姿态 CSV：

```bash
ros2 run sensors_cpp sensors_sub_jilushuju \
  --ros-args \
  -p imu_source:=calibrated \
  -p output_dir:=/home/lamost/robot_data/imu-calibrated \
  -p duration_s:=60.0
```

原 CSV 字段顺序保留，末尾新增 `imu_source` 和 `imu_reference_saved_at`：前者标明选择的输入，
后者记录接收到的基准状态中的保存时间。默认原始记录的后者为空。
只按固定列数读取 CSV 的旧分析脚本需适配新增列。
状态与测量是独立 ROS 消息，校正切换瞬间的时间标记可能存在发现/传输延迟；
建议先完成校正并确认状态，再开始一段记录，不在同一记录期间切换基准。

需要保留完整证据时，同时录制原始、校正和状态：

```bash
mkdir -p /home/lamost/robot_data/bags
ros2 bag record \
  -o "/home/lamost/robot_data/bags/imu-reference-$(date +%Y%m%d_%H%M%S)" \
  /underwater_robot/sensors/imu/state \
  /underwater_robot/sensors/imu/calibrated \
  /underwater_robot/sensors/imu/calibration_status
```

回放保持在隔离 ROS 域，并设置查看入口 `auto_start=false`；参见 [录包与回放](02-sensors-recording.md#bag)。

<a id="persistence"></a>
## 保存、重启加载和清除

默认使用 `$ROS_HOME/<机器人命名空间>/imu_reference.txt`，`ROS_HOME` 未设置时使用 `~/.ros`。
当前 `/underwater_robot` 命名空间和 `lamost` 用户对应：

```text
/home/lamost/.ros/underwater_robot/imu_reference.txt
```

实际路径以服务响应或状态话题为准。文件保存：格式版本、欧拉角约定、传感器 frame、
基准四元数、目标 RPY、样本数和 UTC 保存时间。数据位于源码仓库之外。

查看记录：

```bash
cat /home/lamost/.ros/underwater_robot/imu_reference.txt
```

每次成功校正替换当前基准。若需要保留多个实验的基准记录，可在下一次校正前复制：

```bash
mkdir -p /home/lamost/robot_data/imu-references
cp /home/lamost/.ros/underwater_robot/imu_reference.txt \
  "/home/lamost/robot_data/imu-references/reference-$(date +%Y%m%d_%H%M%S).txt"
```

需要自定义路径时，在 [sensors.yaml](../../internal/underwater_bringup/config/instances/sensors.yaml) 设置：

```yaml
imu_calibration_file: /home/lamost/robot_data/imu_reference.txt
```

路径支持绝对路径或 `~/`；参数在启动时读取，修改后重新编译/加载配置并正常重启对应后台。
不要让两个不同机器人或测试实例共用一个校正文件。

同一文件、同一 IMU frame 下重启会自动加载。不存在时保持未校正；格式损坏、frame 不匹配或读取失败时明确报告错误，
不会把损坏基准当作零姿态，原始传感器话题仍可用。保存采用同目录临时文件刷新后原子替换，
保存失败不会激活新基准。

清除当前基准：

```bash
ros2 service call /underwater_robot/sensors/imu/clear_calibration \
  std_srvs/srv/Trigger "{}"
```

清除会取消正在进行的采样，并删除当前保存文件；原始话题不受影响。已有历史副本和 ROS bag 不会被删除。
重新校正前，`imu_source=calibrated` 的角度显示 NaN。

更换 IMU、重新安装、改变固件姿态算法，或 MCU 重启导致航向参考重新初始化后，应重新确认/采集基准。
文件仍能加载不等于物理基准依然有效。本功能不会在断线重连时偷偷归零。

<a id="status"></a>
## 状态与采样要求

查看状态：

```bash
ros2 topic echo /underwater_robot/sensors/imu/calibration_status --once
```

| 字段 | 含义 |
|---|---|
| `calibrated` | 当前是否有有效的已保存基准；不等于此刻有新鲜 IMU |
| `collecting` | 是否正在采集新基准 |
| `attitude_fresh` | 当前原始姿态是否新鲜有效 |
| `samples_collected` / `samples_requested` | 本进程最近一次采样请求的进度；重启后为 0 |
| `captured_rpy_deg` | 已激活基准对应的原始 RPY，度 |
| `reference_rpy_deg` | 用户要求赋给该基准的目标角，度 |
| `saved_at` | 当前基准的 UTC 保存时间 |
| `calibration_file` | 实际文件路径 |
| `detail` | 当前状态或失败原因 |

默认采样条件：

- 有效原始陀螺仪必须在本次请求后到达，且与姿态属于同一次连接。
- 陀螺仪向量模长不超过 0.05 rad/s。
- 姿态相对首个接受样本的旋转距离不超过 1°。
- 至少覆盖 1 秒，样本按最短间隔接收，避免一批积压数据瞬间完成校正。
- 请求样本数为 10～2000；超时必须大于最短采样时长且不超过 120 秒。
- 默认 100 个样本、15 秒超时；实际采样时间取决于数据频率。

阈值配置为 `imu_calibration_max_gyro_rad_s`、`imu_calibration_max_spread_deg`、`imu_calibration_min_duration_s`。
运动、连接变化、无效数据、超时或写文件失败都会让本次请求失败，并保留上一次已激活的基准。
这些检查不能证明机器人完全没有平移；采样时仍需人工固定机器人。

<a id="math"></a>
## 数学约定与边界

输入使用当前协议的 `euler_rpy_rad`，按右手 ZYX 顺序构造旋转：`Rz(yaw) * Ry(pitch) * Rx(roll)`。
不把未经确认的设备四元数和欧拉角混用。采样将四元数符号统一到同一半球后平均并归一化，
避免 +179.9° 和 -179.9° 平均成 0°。

```text
q_baseline = 稳定姿态样本的平均四元数
q_target   = 用户指定的 reference_rpy_deg 对应四元数
q_current  = 当前原始欧拉角对应四元数
q_output   = q_target * inverse(q_baseline) * q_current
```

在基准姿态时输出接近目标姿态。该乘法次序定义的是以初始传感器朝向为参考的相对旋转，再施加目标姿态标签。
它不是完整的机械安装外参辨识，也没有承诺 ENU/NED 或真实机体轴已经确认。
单个姿态无法从未知世界参考中唯一分离安装旋转和航向偏置。

输出欧拉角在 ±180° 存在表示换界，在 pitch 接近 ±90° 时存在欧拉角奇异性；需要连续旋转处理时使用校正四元数。
该功能不替换 `sensors/imu/data_raw`，也不把参考姿态写成标准 ROS 世界姿态供控制器自动使用。
`header.frame_id` 和原始加速度/角速度仍表示 IMU 传感器轴。

参考：[ROS REP-145 IMU 坐标与数据约定](https://github.com/ros-infrastructure/rep/blob/master/rep-0145.rst)。

<a id="verification"></a>
## 验证与排障

单元与离线检查：

```bash
bash tools/validate.sh offline
```

专项集成验证：

```bash
bash tools/validate.sh imu-calibration
```

专项测试仅启动设备解析节点并注入合成帧，不启动串口网关或控制器。
使用 ROS 域 184、进程专属命名空间和临时文件，覆盖采样、保存、原始数据保留、运动拒绝、
角度换界、过期失效、重启加载、清除和写入失败。
本轮构建、测试及证据范围见 [2026-10-08 验证记录](../experiments/2026-10-08-imu-reference-validation.md)。

| 问题 | 检查 |
|---|---|
| 找不到服务类型或服务 | 是否完整编译、加载新环境，并重启旧后台 |
| 采样一直等待新鲜 gyro | 原始组是否有效、是否收到本次请求后的数据 |
| `robot is rotating` | 陀螺仪是否超过阈值，先固定机器人，不要盲目放宽限制 |
| `attitude moved` | 姿态样本波动过大；检查振动和设备姿态漂移 |
| 超时 | IMU 频率是否足以收集所需样本，是否掉线或数据过期 |
| 保存失败 | 目录权限、磁盘、实际路径及文件类型；旧基准继续有效 |
| 成功后界面仍是旧角度 | 查看入口是否设置 `imu_source:=calibrated` |
| 校正角为 NaN | 是否存在有效基准、姿态是否新鲜，是否刚清除或文件加载失败 |
| 重启后方向不一致 | 固件世界/航向参考是否改变；重新摆放基准并校正 |

[返回教程总目录](README.md) · 上一篇：[04 维护与故障排查](04-maintenance.md)

# UnderWater-Robot-ROS2 完整使用教程

[返回项目 README](../../README.md) · [v1.0.0 发布说明](../../CHANGELOG.md)

本教程面向 `v1.0.0`，运行平台为 Jetson Orin NX、Ubuntu 22.04、ROS 2 Humble，业务实现为 C++17。
命令默认在 Jetson 的 Bash 终端执行，示例部署位置为 `/home/lamost/UnderWater-Robot-ROS2`。
其他用户或安装位置需要替换路径。Windows 侧用于源码维护，编译和设备运行在 Jetson 完成。

## 本页索引

- [阅读路线](#reading)
- [版本能力](#capabilities)
- [已有工作空间快速开始](#quick-start)
- [日常操作顺序](#daily)
- [命令速查](#commands)
- [专题资料](#references)

<a id="reading"></a>
## 阅读路线

| 顺序 | 文档 | 内容 |
|---|---|---|
| 1 | [安装与编译](01-installation.md) | 系统准备、GitHub 源码、依赖、编译、目录和每个终端的环境 |
| 2 | [传感器与数据记录](02-sensors-recording.md) | 串口、后台、IMU、深度、有效位、水面归零、CSV、录包和回放 |
| 3 | [手动控制](03-manual-control.md) | 隔离预览、按键、档位保持、实际输出、停机、后台管理和电机反馈 |
| 4 | [维护与故障排查](04-maintenance.md) | 参数修改、测试、版本更新、日志、常见故障和启动服务 |

新机器从第 1 篇开始。已经编译好的工作空间可以从下面的快速开始进入第 2 篇。
准备控制推进器前，应完整阅读第 3 篇。

<a id="capabilities"></a>
## 版本能力

| 功能 | 当前能力 |
|---|---|
| MCU 通信 | USB CDC 串口网关、协议解析、连接与有效性状态 |
| IMU | 加速度、角速度、姿态；原始数据与姿态分别判断有效性 |
| 深度计 | 压力、温度、深度与显式水面归零 |
| 电机反馈 | 节点状态、转速、电压、估算母线电流、MCU 温度 |
| 记录 | 10 Hz CSV、ROS bag 消息录制 |
| 控制 | 键盘六轴档位、八推进器分配、启用与超时门禁 |
| 验证 | 协议和设备测试、合成遥测、真实 ROS 节点加伪串口测试 |

```text
IMU / 深度计 / 电机反馈 → MCU → USB CDC → 网关 → 设备解析 → 查看 / 记录
键盘 → 六轴档位 → 动力分配 → 连接、启用和超时检查 → MCU → 电机
```

19 个兼容命令中，8 个接入现有能力。下列 11 个命令仅保留名称，正常执行和 `--check` 返回 69：

- `sensors_cpp`：`camera_display`、`camera_save`、`camera_ros`、`camera_parallel_ros`。
- `sensors_cpp`：`auto_control`、`auto_control_new`、`semi_control`、`semi_control_fc`、`semi_control_fcadrc`。
- `global_camera`：`global_cam_pub`、`data_sub`。

未实现相机、定深、定向、自动航行等能力。`imu_py`、`depth_py` 保留旧名称，实际代码是 C++。
IMU 气压需要匹配 MCU 固件，气压高度不能替代水深。URDF 目前只有 `base_link`。

<a id="quick-start"></a>
## 已有工作空间快速开始

每个终端先加载环境：

```bash
cd /home/lamost/UnderWater-Robot-ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

先用一个终端汇总查看传感器：

```bash
ros2 run sensors_cpp sensors_sub
```

需要记录时，在另一个已加载环境的终端运行：

```bash
ros2 run sensors_cpp sensors_sub_jilushuju
```

记录位置会打印在终端中。当前示例工作空间默认位于 `Recodes/`。
每个入口用 `Ctrl+C` 正常退出。程序会准备或复用后台，不需要先手工启动其他传感器入口。

这两条命令不会主动申请电机启用；如果复用已有控制后台，它们不会改变该后台的发送配置。
遇到 `NaN` 时，按 [数据有效性](02-sensors-recording.md#validity) 检查，不能把缺测当作零值。

<a id="daily"></a>
## 日常操作顺序

1. 检查 MCU USB 枚举、串口权限和已有网关，避免重复占用串口。
2. 在每个终端加载 ROS 和项目环境。
3. 启动 `sensors_sub`，确认数据有效。
4. 需要时在空气或水面执行深度归零。
5. 启动 CSV 或 ROS bag 记录。
6. 需要控制时，按 [手动控制教程](03-manual-control.md) 确认现场条件和后台模式。
7. 结束时先在控制入口按 `Q`，再停止记录和查看程序。
8. 手动启动的后台，最后在其终端用 `Ctrl+C` 结束。

由兼容入口创建的后台，在最后一个兼容入口退出后关闭。外部启动的后台由启动者负责关闭。
发布前的软件验证未操作真实电机；真实方向、推力曲线和实机响应仍需要单独验证。

<a id="commands"></a>
## 命令速查

| 用途 | 命令 | 说明 |
|---|---|---|
| 汇总查看 | `ros2 run sensors_cpp sensors_sub` | 直接订阅内部 IMU 和深度状态 |
| IMU | `ros2 run imu_py imu_publisher` | 查看并发布 `/imu_data` |
| 深度 | `ros2 run depth_py depth_publisher` | 查看并发布 `/depth_temp` |
| 另一深度入口 | `ros2 run sensors_cpp depth` | 与上一条二选一 |
| CSV 记录 | `ros2 run sensors_cpp sensors_sub_jilushuju` | 默认 10 Hz |
| 键盘实际控制 | `ros2 run manual_control manual_control` | 首个方向键申请启用；先阅读控制篇 |
| 实验激励 | `ros2 run manual_control forces_pub` | 启动后主动申请启用；不是普通查看命令 |
| ROS 时间 | `ros2 run manual_control time_read_test` | 打印时间后退出，不启动后台 |

所有兼容入口支持 `--help` 和 `--check`；这两个检查选项不操作硬件。
已实现入口的 `--check` 返回 0；未实现入口返回 69；`--help` 返回 0。

<a id="references"></a>
## 专题资料

- [ROS 话题与消息含义](../interfaces/topics.md)
- [MCU 线协议](../protocols/mcu-wire.md)
- [深度归零实现与历史验证](../deployment/depth-zero.md)
- [动力分配与手动转速](../deployment/thrust-allocation.md)
- [CAN 抓包分析教程](../deployment/can-command-analysis.md)
- [兼容入口验证](../deployment/entry-compatibility.md)
- [当前硬件](../hardware/current-hardware.md)
- [软件架构](../architecture/architecture.md)

下一篇：[01 安装与编译](01-installation.md)

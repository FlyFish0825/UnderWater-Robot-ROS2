# UnderWater-Robot-ROS2

首个公开发布版：**v1.0.0**（2026-10-08）。版本范围和验证记录见 [发布说明](CHANGELOG.md)。

ROS 2 Humble / Jetson Orin NX / C++17。业务入口采用 fc_sensors_ros2 的包名和程序名，
原有 underwater_* 实现放在 internal，原调试命令与原生消息继续可用。

业务级六自由度远程控制更新位于 `codex/dashboard-controller-input` 功能分支：
GUI 持有操作权，Nano 保留动力分配和电机执行链。使用前阅读
[远程操控教程](docs/usage/10-remote-manual-control.md)；两端必须匹配 TCP 9003 v2。
本次提交的范围、离线核验和待验收项见 [2026-10-12 整理记录](docs/experiments/2026-10-12-remote-manual-verification.md)。

## 文档索引

**日常转发操作从 [串口与相机转发手册](docs/usage/09-forwarding-handbook.md) 开始；完整使用教程见 [教程总目录](docs/usage/README.md)。** 新机器按安装、传感器、控制、维护的顺序阅读；
已有 Jetson 工作空间可直接进入传感器与记录篇。各篇均带章节索引、上一篇和下一篇链接。

| 需要了解的内容 | 入口 |
|---|---|
| 完整使用教程、日常操作顺序 | [教程总目录与快速开始](docs/usage/README.md) |
| Nano 开机后启动 ROS 2、查看 IMU/深度、进入手动控制 | [00 Nano 从开机到运行 ROS 2](docs/usage/00-nano-first-start.md) |
| 新机器准备、源码获取、编译、终端环境 | [01 安装与编译](docs/usage/01-installation.md) |
| 串口检查、IMU、深度归零、CSV、ROS bag | [02 传感器与数据记录](docs/usage/02-sensors-recording.md) |
| 平放/朝向基准采样、姿态校正、保存与重启加载 | [05 IMU 基准姿态校正](docs/usage/05-imu-calibration.md) |
| 串口转发使用教程：9000 原始数据、9002 控制权、GUI/Nano 接管与释放 | [06 串口 TCP 转发教程](docs/usage/06-nano-tcp.md) |
| 相机转发使用教程：9001 双目图像、预览、记录与实际帧率 | [07 相机 TCP 转发教程](docs/usage/07-stereo-camera.md) |
| 从开机到双路连接、控制权切换、停止与故障检查的完整手册 | [09 串口与相机转发操作手册](docs/usage/09-forwarding-handbook.md) |
| 固定局域网 IP、随 ROS 2 同启两条转发、远程回退与同时接收验收 | [08 局域网与 TCP 恢复](docs/usage/08-lan-recovery.md) |
| 双目相机采集、左右图、JPEG 转发与上位机接口 | [07 双目相机](docs/usage/07-stereo-camera.md)、[上位机接入报告](docs/architecture/deployment/stereo-camera-host-report.md) |
| 隔离预览、键盘档位、实际控制、停机与反馈 | [03 手动控制](docs/usage/03-manual-control.md) |
| 上位机业务操作权 / Nano 执行链、TCP 9003 v2、会话确认与断线停止 | [10 业务级远程手动控制](docs/usage/10-remote-manual-control.md) |
| 配置修改、重新编译、测试、更新与故障排查 | [04 维护与故障排查](docs/usage/04-maintenance.md) |
| 编译与运行命令 | [编译和环境](#编译和环境)、[沿用原命令](#沿用原命令) |
| 键盘按键、档位与启停 | [键盘控制](#键盘控制)、[动力分配与手动转速控制](docs/architecture/deployment/thrust-allocation.md) |
| 如何分析 CAN CSV 中的力与转速指令 | [抓包分析教程](docs/architecture/deployment/can-command-analysis.md) |
| 111.csv、112.csv 的方向、时间线与曲线 | [2026-10-08 抓包实例记录](docs/experiments/2026-10-08-can-command-analysis.md) |
| 传感器修复、真实 GUI/Nano 控制权与原始串口验收 | [2026-10-10 最终实机验收](docs/experiments/2026-10-10-sensor-fix-control-acceptance.md)、[排障阶段记录](docs/experiments/2026-10-10-control-ownership.md) |
| ROS 话题与串口协议 | [话题接口](docs/interfaces/topics.md)、[MCU 线协议](docs/protocols/mcu-wire.md) |
| 深度归零与硬件范围 | [深度归零](docs/architecture/deployment/depth-zero.md)、[当前硬件](docs/hardware/current-hardware.md) |
| 验证方法与兼容入口 | [验证说明](docs/architecture/deployment/validation.md)、[兼容入口验证](docs/architecture/deployment/entry-compatibility.md) |
| 软件结构与发布范围 | [架构说明](docs/architecture/architecture.md)、[发布说明](CHANGELOG.md) |
| 分支、提交范围、生成文件与验证要求 | [开发和提交规范](CONTRIBUTING.md) |

## 当前目录

```text
src/
├── msg_pkg/          对外消息定义
├── imu_py/           imu_publisher；实际为 C++
├── depth_py/         depth_publisher；实际为 C++
├── sensors_cpp/      深度、查看、记录入口及共享后台
├── manual_control/   键盘、实验激励和时间查看入口
└── global_camera/    双目采集、左右图、JPEG TCP 转发与图片记录
internal/
├── underwater_interfaces/    原内部消息
├── underwater_mcu_gateway/   串口、协议、命令及深度归零
├── underwater_devices/       IMU、深度、电机反馈解析
├── underwater_control/       八推进器分配、启停和超时
├── underwater_bringup/       原 XML 启动和参数
├── underwater_description/   最小 URDF
└── underwater_robot/         基础软件心跳
third_party/                  动力分配及 OSQP/Eigen 等依赖
tests/integration/            C++ 集成探针
tools/                        验证脚本
docs/                         接口、协议、部署和实验记录
deploy/                       服务模板
```

共 13 个实际 ament_cmake 包：6 个入口包和 7 个内部包。没有包内嵌套包。
不创建未来功能的空目录；未实现功能只有返回明确错误的命令入口，没有空驱动或假数据。

## 编译和环境

首次获取包含当前转发、控制权及传感器修复的源码（仅用于目标目录尚不存在时）：

```bash
git clone --branch main https://github.com/FlyFish0825/UnderWater-Robot-ROS2.git
cd UnderWater-Robot-ROS2
```

`main` 已包含相机转发、原始串口控制权和传感器修复；本次业务操控更新另在功能分支提交，不表示已合并到主分支。
首次获取业务操控版时，将上面的 `--branch main` 改成 `--branch codex/dashboard-controller-input`。
`v1.0.0` 是历史固定发布标签，不包含上述后续功能。已有工作空间时进入原项目目录，按[维护教程](docs/usage/04-maintenance.md)核对版本和未提交内容后再更新。以下命令以当前 Jetson 的部署路径为例：

```bash
cd /home/lamost/UnderWater-Robot-ROS2
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

在工作空间根目录直接执行 `colcon build` 即可全量编译 13 个项目包。
项目自带 `colcon_defaults.yaml`，自动设置 src/internal 包搜索范围、逐包构建、符号链接安装和 RelWithDebInfo，
无需再手动添加构建参数；不会将 third_party、artifacts 或 tmp 中的源码/备份识别为额外包。
查看包列表直接使用 `colcon list`。
Windows 可核验不依赖 ROS 的控制核心和协议，完整 ROS 编译、POSIX 网络及设备验证在 Nano/Linux 完成；build/install/log 是生成目录。

## 沿用原命令

| 命令 | 当前行为 |
|---|---|
| ros2 run imu_py imu_publisher | 查看 IMU 并发布 /imu_data，角度为度，角速度为度/秒 |
| ros2 run depth_py depth_publisher | 查看深度、温度和压力，发布 /depth_temp |
| ros2 run sensors_cpp depth | 同上，和 depth_publisher 二选一 |
| ros2 run sensors_cpp sensors_sub | 汇总查看现有 IMU 和深度 |
| ros2 run sensors_cpp sensors_sub_jilushuju | 10 Hz CSV 记录，并打印保存位置 |
| ros2 run manual_control manual_control | 方向按键输入六轴手动档位，首个方向键请求启用 |
| ros2 run manual_control forces_pub | 原 Fx 实验激励时序，默认执行 100 秒然后请求停机 |
| ros2 run manual_control time_read_test | 打印 ROS 时间，不启动后台 |

每个终端先 source install/setup.bash。传感器、查看和记录入口可单独运行，
所需后台会自动准备。记录默认保存到项目根目录的 Recodes/，Nano 上为
/home/lamost/UnderWater-Robot-ROS2/Recodes；此默认路径在编译时确定，不随启动终端的当前目录改变。
参数 output_dir 可改保存位置。记录包含时间、深度、温度、压力、加速度、
角速度、欧拉角和有效位，缺测字段为 NaN，不伪造相机数据。

可选 ROS 参数：robot_namespace（默认 /underwater_robot）、auto_start（默认 true）、
serial_port（自动后台的 USB CDC 路径）、duration_s（0 表示持续运行；激励默认 100 秒）。
例如 --ros-args -p duration_s:=5.0。测试接入已有后台时可设 auto_start:=false。
手动和激励入口支持 preview:=true：发布预览请求，不调用启用服务。
preview 不是全局禁止发送开关；预览应使用隔离环境或确认 motor_tx=false，
具体步骤见 [隔离预览](docs/usage/03-manual-control.md#preview)。

### IMU 基准姿态校正（v1.0.0 之后新增）

把机器人固定在已知水平和朝向状态，调用 `sensors/imu/calibrate` 采集并保存基准。
兼容查看与记录入口使用 `--ros-args -p imu_source:=calibrated` 选择校正姿态，默认仍为 `raw`。
基准在主机保存并在重启时加载；原始话题、加速度和角速度不改动，不需要位置 XYZ。
完整步骤、已知非零姿态、状态与清除命令见 [IMU 校正教程](docs/usage/05-imu-calibration.md)。
该功能是初始姿态参考校正，不是机械外参、陀螺零偏或机器人中心加速度标定。

原生 /underwater_robot/... 话题不变。外部 /imu_data、/depth_temp、/forces 使用
src/msg_pkg/msg 中的新消息定义；命令名兼容不等于旧消息二进制 ABI 兼容。
/forces 是请求量，不是测得推力。原始 IMU 与姿态分别判断有效性，角度展示使用度。

## 键盘控制

| 按键 | 操作 |
|---|---|
| W / S | Fx 增减 1 |
| A / D | 偏航力矩 N 增减 1 |
| I / J | Fz 增减 1 |
| T / F | Fy 增减 1 |
| Y / G | 横滚力矩 K 增减 1 |
| U / H | 俯仰力矩 M 增减 1 |
| Q / Ctrl+C | 发送零请求、尝试禁用并退出 |

各轴手动档位范围为 -100 到 100：0 停机，1 档起步，100 档为上限；档位不是 N 或 N·m。
按键累加档位，松开按键不会自动归零；长按可能连续加档。结束操作使用 Q 或 Ctrl+C。
0～7 的 control_flag 含义未从旧 MCU 核实，
会明确提示不支持，不猜测或发送模式字节。半自动 P/O 等功能尚未实现。
手动程序等待首个方向键才请求启用；forces_pub 是主动激励命令，启动后请求启用。
底层保留单来源、连接/会话、零输出握手、时间戳和超时检查。
拒绝抢占已有 wrench 发布者；失效后需要新的操作请求启用。
主机停机依赖通信，不能替代电机端看门狗和独立断电措施。

手动入口使用独立的 control/manual_request：先按六轴方向分配，再统一调整比例，
力矩基准为八推进器中心 `thruster_center`，手动六轴按 `[-1,1,1,-1,1,-1]` 映射到 NED。
默认布局六轴单独 ±1 均有四路 ±1200 RPM，100 档峰值为 6000 RPM。
必要通道保持共同缩放比例并跨过死区；无法在转速范围内保持比例的组合会拒绝输出并停机。
单独 Fx=1 为 `[1200,-1200,1200,-1200,0,0,0,0]` RPM，
Fz=1 为 `[0,0,0,0,1200,1200,-1200,-1200]` RPM；回到六轴全零立即请求停机。
终端显示 Manual levels 和 Proposed RPM；/forces 发布分配器换算后的模型请求量。

物理力接口 control/wrench_request、wrench_command 和 forces_pub 保持原临时线性比例，
不套用手动起步增益。手动与物理力输入合计只允许一个发布者。
以上是目标转速；实际推力曲线和实机响应仍需要标定。
详见 [动力分配说明](docs/architecture/deployment/thrust-allocation.md)。
对照 CAN 抓包检查实际下发值，见 [抓包分析教程](docs/architecture/deployment/can-command-analysis.md)；
包括水平推进、偏航、横滚、俯仰及组合指令的 [111/112 实例](docs/experiments/2026-10-08-can-command-analysis.md)。

## 共享后台

入口先检测网关，外部启动的后台只复用，不重启或改变其配置；自动管理的后台可按下述条件切换。
不存在时，自动启动原 manual.launch.xml 的网关、设备解析和分配器。
传感器、查看、记录及预览入口使用 motor_tx=false，不发送运动指令；网关启动/停止仍可能发送保护性停止帧。
实际控制入口请求 motor_tx=true，但分配器初始 disarmed，先发送停机握手。
若自动后台此前为监测模式，控制入口会请求其管理器重启为允许发送的模式；
期间传感器可能短暂显示无效，随后恢复。外部启动的后台不自动切换模式。

同一用户、ROS_DOMAIN_ID 和 robot_namespace 下共享一份自动后台。
最后一个兼容入口退出后关闭其创建的后台；外部启动的后台不会被关闭。
日志位于 /tmp/underwater-entry-用户号-会话标识/backend.log。
复用的外部后台若没有分配器或禁用了 motor_tx，控制入口会报告不可用，
不会擅自重启后台或改写参数。

## 仅保留名称的入口

以下 9 个命令尚未实现，返回退出码 69，不启动后台或电机：

- sensors_cpp：camera_display、camera_save、camera_ros、camera_parallel_ros。
- sensors_cpp：auto_control、auto_control_new、semi_control、semi_control_fc、semi_control_fcadrc。

19 个旧命令均可解析，其中 10 个接入已有能力。global_camera 的 global_cam_pub
独立采集双目并在 TCP 9001 转发拼接 JPEG；data_sub 订阅检查并可保存左右 PNG。
这两个入口不启动 MCU 后台；原 sensors_cpp 的四个相机入口暂不重定向。
所有入口支持 --help 和 --check：
--help 不操作硬件；--check 可用返回 0，未实现返回 69。

## 原调试入口

```bash
ros2 launch underwater_bringup robot.launch.xml
ros2 launch underwater_bringup manual.launch.xml
ros2 run underwater_control wrench_command 0 0 0 0 0 0
ros2 launch underwater_robot robot.launch.xml
```

原 manual.launch.xml 仍默认 motor_tx=false。
underwater_robot 的启动仅为软件心跳；不要对已占用串口另起网关。
深度归零沿用 [原服务](docs/architecture/deployment/depth-zero.md)，不在启动或重连时自动归零；深度计型号固定只读，保留设备现有 P0。

## 测试与录包

```bash
bash tools/validate.sh offline
bash tools/validate.sh synthetic
bash tools/validate.sh allocation
```

协议、设备有效性和动力分配测试不代表实机试转。
入口验证见 [兼容入口验证](docs/architecture/deployment/entry-compatibility.md)。
录包仍使用 ros2 bag；话题表在 internal/underwater_bringup/config/recording/topics.yaml。
大型数据保存到仓库外，例如 ~/robot_data。

## 后续规划

本阶段不补做缺失的控制器、视觉或其他驱动，实际开发时再创建代码和测试。

| 方向 | 后续内容 |
|---|---|
| 旧操作完整兼容 | 核对 control_flag、旧分析脚本、单位及操作响应 |
| 闭环控制 | 定深、定向、姿态、速度、定点、PID/滑模/ADRC |
| 推进器模型 | 实测推力–RPM 曲线、限制策略、电机端看门狗 |
| 相机与视觉 | 驱动、显示、保存、双目、全局相机、轨迹采集；需型号与标定 |
| 其他传感器 | 激光雷达、声呐、DVL、声学定位、GNSS、水质、漏水、电源、缆绳 |
| 其他执行器 | 照明、云台、机械臂、浮力调节、泵阀、电源开关、绞车 |
| 外部通信 | 遥测、命令、视频、发现、批量数据、旧接口适配 |
| 算法与任务 | 状态估计、感知、导航、SLAM、任务状态机、行为及结果管理 |
| 运行与仿真 | 健康、记录、实验配置、URDF/TF、几何/惯量、标定及仿真 |

IMU 气压需匹配 MCU 固件，气压高度不替代水深。简化 URDF 定义 base_link 与八推进器中心 thruster_center，
尚未建模完整惯量与传感器外参。
历史实验记录保留当时状态；当前范围以 README、源码和实时状态为准。

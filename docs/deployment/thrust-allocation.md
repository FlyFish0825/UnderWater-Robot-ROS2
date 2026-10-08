# 动力分配与手动转速控制

项目仍为 /home/lamost/UnderWater-Robot-ROS2；所有协议、分配和发送实现均为 C++。
输入顺序是 **Fx Fy Fz Mx My Mz**，分别为模型中的 N 和 N·m，坐标系是布局使用的 base_link。
当前标尺没有经过真实推力标定，不能声称产生了输入所指定的真实六轴力。

## 1. 布局和临时映射

布局唯一来源是用户指定的 F:/file/BaiduSyncdisk/Project/OSQP/main.cpp。
没有采用 SDK 自带的不同垂直推进器位置。用户已确认 fan1..8 对应 CAN 节点 1..8，
正转推力沿该文件的方向向量。

| fan | 位置 m | 正推力轴 |
|---|---|---|
| 1 | (0.23016, -0.18359, -0.040532) | (-0.70711, -0.70711, 0) |
| 2 | (0.23016, 0.18359, -0.040532) | (-0.70711, 0.70711, 0) |
| 3 | (-0.25591, -0.18359, -0.040532) | (0.70711, -0.70711, 0) |
| 4 | (-0.25591, 0.18359, -0.040532) | (0.70711, 0.70711, 0) |
| 5 | (0.25239, -0.26163, 0) | (0, 0, 1) |
| 6 | (0.25239, 0.26163, 0) | (0, 0, 1) |
| 7 | (-0.27087, -0.26163, 0) | (0, 0, 1) |
| 8 | (-0.27087, 0.26163, 0) | (0, 0, 1) |

使用 ThrustAllocator_API_CurrentROV 的 QP 求解器，OSQP/Eigen/QDLDL/OsqpEigen
从随库源码在 ARM64 编译，未链接 Windows 二进制。位置叉乘推力得到力矩，反扭矩系数暂为 0。

临时单机映射：

```text
raw_rpm = allocated_force_N / nominal_max_thrust_N × max_rpm

|raw_rpm| < 1000  -> 0，停机
其他值            -> 四舍五入后限幅到 [-6000,-1000] 或 [1000,6000]
```

默认 nominal_max_thrust_N=500、max_rpm=6000，即 12 rpm/N。
500 N 是原示例的临时求解边界/比例，不是实测能力。
所以每台分配力小于约 83.33 N 时都落在停机死区；例如 Fz=50 时会全部停机，
这不是没有收到输入。不能为了制造转速把每次输入自动归一化到 6000 rpm。

2026-10-08 使用 Nano 当前分配库离线计算（未发布控制请求）：纯 Fx=1 时，
fan1/2 的分配力约 -0.355324 N，fan3/4 约 +0.351783 N，
fan5/6 约 -0.038730 N，fan7/8 约 +0.038730 N；乘 12 后仍远低于 1000 RPM，
最终八路目标全部为 0。纯 Fx=100 同样全部为 0。
这是物理力接口 control/wrench_request 的行为。键盘入口现使用下面的独立手动档位映射，
不再将 1 档当作 1 N 直接套入此比例；物理力接口及实验激励保持原行为。

离线例子：Fz=400，其他轴为 0，前四台为 0，后四台约 1242、1242、1158、1158 rpm。
此例只是暂定模型计算，不意味着实机输出 400 N。
死区会改变分配后的合力/力矩，因此同时发布 allocated_wrench 与 modeled_rpm_wrench，
后者仅是按临时线性模型、考虑死区后的预测，不是传感器实测。

## 手动档位与起步转速

键盘通过 control/manual_request（geometry_msgs/WrenchStamped）提交六轴档位，
各字段是 -100 到 100 的操作量，不是物理力。取六轴绝对值最大值 L：

- L 小于 1：目标全零，包括浮点噪声。
- 1 <= L <= 100：用归一化方向进行 QP 分配，再对全部分配量使用同一比例。
- 最大通道目标 = manual_start_rpm + (max_rpm - manual_start_rpm) * (L - 1) / 99。
- 默认 manual_start_rpm=1200，max_rpm=6000；零档不会套用起步转速。
- 比例调整后仍由原 linear_rpm 执行小于 1000 RPM 停机、舍入和上限检查。

这使主要推进器跨过起步死区，同时不把弱补偿通道或求解残差逐个抬到 1000 RPM。
默认配置单独 Fx=1 档约为 [-1200,-1200,1188,1188,0,0,0,0] RPM。
组合方向共用 L 和一个比例，不会将每个轴分别拉到最大转速。

control/allocation.requested_wrench 保存放大后的等效模型请求，thruster_force_n 为对应模型推力，
proposed_motor_rpm 是经过停机死区后的建议转速。它们不代表实测力或实测转速。
手动与物理力话题共用来源数量检查、时间戳、启停、会话、超时和重连门禁。

## 2. 编译

```bash
cd /home/lamost/UnderWater-Robot-ROS2
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

每个新终端都需要 source /opt/ros/humble/setup.bash 和本工作空间 install/setup.bash。

## 3. 先预览分配，不发送电机命令

终端 A：

```bash
ros2 launch underwater_bringup manual.launch.xml
```

默认 motor_tx=false，不发送电机命令；独立深度归零服务仍可按需使用。终端 B 发送一个 5 秒示例输入：

```bash
ros2 run underwater_control wrench_command 0 0 400 0 0 0 \
  --ros-args -r __ns:=/underwater_robot -p duration_s:=5.0
```

终端 C 查看：

```bash
ros2 topic echo /underwater_robot/control/allocation
```

重点字段：
- requested_wrench：输入六轴量。
- thruster_force_n：按 fan1..8 的 QP 推力。
- proposed_motor_rpm：按 CAN 节点 1..8 的建议转速。
- input_fresh、solver_ok：本次结果是否可以使用。
- armed、gateway_ready：当前能否把建议值送到电机。
- modeled_rpm_wrench：临时 RPM 模型考虑死区后的预测。

不运行控制模块、只查看传感器，继续使用原来的 robot.launch.xml。

## 4. 实际电机命令入口

下面的 motor_tx=true 与 enable 会开放真实电机下发。这不是预览。
确认现场允许推进器转动后，停止预览实例，终端 A 启动：

```bash
ros2 launch underwater_bringup manual.launch.xml motor_tx:=true
```

串口打开后先发送明确的全节点停机命令；控制节点默认仍未启用。
终端 B 等网关连接/心跳正常后启用：

```bash
ros2 service call /underwater_robot/control/enable std_srvs/srv/SetBool "{data: true}"
```

收到 success=true 后，再使用第 3 节的 wrench_command 输入。
该示例会命令 5～8 号推进器运行，因此只有准备试转时才执行。
任意方向用同一入口输入对应的 Fx/Fy/Fz/Mx/My/Mz；负值对应反向模型目标。
没有 ROS TF 自动转换，frame_id 必须与 wrench_frame_id（默认 base_link）一致。

主动停止：

```bash
ros2 service call /underwater_robot/control/enable std_srvs/srv/SetBool "{data: false}"
```

- wrench_command 结束前发送零输入；Ctrl+C 中断时由输入超时停止。
- 接收到有效输入后，连续 500 ms 没有新输入会停机并取消启用，需要再调用 enable。
- 允许 enable 后等待第一份新输入，等待期间只发送零输出，不复用 enable 前的旧输入。
- wrench_request 与 manual_request 合计只允许一个发布者；多个输入来源会取消启用。
- 不可用网关、旧时间戳、错误坐标系、NaN/Inf 或求解失败会停止控制。
- 网关另有独立的 500 ms 命令看门狗；过期后同一会话不能自行恢复。
- 串口重连产生新 connection_id，旧命令被拒绝，必须重新 enable。
- 退出正常启动程序时，网关会尽力写入停机帧，再释放串口。

重要边界：主机停机要求串口/CAN 链路仍可用。本次检查的电机固件没有找到已实现的
CAN 命令超时停机逻辑；USB/CAN 物理断链、网关进程被强杀或整机掉电时，
不能凭主机看门狗保证电机停止。实际带动力测试应具备独立断电/急停手段。
本次没有修改或烧录 MCU 固件，也没有实际电机试转。

enable 是运行状态门禁，不是网络身份认证；当前 ROS 域应属于可信机器人内部网络。

## 5. 发送协议与状态

内部 MotorSetpoint 绑定 connection_id、enable 会话和递增 sequence。
网关只接收节点 1..8、0 或 ±1000..6000 rpm，运行位必须与非零转速严格对应。

下行是旧紧凑 AA55，BODY_LEN=32、CAN ID=0x100、FLAGS=0x02（FD、关闭 BRS）、
24 字节电机 payload，总长 38 字节。下行不带上行的设备时间戳。

运行时先发送 SPEED_VECTOR(0x10，RunMask=0) 更新速度，再发送 RUN_VECTOR(0x11)
设置各节点运行位。停止时 RUN_VECTOR(RunMask=0) 优先，其后清零速度。
现有固件的 SPEED_VECTOR(RunMask=0) 不会停止已经运行的节点，所以不能仅发送零速度。

gateway/status 中 tx_bytes、tx_frames 统计完整写入的帧，
motor_session_id、motor_rearm_required、motor_command_detail 表示发送门禁状态。
它们不是 CAN 应答或真实转动证明；实测速度应查看 actuators/thrusters/state。

## 6. 以后修改的位置

- 布局、节点映射、符号、临时推力上限、QP 权重、RPM 范围：
  internal/underwater_control/config/allocation.yaml。
- 推力到 RPM 的替换点：
  internal/underwater_control/src/allocation/linear_rpm.cpp。
- QP 适配：
  internal/underwater_control/src/allocation/allocator.cpp。
- 电机命令编码/会话门禁：
  internal/underwater_mcu_gateway/src/commands/motor_commands.cpp。

配置在启动时读取，修改后重启。可将正反向实测曲线接入独立 RPM 映射函数，
同时更新反算的 modeled_rpm_wrench；不需要改 ROS 六轴输入或底层 CAN 格式。

## 7. 验证

```bash
bash tools/validate.sh offline
bash tools/validate.sh allocation
```

allocation 验证使用独立域和专用伪终端，启动真正的分配/网关节点并解析它们的串口输出。
它不打开真实机器人串口，也不是实机带载证明。

[本轮移植与验证记录](../experiments/2026-10-07-thrust-allocation-validation.md)。

# 水下机器人测试体系与控制中枢 (Testing Center)

本项目遵循 **ROS 2 官方标准规范（REP-144 标准工程布局）**，构建了分层解耦的自动化测试与现场诊断体系。

所有测试代码分为三个层次：**包内单元测试**、**系统级集成探针**与**端到端网络运维验证**。在常规构建与实机部署中，测试代码受到构建开关保护，**零性能损耗、零运行内存占用**。

---

## 一、 快速运行指南

### 1. 离线快速验证（无需硬件、1 秒出结果）
在编译完成后（`colcon build`），直接在仓库根目录或 `tests/` 目录下运行：
```bash
# 运行离线全量单测（推荐）
bash tests/run_tests.sh offline

# 或者使用 ROS 2 标准命令
colcon test --packages-select underwater_mcu_gateway underwater_control underwater_devices
colcon test-result --all --verbose
```

### 2. 远程业务控制与手柄链路验证
```bash
# 离线验证手柄控制状态机与 TCP 协议
bash tests/run_tests.sh remote-manual
```

### 3. 网络与 TCP 转发验证
```bash
# 验证 9000~9003 四大端口端到端传输
bash tests/run_tests.sh network
```

### 4. 系统级集成验证（带探针）
```bash
# 虚拟仿真环境下的遥测与传感器集成验证
bash tools/validate.sh synthetic

# 真机现场串口连线验证（只接收，不发推力）
bash tools/validate.sh live
```

---

## 二、 全项目测试文件全景索引（共 29 个）

### 1. 控制层与手柄/动力学测试（`internal/underwater_control/test/`）

| 测试文件 | 语言 | 类型 | 验证目标与安全意义 | 硬件依赖 |
| :--- | :---: | :---: | :--- | :---: |
| `remote_manual_test.cpp` | C++ | 单元测试 | **手柄纯逻辑状态机**：校验手柄六轴档位输入、防高频刷包熔断、500ms 心跳超时强制归零。 | 否 |
| `remote_manual_ros_test.py` | Python | 节点集成 | **手柄 ROS2 节点链路**：模拟上位机 TCP 9003 连接，测试租约握手与 `/control/manual_request` 话题。 | 否 |
| `remote_tcp_test.cpp` | C++ | 单元测试 | **非阻塞网络 I/O**：测试 TCP 接收/发送缓冲区、粘包拆包与断线资源回收。 | 否 |
| `allocation_test.cpp` | C++ | 契约测试 | **八推进器分配算法**：测试二次规划（OSQP）求解器能否将六轴期望力精准分配到 8 个电机的 RPM。 | 否 |
| `allocation_examples.cpp` | C++ | 算例演示 | **推力分配离线算例**：不依赖 ROS、不操作硬件的独立数学验证用例。 | 否 |

### 2. 串口网关与安全门禁测试（`internal/underwater_mcu_gateway/test/`）

| 测试文件 | 语言 | 类型 | 验证目标与安全意义 | 硬件依赖 |
| :--- | :---: | :---: | :--- | :---: |
| `gateway_motor_authority_test.cpp` | C++ | 门禁测试 | **电机发送门禁**：测试 `motor_tx=false` 强制禁发，以及控制权丢失时必然下发急停全零帧。 | 否 |
| `manual_ownership_test.cpp` | C++ | 租约测试 | **动态租约时效**：测试上位机私有动态令牌（Lease Token）的时效与伪造拦截。 | 否 |
| `control_ownership_test.cpp` | C++ | 仲裁测试 | **控制权状态机**：验证 `GUI_MANUAL`（手柄）、`GUI`（串口透传）与 `Nano`（机载）之间的互斥仲裁。 | 否 |
| `control_protocol_test.cpp` | C++ | 协议测试 | **TCP 9002 管理协议**：解析 CLAIM、TAKEOVER、RELEASE、PING 心跳指令。 | 否 |
| `motor_command_test.cpp` | C++ | 协议测试 | **电机协议打包**：测试 8 通道电机 RPM 帧序列化、校验和及限幅保护。 | 否 |
| `protocol_test.cpp` | C++ | 通讯测试 | **底盘 MCU 协议解包**：测试数据包帧头尾校验、CRC 校验与坏包过滤。 | 否 |
| `serial_io_test.cpp` | C++ | 驱动测试 | **POSIX 串口底层 I/O**：测试波特率配置、非阻塞读写与串口无回显（No-Echo）配置。 | 否 |
| `tcp_serial_bridge_test.cpp` | C++ | 透传测试 | **TCP 9000 桥接**：测试网络串口透传通道在不同控制权模式下的读写隔离。 | 否 |
| `depth_zero_test.cpp` | C++ | 标定测试 | **深度计基准零点**：验证 MS5837 传感器水面大气压 $P_0$ 标定与固定气压公式回归。 | 否 |

### 3. 传感器与机载设备测试（`internal/underwater_devices/test/`）

| 测试文件 | 语言 | 类型 | 验证目标与安全意义 | 硬件依赖 |
| :--- | :---: | :---: | :--- | :---: |
| `attitude_reference_test.cpp` | C++ | 单元测试 | **姿态基准转换**：验证 IMU 欧拉角、四元数与机器人机体坐标系（NED/FLU）转换。 | 否 |
| `device_test.cpp` | C++ | 单元测试 | **传感器抽象基类**：测试传感器设备生命周期与异常状态捕获。 | 否 |

### 4. 双目相机与推流测试（`src/global_camera/test/`）

| 测试文件 | 语言 | 类型 | 验证目标与安全意义 | 硬件依赖 |
| :--- | :---: | :---: | :--- | :---: |
| `stream_test.cpp` | C++ | 流水线测试 | **JPEG 视频流**：测试双目图像拼接、压缩并推流至 TCP 9001 端口。 | 否 |
| `stereo_transform_test.cpp` | C++ | 几何测试 | **双目几何外参**：测试左右镜头基线变换与坐标对齐。 | 否 |
| `stereo_ros_probe.cpp` | C++ | 性能探针 | **图像发布性能**：测试双目图像话题的发布帧率与延迟。 | 否 |

### 5. 系统级集成探针（`tests/integration/`）

| 测试文件 | 语言 | 类型 | 验证目标与现场排障功能 | 硬件依赖 |
| :--- | :---: | :---: | :--- | :---: |
| `telemetry_probe.cpp` | C++ | 集成探针 | **整机遥测诊断**：汇总整机所有传感器话题，实时统计频率、时延与丢包率。 | 可选（支持仿真） |
| `serial_control_probe.cpp` | C++ | 硬件探针 | **串口硬件回环**：下发测试帧并监听单片机回包，验证硬件串口连通性。 | 虚拟串口或真机 |
| `imu_calibration_probe.cpp` | C++ | 标定探针 | **IMU 零漂标定**：静置采集 IMU 数据并计算零漂方差，评估校准质量。 | 虚拟串口或真机 |
| `compat_probe.cpp` | C++ | 兼容探针 | **接口向后兼容**：验证原 `fc_sensors_ros2` 话题与新内部架构的数据一致性。 | 否 |

### 6. 运维工具与网络测试（`tools/` & `bringup`）

| 测试文件 | 语言 | 类型 | 验证目标与用途 | 硬件依赖 |
| :--- | :---: | :---: | :--- | :---: |
| `tools/validate.sh` | Bash | 自动化中枢 | **全项目一键验证入口**：统一串联单测与三大集成探针（支持 5 种模式）。 | 否 / 可选 |
| `tools/test_tcp_forwarding.py` | Python | 集成测试 | **TCP 四大端口集成测试**：自动拉起服务端并模拟客户端全双工通信。 | 否 |
| `tools/test_tcp_forwarding_client.py`| Python | 压力测试 | **TCP 模拟压力测试客户端**：高频下发数据流测试网关承载力。 | 否 |
| `tools/test_robot_static_ip.sh` | Bash | 网络安全测试 | **静态 IP 切换演练**：在隔离模拟网络中测试静态 IP 配置与回滚，防止设备失联。 | 否 |
| `internal/underwater_bringup/test/remote_manual_launch_test.py` | Python | 语法测试 | **启动脚本合规性**：验证 XML 参数映射、依赖与默认禁发配置正确。 | 否 |

---

## 三、 ROS 2 架构设计说明（为什么不物理合并到单一文件夹？）

根据 **ROS 2 官方标准规范（REP-144 / Ament CMake 标准）**：
1. **模块内聚**：每个功能包（Package）都是自包含的单元，其单元测试放置在各包的 `test/` 目录下，直接链接该包生成的内部私有目标（如内部静态库），避免将私有符号暴露给外部。
2. **构建解耦**：使用 `colcon build --packages-select <pkg>` 单独构建某一模块时，该模块的测试能独立自洽编译。
3. **长期维护**：保证与外部上游/协作仓库的分支结构保持标准一致，杜绝产生跨目录的文件增删冲突。

本 `tests/` 目录作为**顶层中枢**，通过统一调度脚本与本索引文档，为整机提供一站式的测试视图与执行能力。

# 04 维护与故障排查

[教程总目录](README.md) · 上一篇：[手动控制](03-manual-control.md) · [项目 README](../../README.md)
下一篇：[IMU 基准姿态校正](05-imu-calibration.md)

## 本页索引

- [修改位置与重新编译](#configuration)
- [软件测试](#tests)
- [版本核对与源码更新](#versions)
- [故障速查](#troubleshooting)
- [查看日志与 ROS 环境](#logs)
- [启动服务与正常结束](#services)

<a id="configuration"></a>
## 修改位置与重新编译

下列链接均指向项目内实际文件或目录：

| 内容 | 修改位置 |
|---|---|
| 网关稳定等待、数据延迟、归零服务开关 | [mcu.yaml](../../internal/underwater_bringup/config/instances/mcu.yaml) |
| 设备解析配置 | [sensors.yaml](../../internal/underwater_bringup/config/instances/sensors.yaml) |
| 推进器布局、分配权重、转速范围 | [allocation.yaml](../../internal/underwater_control/config/allocation.yaml) |
| 启动方式与默认串口 | [launch](../../internal/underwater_bringup/launch) |
| 兼容入口与自动后台 | [sensors_cpp/src](../../src/sensors_cpp/src) |
| 串口与协议 | [underwater_mcu_gateway](../../internal/underwater_mcu_gateway) |
| 设备解析 | [underwater_devices](../../internal/underwater_devices) |
| 动力分配与控制 | [underwater_control](../../internal/underwater_control) |

默认控制参数示例：

```yaml
min_rpm: 1000
max_rpm: 6000
manual_start_rpm: 1200
input_timeout_ms: 500
```

参数影响真实目标，修改时必须与硬件能力和模型相符。布局、方向符号和推力曲线应由测量与配置依据决定。
配置在启动时读取，保存 YAML 不会自动更新正在运行的节点。

正常结束相关程序后重新编译：

```bash
cd /home/lamost/UnderWater-Robot-ROS2
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

编译成功后再重新启动。不要把一次编译成功当作硬件行为已经验证。

<a id="tests"></a>
## 软件测试

在编译完成的工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
bash tools/validate.sh offline
```

包含 5 个 CTest：协议、电机命令、深度归零、设备有效性和动力分配。
后续 IMU 校正实现新增第 6 个 CTest，覆盖基准旋转、采样条件和文件持久化。
TCP 模块再增加一个本机回环测试，覆盖原始字节、单客户端、只读、缓存限制和不同客户端 IP 重连。

合成遥测：

```bash
bash tools/validate.sh synthetic
```

真实 ROS 节点加伪串口控制：

```bash
bash tools/validate.sh allocation
```

兼容入口：

```bash
ROS_DOMAIN_ID=185 ROS_LOCALHOST_ONLY=1 \
ros2 run sensors_cpp compat_probe
```

这些测试不打开真实机器人串口。合成遥测使用域 189，伪串口控制使用域 190，
兼容入口示例使用域 185；避免在这些域同时运行其他同类测试。
`tools/validate.sh live` 属于真实设备读取路径，不包含在上述离线测试流程中。

IMU 校正专项测试使用域 184，只注入合成数据：

```bash
bash tools/validate.sh imu-calibration
```

`v1.0.0` 发布前，上述测试均通过。日志中的 `PASS` 证明对应软件测试通过，不是实机带载或转动证据。
详细验证记录见 [兼容入口验证](../architecture/deployment/entry-compatibility.md) 和 [发布说明](../../CHANGELOG.md)。

<a id="versions"></a>
## 版本核对与源码更新

先区分两种目录：

- GitHub 克隆目录有自己的 `.git`，可核对版本、分支和修改。
- 手工部署的源码目录可能没有 `.git`，不能直接 `git pull`。

2026-10-08 教程编写时，Jetson 的现有部署属于后者。2026-10-10 已在保留原源码、安装目录与运行后台的前提下建立 Git 管理，并与本地统一为 `main`；开发分支内容已完整合入，历史发布标签保留。其他部署以后按实际检查结果判断。
Git 克隆工作空间可执行以下只读检查：

```bash
git rev-parse --show-toplevel
git status --short --branch
git remote -v
git describe --tags --always
```

根路径和远端应对应自己的项目。固定发布版用标签复现，开发分支则可能包含发布后的修改。
`git clone --branch v1.0.0` 处于分离 HEAD，直接 `git pull` 不是升级标签的方式。

如果需要从当前固定版本开始开发，可在确认位于正确仓库后创建自己的分支：

```bash
git switch -c work/my-change
```

更新前先正常停止控制和后台、保留本地配置及采集数据，查看并保存尚未提交的修改。
获取远端引用不会直接替换工作区：

```bash
git fetch origin --tags
git tag --list
```

然后选择实际存在的发布标签或开发分支，再编译和测试。不要使用 `reset --hard`、
`clean -fd` 或覆盖复制来掩盖工作区差异；无 Git 的现有部署应先规划源码同步或独立克隆迁移。

### 当前主分支同步与工作区整理

本地与 Nano 都保留 `main`。正常网络下，先确认工作区干净再更新：

```bash
git status -sb
git branch --show-current
git fetch origin
git merge --ff-only origin/main
git rev-parse HEAD
```

双方最后一条命令的完整提交号应一致，`git status --porcelain` 应无输出。Nano 网络异常时，使用本地导出的 Git bundle 同步对象和分支引用，再校验对应源码文件；不要用强制重置掩盖修改。

`build/`、`install/`、`log/`、`Recodes/`、`artifacts/`、`tmp/`、`output/` 和部署归档/日志属于本地生成或现场证据，按 `.gitignore` 保留，不作为源码提交。`deploy/*.service.example` 仍跟踪。旧诊断文件先归档、验证哈希再整理；不删除采集记录、恢复备份、第三方源码或当前后台使用的安装目录。

2026-10-10 归档在 `artifacts/maintenance/20261010-workspace-cleanup/`：Windows 的 `local-tmp/` 保留 30 个原临时文件，Nano 的 `nano-tmp/` 保留 13 个原临时文件，Nano 原 `gui.zip` 也保留，均验证移动前后 SHA-256 相同。原部署恢复点继续保留在 `deploy/backup-*`，本地分支整理前的完整 Git bundle 在归档目录中。当前测试后台保持运行。

<a id="troubleshooting"></a>
## 故障速查

| 现象 | 检查方法 |
|---|---|
| `ros2: command not found` | 加载 `/opt/ros/humble/setup.bash` |
| `Package ... not found` | 检查构建结果、`install/setup.bash` 和 `ros2 pkg prefix` |
| 找不到 `/dev/ttyACM0` | 检查 USB 连接、供电、枚举及实际 by-id 路径 |
| 串口 `Permission denied` | 检查 `dialout`，加入后重新登录 |
| 串口被占用 | `fuser -v /dev/ttyACM0`，确认已有网关或其他串口程序 |
| 修改 `serial_port` 无效 | 入口复用了已有后台；该参数不重配置已有网关 |
| IMU/深度一直为 NaN | 检查连接、有效帧、对应分组及有效位 |
| ADC 有值但深度无效 | 检查型号、压力与零点状态，按条件执行水面归零 |
| 启用被拒绝 | 检查连接、稳定等待、心跳、`tx_enabled` 和 `detail` |
| 没有启用服务 | 已有后台可能只有传感器；控制需要带分配器的后台 |
| 提示另一个控制来源存在 | 结束重复的键盘、激励或原生力输入入口 |
| `manual_control needs an interactive terminal` | 使用交互终端；SSH 分配终端 `-t` |
| 松开键后仍保持目标 | 档位保持是当前行为，反向减档或 `Q` 退出 |
| 建议 RPM 有值但没有实机反馈 | 检查启用、发送门禁、CAN、电源及反馈有效性 |
| 数字模式键或 P/O 无功能 | 这些功能尚未实现 |
| 回放时意外启动网关 | 回放查看入口必须 `auto_start=false`，使用隔离 ROS 域 |
| CSV 不在当前目录 | 默认路径编译时确定，查看终端打印或设置 `output_dir` |

<a id="logs"></a>
## 查看日志与 ROS 环境

列出自动后台目录：

```bash
ls -td /tmp/underwater-entry-*
```

根据实际目录替换下面路径：

```bash
tail -n 100 /tmp/underwater-entry-实际目录/backend.log
```

构建日志在工作空间 `log/` 下；ROS launch 也会打印其日志位置。
不要因为一个入口启动失败，就直接删除其他运行实例的锁文件或强杀所有 ROS 进程。

检查节点、话题、服务以及终端域设置：

```bash
ros2 node list
ros2 topic list
ros2 service list
printenv ROS_DOMAIN_ID
printenv ROS_LOCALHOST_ONLY
```

不同 ROS 域不能相互发现。教程的 185、186、187、189、190 用于隔离测试，
日常查看和控制必须与真实后台使用相同域。

<a id="services"></a>
## 启动服务与正常结束

[underwater-robot.service.example](../../deploy/underwater-robot.service.example) 是模板，
仓库发布或编译不会自动将它安装成开机服务。它默认启动传感器与心跳后台，不包含控制分配器。
日常可以继续使用终端入口；需要配置开机运行时，再核对用户、路径、串口和所需后台类型。

若以后配置了监测服务，该外部后台不会被键盘入口自动切换到控制模式。
结束控制应先让控制入口发送停机请求，再退出记录和查看程序，最后结束自己启动的后台。
遇到物理链路故障时使用独立急停/断电措施，不把停止主机进程当成电机已停的证明。

[返回教程总目录](README.md) · 上一篇：[03 手动控制](03-manual-control.md) · [返回项目 README](../../README.md)
下一篇：[05 IMU 基准姿态校正](05-imu-calibration.md)

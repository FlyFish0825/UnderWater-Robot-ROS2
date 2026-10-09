# 01 安装与编译

[教程总目录](README.md) · [项目 README](../../README.md) · 下一篇：[传感器与数据记录](02-sensors-recording.md)

## 本页索引

- [平台与现有部署](#platform)
- [准备 ROS 与工具](#dependencies)
- [获取固定版本](#source)
- [编译全部包](#build)
- [每个终端加载环境](#environment)
- [目录说明](#directories)

<a id="platform"></a>
## 平台与现有部署

项目运行平台为 Jetson Orin NX、Ubuntu 22.04、ROS 2 Humble；主要语言是 C++17。
本文默认 Bash 和用户 `lamost`，工作空间为 `/home/lamost/UnderWater-Robot-ROS2`。
使用其他用户时替换路径；在 SSH 登录后的 Jetson 终端执行命令。

2026-10-08 编写教程时，现有 Jetson 部署目录有源码和构建产物，但没有 `.git`。
这是当时的检查结果：以后可用下面的命令判断当前目录是否为 Git 工作树：

```bash
git -C /home/lamost/UnderWater-Robot-ROS2 rev-parse --show-toplevel
```

返回值应恰好是该项目根目录。报错或返回其他上级路径时，不能直接套用此项目的 `git pull` 更新流程。
已部署并编译好的机器可跳到 [加载环境](#environment)，不必重复克隆或安装。

<a id="dependencies"></a>
## 准备 ROS 与工具

新机器先按 [ROS 2 Humble 官方 Ubuntu 安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
配置 ROS 软件源和 UTF-8 环境。Humble 的 Ubuntu 二进制目标为 22.04；Jetson 使用 ARM64 包。
已有 JetPack/Ubuntu 环境按设备要求维护，不需要为了运行本项目重装系统。

软件源配置完成后，安装工具与 ROS 基础运行环境：

```bash
sudo apt update
sudo apt install \
  git \
  build-essential \
  cmake \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-ros-base \
  ros-humble-ros2launch \
  ros-humble-rosbag2
```

Python 工具用于 ROS 构建和依赖管理，项目业务模块仍然是 C++。

检查环境：

```bash
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
command -v ros2
command -v colcon
cmake --version
```

`ROS_DISTRO` 应为 `humble`；项目控制包要求 CMake 至少 3.22。

<a id="source"></a>
## 获取固定版本

仓库：[FlyFish0825/UnderWater-Robot-ROS2](https://github.com/FlyFish0825/UnderWater-Robot-ROS2)。
以下命令仅用于目标位置尚无项目目录的新部署，不覆盖已有源码或记录：

```bash
cd ~
git clone --branch v1.0.0 \
  https://github.com/FlyFish0825/UnderWater-Robot-ROS2.git
cd ~/UnderWater-Robot-ROS2
```

克隆发布标签出现 `detached HEAD` 是正常现象，表示固定在 `v1.0.0`。
核对版本：

```bash
git describe --tags --exact-match
git status --short --branch
```

首次使用 `rosdep` 时初始化：

```bash
sudo rosdep init
rosdep update
```

已经初始化的机器跳过 `sudo rosdep init`，不要删除已有 rosdep 配置。
进入工作空间并安装声明的依赖：

```bash
source /opt/ros/humble/setup.bash
cd ~/UnderWater-Robot-ROS2
rosdep install \
  --from-paths src internal \
  --ignore-src \
  --rosdistro humble \
  -r -y
```

<a id="build"></a>
## 编译全部包

在工作空间根目录编译，`src/` 和 `internal/` 都参与包发现：

```bash
cd /home/lamost/UnderWater-Robot-ROS2
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

不要只对 `src/` 编译而漏掉内部包。检查包清单：

```bash
colcon list
```

应发现 13 个项目包：6 个兼容入口包和 7 个内部包。
项目根目录的 `colcon_defaults.yaml` 自动提供构建默认值，命令无需附加参数。
首次编译会从附带源码构建动力分配及相关依赖。构建成功时应显示 13 个包完成，且没有失败包。

`v1.0.0` 发布前在独立 Jetson 工作空间全量编译通过，当次约 7 分 29 秒；这不是固定耗时承诺。

<a id="environment"></a>
## 每个终端加载环境

每个新终端执行：

```bash
cd /home/lamost/UnderWater-Robot-ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

确认实际加载的包来自准备使用的工作空间：

```bash
ros2 pkg prefix sensors_cpp
ros2 pkg prefix underwater_mcu_gateway
```

它们应该指向该工作空间下的 `install/`。如果指向旧克隆或临时验证目录，开一个干净终端重新加载。

先执行不操作硬件的入口检查：

```bash
ros2 run imu_py imu_publisher --check
ros2 run depth_py depth_publisher --check
ros2 run manual_control manual_control --check
ros2 run manual_control manual_control --help
```

已实现入口显示 `available`。打印当前 ROS 时间可用：

```bash
ros2 run manual_control time_read_test
```

它打印后退出，不启动硬件后台。多个终端之间还应保持 `ROS_DOMAIN_ID` 和 `ROS_LOCALHOST_ONLY` 设置一致。
默认未设置 `ROS_DOMAIN_ID` 时使用域 0；教程中的测试命令使用单条命令的环境前缀，不修改后续终端环境。

<a id="directories"></a>
## 目录说明

| 目录 | 内容 |
|---|---|
| `src/` | 兼容命令入口、对外消息、共享后台 |
| `internal/` | 协议、串口网关、设备、控制、启动配置、URDF |
| `third_party/` | 动力分配及 Eigen、OSQP 等依赖源码与许可证 |
| `tests/` | 集成探针源码 |
| `tools/` | 验证脚本 |
| `docs/` | 本教程、架构、协议、部署与实验记录 |
| `deploy/` | 系统服务示例模板 |
| `build/` | 编译中间文件和缓存，生成目录 |
| `install/` | 程序、库、消息与环境脚本，生成目录 |
| `log/` | 构建和测试日志，生成目录 |
| `Recodes/` | CSV 默认记录目录，按当前拼写使用 |
| `artifacts/` | 本地验证资料，若存在则由 Git 忽略 |

项目内部的 `install/` 是正常构建产物，与用户主目录中曾存在的 `/home/lamost/install` 是不同路径。
不要删除项目的 `install/` 后继续使用旧环境；如清理了构建产物，需要重新编译。

[返回教程总目录](README.md) · 下一篇：[02 传感器与数据记录](02-sensors-recording.md)

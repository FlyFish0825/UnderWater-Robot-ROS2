# 实际 URDF 推进器布局与正转速极性修正

日期：2026-10-09。此记录覆盖 Windows 本地源码、Nano ARM64 编译以及离线分配/伪串口验证。
本记录是该日第一轮布局/极性修正的历史结果。用户随后确认手动六轴应整体取反，
当前手动正方向与示例见 [后续方向修正记录](2026-10-09-manual-direction-examples.md)；物理力接口结果保持不变。
未打开真实机器人串口、未启用实机电机、未发送实机运动指令；结果不代表实测推力或机器人运动。

## 几何与坐标依据

用户确认实际机器人使用以下 URDF：
`F:/file/BaiduSyncdisk/文件/中科大/我自己/论文/水下机器人/水下机器人模型/URDF/ROV_Product_FV.SLDASM/urdf/ROV_Product_FV.SLDASM.urdf`。

- URDF SHA-256：`0fe86691b57d855aea2143762f3f95c43a08cb8934f68c80934a5d7e45b9a9ec`。
- 坐标采用北东地（NED）：+X 北、+Y 东、+Z 竖直向下。请求 frame_id 仍为 base_link，无自动 ENU/NED 转换。
- 八个 joint 的 parent 均为 base_link；安装位置使用 joint origin xyz，轴线通过 origin rpy 旋转到 base_link。
- URDF 的 rpy 只保留有限小数，轴线核对使用 1e-5 容差；分配保留理想 45° 水平轴和竖直轴。
- 1～4 号位置与旧配置一致。5/6 改为 `(0.062129, ±0.125, -0.040532)` m，
  7/8 改为 `(-0.087871, ±0.125, -0.040532)` m，位于 1～4 号内侧。
- 修正后的 6×8 分配矩阵 rank=6。

URDF 提供安装位置和旋转轴线；电机正转速对应的推力极性来自用户确认：
1 西南、2 西北、3 东南、4 东北、5/6 竖直向下、7/8 竖直向上。
模型 axes 保持原有正推力轴，使用 `motor_direction=[1,-1,-1,1,1,1,-1,-1]` 转换到电机转速。

## 修改范围

- 更新 allocation.yaml 和 C++ Config 中的安装位置、极性与 NED 注释。
- control_node 的 motor_direction 默认参数从 Config 取得，避免不加载 YAML 时仍使用全 +1。
- 分配回归使用独立 URDF 位置预设和用户确认的正转速物理轴，反算八路 RPM 的模型合力/力矩，
  核对六轴正负方向及 modeled_rpm_wrench；覆盖死区和正负转速。
- 伪串口集成的 Fz 输入改为 500，使四台垂直推进器均跨过死区，检查正向 Fz 的前正后负、
  正向 Fx 的 `- + - +` 转速，并保留预览、启用、超时、重连和多输入来源门禁检查。
- 更新当前部署、硬件和 CAN 分析说明。2026-10-08 的历史抓包与实验数值保留采集时配置含义。

## Nano 验证

目标：`lamost@192.168.20.70`，项目 `/home/lamost/UnderWater-Robot-ROS2`。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install --executor sequential \
  --packages-select underwater_control underwater_bringup --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
ctest --test-dir build/underwater_control --output-on-failure
build/underwater_control/allocation_test
bash tools/validate.sh allocation
```

- underwater_control、underwater_bringup：两个包编译通过（RelWithDebInfo）。
- CTest：1/1 通过；分配回归：`PASS 1582 allocation checks`。
- 伪串口集成：通过预览、启用、超时停机、重新启用、串口重连、手动起步/归零/超时和输入来源互斥。
  测试创建 `/tmp/underwater-allocator-XXXXXX/serial`，使用独立 ROS 域 190，不打开实际串口。
- 本地配置与 C++ 默认值直接对照实际 URDF 核验通过；五个修改的代码/测试文件与 Nano 逐字节核对。
- installed allocation.yaml 解析至源码配置；control_node 和 serial_control_probe 安装入口使用新构建产物。

节点 1～8 顺序的离线输出：

| 输入 | RPM |
|---|---|
| 物理力接口 Fz=400，其余为零 | `[0,0,0,0,1406,1406,0,0]` |
| 物理力接口 Fz=500，其余为零 | `[0,0,0,0,1757,1757,-1243,-1243]` |
| 手动 Fx=+1 档，其余为零 | `[-1200,1200,-1188,1188,0,0,0,0]` |
| 手动 Fz=+1 档，其余为零 | `[0,0,0,0,1200,1200,0,0]` |

## 边界与恢复

临时 6000 RPM ↔ 500 N 比例、1000 RPM 停机死区、6000 RPM 上限保持原有行为，未做真实推力标定。
实际前后垂直力臂不同，Fz=400 和手动 Fz=1 档的后侧补偿落入死区，因此死区后的模型输出
存在俯仰残差；不能把 QP 死区前的纯 Fz 目标视为死区后的纯 Fz 输出。

本地证据和原文件位于 `artifacts/thrust-urdf-2026-10-09/`，该目录被 Git 忽略。
Nano 原文件备份位于 `/home/lamost/UnderWater-Robot-ROS2/log/thrust-urdf-2026-10-09-4f5f2cd1/`。
恢复源文件后需重新编译对应包；配置在节点启动时读取，已运行的节点需重启才能使用新值。
此次没有提交或推送，也没有修改已有 TCP/IMU 工作或停止已有实机服务。

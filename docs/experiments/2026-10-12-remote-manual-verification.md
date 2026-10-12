# 2026-10-12 业务操控整理与验证边界

[项目说明](../../README.md) · [操作与协议教程](../usage/10-remote-manual-control.md) · [提交规范](../../CONTRIBUTING.md)

## 提交范围与版本

本轮整理此前完成的 GUI/Nano 业务操作权修正，不再改变终端键位、分配算法、推进器顺序或源码发送默认值。
源码、测试和文档的发布目标为两个仓库的 `codex/dashboard-controller-input` 同名功能分支：

- 作者：[FlyFish0825/UnderWater-Robot-ROS2](https://github.com/FlyFish0825/UnderWater-Robot-ROS2)。
- 个人：[MG-521-LY/UnderWater-Robot-ROS2](https://github.com/MG-521-LY/UnderWater-Robot-ROS2)。

本轮推送预检：个人仓库允许创建分支；作者仓库向账号 `MG-521-LY` 返回 HTTP 403，需作者授予仓库写权限后再推送。
因此不能把个人仓库发布描述成作者仓库也已更新；此次不以新建 PR 或修改主分支代替授权。

本地基线 `54e4b5943492281b3e2e4d2b8ecc48aaad5e5331`；检查时作者 main 为
`5784419d468c5f2597ba6edcd61972d20de56c95`，包含本地未集成的测试/格式整理与合并更新。
本次不更新任一 `main`，不把未经回归的主分支合并混入本轮提交。合并前另做冲突和接口回归。
匹配上位机为 Underwater_GUI 的同名功能分支，已提交版本 `0342970d3d14285d7d3351e86d74ba292309ded5`，使用 9003 v2。
代码版本与部署版本必须分别核对，分支已推送不等于 Nano 已部署最新文档/源码。

## 本轮本机核验（Windows，无 ROS）

工具链：MinGW 8.1 / C++17、CMake 3.28.6、Ninja 1.11.1、Python 3.12.10。
本轮不连接 Nano、不启动网络监听或 ROS 节点、不接管控制权、不发送运动命令。

| 检查 | 结果及覆盖范围 |
|---|---|
| REMOTE_MANUAL_CORE_ONLY 构建和 CTest | 1/1 通过；直接执行 remote_manual_test 为 337 checks，覆盖协议/会话、行缓冲和背压核心 |
| 网关独立 C++ 测试 | manual_ownership 41、control_protocol 18、gateway_motor_authority 42、protocol 1292、motor_command 25、depth_zero 42；共 1460 checks |
| 启动 XML 合同 | Python unittest 2/2 通过，检查入口和默认禁止发送参数透传，不执行 launch |
| GUI/ROS2 手动线协议交叉检查 | 匹配 GUI 工作空间的 tools/test-manual-contract.ps1 -Ros2Root E:/ROV/ROS2：4836 checks，0 failures；不启动 ROS 或访问设备 |
| Python/XML 静态语法 | 40 个 Python、22 个 XML 通过；不导入 ROS、不执行节点 |

核心构建复现见[教程离线验证](../usage/10-remote-manual-control.md#离线验证)。
六个网关测试源文件位于 `internal/underwater_mcu_gateway/test/`，以 C++17 编译；
权限测试使用 `control_ownership.cpp`/`control_protocol.cpp`，帧与电机门禁测试复用
`src/protocol/{codec,stream_parser}.cpp` 和 `src/commands/{motor_commands,depth_zero}.cpp`。
编译的可执行文件仅放 `build/test/gateway-authority/`，不提交到远程仓库。

## 先前 Nano 验证（历史记录，不是本轮重跑）

此前 v2 修正在 Nano/Linux ROS2 Humble 独立副本与正式工作空间完成 8 包编译，正式构建约 4 分 39 秒，
13 项离线 CTest 通过；私有 localhost DDS 使用真实适配节点、模拟状态/服务进行集成核验，
不启动串口网关、不访问真实电机。只读连接收到 `HELLO ROV_MANUAL 2` 和 24 字段状态。

2026-10-12 16:53，用户明确确认现场看护和急停条件后，临时用户服务曾重启为
`motor_tx=true`、`tcp_allow_tx=false`；读回 `armed=false`，八路建议 RPM 为零。
这不是本轮重新查到的运行状态，也不代表实机运动或物理停机已验收。源码启动默认仍为 `false`。
临时服务不会开机自动恢复，SSH 断开与服务停机也不是同一件事。

保留原 Nano 回退资料：`/home/lamost/rov-authority-backup.OhRvsM5h/source-before-v2.tar.gz`，
编译记录 `formal-build.log` 位于该备份目录；旧 v1 回退包为
`/home/lamost/rov-manual-backup.01huQRt0/original-source.tar.gz`。这些是机器上的历史路径，不随 Git 源码发布。

## 清理范围与待验收项

四个重复本地构建目录 `build/remote-manual-core`、`build/remote-manual-core-locked`、
`build/remote-manual-core-locked-2`、`build/remote-manual-local` 归档到
`build/test/precommit-cache-20261012/`，可恢复原位置。移动后的 CMake 缓存可能带旧绝对路径，不能直接当新构建使用。
本轮测试集中在 `build/test/remote-manual-core/` 和 `build/test/gateway-authority/`。
部署源码归档、实验数据、必要第三方源码和服务模板保留；生成目录与备份按 `.gitignore` 排除。

待现场验收：真实手柄来源/释放/失联，六轴正负方向和档位，八推进器实际反馈，网络/进程/MCU 故障下的物理停机。
测试通过、下位机 armed、建议 RPM 或串口写入统计都不能代替这些验收。
本次本机无法重新完成 Linux ROS、POSIX socket 和真实 DDS 集成运行，历史结果单独列示，不伪装成本轮结果。

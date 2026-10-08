# 兼容入口验证

在工作空间根目录编译全部 src 和 internal 包后，source install/setup.bash。

## 无硬件入口检查

所有 19 个原入口都支持 --help 和 --check。--help 返回 0；--check 对已实现入口返回 0，
对尚未实现的视觉/闭环/轨迹入口返回 69。两种检查都不启动后台或操作硬件。

## 隔离消息与键盘测试

```bash
ROS_DOMAIN_ID=185 ROS_LOCALHOST_ONLY=1 ros2 run sensors_cpp compat_probe
```

探针启动独立的假数据发布者和假 enable 服务，所有子入口设置 auto_start=false，
不启动真实网关，不打开设备串口。测试覆盖角度单位转换、数据过期、深度、CSV、
手动程序等待按键、W/A/T/I 的方向映射、Q 退出禁用，以及预览模式 Ctrl+C。
探针输出文件保留在 /tmp/underwater-compat-probe-进程号。

## 自动后台生命周期

使用另一个独立 ROS_DOMAIN_ID、ROS_LOCALHOST_ONLY=1，并将 serial_port 指向不存在的
/tmp/underwater-no-such-port，启动两个不同持续时间的入口。
确认只出现一个网关、一个设备节点和一个分配器；第一个入口结束不停止其他入口，
最后一个退出后自动后台关闭。该测试不使用真实串口。

## 实机读取

复用已有后台，使用 imu_publisher 或 depth_publisher 的 duration_s 参数进行有界读取。
不能因为入口解析成功、预览成功或串口写入就声称电机方向、实际转速或闭环已验证。
手动实际输出、实验激励、模式键和缺失算法应分别验证，不在入口测试中自动执行。

手动档位更新后，键盘使用 control/manual_request；forces_pub 仍使用物理力话题。
allocation_test 覆盖六轴正反首档、零档、噪声、全档位范围和物理力接口不变。
serial_control_probe 使用真实分配器和伪串口检查首档非零 RPM、零档停机、
手动超时及与物理力来源冲突时禁用；不打开机器人串口。

## 2026-10-08 Nano 验证结果

- 13 个包完成全量编译；随后对入口修正重新编译通过。
- 19 个原命令的 --check 全部符合预期：8 个可用返回 0，11 个未实现返回 69。
- 原有 5 个 CTest 测试通过：协议、电机命令、深度归零、设备有效性、动力分配。
- 原有合成遥测集成与伪终端控制集成通过，未使用真实电机输出。
- compat_probe 通过：单位转换、过期失效、深度、CSV、首键启用、方向键、Q 停机、
  真实 PTY Ctrl+C 事件以及激励预览。各子进程均正常退出。
- ROS_DOMAIN_ID=186 的不存在串口测试通过：单一后台、监测模式不启用电机发送、
  一个入口退出不影响另一个、最后退出后后台关闭。
- ROS_DOMAIN_ID=187 的不存在串口测试通过：监测转控制模式；未按方向键时电机命令未激活。
- 复用原有实机后台读取 IMU 和深度成功；深度订阅发现后读取到约 -0.037 m、24.8 摄氏度。
  初始未收到数据时正确显示 NaN。未执行归零、实机启用或非零电机请求。
- 迁移后本地与 Nano 的 108 个源码/配置/文档文件已做 SHA-256 一致性核对。

Nano 验证日志位于 /tmp/underwater-entry-*.log；原目录和构建产物备份保存在
/home/lamost/underwater-before-entry-layout-9yuslspw/。

## 2026-10-08 手动首档修正验证

- 新增独立 manual_request 档位输入，物理力接口保持原换算。
- 本地和 Nano 的动力分配测试通过，共 1397 项检查；六轴正反首档均有四路有效起步目标。
- 真实节点加伪串口测试通过：Fx=1 档实际编码为
  [-1200,-1200,1188,1188,0,0,0,0] RPM；零档、超时、双来源冲突均停止并保持原门禁。
- 兼容入口集成测试通过，包括方向键、Q、Ctrl+C 和激励预览；未执行实机非零转速试验。
- 在确认原控制器未使能、无有效输入后重启后台，加载 manual_start_rpm=1200。
- 修改前源码备份位于 /home/lamost/underwater-before-manual-start-ai0a4lac/；
  构建与验证日志位于 Nano 的 /tmp/underwater-manual-start-*.log。
- 部署后检查 armed=false、motor_command_active=false。此时 MCU USB 未枚举，
  /dev/ttyACM0 及其 by-id 路径不存在，connected=false；后台等待自动重连，未声称实机转动已验证。

## 2026-10-08 发布前目录清理

上文记录的 `underwater-before-entry-layout-9yuslspw` 和
`underwater-before-manual-start-ai0a4lac` 为当时的备份位置，现已清理，不能再用作恢复来源。
当前源码以 Git 提交和发布标签为版本依据。

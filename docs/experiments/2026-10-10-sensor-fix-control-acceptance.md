# 2026-10-10 传感器修复与控制权实机验收

## 传感器结果

H750 修复写完成回调中立即启动读阶段的时序：改为主循环在原事务期限内提交读阶段，HAL_BUSY 延后重试，真实错误按类型返回。实际引脚、固定 MS5837-02BA 和空气参考 P0=100965 Pa 均未改变，也没有校零。

修复固件经 WCH-Link/OpenOCD 写入并 Verified OK；ELF SHA-256 为 `C88A8E555F98CADB5516BD88631A34A3F46E64BA342778D58C890F9AD2B5FE07`，BIN 为 `AB09632BA27AA4C8C41D8715932FA4D5ACD142DC35952507414CF8E9F6AC21B2`。H750 主机回归 51 项通过、ARM Debug 编译通过。

- 深度计原始 D1/D2、压力、温度、水深、P0 全有效。压力 100949 Pa，温度 26.41 °C，D1=6295611、D2=7928969，水深约 -0.001586 m。
- 实测深度采样 24.994～25.005 Hz；成功样本从 938 增至 5838，设备错误始终为 0。
- WCH-Link 拔下后 IMU 0x80 原始量、0x81 四元数/欧拉角、0x85 气压四项同时有效；气压约 101540.67 Pa、温度 31.77 °C。气压高度不替代水深。

## 真实 GUI/Nano 控制权与串口路径

Windows 验收程序使用正式 GUI 的 BootloaderCommunicationService、TcpControlTransport、TcpTransport 和协议解析器。程序只向 MCU 发送一次 20 字节深度 GET_STATUS；没有电机运行、深度归零或参数写入。

1. Nano 初始持有控制权；GUI 普通 CLAIM 收到 BUSY NANO，并提示 TAKEOVER；未经授权的原始字节写入被拒绝。
2. GUI 明确 TAKEOVER，管理令牌与 9000 OWNER 握手完成后才允许写入。
3. GUI 发送编码后的原始 GET_STATUS，经 Nano 到 MCU，收到序号匹配、CRC 已由生产解析器验证的 0x42 成功回复；未额外包裹串口请求。
4. GUI RELEASE 后由 Nano 持有；GUI 写权限撤销。
5. GUI 再次接管；Nano service action 0 普通申请失败（owner=GUI），action 1 明确接管成功（owner=NANO）；GUI 旧会话关闭，后续写入被拒绝。
6. Nano action 2 释放到 NONE，action 0 再申请回 NANO，均 success=true。

现场最后状态：control_owner=NANO、tx_enabled=false、motor_command_active=false。网关累计写入 704 字节：18 个安全停止帧共 684 字节，加唯一的 20 字节 GET_STATUS。没有电机 RUN 非零指令、ZERO_DEPTH 或新 P0。累计 CRC 错误 3 发生在本轮多次刷写/USB 重连期间，恢复稳定之后未增长。

## 构建与文档

GUI 完整 CTest 21/21 通过；硬件验收程序返回 PASS。手动实机探针为 GUI `rov_tcp_control_hardware_probe`，默认不加入自动 CTest，须在现场测试暂歇时调用。Nano 教程见[原始 TCP 与控制权](../usage/06-nano-tcp.md)，传感器使用见[传感器与记录](../usage/02-sensors-recording.md)。

### 正式目录部署与兼容入口复测

机器人标准目录 `/home/lamost/UnderWater-Robot-ROS2` 已从代码基线 `34f8af3` 更新，13 个包全部编译通过，网关 5、设备 2、分配器 1、相机协议 2 项测试共 **10/10 通过**。新终端加载标准 `install/setup.bash` 后，网关包来自 `install/underwater_mcu_gateway`，ControlOwnership 接口可用。原源码、旧包构建目录与安装目录备份在 `deploy/backup-standard-before-34f8af3-20261010-134748`。

实机检查发现兼容入口原先只等 1.5 秒发现已有后台，会在发现较慢时尝试重复启动；第二网关因端口冲突退出，原串口后台未被关闭。将发现等待上限延长到 5 秒并重编 `sensors_cpp` 后，连续三次 `sensors_sub` 均显示 `Reusing existing robot backend`，同时读到 `raw=1 attitude=1 depth=1`，退出后仍仅原 PID 17784 占用串口。修复文件本机/Nano SHA-256 相同：`391EFECA6DFA577D428C4FD8177DB1FAF4802E961ACCDEB7FB67F0D5DFDB6BAE`。

复测后深度成功样本 **29788**、设备错误 **0**；网关 CRC 累计仍为 3，写入仍为 704 字节，没有额外串口命令。记录在机器人 `deploy/standard-runtime-validation-20261010.log` 与 `deploy/standard-reuse-validation-20261010.log`。

按用户要求，当前测试后台继续运行，9000/9002 可用。当前专门运行传感器后台 `camera_enabled=false`，9001 未开启，本次没有重测相机帧率。下一次正常启动可使用正式目录的 `ros2 launch underwater_bringup robot.launch.xml`，由该 launch 同时管理串口、控制权和相机转发；开机不自动启动转发。

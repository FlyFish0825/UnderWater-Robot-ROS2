# Nano 验证

先 source /opt/ros/humble/setup.bash 和工作空间 install/setup.bash。

## 离线 C++ 测试

```bash
ctest --test-dir build/underwater_mcu_gateway --output-on-failure
ctest --test-dir build/underwater_devices --output-on-failure
```

协议测试使用独立字面量帧，覆盖分片边界、所有单比特破坏、CRC、
混流、垃圾前缀、半帧超时和合法负载中的嵌套帧头。
设备测试覆盖 g 换算、有效位、NaN、电机量纲和心跳 CRC。

## 合成端到端测试（不打开串口）

两个终端均设置：

```bash
export ROS_DOMAIN_ID=189
export ROS_LOCALHOST_ONLY=1
```

终端 A：

```bash
ros2 launch underwater_bringup base.launch.xml gateway_enabled:=false camera_enabled:=false tcp_port:=0 tcp_control_port:=0 namespace:=fixture
```

终端 B：

```bash
ros2 run underwater_bringup telemetry_probe --ros-args -r __ns:=/fixture -p synthetic:=true
```

验证内部帧经过真正设备节点后的数据和 500 ms 超时通知；完成后 Ctrl+C 停止终端 A。
telemetry_probe 是 BUILD_TESTING 打开时编译的 C++ 测试工具，不是常驻业务节点。

## 实机只读测试

两个终端均使用 ROS_DOMAIN_ID=188、ROS_LOCALHOST_ONLY=1，避免与其他实验域混流。
终端 A：

```bash
ros2 launch underwater_bringup robot.launch.xml
```

终端 B：

```bash
ros2 run underwater_bringup telemetry_probe --ros-args -r __ns:=/underwater_robot
```

探针运行约 10 秒，核对网关连接、IMU、深度状态及无效值处理。
默认不要求有电机帧：电机在线与否由实际总线反馈决定，不能由该测试推断。
探针和启动程序都不发送串口协议命令。停止终端 A 后可用 fuser /dev/ttyACM0 确认串口释放。

自动运行上述有限时长检查并停止测试节点：

```bash
bash tools/validate.sh offline
bash tools/validate.sh synthetic
bash tools/validate.sh live
```

[本轮实机结果](../../experiments/2026-10-07-initial-validation.md)。

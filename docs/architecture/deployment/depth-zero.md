# 深度计水面归零

本机只有一只固定的 MS5837-02BA，不提供型号确认、选择或写入。固件上电使用固定空气参考 `P0=100965 Pa`（此前空气实测值）。只有现场确实需要修正参考压力时，才在探头位于空气/水面时显式调用：

```bash
ros2 service call /underwater_robot/sensors/depth/zero std_srvs/srv/Trigger "{}"
```

一个调用会查询当前状态，等待一帧新的有效压力，发送一次 `ZERO_DEPTH`，读回实际 P0，
再等待具有压力、深度和零点有效位的新测量。成功后返回 `success=true` 和实测参考压力/深度；
不会把收到 ACK 单独当作成功，也不会检查或设置型号。

以后确有偏差时，把探头重新放回空气或水面，再调用同一个接口即可。
它只调整零点，不改 OSR、输出频率、水密度和滤波系数，
不发送 IMU 硬件校准命令，也不启用电机。

## 使用边界

- 固件默认 P0 固定为 `100965 Pa`，不从实时气压自动采样，也不在启动或重连时重写。显式 `ZERO_DEPTH` 的现场修正只保存在 MCU RAM；MCU 复位后恢复这个固定空气参考。
- 该固定值来自此前空气中的实测参考，不是实时气压；只有实际观察到偏差时才需要手动修正。
- 重连和普通启动不会自动归零，避免在水下重连时把真实深度抹掉。
- 归零不是完整压力/温度标定，也不标定水密度、传感器增益或安装误差。
- 串口由网关独占；服务异步等待回复，不阻塞接收和电机命令超时处理。
- 命令、目标、序号和回复内容必须匹配；没有自动重发。超时/断线返回失败，保留明确状态。
- enable_depth_commands 默认 true，串口可写仅供这个显式零点服务按需使用；
  enable_motor_tx 仍独立控制电机发送。两者都关闭时串口以只读方式打开。
- 可在 bringup/config/instances/mcu.yaml 中关闭 enable_depth_commands 后重启。

## 本轮验证（2026-10-07）

网关构建成功，1292 项协议检查、25 项电机命令检查、47 项归零检查通过。
实机 Trigger 返回 success=true，型号/零点读回与新测量验证通过。

最后一份检查到的 ROS 测量：
- device_status=2547（0x9F3）。
- fresh、raw_valid、pressure_valid、temperature_valid、depth_valid、zero_valid 全为 true。
- 水面参考压力 P0=101453 Pa。
- 当前压力=101454 Pa，温度约22.17°C。
- raw_depth_m=filtered_depth_m=0.0000990978 m，约0.10 mm，接近水面零点。
- 电机控制 armed=false。

为了加载新接口，原手动启动实例已正常停止，以相同参数在后台恢复。
本次后台启动 PID 记录在 log/depth-zero-live/launch.pid，日志在同目录 launch.log。
不是开机自启动，不要再启动第二个实例抢占同一串口。

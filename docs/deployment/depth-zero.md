# 深度计水面归零

当前探头按原项目已确认的 MS5837-02BA 使用。需要把空气/水面处作为深度 0 时，
在已启动的 ROS 工作空间中调用：

```bash
ros2 service call /underwater_robot/sensors/depth/zero std_srvs/srv/Trigger "{}"
```

一个调用会完成：
1. GET_INFO 核对能力与型号。
2. 型号尚未配置为 02BA 时，设置 0x0105=2，并读回确认。
3. 等待新的有效压力，再发送 ZERO_DEPTH，把当前压力记为水面参考。
4. 读回 P0，并等待归零后的新 0x82 测量具有压力、深度、零点有效位。
5. 返回 success=true 和实际参考压力/深度。不会把收到 ACK 当作测量成功。

以后不准时，把探头重新放回空气或水面，再调用同一个接口即可。
它只调整零点及必要的型号，不改 OSR、输出频率、水密度和滤波系数，
不发送 IMU 硬件校准命令，也不启用电机。

## 使用边界

- 当前固件的型号和 P0 保存在 MCU RAM，掉电后需在空气/水面重新调用；未伪装成 Flash 保存。
- 重连和普通启动不会自动归零，避免在水下重连时把真实深度抹掉。
- 归零不是完整压力/温度标定，也不标定水密度、传感器增益或安装误差。
- 串口由网关独占；服务异步等待回复，不阻塞接收和电机命令超时处理。
- 命令、目标、序号和回复内容必须匹配；没有自动重发。超时/断线返回失败，保留明确状态。
- enable_depth_commands 默认 true，串口可写仅供这个受限服务按需使用；
  enable_motor_tx 仍独立控制电机发送。两者都关闭时串口以只读方式打开。
- 可在 bringup/config/instances/mcu.yaml 中关闭 enable_depth_commands 后重启。

## 本轮验证（2026-10-07）

网关构建成功，1292 项协议检查、25 项电机命令检查、47 项归零检查通过。
实机 Trigger 返回 success=true，型号/零点读回与新测量验证通过。

最后一份检查到的 ROS 测量：
- device_status=2547（0x9F3）。
- fresh、raw_valid、pressure_valid、temperature_valid、depth_valid、zero_valid、model_confirmed 全为 true。
- 水面参考压力 P0=101453 Pa。
- 当前压力=101454 Pa，温度约22.17°C。
- raw_depth_m=filtered_depth_m=0.0000990978 m，约0.10 mm，接近水面零点。
- 电机控制 armed=false。

为了加载新接口，原手动启动实例已正常停止，以相同参数在后台恢复。
本次后台启动 PID 记录在 log/depth-zero-live/launch.pid，日志在同目录 launch.log。
不是开机自启动，不要再启动第二个实例抢占同一串口。

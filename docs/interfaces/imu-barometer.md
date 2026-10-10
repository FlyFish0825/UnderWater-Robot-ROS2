# IMU 内置气压计：ROS 数据与固件要求

## 当前交付状态

2026-10-10 已完成 H750 烧录与真实 `group=133/0x85` 验收：压力、温度、高度与参考压力均有效。WCH-Link 拔下后原始、姿态、气压三组同时上报；气压约 101540.67 Pa。详细记录见[传感器修复与控制权实机验收](../experiments/2026-10-10-sensor-fix-control-acceptance.md)。

## 1. 查看方式

加载工作空间环境并启动原来的 robot.launch.xml 或 manual.launch.xml 后：

```bash
ros2 topic echo /underwater_robot/sensors/imu/barometer --qos-reliability best_effort
```

这个专用话题只发布气压组及其失效通知。完整 IMU 话题仍为：

```bash
ros2 topic echo /underwater_robot/sensors/imu/state --qos-reliability best_effort
ros2 interface show underwater_interfaces/msg/ImuState
```

ImuState 增加了字段，更新后需重新 source install/setup.bash，并重开原先的 echo/订阅程序。

## 2. 分组及新增字段

ImuState 三组共用一种消息结构，每组只填自己的字段：

| group | 十六进制 | 有效数据 |
|---|---|---|
| 128 | 0x80 | 加速度、角速度、磁场 |
| 129 | 0x81 | 四元数、欧拉角 |
| 133 | 0x85 | IMU 内置气压计 |

| 新字段 | ROS 类型 | 含义 |
|---|---|---|
| baro_pressure_pa | float64 | IMU 气压，Pa |
| baro_temperature_c | float64 | 原生气压帧中的温度，°C，不作为水温 |
| baro_height_m | float64 | 模块给出的气压高度，m，参考由模块定义，不冒充绝对海拔 |
| baro_reference_pa | float64 | IMU 气压高度使用的参考气压，Pa |
| baro_pressure_valid | bool | 本条压力字段有效 |
| baro_temperature_valid | bool | 本条温度字段有效 |
| baro_height_valid | bool | 高度有效，且压力/参考气压均有效 |
| baro_reference_valid | bool | 参考气压有效 |

仍需检查 fresh。无效浮点字段是 NaN；group=128/129 的新气压字段始终为 NaN/false。
设备整体 device_status 的压力位为1，不表示任意分组里都有压力数值。

气压组独立缓存，默认500 ms超时；持续到来的加速度或姿态不会延长它的有效期。
过期后两个自定义 IMU 话题发布 fresh=false、相应有效位=false、数值NaN的通知。
标准传感器话题停止发布失效值，消费方也需要自己的超时判断。

测试向量示例（不是实机读数）：

```yaml
group: 133
fresh: true
raw_valid: false
quaternion_valid: false
euler_valid: false
baro_pressure_valid: true
baro_temperature_valid: true
baro_height_valid: true
baro_reference_valid: true
baro_pressure_pa: 101325.0
baro_temperature_c: 22.5
baro_height_m: 12.5
baro_reference_pa: 101500.0
```

## 3. 标准 ROS 话题

| 相对话题（默认加 /underwater_robot） | 类型 | 发布条件 |
|---|---|---|
| sensors/imu/barometer | underwater_interfaces/ImuState | 0x85及其失效通知 |
| sensors/imu/fluid_pressure | sensor_msgs/FluidPressure | 气压有效 |
| sensors/imu/temperature | sensor_msgs/Temperature | 气压计温度有效 |

frame_id 仍为 imu_sensor。header.stamp 是 Nano 接收时刻，device_timestamp_us
是 MCU 收到原生气压帧的时间低32位，分辨率按现有固件为1 ms；不是UTC同步采样时间。
标准消息 variance=0 表示方差未知，没有虚构标定精度。

独立 MS5837 深度计保持自己的 sensors/depth/state、sensors/pressure/fluid_pressure、
sensors/pressure/temperature。气压高度不写入水深，气压参考也不复用MS5837水面零点。
depth/zero 服务不会修改 IMU 气压参考或 IMU 硬件校准。

## 4. 新上行协议

原生IMU 0x32 → imu_sensor 的 baro 四字段 → H750服务层新增 AA5B TARGET=1、CMD=0x85。
VERSION=1、FLAGS=8、LEN=20，总长40字节；CRC与现有AA5B相同。

| Payload偏移 | 格式 | 字段 |
|---|---|---|
| 0 | u32 LE | status |
| 4 | f32 LE | height_m |
| 8 | f32 LE | temperature_c |
| 12 | f32 LE | pressure_pa |
| 16 | f32 LE | reference_pa |

保持原生0x32的四字段顺序，不追加到已有0x80/0x81，旧解析器不需要改变原帧长度。
单位沿用当前固件驱动契约：m、°C、Pa、Pa，没有额外乘100。
状态bit4用于气压组，bit5用于温度；ONLINE跟随raw组，不能代替气压组新鲜度。
ROS另外检查有限数、压力与参考气压大于0，不凭状态位制造测量。

GET_INFO在观察到原生baro样本后增加压力/温度能力位，1E7→1FF；型号仍为0，不做型号猜测。
数据仍受IMU目标的START/STOP_STREAM控制，排队失败不推进baro发送游标。
本轮不修改IMU采样率、算法模式、UART配置或校准。

## 5. 固件与部署

下位机源工程：
F:/file/BaiduSyncdisk/Project/CAN_To_Uart/CAN_To_Uart

本轮改动：
- Core/Src/sensor_service.c：补齐发送与能力位。
- Core/Inc/sensor_imu_baro.h：纯C数据打包。
- tests/imu_baro_stream_test.c：布局、CRC、独立时间、重复样本和无效值测试。
- docs/imu-aa5b-host-protocol.md、docs/imu-sensor.md：同步协议。

已生成：
- build/Release/CAN_To_Uart.elf
- build/Release/CAN_To_Uart.bin

Release使用Flash60468字节（128KiB的46.13%），Debug使用118912字节（90.72%）。
原工程的UART开关、板卡时钟及CAN配置没有更改。当前未检测到烧录器，未烧录。

烧录会重启H750，其MS5837型号/水面零点在RAM中，会丢失。
更新完成且探头在空气/水面时，重新调用 sensors/depth/zero 即可恢复，不要在水下随意归零。
现有硬件记录WCH自带串口会占用PA9/PA10；烧录后需按既有接线要求退出/拔除调试器再检查IMU流。

旧版Underwater_GUI有0x85的长度兼容分支，但没有显示这四个气压字段；本轮只完善ROS端显示接口。

[本轮构建、测试与未烧录边界](../experiments/2026-10-07-imu-barometer-validation.md)。

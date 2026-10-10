# 推进器反馈与心跳的聚合状态：两类帧分别更新，各自有独立有效期。
# 单位：转速 RPM(带符号)、母线电流 A、母线电压 V、MCU 温度 ℃。
# feedback_valid 为假时四个浮点量均为 NaN；heartbeat_valid 为假表示 2.5 s 内无心跳。
std_msgs/Header header  # 最近一次更新时刻
uint64 connection_id  # 串口连接标识，变化表示重连
uint8 node_id  # CAN 节点号 1..8
bool online  # 反馈或心跳任一有效即为在线
bool feedback_valid  # 周期反馈数据是否有效
bool heartbeat_valid  # 2.5 s 内是否收到心跳
bool device_timestamp_valid  # 设备时间戳是否可信
uint32 device_timestamp_us  # 设备侧微秒时间戳
# Values below are NaN when feedback_valid is false.
float64 speed_rpm  # 转子转速 (RPM)，带符号，由 16 位补码解码
float64 estimated_bus_current_a  # 母线电流估计值 (A)，0..10 A 量程
float64 bus_voltage_v  # 母线电压 (V)，0.01 V/LSB
float64 mcu_temperature_c  # MCU 温度 (℃)，-20..150 ℃ 量程
uint8 state  # 驱动器状态码 0..2
bool current_calibration_done  # 电流标定是否完成
bool speed_loop_enabled  # 速度环是否使能
bool voltage_limited  # 是否因电压不足而降额
uint8 feedback_sequence  # 反馈帧序号，用于丢失检测

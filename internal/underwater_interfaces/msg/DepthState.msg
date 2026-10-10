# MS5837 深度/压力传感器状态，由 AA5B target=2、command=0x82 帧解码。
# 单位：压力 Pa、温度 ℃、深度 m；有效性由 device_status 位与各 *_valid 标志共同决定。
# 校验失败的物理量一律为 NaN，不提供占位零值。
std_msgs/Header header  # 宿主接收时间与传感器坐标系
uint64 connection_id  # 串口连接标识，变化表示重连
uint32 device_timestamp_us  # 设备微秒时间戳
uint32 sequence  # 设备帧序号，用于去重
uint32 device_status  # 设备状态位，同时指示各量有效性
bool fresh  # 本帧是否未超时
bool raw_valid  # 原始 ADC 数据(d1/d2)是否有效
bool pressure_valid  # 压力是否有效
bool temperature_valid  # 温度是否有效
bool depth_valid  # 深度是否有效
bool zero_valid  # 水面零点标定是否有效
# Invalid floating-point quantities are NaN, not placeholder zero.
float64 pressure_pa  # 绝对压力 (Pa)
float64 temperature_c  # 水温 (℃)
float64 raw_depth_m  # 未滤波深度 (m)
float64 filtered_depth_m  # 滤波后深度 (m)
float64 surface_pressure_pa  # 零点标定的水面压力 (Pa)
uint32 d1  # 压力传感器原始 ADC 采样值 1，仅 raw_valid 时有效
uint32 d2  # 压力传感器原始 ADC 采样值 2，仅 raw_valid 时有效

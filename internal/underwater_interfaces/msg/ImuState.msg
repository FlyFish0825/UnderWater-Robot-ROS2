# IMU 状态消息：由 AA5B target=1 的 0x80/0x81/0x85 三种帧解码，按 group 分批填充。
# 单位：加速度 m/s²、角速度 rad/s、欧拉角 rad、四元数 wxyz、气压 Pa、温度 ℃、高度 m。
# 无效物理量一律为 NaN，须先检查对应 *_valid 标志；不属于本组 group 的字段也保持 NaN。
std_msgs/Header header  # 宿主接收时间；0x81 时 frame_id 用于标定一致性校验
uint64 connection_id  # 串口连接标识，变化表示重连
uint32 device_timestamp_us  # 设备微秒时间戳
uint32 sequence  # 设备帧序号，用于去重
uint32 device_status  # 设备状态位，同时指示各物理量有效性
# 0x80 raw, 0x81 attitude, 0x85 barometer; other groups' values are NaN.
uint8 group  # 帧组：0x80 原始、0x81 姿态、0x85 气压计
bool fresh  # 本帧是否未超时
bool raw_valid  # 加速度/角速度/磁字段是否有效
bool quaternion_valid  # 四元数是否有效（含模长 0.5..1.5 检查）
bool euler_valid  # 欧拉角是否有效
float64[3] acceleration_m_s2  # 线加速度 (m/s²)，传感器体轴系
float64[3] angular_velocity_rad_s  # 角速度 (rad/s)，传感器体轴系
# Device protocol units; not proven to be tesla.
float64[3] magnetic_protocol_units  # 磁场，保留设备协议原始单位（未证实为特斯拉）
# Device convention, not yet certified as REP-103/145 orientation.
float64[4] quaternion_wxyz  # 姿态四元数，顺序 w,x,y,z，采用设备约定坐标系
float64[3] euler_rpy_rad  # 欧拉角 roll/pitch/yaw (rad)，右手 ZYX 顺序
# IMU air barometer, independent of the MS5837 underwater depth sensor.
bool baro_pressure_valid  # 气压是否有效
bool baro_temperature_valid  # 气压计温度是否有效
bool baro_height_valid  # 气压高度是否有效（需压力与基准同时有效）
bool baro_reference_valid  # 气压基准值是否有效
float64 baro_pressure_pa  # 气压 (Pa)
float64 baro_temperature_c  # 气压计温度 (℃)
float64 baro_height_m  # 气压高度 (m)
float64 baro_reference_pa  # 气压基准值 (Pa)

# 网关原始帧：MCU 串口协议解析后的通用容器，供各设备解码器分派。
# family/command/flags/target 共同决定帧类型，payload 为未经解释的原始字节。
# 约定：header.stamp 是宿主接收时间，设备自身时间见 device_timestamp_us。
# header.stamp is host receipt time, NOT synchronized MCU acquisition time.
std_msgs/Header header  # 宿主接收时间，非 MCU 采样时刻
uint64 connection_id  # 串口连接标识，变化表示重连
uint8 family  # 帧族标识（如 0x5b 传感器、0x55 电机）
uint8 command  # 帧内命令码（如 0x80/0x81/0x85）
uint8 flags  # 子帧类型标志（如 8 表示数据帧）
uint8 target  # 目标设备：1=IMU，2=深度/压力
uint32 sequence  # 帧序号，用于去重与回绕判断
uint32 can_id  # 原始 CAN 标识符（电机通道使用）
bool device_timestamp_valid  # 设备时间戳是否可信
uint32 device_timestamp_us  # 设备侧微秒时间戳
uint8[] payload  # 未解释的原始负载字节，长度由帧类型决定

# 传感器体健康状态：由 AA5B command=0x83 状态帧解码，按 target 区分 IMU 与深度传感器。
# online 需同时满足 device_status bit0 与样例未超时；good_frames/errors 为设备侧累计值。
# sample_age_ms 为 -1 表示设备从未上报过样例。
std_msgs/Header header  # 状态帧的宿主接收时刻
uint64 connection_id  # 串口连接标识，变化表示重连
uint8 target  # 目标设备：1=IMU，2=深度/压力
bool fresh  # 状态是否在刷新窗口内
bool online  # 设备是否在线（bit0 且样例未超时）
uint32 device_status  # 设备状态位，bit0 表示在线
uint32 sample_sequence  # 最近一次样例序号
# -1 means no sample has ever been reported.
int64 sample_age_ms  # 最近样例的年龄 (ms)，-1 表示从未上报
uint32 good_frames  # 累计正常帧数
uint32 errors  # 累计错误数

# 串口网关运行状态：连接、收发统计、电机指令会话与 TCP 控制口信息。
# tx_* 计数表示已写入串口 FD 的字节/帧，并不代表电机已确认接收。
# heartbeat_age_ms 为 -1 表示尚无心跳；buffer 百分比仅在 buffer_status_valid 时有效。
std_msgs/Header header  # 状态发布时刻
string port  # 当前使用的串口设备路径
bool connected  # 串口是否已打开
bool settling  # 是否处于打开后的稳定等待期
bool tx_enabled  # 是否允许向 MCU 发送数据
uint64 connection_id  # 连接标识，变化表示重连
uint64 rx_bytes  # 累计接收字节数
uint64 valid_frames  # 累计通过校验的帧数
uint64 crc_errors  # 累计 CRC 校验失败数
uint64 shape_errors  # 累计结构/长度非法帧数
uint64 discarded_bytes  # 累计丢弃的无法同步字节数
uint64 stale_frames  # 累计因过旧被丢弃的帧数
int64 heartbeat_age_ms  # 距最近一次 MCU 心跳的毫秒数，-1 表示从未收到
bool buffer_status_valid  # 缓冲区占用统计是否有效
uint8[5] buffers_percent  # 五路缓冲区的占用百分比 0..100
string detail  # 人类可读的状态说明
# Written to the serial FD, not an acknowledgement from a motor.
uint64 tx_bytes  # 累计写入串口的字节数（非电机确认）
uint64 tx_frames  # 累计写入串口的帧数（非电机确认）
uint64 motor_session_id  # 网关切换到的电机指令会话号，用于与分配器握手
bool motor_command_active  # 网关是否正在转发电机指令
bool motor_rearm_required  # 是否要求上层重新武装后才接受电机指令
string motor_command_detail  # 电机指令通道的补充说明
string control_owner  # 当前持有控制权的客户端标识
uint16 tcp_control_port  # 控制权服务的 TCP 端口

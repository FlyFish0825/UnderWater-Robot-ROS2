# msg_pkg 兼容话题 /depth_temp 的消息定义：由 sensors_cpp/telemetry 从 underwater_interfaces/DepthState 转发。
# 字段无效时统一填 NaN；fresh 表示本条是否为设备新采样（其余 bool 均以 fresh 为前提）。
std_msgs/Header header
float64 depth      # 滤波后水深（米）
float64 temp       # 水温（摄氏度）
float64 pressure_pa  # 绝对压力（帕）
bool fresh         # 本条数据是否为最新采样，超时后置 false
bool depth_valid   # 深度是否有效
bool temperature_valid  # 温度是否有效

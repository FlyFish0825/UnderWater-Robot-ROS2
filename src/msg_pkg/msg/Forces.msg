# Requested command, not measured force.
# msg_pkg 兼容话题 /forces 的消息定义：期望力/力矩指令（非实测力），由 sensors_cpp/manual 相对 underwater_interfaces 的 requested_wrench 转发。
std_msgs/Header header
float64 fx     # 纵向（x 轴）期望推力（牛）
float64 fy     # 横向（y 轴）期望推力（牛）
float64 fz     # 垂向（z 轴）期望推力（牛）
float64 tyaw   # 偏航（绕 z 轴）期望力矩（牛·米）

# Legacy-facing values: acceleration m/s^2, gyro deg/s, angles deg.
# Raw and attitude validity are independent; invalid fields are NaN.
# msg_pkg 兼容话题 /imu_data 的消息定义，由 sensors_cpp/telemetry 从 underwater_interfaces/ImuState 转发；
# 与内部接口不同，这里保留“度”为单位以兼容旧上位机。
std_msgs/Header header
float64 acc_x   # x 轴加速度（m/s^2）
float64 acc_y   # y 轴加速度（m/s^2）
float64 acc_z   # z 轴加速度（m/s^2）
float64 gyro_x  # 绕 x 轴角速度（deg/s）
float64 gyro_y  # 绕 y 轴角速度（deg/s）
float64 gyro_z  # 绕 z 轴角速度（deg/s）
float64 angle_x # 横滚 roll（度）
float64 angle_y # 俯仰 pitch（度）
float64 angle_z # 偏航 yaw（度）
bool raw_valid       # 原始加速度/角速度是否有效（无效时对应字段为 NaN）
bool attitude_valid  # 姿态角是否有效（依赖已保存的 IMU 参考）

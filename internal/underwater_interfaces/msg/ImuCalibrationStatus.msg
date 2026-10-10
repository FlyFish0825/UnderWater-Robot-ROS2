# IMU 姿态参考标定状态：报告是否已标定、是否正在采集以及最近姿态是否新鲜。
# 角度数组单位为度，顺序 roll/pitch/yaw（ZYX）；无数据时为 NaN。
# saved_at 为 UTC ISO-8601 字符串，calibration_file 为实际使用的绝对路径。
std_msgs/Header header  # 状态发布时刻与 IMU 坐标系
bool calibrated  # 是否已加载或保存了有效的姿态基准
bool collecting  # 是否正在采集静止姿态样本
bool attitude_fresh  # 最近一帧姿态是否仍在超时窗口内
uint32 samples_collected  # 本次采集已接受的样本数
uint32 samples_requested  # 本次采集请求的目标样本数
float64[3] captured_rpy_deg  # 采集到的基准姿态 roll/pitch/yaw (deg)，无基准时为 NaN
float64[3] reference_rpy_deg  # 采集时声明的目标姿态 roll/pitch/yaw (deg)
string saved_at  # 参考文件保存时间，UTC ISO-8601
string calibration_file  # 参考文件的绝对路径
string detail  # 人类可读的标定进度或失败原因

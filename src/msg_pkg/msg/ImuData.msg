# Legacy-facing values: acceleration m/s^2, gyro deg/s, angles deg.
# Raw and attitude validity are independent; invalid fields are NaN.
std_msgs/Header header
float64 acc_x
float64 acc_y
float64 acc_z
float64 gyro_x
float64 gyro_y
float64 gyro_z
float64 angle_x
float64 angle_y
float64 angle_z
bool raw_valid
bool attitude_valid

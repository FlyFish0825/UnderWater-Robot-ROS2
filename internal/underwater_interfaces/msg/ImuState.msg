std_msgs/Header header
uint64 connection_id
uint32 device_timestamp_us
uint32 sequence
uint32 device_status
# 0x80 raw, 0x81 attitude, 0x85 barometer; other groups' values are NaN.
uint8 group
bool fresh
bool raw_valid
bool quaternion_valid
bool euler_valid
float64[3] acceleration_m_s2
float64[3] angular_velocity_rad_s
# Device protocol units; not proven to be tesla.
float64[3] magnetic_protocol_units
# Device convention, not yet certified as REP-103/145 orientation.
float64[4] quaternion_wxyz
float64[3] euler_rpy_rad
# IMU air barometer, independent of the MS5837 underwater depth sensor.
bool baro_pressure_valid
bool baro_temperature_valid
bool baro_height_valid
bool baro_reference_valid
float64 baro_pressure_pa
float64 baro_temperature_c
float64 baro_height_m
float64 baro_reference_pa

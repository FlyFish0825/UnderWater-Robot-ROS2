std_msgs/Header header
uint64 connection_id
uint8 node_id
bool online
bool feedback_valid
bool heartbeat_valid
bool device_timestamp_valid
uint32 device_timestamp_us
# Values below are NaN when feedback_valid is false.
float64 speed_rpm
float64 estimated_bus_current_a
float64 bus_voltage_v
float64 mcu_temperature_c
uint8 state
bool current_calibration_done
bool speed_loop_enabled
bool voltage_limited
uint8 feedback_sequence

# header.stamp is host receipt time, NOT synchronized MCU acquisition time.
std_msgs/Header header
uint64 connection_id
uint8 family
uint8 command
uint8 flags
uint8 target
uint32 sequence
uint32 can_id
bool device_timestamp_valid
uint32 device_timestamp_us
uint8[] payload

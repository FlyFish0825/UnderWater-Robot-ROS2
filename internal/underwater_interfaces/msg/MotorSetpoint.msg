std_msgs/Header header
# Binds commands to the current serial connection, not a previous USB session.
uint64 connection_id
# A new identifier is issued for every explicit enable/rearm.
uint64 session_id
uint64 sequence
bool armed
uint8 node_mask
uint8 run_mask
# Indexed by CAN NodeID - 1, never geometry row after remapping.
int16[8] rpm

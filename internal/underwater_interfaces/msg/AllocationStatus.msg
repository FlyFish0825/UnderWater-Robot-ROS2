std_msgs/Header header
bool armed
bool input_fresh
bool gateway_ready
bool solver_ok
bool rpm_deadband_applied
bool rpm_saturated
uint64 session_id
float64[6] requested_wrench
float64[6] allocated_wrench
# Prediction using the temporary linear curve AFTER deadband, not measured wrench.
float64[6] modeled_rpm_wrench
float64[8] thruster_force_n
int16[8] proposed_motor_rpm
string detail

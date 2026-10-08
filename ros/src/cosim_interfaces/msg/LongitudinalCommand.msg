# Continuous lidar controller output. No lead-vehicle ground-truth input.
std_msgs/Header header
uint64 sequence
string target_vehicle
float64 target_speed_mps
float64 acceleration_mps2
builtin_interfaces/Time measurement_stamp
bool measurement_valid
float64 measured_gap_m
string state

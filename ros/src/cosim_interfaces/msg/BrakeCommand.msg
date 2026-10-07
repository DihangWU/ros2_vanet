# header.stamp 是控制器生成命令的仿真时间。
std_msgs/Header header
string event_id
string target_vehicle
float64 target_speed_mps
# 最终停车净间距（前车车尾到后车车头）。
float64 desired_gap_m
bool simulated

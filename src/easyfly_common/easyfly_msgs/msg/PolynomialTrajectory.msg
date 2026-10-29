# Scheduled polynomial; start time is a command schedule, not a sensor stamp.
# Each quintic piece stores x,y,z rows of six descending power coefficients.
std_msgs/Header header
string navigation_session_id
uint32 trajectory_id
string map_session_id
uint64 map_generation
uint64 map_revision
uint64 validated_map_revision
builtin_interfaces/Time map_stamp
builtin_interfaces/Time validated_map_stamp
builtin_interfaces/Time state_stamp
float64[] durations
float64[] coefficients
float64 max_speed
float64 max_acceleration
float64 max_jerk

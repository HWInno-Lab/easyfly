# Freshness/admission contract for an explicitly selected localization backend.
# ESTIMATING means fresh finite output; it is not proof of visual tracking quality
# or flight readiness. Consumers must match session_id/generation and pose_stamp.
uint8 WAITING_INPUT=0
uint8 INITIALIZING=1
uint8 ESTIMATING=2
uint8 STALE=3
uint8 RESET_REQUIRED=4
uint8 FAILED=5

std_msgs/Header header
string backend_id
string session_id
uint64 generation
uint8 state
string reason
string tracked_frame
bool pose_valid
bool velocity_valid
bool covariance_available
bool tracking_quality_available
builtin_interfaces/Time pose_stamp
builtin_interfaces/Time camera_stamp
builtin_interfaces/Time imu_stamp
uint64 camera_forwarded
uint64 camera_rejected
uint64 imu_forwarded
uint64 imu_rejected
uint64 pose_rejected
string last_rejection

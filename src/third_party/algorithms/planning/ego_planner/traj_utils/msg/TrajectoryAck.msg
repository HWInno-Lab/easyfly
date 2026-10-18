# Acknowledgement emitted only after traj_server has validated and activated a
# complete B-spline candidate.  PositionCommand is deliberately not an ACK.
std_msgs/Header header
string navigation_session_id
uint32 trajectory_id
uint8 status
uint64 activation_sequence
string source_node

uint8 STATUS_ACCEPTED=1

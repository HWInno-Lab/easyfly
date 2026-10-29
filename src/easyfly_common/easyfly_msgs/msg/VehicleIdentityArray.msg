# Original image header; consumers must also check valid and producer session.
std_msgs/Header header
string camera_id
string session_id
string gallery_sha256        # Empty for discovery-only mode; explicit reference manifest otherwise
string model_sha256
bool valid
string reason
VehicleIdentity[] identities

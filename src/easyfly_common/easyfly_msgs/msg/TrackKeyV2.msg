# EXT-02 versioned contract; V1 wire definitions remain unchanged.
unique_identifier_msgs/UUID tracker_session_id
uint64 tracker_epoch
int64 raw_id
bool stable_id_valid
int64 stable_id
# Producer-owned compatibility aliases; never truncate raw_id.
bool legacy_id_valid
int32 legacy_raw_id
int32 legacy_stable_id

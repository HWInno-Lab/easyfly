# Gazebo camera scene-time compatibility

Private Gazebo Classic 11.10.2 sensor library for `easyfly_gzserver` only.
The system installation and other Gazebo executables are unchanged.

The original non-lockstep image scheduler treats equal render-scene time as a
world reset, clears measurement history, then admits another render at the same
stamp. Under a faster renderer and bounded CPU contention this generates different
pixels with the same native/ROS timestamp. Preserve history for positive equal
image time; retain native behavior for rollback, zero time, other sensors and
strict-rate cameras. No frame is retimestamped, no ROS filter drops the evidence,
and no freshness or rate limit is relaxed.

All source/header snapshots in `upstream/` retain Apache-2.0 notices and exact
bytes from `gazebo11_11.10.2`, source archive SHA256
`51976f29e375463e3bbf1343b7532b038728f13389e94e78b2fa8271911b2dcb`.
`SHA256SUMS` records every imported file. Only `src/sensor.cpp` derives from
`upstream/Sensor.cc`. Public object layouts remain unchanged. Internal factory
binding matches upstream to avoid a name collision with ROS `gazebo_msgs`.
The private server requires the compatibility marker symbol, preventing a silent
fallback to an unpatched sensor library.

The native clock regression executes the actual scheduler with only its scene
clock replaced by a deterministic input. It checks equal-time history, the
sampling period, forward progress, real rollback and zero-time initialization.
It is separate from the actual rendered full-stack acceptance evidence under
`artifacts/v01-v16-20261025`; diagnostic renders do not substitute for installed
ordinary runs. CPU-only full-stack rendering under four-core/two-worker pressure
has a separately recorded 250ms image-gap failure; this scheduler fix does not
claim to repair renderer throughput.

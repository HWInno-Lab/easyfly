# Gazebo IMU measurement snapshots

This independently named ROS sensor plugin derives from `gazebo_ros_pkgs` 3.9.0;
original source, Apache-2.0 notices and hashes are retained locally. It does not
replace a system library. Curated MID-360 runtime models select
`libeasyfly_gazebo_ros_imu.so`. Since the V-16 full-stack regression on
2026-10-26, the legacy MID-360 entry selects the same plugin: its original
strict-rate path emitted zero source stamps. Only the plugin filename changes;
legacy physics, noise, rate and frame parameters remain unchanged.

Gazebo 11.10.2 strict-rate `Sensor::Update` does not advance `LastUpdateTime`.
The upstream ROS plugin reads that scheduler time, causing repeated zero stamps
in the native lockstep fixture. This derivative copies a single mutex-protected
`ImuMessage` snapshot, including its physical measurement timestamp, orientation,
angular velocity and acceleration. It skips uninitialized and byte-identical
snapshots; it does not fill, interpolate, retimestamp or suppress backwards or
conflicting measurements. Configured physics, rate, noise and covariance remain.

The default reliable writer history is bounded at 1000 instead of the upstream
5 samples, matching the OpenVINS guard reader. In a reproduced managed run, an
independent reliable subscriber saw a maximum source gap of 9 ms while the guard
lost 111 ms and correctly latched a fault. Increasing retained writer history
addresses this observed delivery pressure; it does not guarantee lossless
communication under arbitrary load. The guard's 25 ms limit stays unchanged.
An actual-QoS depth diagnostic is emitted once at startup.

Native A/B validation uses the same private Gazebo wrapper and camera with the
original versus curated IMU plugins. The original repeats zero stamps; the
corrected strict-rate fixture has increasing measurement time, unchanged noise
checks and clean exit. Strict-rate mode itself may run the IMU near the physics
rate rather than its requested 200 Hz; the fix does not resample this stream.
Production simulation remains in its previous non-strict mode.

Sampling and ROS publication now use separate execution paths. The sensor callback
copies each complete measurement into a FIFO bounded at 1000 items; one joined
worker publishes the original message in order. DDS discovery or a stalled
reliable writer cannot block the measurement callback. Queue overflow or a publish
exception latches output stopped and logs an error; there is no silent overwrite,
retimestamp or automatic recovery. Normal shutdown disconnects sampling, drains
the finite queue and joins the worker before releasing the ROS publisher. Existing
consumer freshness/continuity gates remain unchanged.

A test-only interposer blocks one actual `rcl_publish` call. With a 300 ms block,
the original synchronous plugin lost 302 ms of source time; the queued plugin
retained all 461 observed measurements with a 6 ms maximum source gap. A 7-second
block filled the queue and verified the stopped-output latch and normal cleanup.
These are controlled native ROS/Gazebo regressions, separate from ordinary
full-stack and flight qualification. The interposer is not installed.

## Opt-in producer timing

Set `EASYFLY_IMU_TIMING_TRACE` to an absolute file prefix in an existing output
directory before launching Gazebo. The plugin records host `CLOCK_MONOTONIC`
times at callback entry, after the measurement snapshot, and before/after the
publish call, together with the original measurement stamp. It writes
`PREFIX.PID.NODE.csv` during normal teardown. Tracing is disabled by default.
The buffer retains at most 60,000 callbacks, counts discarded records, and does
not perform per-sample file I/O or publish extra ROS messages. Result codes are
0 (uninitialized), 1 (identical snapshot), 2 (published), and 3 (publication
failure). The header also records `publish_failures`; a nonzero count invalidates
input evidence. Entry/snapshot times refer to the sensor callback, while publish
times refer to the worker, so queue delay remains measurable.

Pair this with `tools/algorithms/probes/openvins_input_trace.py`, then analyze
the producer CSV and observer JSONL using
`tools/algorithms/probes/analyze_imu_delivery.py --producer CSV --receivers JSONL
--output REPORT.json`. This requires both traces from the same host monotonic
clock. Publication return alone is not proof of subscriber delivery.

Upstream references:
- https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.9.0/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp
- https://github.com/gazebosim/gazebo/blob/gazebo11_11.10.2/gazebo/sensors/Sensor.cc
- https://github.com/gazebosim/gazebo/blob/gazebo11_11.10.2/gazebo/sensors/ImuSensor.cc

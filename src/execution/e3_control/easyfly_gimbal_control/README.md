<!--
  SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
  SPDX-FileCopyrightText: 2026 贾国源
  License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_gimbal_control/licenses/.
  This revision offers no Apache-2.0 option for the listed original code.
-->
# Simulation-only gimbal

This backend cannot control physical hardware. It never publishes aircraft
setpoints. Angles are degrees in body FLU: positive yaw left, positive pitch up,
positive roll around forward. `/gimbal/set_angle` accepts finite bounded commands;
success means accepted, not actual arrival. Observe `/gimbal/state` for movement.
Each axis is limited to `rate_deg_s`; out-of-range requests are rejected, not clamped.

`/gimbal/set_tracking` requires a nonnegative explicit target ID and fresh odom
`/target_positions` and `/uav_state` evidence. Full roll/pitch/yaw orientation is
used. Missing, duplicate, stale, invalid or unreachable selected targets stop motion
and disable tracking. Recovery never reactivates the old command. A manual accepted
angle command supersedes tracking. Disabled tracking holds current angles.

ROS source age and monotonic receipt age are both checked; zero/backward/frozen
ROS time stops old motion. State headers use ROS time (zero time is not trusted).
The simulator's heartbeat is independent of ROS time.
`/gimbal/simulate_connection` (`std_srvs/SetBool`) is **only fault injection**:
false stops heartbeat and pending motion, then connection expires at the configured
steady timeout; true reconnects while holding current angles. It is not a device
protocol. Device disconnection rejects new commands.

All limits/timeouts are startup configuration. Use the supplied `config/gimbal.yaml`
with `--ros-args --params-file ...`, or `ros2 launch easyfly_gimbal_control
gimbal.launch.py use_sim_time:=true`. No physical backend or accuracy claim exists.

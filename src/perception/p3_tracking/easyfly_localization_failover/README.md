# Explicit dual localization / external vision

This explicit simulation profile passed native acceptance in both copy and symlink installations. It is not enabled by existing launches.
See `artifacts/stages1-3-20261028/failover-signoff.json` in the workspace for
original inputs, physical quality, native PX4 fusion, faults and normal shutdown.

The reviewed simulation profile runs FAST-LIO and independent monocular OpenVINS
on original Gazebo LiDAR, camera and IMU messages. Start the normal FAST-LIO
`perception_sitl.launch.py` with `timesync_mode:=PASSTHROUGH` and a 100 Hz simulation
clock (the explicit debug-world entry exposes `debug_clock_rate_hz:=100`). Then run
`ros2 launch easyfly_localization_failover failover.launch.py enable_external_vision:=true
px4_pid:=PID px4_binary:=ABSOLUTE_NATIVE_PX4 px4_rootfs:=ABSOLUTE_PRIVATE_ROOTFS`.
The second launch owns only its four processes; exiting an estimator never
terminates PX4 or the flight control bridge.

Before activation, explicitly fetch the complete MAVROS parameter catalogue with
`/mavros/param/pull` and `force_pull: true`: a partial cache can otherwise report
a successful pull while required parameters remain undeclared. A disarmed
operator must then configure and read back PX4
`EKF2_EV_CTRL=3`, `EKF2_EV_NOISE_MD=1`, `EKF2_EVP_NOISE=0.3`,
`EKF2_EVA_NOISE=0.15`, `EKF2_EV_DELAY=0`, `EKF2_EV_QMIN=0`, and all three
`EKF2_EV_POS_X/Y/Z=0` (poses already represent the base origin). No parameter changes are made by this
node, especially in flight. MAVROS must use PASSTHROUGH sample time and its
vision-pose TF listener must be disabled. GPS/barometer remain available: this
profile is not a GPS-denied acceptance claim.

The native clock witness verifies the exact local PX4 1.16.2 process UID, PID/start
time, binary hash, rootfs and version. It reads original uORB `timesync_status`
snapshots through that binary’s native IPC client. Activation requires 510
distinct consecutive 10 Hz clock observations with <10 ms round trip, ≤20 ms
offset, bounded source gaps and fresh receipt; ambiguity or process replacement
refuses admission. This takes at least 51 simulated seconds. PX4 substitutes its
receipt time before convergence; the witness prevents EV output during that
phase. A 100 Hz simulation clock keeps the measured offset within the bound.
After convergence, low-RTT raw jitter is judged against the pinned native
100 ms deviation/reset threshold. The actual filtered estimate used by native
`sync_stamp()` must stay within 20 ms and change by no more than 1 ms per
observed update; the original cadence and freshness guards remain enforced.
The initial 510-observation qualification retains the stricter 20 ms raw bound.
One distinct native high-RTT rejection may retain the proven
filter only when its estimate is exactly unchanged. It never increments the
accepted count or refreshes the last good receipt. The entire interruption is
bounded to 0.5 simulated seconds / 0.8 steady seconds from the last good sample;
a second rejection, changed estimate/protocol or expired receipt resets proof.
An unchanged native snapshot can preserve the converged filter during the same
fixed interval before that delayed exchange arrives. The witness itself must
continue reporting within 0.4 steady seconds. Duplicates cannot renew either deadline. This follows the pinned native filter
branch that ignores RTT ≥10 ms before considering offset deviation. Bounded IPC
snapshots ahead of queued original ROS clock callbacks are deferred without
counting or renewing freshness. No estimator or truth samples are generated.

Before activation, explicitly initialize the monocular estimator with controlled
motion along both horizontal axes in verified clear space. The native acceptance
uses a 0.6 m square at 1.5 m height, with no EV output. Pure vertical initialization
produced a captured 3 m ground drift and was correctly refused; do not relax the
ground-fit limits or infer scale from FCU reference samples.
After both estimators initialize, land and wait for fresh settled ground poses,
then call `/localization/failover/activate` (`std_srvs/srv/Trigger`). The one-shot
activation freezes yaw and translation gauges using source-stamped FCU position
and independently stamped IMU attitude, while no EV output has yet been sent.
All reference buffers are then discarded and never used to refit. IMU-to-base
translation is `[0.011, 0.02329, -0.12412]` m for the installed gazebo_mono model.
The manager publishes no global TF, `/odom`, or replacement flight state.

Two seconds of simultaneous agreement (20 cm / 0.12 rad bounds) qualifies both
sources. Primary loss can select an already qualified backup; return to primary
requires a new agreement window and at least three seconds of residence. Source
pose age is limited to 0.25 s and steady-clock pose receipt age to 0.35 s.
These limits precede the native OpenVINS receiver’s 0.5 s interruption latch.
Original input health permits at most 0.15 s lead over the quantized simulation
clock; EV measurement admission retains the stricter 0.02 s future bound. At most five
seconds without independent agreement is allowed. Both lost stops output;
recovery requires both to qualify again. A pause exceeding a native receiver’s
own deadline remains refused and requires a new ground session; the manager
does not override the estimator’s rejection. Identity changes, resets, coordinate
discontinuities and clock rollback require a new ground session. Disagreement
stops the whole session because two sources cannot identify which one is wrong.

Only `/mavros/vision_pose/pose_cov` is written, with original selected timestamps
and unknown covariance (NaN), because the fixed ground alignment has unestimated
uncertainty. Verified PX4 parameter-noise mode supplies the explicit observation
noise; this is not an estimated global covariance. MAVROS 2.14.0 converts ENU/FLU
to NED/FRD. Only position fusion is selected; velocity and yaw aiding are excluded. PX4 can apply
a further GNSS noise lower bound while GPS is retained, so 0.3 m is the selected
EV noise parameter, not a claim that the final EKF observation variance is
exactly 0.09 m². Original uORB clock translations and EKF aiding must be audited.
Per-user/domain locking plus DDS publisher checks reject competing writers. A
ROS graph is a trusted local boundary, not a security boundary against malicious
publishers. Fault checks run on a steady timer even if simulation time freezes.

`redundant_waypoint --execute --target X Y Z --output NEW_DIR --ros-args -p use_sim_time:=true`
owns one horizontal native waypoint (10 m / 0.6 m/s limits, settled offboard
hover required). It retains a durable exact goal UUID, cancels only that goal on
EV admission loss and drains its terminal response. Recovery never resubmits an
old mission. An unresolved transaction blocks new commands in the same native
controller session. This wrapper adds no obstacle planner; choose a verified
clear path. Existing independent missions are not silently taken over. Takeoff
and Land remain native explicit operations and are available outside this wrapper.

Hardware, independent sensor failures beyond this shared-IMU simulation, automatic
process restart, unrestricted solo operation, and GPS-denied flight are outside
this profile. The LiDAR fixture uses a 20,000-point spherical scan; acceptance
does not assert the hardware MID-360 field of view. The retained canonical input
audits compare all fields at original stamps, excluding only CDR padding.

# D5 precision landing (simulation first)

Fixed downward camera, static AprilTag 36h11 board only. No physical-device or
moving-pad support; the simulated gimbal is independent of camera calibration.
This package provides observations, a pure policy, and an accepted-only gateway.
The existing FlightController hosts `/precision_land` and remains the sole
aircraft-command publisher. See [D5 contract](../../../../docs/development/d5-landing.md).

Defaults do not enable precision flight. Supply explicit tag ID/size, unit camera
quaternion, FLU lever arm, odom plane height, and flight bounds. All are startup
parameters; the simulation example is not a hardware safety configuration.
Both `precision_descent_floor_height` and `precision_handoff_height` must be
explicit, finite, and satisfy `0.15 <= floor < handoff <= 1.0` metres above the
observed board. The floor bounds generated descent targets, not actual flight.
The current simulation example uses handoff 0.6 m / floor 0.5 m to retain the
marker and its white margin within the camera field of view before handoff.

```sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch easyfly_precision_landing precision_landing.launch.py \
  config_file:=$(ros2 pkg prefix easyfly_precision_landing)/share/easyfly_precision_landing/config/d5_simulation.yaml
```

This starts no flight controller by default and sends no goal. The existing
controller must separately receive matching explicit parameters. Only use
`start_flight_controller:=true` when no flight controller is already running.
The supplied launch requires `use_sim_time=true`. Calibrate the example's plane
height and fixed camera against the actual simulation fixture before use.

`PrecisionLand(tag_id, timeout)` provides real feedback and terminal results.
Python uses `precision_land(tag_id, timeout)`; C++ accepts a `PrecisionLand::Goal`.
Formal mission YAML accepts
`type: precision_land` with explicit `tag_id`. `/trigger_landing` accepts only
NaN XY/default speed and reports admission, never landing success. Admission
timeout can leave a pending goal: do not blindly retry.

Tag loss stops descent; trusted owned OFFBOARD may hold while reacquiring.
Five consecutive distinct valid observations confirm reacquisition; intermittent
good frames neither move the first loss hold point nor renew its deadline.
Fresh alignment is still required afterwards before descent/handoff.
The transform retains at most 32 source frames awaiting a trusted vehicle
snapshot. It chooses the newest still-fresh frame that fits the original state
pairing window, preserving each original source stamp and first receipt TTL.
Ingress and published watermarks are separate: output is strictly increasing,
older candidates are discarded after a successful pair, and newer candidates
may remain pending. Individual expiry removes only that candidate; invalid new
evidence, geometry failure, clock invalidity and reader rebuild purge history.
Bad geometry never falls back to older frames. Late state cannot revive expired
evidence or authorize an extrapolated vehicle pose.
If local steady processing continuity exceeds the existing source-age budget
(0.3 s by default), the state, target or watchdog entry rejects its triggering
sample, clears evidence and rebuilds both volatile readers. Retired reader
callbacks cannot refresh the new generation; recovery requires newly received
state and target evidence with the original freshness/pairing checks. Capability
queries cannot renew processing continuity. The pure `/landing_target` reader
uses best-effort, volatile, depth 1 (including after rebuild) to prefer current
observations over ordered retransmission. It accepts reliable or best-effort
writers. The state reader remains reliable depth 10 and guidance output reliable
depth 1; freshness, replay rejection and loss protection are unchanged.
RMW wall timestamps are diagnostic only. This protects local reader
blackouts, not upstream acquisition timestamps or every form of transport delay.
State failure or external takeover stops platform output. At stable low height,
the controller captures a fixed hold point and a source-time barrier, then waits
for fresh original MAVROS mode evidence on `/flight_status` strictly after that
barrier. Aggregated `/uav_state` timestamps cannot authorize handoff. During
`handoff_wait_mode`, ordinary loss, cancellation and total timeout rules still
apply; new external mode evidence stops output even before aggregate state catches
up. Only then may an exact internal ordinary Land goal handle final touchdown. Parent
ownership persists until its child drains. Unknown mode-command results keep an
independent fault lock even after the public action ends.

Build/test:

```sh
colcon build --packages-up-to easyfly_sim_launcher --symlink-install
colcon test --packages-select easyfly_precision_landing easyfly_flight_control
colcon test-result --verbose
```

Unified installed entry: `ros2 run easyfly_sim_launcher d5_simulation`.
It defaults to startup only: no takeoff, landing or mission Goal. Use only for
local simulation, never hardware or an existing flight stack. First create an
empty evidence directory; `--px4-dir` must be a compiled real ELF PX4 tree.
The entry generates its own independent rootfs with an explicit `px4 -w/-s`
wrapper; it never copies or changes the original PX4 parameters. Shared plugins,
Tools and startup scripts are inputs by convention, not an OS read-only mount.
Example on this workspace (adjust the source PX4 path on another machine):

```sh
d5_session_dir="$(mktemp -d /tmp/easyfly-d5.XXXXXX)"
ros2 run easyfly_sim_launcher d5_simulation \
  --directory "$d5_session_dir" \
  --px4-dir /home/jia/PX4-Autopilot-1.16.2 \
  --use-bundled-dds-profile
```

Wait for `D5_READY`, then Ctrl-C to stop. This is startup readiness, not permission
to fly or proof of a visible tag on the ground. `--startup-duration 10` instead
observes for ten seconds after readiness, then checks all process exits.
Flight/command/observation capture is capped at 10,000 entries per stream;
numbered camera files at 100, while the latest image continues to update.
`--prepare-only` creates inputs without starting Gazebo/PX4/ROS nodes.
For a new, empty evidence directory, explicit `--mission` flies the formal demo;
`--direct-action` flies the direct action check. These modes are mutually exclusive.
Source `scripts/sitl_smoke.py` now delegates to this same engine. Its path remains
available, but old manual staging is refused and flight is no longer implicit.
`--px4-staging` accepts only a generated manifest with validated path/wrapper
bindings; the manifest is structural isolation evidence, not authentication.

`--mission` runs the actual formal YAML through the mission executor and SDK:
takeoff, hover, precision landing. It requires measured stable hover, all three
steps completed, real success, exact mission UUID release and fresh disarmed
feedback. The DDS option freezes a profile for the entire isolated process tree;
it does not change global settings or production defaults. The profile budgets
64 MiB SHM / 4 MiB messages and retains UDPv4 for this same-host simulation.
Alternatively supply an explicit `--dds-profile` XML instead of the bundled
profile flag. Namespace/default participant/transport references and image
budget are checked, not just XML syntax. Before bridge/flight start, the owned
simulation must pass its exclusive reservation and publish a clock; an existing
flight/perception graph is refused. This is not a global ROS security boundary.
SIGINT/SIGTERM trigger owned cleanup; interrupted motion attempts exact cancel
and terminal draining before stopping the stack. SIGTERM reports failure, never
completion. Supervisors should run the installed native executable directly or
signal its process group, not assume a signal to the `ros2 run` wrapper PID is
forwarded. Unconfirmed remote cancellation must never be interpreted as stopped.

The unified simulation entry no longer assumes `world -> odom` z is `-0.10 m`.
It computes the upright fixture's ground height from all supported collision
primitives and explicitly resolved MID360/GPS assets (unknown URI, ambiguous
frame/pose/scale or mesh collision is refused). It queries the generated world's
stamped `GetEntityState` for `easyfly_iris::base_link` and pairs ten fresh, distinct,
disarmed stationary odom samples over at least 1.8 steady seconds. Stable offsets
are frozen before precision nodes start; generated transform/flight plane heights
are revalidated together. The tag plane is the tilted cell top face, not its model
origin. No tag estimate fits its own plane, no set-state service is called, and
there is no in-flight refit. Failure/unknown query, new untrusted state, clock
rewind or unstable reference prevents precision startup. `--prepare-only` emits a
nominal ground-zero configuration only, not a calibrated flight-ready contract.
This is isolated simulation calibration, not automatic hardware calibration.

For sparse saved-frame diagnosis, the source-only utility is:

```bash
python3 src/execution/e3_control/easyfly_precision_landing/scripts/replay_tag_frames.py \
  --artifact /absolute/path/to/sitl-evidence --output /absolute/path/to/new-replay
```

It requires a local C++ compiler, `pkg-config opencv4` and this source workspace;
it calls the actual estimator and preserves raw-image hashes and ambiguity.
`--calibration-json` optionally supplies captured CameraInfo evidence. Inferred
SDF intrinsics or another run's intrinsics are labelled, not certified as the
original frame's calibration. It currently diagnoses fixture ID7 only. Same-stamp
vehicle samples yield an envelope, not a guessed first receipt or actual pairing.

`--transform-diagnostics` is off by default. On this installed Humble version,
it retains RMW source/receive/take wall timestamps, callback duration and pairing
waits; observer output timing also stays in `result.json`. It performs one native
take, with unchanged QoS and source/receipt safety clocks. Wall-clock rollback
and unsupported timestamps are not valid latency evidence; these are software
diagnostics, not sensor acquisition guarantees. S21's old delay did not recur in
S24/S30 and remains unclosed, not fixed merely because the tasks passed.
The flag now also enables the flight controller's read-only, default-off
`precision_timing_diagnostics`. `landing_diag` retains source-correlated native
take, target/state entry, pending/selected pairing, invalidation and publication
start/end events. Publication end includes `succeeded`; an exception is not a
successful output. `precision_frame_diag` records guidance callback entry,
native receive metadata, flight mutex wait, admission outcome and session state;
its `generation` is current, while `callback_generation` identifies the reader.
Callback entry is not a native take timestamp or proof of control-loop use.
Each event ring is capped at 256 with cumulative `dropped_events`. Python flushes
at 1.05 seconds (still no more than once per second), and drains its final tail
on shutdown. Missing/dropped/ambiguous stages remain unknown. Source stamps may
repeat after clock rewind: correlate generation, first receipt and event order,
not just source stamp. Observer receipt steady time is retained in `result.json`;
cross-process steady subtraction requires the same host/clock origin. A consumer
may receive while `publish()` is still returning, so its offset from publish end
can legitimately be negative. These traces remain observational and opt-in.
SITL27/30 passed via the installed entry with 0.2 m board, all cameras 30 Hz,
explicit DDS and 0.6/0.5 m band. Full algorithm load, long-term
robustness and hardware remain unverified. Earlier failures remain in the record.

For the larger generated visual fixture, supply `--fixture-scale 2
--handoff-height 0.6 --descent-floor-height 0.5`. This is a separate explicit
simulation configuration, not evidence that the 0.2 m example is robust.
For load diagnosis only, `--unused-camera-rate 1` reduces the generated fixture's
unused forward color/depth cameras to 1 Hz (default 30 Hz). Down-camera rate,
physics and lidar remain unchanged. A pass here cannot certify full sensor load.

动态相机和低速移动板的显式配置、限制与当前验收状态见 [DYNAMIC.md](DYNAMIC.md)。

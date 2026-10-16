# EasyFly PX4 SITL

Gazebo Classic simulation uses the guarded project-local PX4 compatibility
build, a single-vehicle reservation, and the maintained EasyFly Iris SDF.
Missing or mismatched compatibility inputs fail closed; the original PX4
installation is not patched by this launcher. No automatic flight goal is sent.

## Optional magnetometer sample interval compatibility

The pinned PX4 1.16.2 magnetometer reset filter is initialized with a 10 ms
EKF step, but updated only when magnetometer samples arrive. The default native
magnetometer publication rate is 15 Hz. Using the prediction interval for these
arrivals stretches the intended 0.09 s filter time constant, producing a stale
body-frame field during turns and a large heading correction at airborne
magnetic alignment. The retained S4 ULog and native update-expression replay
record this failure separately from scene qualification.

An explicit simulation-only build can correct the interval using successive
native magnetometer sample timestamps. It retains the 0.09 s constant, noisy
measurements, all fusion/reset conditions, and the existing HRT repair:

```bash
ros2 run easyfly_px4_sitl prepare_px4_hrt \
  --px4-dir /home/jia/PX4-Autopilot-1.16.2 \
  --output-dir /home/jia/easyfly_ws/build/easyfly_px4_mag_compat \
  --mag-lpf-interval
export EASYFLY_PX4_HRT_COMPAT_DIR=/home/jia/easyfly_ws/build/easyfly_px4_mag_compat
```

The output must be a new directory. Preparation checks the source, original
archive, filter header and patch hashes; compiles the original and corrected
update expressions with the actual PX4 AlphaFilter; then replaces one object
in a private EKF archive and relinks a private binary. No upstream source or
previous compatibility build is overwritten. Ordinary tests never build PX4
or launch flight. The option is explicit; HRT-only builds remain supported.

A scene qualification can require `verification.required_px4_compatibility:
[mag_lpf_interval_v1]`. The catalog derives that feature from the selected,
fully checked build or private staging manifest. A missing feature makes its
runtime resources incomplete and managed startup fails before children start.
An offline PASS still does not certify flight readiness.

## Managed scenes

Both entries accept an optional `scene` ID and a `scene_profile` compatibility
label (base_sim for the base entry; full_perception/navigation_dev/landing_dev
for perception). The common resolver checks assets, provenance, registration
and compatibility before acquiring the session reservation or starting children.
It supplies the world, model search roots, aircraft SDF and spawn xyz/yaw.
Unknown, draft, incomplete or conflicting selections fail closed.

An empty scene retains the original defaults and explicit low-level world/model
debugging, including the independent D5 fixture. An empty world argument means
the historical effective default, empty.world or perception_demo.world.
Nonempty scene and world are mutually exclusive. Existing PX4 compatibility,
session ownership, timing and profile safety parameters remain in effect.

## Scoped delayed startup on ROS Humble

Both launch entries use `ScopedTimerAction` for staged startup. Humble's plain
`TimerAction` retains context locals but not scoped launch configurations or
environment variables. In an isolated profile include, the parent scope can
end before a delayed action resolves `headless` or another input.

The project wrapper snapshots the action's inputs, restores them only while
visiting its delayed actions, and then restores the caller's scope. Original
startup periods and shutdown cancellation are unchanged. Nested scope,
environment isolation and cancellation have process-free LaunchService tests.
FAST-LIO resolves its delayed component's boolean clock parameter before its
own include scope ends; it does not acquire a dependency on this SITL wrapper.

## Explicit simulation IMU noise densities

The EasyFly model explicitly sets `gyroscopeNoiseDensity` to
`0.00008726646259971648` rad/s/sqrt(Hz), equivalent to the IIM-42653 typical
0.005 deg/s/sqrt(Hz) spectral density in DS-000529, gyroscope specifications.
The separate 0.05 deg/s RMS figure at 100 Hz is not a density. The installed
Gazebo plugin's default confused these figures, making a stationary simulated
vehicle repeatedly exceed PX4's gyro-vibration rest threshold.

The accelerometer also explicitly uses `0.0006864655` m/s^2/sqrt(Hz):
DS-000529 table 2 gives 65 ug/sqrt(Hz) for X/Y and 70 ug/sqrt(Hz) for Z.
The scalar plugin conservatively uses the larger value on every axis, converted
with standard gravity 9.80665 m/s^2. Its default `0.00637` incorrectly treats
0.65 mg RMS at 100 Hz as a spectral density. The reference and model hash are
recorded in the model package's `licenses/runtime/imu-noise-reference.json`.

These model-level repairs keep nonzero noise. They do not modify system plugins,
PX4 admission/protection parameters, random walk, bias settings, or hardware
configuration. Legacy launches retain their 0.83 m spawn default; both registered
warehouse scenes explicitly select 0.10 m in their scene manifests. This typical
simulation reference is not a measurement or calibration of a real device.
See the [TDK product and datasheet entry](https://www.invensense.tdk.com/en-us/products/inertial-sensors/industrial-motion/iim-42653).

The SDF regression checks physical inertias, camera/TF contracts and the unique,
finite, nonzero densities including degree-to-radian and micro-g conversion. Omission,
zero/noise suppression, RMS-as-density and unrelated IMU overrides fail it.
Cold-start native-state checks are separate from flight and UI acceptance;
passing these checks is not hardware readiness or proof that every historical
GPS or shutdown fault has been eliminated.

## Native sampling and managed process shutdown

The curated Iris publishes the original noisy 1000 Hz stream on `/imu_native`.
The private IMU relay checks contiguous native sequence/timestamps, averages four
samples, and limits acceleration to the representable PX4 signed 16-bit FIFO range
before the existing 250 Hz MAVLink sampler. Clipped impacts do not preserve the
full impulse integral. The model package's `imu-sampling-reference.json` records
the sampling and clipping evidence. Missing native samples stop the relay.

Managed scenes use the private `easyfly_fastdds_compat` runtime and the complete
private Gazebo transport build. They preserve the caller's DDS configuration and
add six named component-loader reply profiles; ordinary writer defaults remain
unchanged. Their package READMEs describe native lifetime fixes and regression
coverage. EGO consumes process stop signals on its main thread and destroys its
nodes before shutting down the ROS context. Native tests are separate from scene
acceptance: current scene evidence must also show successful tasks and normal
exit of every owned process.

Draft scene sensor inspection can supply `debug_spawn_pose:="x y z yaw"` together
with an explicit raw `world` and `debug_model_path`. The four values must be finite.
Managed `scene` selection rejects this override; legacy defaults remain unchanged.
`scene_sitl --bootstrap` uses this explicit route for geometry-reviewed draft
spawns and forbids flight goals while collecting alignment and sensor evidence.

Managed scenes may explicitly set `navigation.grid_footprint_mode` to
`observed_cells` for connected indoor environments. The default remains
`cluster_boxes`; CLI overrides of a managed scene are rejected before startup.
The selection is passed to the semantic node and its actual value is checked by
the explicit scene acceptance probe alongside the loaded library hashes.

Dense scenes may explicitly select `runtime.heartbeat_rate_hz: 5` (default 1).
After the owned, pinned PX4 process reports successful startup, the launch parent
uses its native `px4-mavlink` client to configure only HEARTBEAT on the local
14580 / remote 14540 offboard link. PID parent, executable, start ticks, command
results and native stream status must match. Setup failure stops launch. This
changes native feedback cadence, never the arm command, armed state, timestamps
or the controller's three-wall-second confirmation deadline. The explicit probe
also requires six distinct MAVROS states with receipt gaps at most two seconds
before a goal. Raw disarmed draft inspection keeps upstream telemetry.

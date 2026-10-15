# EasyFly PX4 SITL

Gazebo Classic simulation uses the guarded project-local PX4 compatibility
build, a single-vehicle reservation, and the maintained EasyFly Iris SDF.
Missing or mismatched compatibility inputs fail closed; the original PX4
installation is not patched by this launcher. No automatic flight goal is sent.

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

## Explicit simulation gyroscope noise

The EasyFly model explicitly sets `gyroscopeNoiseDensity` to
`0.00008726646259971648` rad/s/sqrt(Hz), equivalent to the IIM-42653 typical
0.005 deg/s/sqrt(Hz) spectral density in DS-000529, gyroscope specifications.
The separate 0.05 deg/s RMS figure at 100 Hz is not a density. The installed
Gazebo plugin's default confused these figures, making a stationary simulated
vehicle repeatedly exceed PX4's gyro-vibration rest threshold.

This model-level repair keeps nonzero noise. It does not modify system plugins,
PX4 admission/protection parameters, the 0.83 m spawn height, accelerometer
noise, random walk, bias settings, or hardware configuration. This typical
simulation reference is not a measurement or calibration of a real device.
See the [TDK product and datasheet entry](https://www.invensense.tdk.com/en-us/products/inertial-sensors/industrial-motion/iim-42653).

The SDF regression checks physical inertias, camera/TF contracts and the unique,
finite, nonzero gyro density including degree-to-radian conversion. Omission,
zero/noise suppression, RMS-as-density and unrelated IMU overrides fail it.
Cold-start native-state checks are separate from flight and UI acceptance;
passing these checks is not hardware readiness or proof that every historical
GPS or shutdown fault has been eliminated.

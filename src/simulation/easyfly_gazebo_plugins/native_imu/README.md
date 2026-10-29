# Native IMU lifetime and timestamp compatibility

Pinned PX4 Classic revision `70683dc759cbd3ebccfa78429b564f8b1fb5d149`,
Apache-2.0 notices retained in `upstream/`. The independently named
`libeasyfly_native_imu.so` uses the existing `libsensor_msgs.so`; it does not
register another protobuf definition. Configure `EASYFLY_PX4_GAZEBO_BUILD` to
the reviewed PX4 Classic build directory. Its message-library fingerprint is
installed for launch-time validation.

Changes: a distinct class name; widen seconds **before** multiplying
by 1000000 (the original int32 sum overflows at 2147.484 s on the 1 ms grid);
release the update event with `reset()` instead of explicitly invoking its
destructor; reject a nonpositive world-time integration interval before noise
calculation. That interval emits no sample, and the next positive interval uses
the actual reset epoch. The relay still latches on the rollback; it does not
resume old filter state. Native timestamps, sequence, noise, physics and sensor channels
otherwise follow the original implementation. The relay still refuses gaps,
nonfinite samples and discontinuities and never fills or retimestamps samples.

The UBSan fixture executes both extracted timestamp expressions at 2147–2149 s.
The actual native producer/relay fixture crosses the boundary
with 6000 real physics samples; the original producer halts the unchanged relay
when 2147483000 us becomes -2147483296 us, while the corrected producer stays
continuous past 2152 s.
It establishes this defect, not the cause of every historical relay halt.

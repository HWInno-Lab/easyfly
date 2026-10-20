# Configured calibration validation

All configured transforms/intrinsics are validated before publishing any static
TF. This loads known calibration; it does not solve calibration online.
The continuous CameraInfo diagnostic checks frame, K/distortion, source/receipt
age and ROS-clock progress, rather than trusting the first valid sample forever.
The diagnostic is not an automatic flight-admission gate. Configuration changes
require restarting the common calibration launch.

`ros2 launch easyfly_camera_calib calib_tf.launch.py` defaults to simulated time.
Set `use_sim_time:=false` for system time; `calib_file` selects a trusted local
file. First format supports plumb_bob/5 coefficients only. Detailed contract and
tests: [D4 guide](../../../../docs/development/d4-perception.md).

The Gazebo MID360 configuration publishes a fifth transform,
`livox_frame -> livox_imu_frame`, at `(-0.011, -0.02329, -0.00588)` metres.
`sensors.lidar.imu_extrinsic.translation` is the IMU origin in the lidar frame;
the laser sensor's 0.05 m offset has already been subtracted. Its optional `child`
enables TF publication. Older configurations without `child` retain validation-only
behavior. This transform is the inverse of FAST-LIO's lidar-in-IMU extrinsic.

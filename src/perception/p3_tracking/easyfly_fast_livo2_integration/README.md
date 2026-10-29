# FAST-LIVO2 integration

Opt-in Gazebo RGB + MID360 LiDAR + IMU fusion. Real upstream estimator outputs
are guarded before source-time body alignment and navigation. Default FAST-LIO
is retained. No automatic backend switching or external-vision FCU injection.

Input timestamps and payloads are preserved. The explicit Gazebo configuration
uses `preprocess.instantaneous_scan=true`: native physics rays are sampled at one
instant; historical CustomMsg synthetic sequential offsets are not physical beam
times. This is an algorithm input interpretation, not re-stamping observations.
The guard subscribes to native IMU with reliable delivery and a bounded 512-sample
history, matching its reliable producer; camera and cloud inputs retain best-effort
delivery. Original source-gap and source/receipt-age limits are unchanged.

Visual diagnostics count accepted photometric residuals divided by patch area;
poses are emitted after visual EKF updates, with actual filter covariance.
The filter uses world position and right-body orientation errors; body conversion
uses the corresponding lever-arm Jacobian. Alignment is frozen after initial
source-time FCU pairing, never continuously fitted to truth. Native Gazebo truth
is read-only acceptance evidence and is never an estimator input.

All three inputs and output require source/receipt freshness; clock rollback,
source rollback, malformed data, duplicates or producer ambiguity latch a reset.
Sensor silence expires admission; restart the entire selected session after a
latched error. Navigation output requires >=8 accepted visual patches. Raw
odometry alone is insufficient. The frontend itself does not perform loop closure
or map reuse. The separate [private LiDAR atlas](../easyfly_lidar_atlas/README.md)
adds bounded geometric map reuse without changing this navigation contract.
Hardware acceptance and runtime fallback remain separate work.

Raw estimator products, including its debug TF, are isolated under `/fastlivo/raw/`.
A single persistent broadcaster replaces the upstream per-output publisher creation.
Only the guarded navigation adapter publishes control-facing TF and odometry.
Queued image matrices own their pixel buffers after the source callback returns.
Raw twist linear velocity is in the IMU child frame; its covariance is unavailable.
The public aligned navigation state uses checked pose differences and explicitly
reports unavailable aligned covariance. Debug path history is bounded to 1000 poses.

Explicit simulation launch (after sourcing the selected install environment):

```bash
ros2 launch easyfly_sim_launcher profile.launch.py profile:=navigation_dev \
  'algorithms_json:={"slam":"fast_livo2","strategy":"combined","provider":"cpu"}' \
  fastlivo_navigation:=true headless:=true rviz:=false
```

Use a registered scene and its documented geometry/view restrictions for acceptance.
Desktop status is read-only; desktop mission execution for this backend stays disabled.

The IMU input uses a bounded 512-message receive history so point-cloud callback
work cannot evict intervening samples. A source-time IMU gap over 20 ms latches
`imu_source_gap`; the 0.5 s source/receipt freshness limits remain unchanged.
No interpolation, copied-last-value IMU or timestamp repair fills missing samples.
After any required sensor expires, all estimator input forwarding stops while
other original streams remain observable for diagnosis.

For the independent atlas, explicitly set `enable_atlas:=true` on
`fastlivo.launch.py` (or `fastlivo_atlas:=true` on `perception_sitl.launch.py`).
The immutable native option defaults to false. It publishes a complete measured
deskewed scan and the exact post-visual odometry on `/fastlivo/raw/atlas_cloud`
and `/fastlivo/raw/atlas_odometry`, only with initialized IMU and >=8 visual
features. One owned scan survives the next IMU pass clearing its work buffer;
stale or repeated source epochs are never reused. Original display clouds remain
camera-cropped. Native floating-seconds timestamps retain the existing <=1 ns
roundoff relative to the original camera header.

When no CMake build type is selected, the native frontend now uses
`RelWithDebInfo`; explicit Debug or other choices are respected. The unoptimized
default could not process the declared dense simulation input rate and reached
its unchanged bounded input-queue refusal. Queue and freshness limits have not
been enlarged.

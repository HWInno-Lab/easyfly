# Camera calibration and stereo rectification

This package provides configured calibration validation, live monocular/stereo
calibration, and explicit application of a selected stereo candidate. These
independent tools do not grant flight admission or replace an active configuration.

## Acquire a new candidate

Copy `config/online.example.json`, measure the physical checkerboard square size
in metres, and set its **inner** corner counts and source topics/optical frames.
The example uses a 7 × 5 inner-corner board (8 × 6 squares). Set `stereo: false`
for monocular calibration; only the left source is then consumed. Set
`use_sim_time: false` for system-clock streams.

```bash
ros2 run easyfly_camera_calib online_calibration \
  --config /absolute/calibration-input.json \
  --output /absolute/new-calibration-session
```

The output directory must not exist; it is created with mode 0700. Capture 20–40
views within the configured 10–600 second budget. Use a rigid, fully visible board
with different tilts, distances and screen positions in **both** cameras. Training
board normals must span at least 20 degrees, centers must reach both sides of each
image midpoint (45%–55%), and board distance must span a factor of at least 1.3.
Nearly parallel, constant-distance or one-sided captures are rejected even when
reprojection residuals are small. Supported inputs are mono8/rgb8/bgr8,
320–1280 × 240–960, and the five-coefficient plumb_bob model.

The collector detects subpixel corners in the actual images. It keeps at most three
pending images per camera and requires exact paired source timestamps, advancing
ROS time, source/receipt ages within 0.7 seconds, and one pinned DDS writer per
source. Duplicate frames cannot renew freshness or become new samples. Resolution
changes, source replacement/duplication, clock regression, insufficient diversity,
failed checks or timeout fail the session without producing a candidate. Source
loss before any samples waits until the deadline. No old session is resumed.

The solver withholds a fixed-seed selection of at least four views before fitting.
Monocular initialization checks focal/principal-point uncertainty and pose
observability. Stereo mode jointly refines both intrinsic models and the rigid
right-from-left transform using training views only. It then rechecks independent
held-out image residuals and left-pose-to-right reprojection. Neither camera truth
nor a known baseline is supplied to the solver. Original monochrome PNGs, pixel
hashes, exact source stamps, writer identities and detected corners remain in the
session for review.

A successful session writes `candidate.json`, `intrinsics-left.yaml` (and right in
stereo mode), `samples.json` and `status.json`. The YAML files are intrinsic snippets
for the configured loader, not complete sensor configurations. Candidate status is
`validated_candidate_not_activated`; consumers must explicitly select it. Reported
monocular uncertainty belongs to `monocular_initialization`; joint intrinsic
covariance is unavailable. Small reprojection error alone does not establish metric
accuracy over an arbitrary distance range. Measure downstream depth accuracy before
using a new camera/board setup. Solver operations follow the
[OpenCV calibration and stereo geometry contract](https://docs.opencv.org/4.5.4/d9/d0c/group__calib3d.html).

## Apply a selected stereo calibration

Review the candidate and record its exact SHA-256, then start:

```bash
ros2 run easyfly_camera_calib rectify_stereo \
  --calibration /absolute/new-calibration-session/candidate.json \
  --sha256 REVIEWED_64_CHARACTER_LOWERCASE_SHA256
```

Defaults are simulated time and `/stereo/left/image_raw`, `/stereo/right/image_raw`.
Use `--system-clock`, `--left-topic` and `--right-topic` when needed. The candidate
must describe a horizontal left-to-right rig with a positive 0.02–1 m baseline.
The selected file is bounded and its exact bytes are verified before use.

Outputs are `/stereo/rectified/{left,right}/{image_raw,camera_info}` with original
source stamps, rectified optical frame names ending in `_rectified`, zero residual
distortion, and rectified K/P. The rectifier publishes only each raw optical frame
to its rectified optical frame; deployment still owns the rig-to-body/world TF and
the physical relation between the raw cameras. Run one publisher for each of these
new frame names. `/stereo/rectified/status` reports candidate hash and freshness.
Images require exact source pairs and independent 0.5 second clock/source/receipt
budgets. Writer replacement/duplication or clock regression latches a fault until
explicit restart. A temporary frozen clock produces no fresh output; genuinely new
source data can resume after time advances.

For `easyfly_depth_estimation`, set its four input topics to these rectified image
and CameraInfo topics and frames to the corresponding `_rectified` optical frames.
It must receive this complete tuple; mixing raw images and rectified CameraInfo is
invalid. The default Iris RGB-D camera is not a physical stereo rig.

## Configured calibration validation

`ros2 launch easyfly_camera_calib calib_tf.launch.py` validates every configured
transform/intrinsic before publishing static TF. It defaults to simulated time;
`use_sim_time:=false` selects system time and `calib_file` selects a trusted file.
The CameraInfo diagnostic continuously checks frame, K/distortion, source/receipt
age and ROS-clock progress. Configuration changes require an explicit restart.
Detailed loader contract: [D4 guide](../../../../docs/development/d4-perception.md).

Gazebo MID360 includes `livox_frame -> livox_imu_frame` at
`(-0.011, -0.02329, -0.00588)` metres. `sensors.lidar.imu_extrinsic.translation`
is the IMU origin in the lidar frame; the laser sensor's 0.05 m offset has already
been subtracted. Optional `child` enables TF publication; older configurations
without it retain validation-only behavior. This is the inverse of FAST-LIO's
lidar-in-IMU extrinsic.

## Verified scope

Native Gazebo stereo pixels with 25 physical checkerboard poses, five withheld
views and no supplied camera truth recover focal lengths within 0.7% and the
0.2 m baseline within 0.3%. Candidate → rectifier → metric stereo evaluation on
independent textured planes at 2/4/8 m passes the unchanged 5% P95 relative error
limit (0.37%/2.26%/3.52%). These metrics apply to the declared central evaluation
region, with 100%/100%/92.44% valid coverage, not the entire image. Earlier failed
captures and 8 m failures are retained with the acceptance evidence.

Tests cover native image collection, monocular input, held-out geometry, metric
stereo output, exact remapped pixels/source headers/optical rotations, clock pause
and regression, duplicate/unpaired input, writer changes, changed candidate bytes,
and normal signal-driven retirement in both installation modes. Existing
`easyfly_time_sync` separately pairs images without rewriting their headers.

Hardware triggering, oscillator synchronization, D435i physical calibration,
rolling-shutter timing, IMU/lidar-camera calibration and real-device metric quality
remain hardware work. Software pairing does not synchronize hardware clocks.

# Independent tag36h11 perception

`apriltag_detection_node` consumes `/camera/image_raw` and `/camera/camera_info`,
publishes `/tag_observations`, and offers `/platform/apriltag/capabilities`. No
flight, map, `/tracks`, or `/target_positions` publisher exists. Launch with
`apriltag_detection.launch.py use_sim_time:=false tag_size_m:=0.2` after verifying
the actual printed tag's metric outer edge. Default size **0 means unconfigured**,
not an assumed physical size.

Only OpenCV's AprilTag 36h11 dictionary is supported. IPPE-square returns candidate
Tag→optical-camera poses; positive corner depths, finite data, unit quaternion,
four-corner RMS reprojection and ambiguity margin gate validity. Numerically
identical candidates are deduplicated, but substantively different poses with
similar errors are rejected. A front-facing observation can therefore remain
invalid. `valid` does not certify real-world calibration accuracy.
Detection refines corners with bounded-window SUBPIX (3 pixels) before IPPE.
Existing reprojection, ambiguity and downstream tilt/plane gates are unchanged.
The standalone process uses a read-only `opencv_threads=2` budget (1..8), avoiding
an implicit thread per available core; it does not configure other OpenCV nodes.
Each successful result retains the exact CameraInfo and its receipt time used
for computation. Readiness, expiration, and publication re-check that evidence;
new images or CameraInfo timestamps cannot extend an old result's calibration
age. Bad CameraInfo or a changed calibration model immediately invalidates prior
results. Fresh updates of the same model do not indiscriminately revoke readiness.

Images must be packed mono8/rgb8/bgr8, at least 16×16 and at most 2,073,600 pixels.
CameraInfo must have matching dimensions and configured optical frame, positive
focal lengths, finite conventional K, and plumb_bob/rational_polynomial distortion
with 0/5 finite coefficients for plumb_bob or 8 for rational_polynomial. Original image stamps are retained. Both source
and steady receipt ages are bounded. Zero/paused/regressed ROS clock, wrong frame,
conflicting/out-of-order image stamps, missing/expired calibration, or malformed
payload produce invalid arrays. A valid image with no tags produces a valid empty
array. At most 64 detections are emitted. DDS input depth 1 and a replaceable
latest-image slot bound buffering; a joined worker processes OpenCV outside ROS
callbacks, and generation/stamp guards discard superseded work.

An exactly identical latest raw image (all header/metadata fields and pixel bytes)
is ignored only while its original source/steady receipt lifetime, progressing
clock, calibration and processing generation remain valid. It emits no new
observation, renews no lifetime and cannot count as another stable frame downstream.
Only one eligible raw image is retained, at most 6,220,800 pixel bytes. Invalid
input/result, changed or invalid calibration, expiration or clock failure clears
this eligibility. Same-stamp different content still invalidates; replaying the
old image after invalidation cannot restore readiness. No hash-only comparisons
or decoded-image equivalence are used.

Read-only startup parameters: `optical_frame=camera_color_optical_frame`,
`tag_size_m=0` (0..10m), `max_age_seconds=.5` (.05..5s),
`max_reprojection_error_px=2` ((0,20]), `ambiguity_margin_px=.2` ((0,5]).
Capabilities separate implementation from fresh input readiness, not flight
permission. Pure C++ tests generate markers and calibrated tilted geometry;
the ROS test uses the installed C++ `tag_fixture` writer, never Python OpenCV.
ROS tests use domains 105 and 106; run serially with other workspace tests using
those domains. Existing ament lint registrations remain; behavior tests do not imply
that all lint checks passed.

## Timestamp tolerance

Source timestamps may lead the current ROS clock by at most
`min(0.2 seconds, max_age_seconds)`; larger future timestamps are rejected.
Positive source time, optical frame, source/receipt freshness and ROS clock
progress remain mandatory. Paused/zero/regressed clocks still invalidate output;
original source stamps are never rewritten. This is perception readiness only,
not permission to fly or publish flight commands.

## ALG-5 算法边界

标志检测与 IPPE 位姿计算迁入 easyfly_tag_pose；CameraInfo 接纳、校准版本、源时间/过期和 worker generation 留在本包。旧 tag_core.hpp 仅为兼容别名。

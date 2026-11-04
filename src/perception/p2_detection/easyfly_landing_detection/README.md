<!--
  SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
  SPDX-FileCopyrightText: 2026 贾国源
  License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_landing_detection/licenses/.
  This revision offers no Apache-2.0 option for the listed original code.
-->
# Selected-tag landing perception adapter

`landing_detection_node` only adapts `/tag_observations` to `/landing_target`.
It never commands a descent, changes flight modes, or publishes map/track inputs.
The observation subscription uses best-effort, volatile KEEP_LAST depth 1: it
accepts reliable or best-effort publishers without waiting for missing older
sequence numbers. Dropped frames are allowed; source/receipt freshness, replay
rejection and missing-target expiry remain mandatory. `/landing_target` output,
actions and services retain their existing reliable delivery policies.
Launch `landing_detection.launch.py selected_tag_id:=7 use_sim_time:=false`.
Default `selected_tag_id=-1` means **no selection**, and always yields invalid.

Exactly one valid observation of the explicit ID is required. Other tags do not
influence selection; duplicate selected IDs, absence, invalid upstream arrays,
bad/non-unit pose, nonpositive optical depth, stale source or receipt time,
duplicate/out-of-order stamps, zero/paused/regressed clock, and wrong frame
invalidate output. No input for the age budget also publishes invalid. Original
image header is retained. `yaw_rad` is the tag local X-axis projected into camera
optical XY, not an ENU vehicle heading. `optical_depth_m` is camera Z, **not ground
height**. An unobservable projected X-axis yaw is rejected. Reprojection error is
preserved, not relabeled as a calibrated quality probability.

Read-only startup parameters: `selected_tag_id=-1`,
`optical_frame=camera_color_optical_frame`, `max_age_seconds=.5` (.05..5s).
`/platform/landing/capabilities` reports implementation and current valid-target
readiness separately. Ready is not permission to land. ROS regression uses
synthetic typed messages, explicit-ID/multiple-tag cases and expiration; flight
integration belongs to later D5 work.
The ROS test uses domain 106; do not run concurrently with other tests on that
domain. Original ament lint registration is retained independently of behavior
test results.
The end-to-end test additionally generates a C++ warped 36h11 marker, feeds real
Image/CameraInfo through both real nodes, checks selected optical depth near 1m
and exact original source stamp, then replaces the image with white pixels and
asserts selected-target invalidation. This remains software perception validation,
not a flight, real camera calibration, or landing accuracy certification.

## Timestamp tolerance

Source timestamps may lead the current ROS clock by at most
`min(0.2 seconds, max_age_seconds)`; larger future timestamps are rejected.
Positive source time, optical frame, source/receipt freshness and ROS clock
progress remain mandatory. Paused/zero/regressed clocks still invalidate output;
original source stamps are never rewritten. This is perception readiness only,
not permission to fly or publish flight commands.

## Optional ingress timing

Read-only `timing_trace_path=""` is disabled by default. An explicit path must
not already exist. The original reader's native writer/receive timestamps,
callback entry wall/ROS times, and publish-call boundaries are retained in an
8192-event ring and written as TSV only on normal node destruction. The file
starts with total/retained/dropped counts. There is no callback-time disk I/O,
extra reader, or change to reliability/history. A crash or forced kill may leave
an empty file; that is not complete evidence. Zero native timestamps and wall
clock jumps prevent a latency conclusion. Compare wall with wall and ROS source
time with ROS time, never across the two clock domains.

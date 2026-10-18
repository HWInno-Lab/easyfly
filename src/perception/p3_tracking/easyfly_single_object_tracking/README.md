# Explicit-ROI CSRT tracking

`single_object_tracking_node` consumes `/camera/image_raw`, publishes independent
`/single_tracks` and `/single_track/state`, and provides `/single_track/manage`.
It does not publish `/tracks`, `/target_positions`, map inputs or flight commands.
Launch `single_object_tracking.launch.py use_sim_time:=false`.

Manage operations are `initialize`, `reset`, `query`. Initialize must name the
**exact latest displayed image header** (positive stamp + optical frame), with a
finite normalized top-left/size ROI fully inside the image and at least 8×8
pixels. Every accepted new ROI gets a new monotonic ID in an independent namespace.
Acceptance means initialization is queued, not that tracking succeeded. Lost
targets require explicit reinitialization; there is no automatic identity guess.
`initializing`/`relocalizing` become `tracking` only after successful CSRT work;
`lost` outputs an empty array and invalid state. Reset yields `idle`. Old ROI
generations cannot overwrite new/reset state.

Bounding boxes are normalized image coordinates. Confidence is **0 (unknown)**,
not a fabricated probability of 1; OpenCV CSRT has no calibrated confidence here.
Trajectory and velocity fields stay empty/zero: no metric 3D or velocity claim.
Frames must be packed mono8/rgb8/bgr8, 16×16 or larger and ≤2,073,600 pixels.
Source/receipt age, ROS clock progress, strictly increasing stamps and optical
frame gate input and output. Invalid/stale input, paused/regressed/zero clock or
expired processed output causes loss even if callbacks keep receiving images.
DDS depth 1 plus a replaceable latest slot bound queued work. A worker-owned CSRT
instance is joined at shutdown; cache is at most latest/ROI snapshot plus in-flight
work, and ROI generation protects initialization/reset races.

Read-only startup parameters: `optical_frame=camera_color_optical_frame` and
`max_age_seconds=.5` (.05..5s). `/platform/single_tracking/capabilities` separates
implemented algorithm from current valid tracking; neither grants flight control.
Pure C++ regression uses textured synthetic translation. ROS tests create images
from pure Python byte buffers, not cv2/numpy, and exercise ROI/reset/new identity,
expiration, stale display rejection, and reset while work is pending.
ROS regression uses domain 107, serially with any other workspace use of that
domain. Original lint checks remain registered; behavior tests are separate.

## Timestamp tolerance

Source timestamps may lead the current ROS clock by at most
`min(0.2 seconds, max_age_seconds)`; larger future timestamps are rejected.
Positive source time, optical frame, source/receipt freshness and ROS clock
progress remain mandatory. Paused/zero/regressed clocks still invalidate output;
original source stamps are never rewritten. This is perception readiness only,
not permission to fly or publish flight commands.

## ALG-5 算法边界

CSRT 模型生命周期已迁入 easyfly_csrt，并通过 easyfly_single_tracking_api 调用；ROI 服务、重定位、图像所有权、源时间/过期和 worker generation 留在本包。未知分数继续发布原 confidence=0，算法使用 nullopt。

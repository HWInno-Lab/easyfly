# Metric visual navigation

This explicit local task connects measured color-line/ellipse observations to
metric guidance and one native `FollowTrajectory` action. It also consumes
obstacle classification and optional target prediction as conservative control
constraints. Start from a settled, armed OFFBOARD hover; takeoff and landing are
separate explicit operations. Existing desktop and detector launches do not start
this task automatically.

```bash
ros2 run easyfly_visual_navigation visual_mission \
  --config /absolute/task.json --output /absolute/new-session
# After reviewing that task, execute into a different new session directory:
ros2 run easyfly_visual_navigation visual_mission \
  --config /absolute/task.json --output /absolute/execution-session --execute
```

The first command writes a review and has no ROS flight side effects. Both modes
require a new absolute output directory, created with permissions 0700. Use
`config/line.example.json` or `ellipse.example.json` as a complete configuration.
The configuration declares the local `odom` endpoint, altitude, ground-surface Z,
control corridor, source topics, optical frame, speed, acceleration and timeout.
Speed is bounded to 0.1–0.5 m/s, acceleration to 0.1–0.5 m/s², goal altitude to
0.8–5 m and the wall-clock task budget to 10–180 seconds. Actual demonstrated
flight uses 0.3 m/s and 0.3 m/s², not every admitted setting.

## Inputs and metric guidance

Run the existing color-line or ellipse detector on a calibrated camera. Provide
its original-source observation array, aligned **32FC1 optical-Z depth in metres**,
CameraInfo with zero distortion, and `odom <- optical_frame` TF at each exact
image source timestamp. The camera/image resolution is bounded to 160–1280 ×
120–720. This is not pixel-only control: each selected visual feature requires
finite local measured depth support, one consistent surface, a source-time rigid
transform, and consistency with the configured surface height and intended route.
The sensor rig and coordinate publisher remain explicit deployment responsibilities.

Line mode follows a single unambiguous measured straight component toward the
requested metric endpoint. Endpoints inside the image are sampled away from its
edge. Competing routes and discontinuous changes fail closed. Ellipse mode centers
above a static measured ellipse within 0.3 m of the requested association seed;
the measured center becomes the native action endpoint. The complete selected
contour requires a 24-pixel border margin. A moved ellipse aborts this static-target
task. Arbitrary curved routes, route intersections, moving pads and unseen space
are outside this task's declared scope.

A complete set of actual DDS writers must remain stable for one second before
initial admission. All subsequent writer replacements/duplications, clock
regression and camera calibration changes latch a failure. Command admission also
requires 0.8 seconds of settled hover, velocity below 0.08 m/s, roll/pitch below
0.04 rad and height within 0.1 m of the request. Source stamps, receipt ages and
clock progress have independent **0.5 second** budgets. Duplicate timestamps
cannot refresh receipt age. The controller retains at most four visual tuples and
selects the newest still-fresh tuple with an available exact-time TF; it never
uses current-time TF to transform an older image. Invalid depth, wrong frames,
unpaired messages and missing observations cannot create new measured geometry.

## Semantic and prediction constraints

Fresh `/obstacles_classified` (`ClassifiedObstacleArray`) is mandatory. Start the
existing `easyfly_semantic_map` obstacle classifier with fresh geometry and measured
`/target_positions`; classification cannot delete geometry. For a known clear test
corridor, an explicit valid empty geometry stream is appropriate. Empty data does
not establish that an arbitrary unseen flight corridor is free: this local task
requires a previously verified corridor and does not replace a general map planner.

Every obstacle is retained with at least a 0.35 m margin. A fresh, uniquely
associated person class enlarges it to 0.7 m. Existing dynamic velocity expands the
geometry through a 1.5 second sweep. With `use_predictions: true`, the existing
`easyfly_target_prediction` output must be current too. A prediction is used only
when ID, class and original observation time match the classification association.
The predicted center and its declared model uncertainty can only enlarge the
existing obstacle envelope, never clear or shrink it. Prediction variance is a
model quantity, not a calibrated physical sensor uncertainty.

The complete upcoming segment and braking distance are checked against these
boxes. Hazards brake the reference to a hold, and fresh clear geometry permits
progress to resume within the same still-owned task. Invalid/stale source data
aborts. Tracking error over 0.35 m, lost OFFBOARD/arming, corridor exit or a control
source-time step over 100 ms also aborts. Commands use bounded velocity and
acceleration, the admitted native stream watermark, and the exact task session.
The task preserves initial yaw and has no separate manual-control override.

## Ownership and cancellation

`SIGINT`/`SIGTERM` requests cancellation. On any failure, command streaming stops,
the controller cancels its exact native UUID once, and drains the native terminal
result for up to 15 seconds. A cancel acknowledgement alone does not release
ownership. A private per-user/domain ledger under `/tmp/easyfly-visual-owner-UID`
records intent before send. Unknown admission/results remain busy; a new task in
the same controller session is blocked. Controller restart is an explicit recovery,
and never restores or adopts the previous flight task. Normal completion leaves
the native vehicle hovering; submit Land separately.

`status.json`, `result.json` and `/visual_navigation/status` expose task identity,
source writers, measured visual source time, native ownership, command counts and
obstacle holds. Input recovery does not automatically restart an aborted task.
A canceled/failed CLI exits nonzero even when native cancellation and cleanup
succeed; inspect `ownership_released` and the exact native terminal state.

## Verified software/simulation scope

Native Gazebo down RGB-D pixels → installed C++ detectors → source-time metric
geometry → native PX4/FollowTrajectory completes straight-line following and static
ellipse centering in both installation modes. The independent acceptance witness
measures endpoint error, not only action acknowledgements. Actual classifier and
predictor processes consume explicit deterministic metric hazard/target fixtures:
person semantics and linked predictions hold motion, fresh hazard removal resumes
the route, and a foreign flight goal is rejected during ownership.

Freezing the actual owned line detector during flight aborts and releases the exact
native action. Resuming it does not restart motion. Explicit SIGTERM cancellation,
Land/disarm and normal stack retirement also pass. Native ROS input checks cover
mismatched stamps, wrong frames, invalid depth, frozen/duplicate clocks, changed
calibration, duplicate/replaced publishers and clock rollback. Earlier fixture,
visibility and initial discovery failures remain in the acceptance evidence.

Hardware operation, arbitrary environment perception accuracy, global map coverage,
all supported speed values, curved-line tracking and moving-target landing remain
outside this bounded software/simulation acceptance.

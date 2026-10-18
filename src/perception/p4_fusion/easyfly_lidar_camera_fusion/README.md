# Independent source-time lidar/camera projection

Launch `fusion.launch.py use_sim_time:=false` outside simulation. Inputs default
to `/camera/color/image_raw`, `/camera/color/camera_info`, `/pointcloud_filtered`.
Exact two-source-time TF via fixed `odom` projects points into
`camera_color_optical_frame`; missing TF never falls back to latest.

Outputs `/lidar_camera/colored_points` preserve source cloud XYZ/header and add
packed UINT32 RGB. `/lidar_camera/depth` preserves image header, 32FC1 optical
Z in metres, NaN elsewhere. Nearest-pixel Z-buffer handles same-pixel occlusion;
this is sparse support, not a replacement aligned-depth sensor or full dynamic
scene compensation. `/lidar_camera/status` and
`/platform/lidar_camera/capabilities` distinguish fresh results from invalid or
expired inputs. No original cloud, map, target or flight topic is published.

Read-only defaults: `max_age=.5`, `tolerance=.05`, `max_points=20000`,
`camera_frame=camera_color_optical_frame`, `fixed_frame=odom`, plus input topic
names. Only finite conventional K and plumb_bob with five coefficients are
supported. Bounded image queue/latest cloud worker, source and receipt budgets,
clock progress/rollback clearing, epoch guards and joined shutdown prevent stale
work from reviving readiness. See [D4 contracts](../../../../docs/development/d4-perception.md)
for exact size/layout budgets and invalid semantics.

Pure projection, deterministic queued-task epoch isolation, real ROS TF/time/CameraInfo negatives, and simulated clock
zero/pause/replay/rollback tests are registered independently of existing lint.
Test domains 110/114/115; keep tests on each domain serial. Synthetic regression is
not real sensor accuracy, SITL or flight safety acceptance.

## ALG-6 算法边界

投影数值计算已迁入 easyfly_projection_py；原 projection.py 保留 ROS 消息字节布局解析和输出序列化。lookup_transform_full 的两源时间、固定帧、worker/epoch、CameraInfo 接纳与源/接收时效规则不变。

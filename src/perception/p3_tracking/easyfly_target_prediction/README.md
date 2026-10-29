# Independent target prediction

`target_prediction_node` subscribes only to `/target_positions` (`Target3DArray`)
and publishes only `/target_predictions` (`TargetPredictionArray`). It does not
publish measurements, control commands, tracking goals, or map inputs. Nothing
automatically connects this model to navigation or target following.

At least two real, strictly ordered observations of a stable nonnegative ID are
required. Every array and entry must have an identical positive observation time
and `odom` frame. Positions and confidence must be finite; position must be valid
and confidence must be within the configured threshold and 1. A malformed whole
frame, duplicate ID, repeated/out-of-order timestamp, or over-limit input clears
all tracks (a deliberately conservative fail-closed policy). Valid empty arrays
and disappeared identities clear history immediately. Class changes, excessive
speed, jumps, or reacquisition require a new baseline plus a subsequent real
observation; invalid predictions never become valid merely by republishing.

The output `header` is the current ROS reference time. `observation_header` retains
the exact source measurement time/frame. With observation age `a` and configured
horizon `h`, the position is `observation + velocity * (a + h)`: `h` is measured
after the output reference time, not after the observation. Velocities are bounded
three-dimensional norms. Reported equal diagonal variances are the simple model
`base_variance + variance_growth * (a + h)^2` in m², **not calibrated sensor
covariance** or a claim of statistically calibrated uncertainty.

Source age uses ROS time; receipt age and the independent clock-progress watchdog
use steady time. Both ages must be within `max_age_seconds`. Zero ROS time, clock
regression, or a ROS clock paused longer than `max_age_seconds` clears history.
Expired tracks emit an invalid entry once (or an empty array after clock failure)
and are removed; subsequent output is empty. New receipts cannot extend old source
time or stalled-clock validity.

Startup-only parameters are read-only:

| Parameter | Default | Allowed range |
| --- | --- | --- |
| `horizon_seconds` | 0.5 | 0..2 s |
| `max_age_seconds` | 0.5 | 0.01..5 s |
| `max_speed` | 10 | (0,100] m/s |
| `max_jump` | 5 | (0,100] m |
| `max_tracks` | 256 | 1..1024 entries and cached tracks |
| `min_confidence` | 0.2 | 0..1 |
| `base_variance` | 0.04 | 0..100 m² |
| `variance_growth` | 1 | 0..100 m²/s² |
| `publish_period_seconds` | 0.1 | 0.05..1 s |

Launch with `ros2 launch easyfly_target_prediction target_prediction.launch.py`,
adding `use_sim_time:=true` when measurements use simulated ROS time.
`/platform/prediction/capabilities` reports this constant-velocity model as
implemented, and ready only while at least one currently valid prediction exists.
Readiness is not flight permission. DDS identities are not authenticated here;
use a trusted publisher or DDS Security when producer trust matters.

Core GTests now belong to `easyfly_motion_prediction`; the functional package
retains actual ROS positive/negative cases on domain 104, including paused and regressed `/clock`. Build only after `easyfly_msgs` has
been regenerated with the prediction interfaces. Run targeted CTests from
`build/easyfly_target_prediction` after sourcing the workspace installation.


ALG-4 moves the complete prediction/history model into the standard-library-only
`easyfly_motion_prediction` package. The node directly consumes its ordinary
C++ types and explicitly supplies ROS/steady timestamps; the old `predictor.hpp`
contains forwarding aliases only. Message conversion, clock sampling, timer and
capability service remain here. Topics, QoS, parameters, model thresholds and
clock/watchdog call order are preserved. No navigation/control consumer is added.

独立的 `easyfly_visual_navigation visual_mission --execute` 现提供显式米制视觉任务，
并可消费类别和预测作为扩大避让范围的约束。其源时间、深度、坐标、控制所有权和
仿真范围见 [视觉导航说明](../../../execution/e3_control/easyfly_visual_navigation/README.md)。
本包仍只产生原有观测，启动本包不会自动开始飞行。

# Managed SUPER

Experimental `navigation_strategy:=super` is available in `navigation_dev`.
It does not change the default planner or start navigation without an explicit goal.

The installed `easyfly_super_planner::super_execution_core` consumes the official
SUPER PlanFromRest pipeline: native A*, CIRI, exploration and backup MINCO-S4.
Each bounded worker imports an already inflated, original source-stamped ray grid
into native ROG-Map and verifies every cell on readback. Outside cells block;
unknown cells block in the default profile and are preserved only for explicit
committed exploration.
CIRI's point radius encloses the complete blocked voxel cube (half diagonal plus
outward numerical padding); the input still includes the .65 m vehicle envelope.

Default execution retains the original complete-goal, known-free profile.
The explicit `execute_committed:=true` profile executes the **original native
committed polynomial**: exploration prefix followed by native MINCO-S4 braking.
It preserves unknown cells for native exploration, but its backup visibility and
CIRI corridor include unknown voxel cubes as obstacles. Independent continuous
Bernstein collision, measured start/C2, terminal rest and exact derivative
certificates are mandatory for every admitted polynomial. No polynomial is
rescaled or replaced by a custom braking curve.

Unknown space is a goal/search capability, not permission to command flight
through unobserved cells. The owner additionally expands unknown cells by the
map's vehicle/tracking envelope using conservative whole-cell box dilation.
Occupied cells are never cleared. Input grids therefore need sufficient three-
dimensional observations around the complete vehicle envelope. A MID-360-only
scan can fail this gate because of its vertical blind region; a valid point
observation at the vehicle centre is insufficient. The simulated spherical scan
used by the extension's acceptance fixture is explicitly a virtual sensor,
not evidence of MID-360 hardware coverage or permission to fill blind regions.

A partial native backup must stop with zero reference velocity/acceleration,
make at least .25 m progress and remain at least .35 m from the parent goal.
After execution, the owner holds the certified endpoint until measured velocity
is below .08 m/s and tracking error below .1 m for .5 s. It requests the existing
coordinator's owned cancel/terminal/STOP/replan cycle only when fresh map
observations support continuation. Waiting is limited to 10 s; the original
parent goal retains its total time and replan budgets. An intermediate stop is
never reported as full-goal success. All remaining coefficients stay under
continuous collision checks, and the .2 m tracking stop remains unchanged.

Enable this profile explicitly (the evidence directory must already exist):

```bash
ros2 launch easyfly_sim_launcher profile.launch.py \
  profile:=navigation_dev \
  algorithms_json:='{"slam":"fast_lio","strategy":"super","provider":"cpu"}' \
  super_execute_committed:=true \
  super_evidence_directory:=/absolute/existing/evidence
```

The direct `super.launch.py` argument is `execute_committed`; the node parameter
is read-only. Default profiles do not opt in. Managed launch rejects this switch
outside `navigation_dev` with SUPER selected. Save adequate sensor observations
before admitting a goal; the switch does not remove the mapper's current extent
or observation requirements.

The owner enforces source and receipt freshness, map identity/revision, one command
publisher, exact session STOP, bounded worker reaping and bounded coordinator
replanning. Worker wall/CPU/memory limits are 3 seconds/3 seconds/1 GiB. Requests in
this Gazebo acceptance use .6 m/s, acceleration 2 m/s² and jerk 8 m/s³.
Native optimization targets are stricter to preserve tracking margin; hard guards
retain the requested bounds.

The headless planner's internal time stays at the measured snapshot while the
held vehicle awaits admission; solver CPU time does not advance a trajectory.
Steady-clock worker deadlines remain active. Backup corridor construction retries
shorter seed lines with the same voxel envelope, then finds a contiguous interval
of the exploration trajectory inside the resulting corridor. The braking-duration
initial guess is independent of the departure-time interval. Native backup checks
and the adapter's independent collision/dynamics certification remain mandatory.
The regression replays the original map-change failure and two return snapshots
three times each, and rejects a fully occupied control grid.

Dependencies are exported installed ROS/PCL/Eigen/yaml/native targets and the
existing Grid interface; this is not a ROS-free standalone binary. No upstream demo
controller or simulator is enabled. Source provenance, licensing discrepancy,
reproduction commands and current acceptance evidence are documented in
`docs/development/super-simulation-acceptance.md` at the workspace root.

跟踪误差按 `/uav_state.header.stamp` 对应的轨迹位置计算，避免把状态传输延迟计为误差。状态新鲜度、当前命令时刻的碰撞／边界检查及 0.2 m 跟踪误差停止门限保持不变。实际 ROS 延迟状态负例与真实 0.3 m 偏差停止测试分别覆盖时间对齐和故障保护。

当前 PX4 命令链只转发位置和速度。原生优化器为此预留跟踪余量：加速度不超过 0.15 m/s²、加加速度不超过 0.3 m/s³，且不得超过请求本身的界限。不满足时最多以更低速度目标重新求解八次，共享原有 worker 时间／CPU／内存预算；不缩放已有系数，不改变实测初速度。每次原生设置与轨迹均保留，原生 backup 检查和独立全轨迹碰撞／边界认证仍必须通过；执行端再次核验序列化结果。较高请求速度只是上限，并非巡航速度保证。

The committed profile tightens native **backup** acceleration/jerk targets
separately, preventing an otherwise slow exploration path from selecting a fast
emergency brake that the current position/velocity-only PX4 chain cannot track.
Requests above .8 m/s additionally tighten exploration targets and penalties;
actual speed still depends on available certified distance. A larger direct
visibility horizon requires a continuously known-free corridor. Native numerical
failures can retry at a lower target speed, sharing the existing eight-attempt
and wall/CPU/memory budgets. The original measured initial velocity is retained.
Native A* storage covers the whole fixed map around an off-centre measured start.

`EASYFLY_SUPER_REQUEST_2` explicitly opts into committed execution. V2 plans carry
native backup start time and a partial flag; V1 request/plan readers remain
supported for the default profile. The owner revalidates serialized metadata
and coefficients, and retains `.backup-events` beside each private job. Raw
native outputs, selected settings, request map and source timestamps remain
available for independent audits. Duplicate source stamps never renew receipt
freshness; out-of-order state delivery is ignored. Global clock rollback and
map identity/generation/revision rollback retain the existing stop contract.

The 2026-10-29 E-03 simulation acceptance used the actual installed native
planner/controller and Gazebo ray producer with the declared spherical sensor
fixture (0.6 m minimum range excludes the simulated airframe; 4.5 m range for
unknown progression, 40 m for the separately pre-observed long corridor).
The 7 m initially unknown goal needed two native backup stops and finished with
6.8 cm error. An actual mapper SIGSTOP produced a failed action terminal in
1.06 s; explicit owned cancellation took .10 s, and neither resumed automatically.
The 20 m known corridor reached **1.275 m/s measured peak**, with 2.2 cm endpoint
error. These numbers do not qualify the upstream high-speed envelope, arbitrary
unobserved environments, MID-360 blind regions or hardware flight. Both flights
landed/disarmed and all native children exited normally. Original failed fixtures
and the short-stop regression are retained in the stage evidence directory.
See `artifacts/stages1-3-20261028/super-committed-signoff.json` at workspace root.

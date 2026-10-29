# GCOPTER simulation backend

The opt-in `gcopter` strategy runs the original GCOPTER/MINCO optimizer from
`easyfly_gcopter_core`. Its pinned MIT source and one C++17 compatibility patch
are registered in `src/third_party/algorithms/planning/gcopter.provenance.json`.
ROS1 examples and their OMPL dependencies are excluded from the runtime.

Select it through the managed simulation entrypoint:

```bash
ros2 launch easyfly_sim_launcher profile.launch.py profile:=navigation_dev \
  scene:=gcopter_3d_lane_v1 headless:=true rviz:=false \
  'algorithms_json:={"slam":"fast_lio","strategy":"gcopter","provider":"cpu"}'
```

Launch is idle and sends no flight goals. Use the existing takeoff, `/navigate`,
mission and land interfaces. A navigation goal with zero speed uses 0.6 m/s for
this strategy; GCOPTER initially targets half the requested speed to leave
tracking margin for the current PX4 controller. Simulation acceptance covers
requests of 0.6 m/s, not the entire admitted speed range. Existing defaults for
other strategies remain 2 m/s.

The position/velocity command path does not forward acceleration. Short,
multi-piece plans can exceed its tracking envelope even below the requested speed.
The simulation profile now requires polynomial acceleration <=0.15 m/s² and
jerk <=0.30 m/s³. If necessary, the native optimizer retries timing with a 25%
lower velocity target, at most eight attempts within the original worker budget.
Numerically failed native solves are also retried within that same budget; every
result must still pass independent validation. Long route boxes retain up to
two additional observed-free cells along an already long axis, providing
braking room beyond the endpoints for measured residual velocity. This adds no free map cells or endpoint assumptions.
It preserves measured initial velocity, endpoints and collision checks; it never
scales an already generated polynomial. The owner independently checks the same
profile before admission. Infeasible profiles are rejected, and the original
0.20 m tracking stop remains unchanged. Requested speed is an upper bound;
these additional limits can produce substantially slower motion. They are a
conservative simulator profile, not validated hardware limits.

## Map and trajectory contract

`/livox/lidar/rays` reads the same cached native physics rays as the existing
Livox output. Each actual hit or maximum-range miss has the original scan stamp.
A miss clears its measured ray only. Invalid returns provide no free evidence.
The original Livox point outputs retain their previous semantics.

`gcopter_voxel_map` waits for `odom <- livox_frame` at that exact scan time.
`/planning/voxel_grid` has bounded row-major x/y/z storage, actual source time,
producer identity, reset generation, monotonic revision and inflation radius.
Values are -1 unknown, 0 observed free and 100 inflated occupied. Unknown is
blocked. By default, occupied cells persist until a complete map reset. The explicit
dynamic profile below can clear a removed obstacle using new ray evidence. Clock rollback clears evidence
and increments generation. Stale input publishes an invalid, zero-stamp grid.
Only the sensor mounting cell can be resolved from unknown to self-free when
valid rays exist; known obstacles are never erased by that operation.

The default map is [-4,8] x [-4,4] x [-0.2,3.4] m at 0.2 m resolution. The owner
requires inflation >=0.65 m and restricts whole planning cells to odom altitude
[0.5,2.5] m and an XY inset by the inflation radius. Inflation accounts for the
reviewed simulated iris body and the 0.20 m tracking bound. It is not a validated
hardware configuration or a claim that unseen space around the center is free.

A 26-connected search rejects diagonal corner cuts. Every convex corridor box
lies wholly in observed-free voxels. The actual GCOPTER optimizer receives exact
measured start position/velocity, zero initial acceleration and a resting goal.
The owner independently rechecks finite coefficients, C2 joins, endpoint states,
exact polynomial speed/acceleration/jerk extrema and continuous collision using
Bernstein bounds and recursive subdivision. Inconclusive checks reject the plan.
Scheduling stamps in `/planning/gcopter_trajectory` are command start times;
map/state source stamps remain separate and unchanged.

By default, the backend plans one complete goal trajectory in a fixed local map. It checks
all remaining pieces against fresh maps during execution. A map change stops
output immediately. If the measured position and current reference remain free
and tracking is valid, `/planning/gcopter_replan` requests the coordinator to
cancel the old child and acknowledge the exact old planner STOP before starting
a fresh session from measured state. The pre-existing maximum of three replans
and original 120 s navigation wall budget apply; neither is reset by replanning.
Current-position collision, stale inputs, duplicate command publishers and
tracking errors abort instead. Input recovery needs a new explicit goal. Map
identity/generation changes and clock rollback latch a full backend restart.

## Ownership and bounded computation

The existing navigation coordinator owns the FollowTrajectory child and exact
session UUID. GCOPTER uses `/planning/gcopter_session`. Its native worker is a
separate owned process with a 3 s wall deadline, 3 s CPU limit and 1 GiB address
space limit. STOP discards the plan, terminates/reaps the child and acknowledges
only after successful reap. Canceled or timed-out results cannot activate later.
Failure evidence remains in a private `gcopter-*` directory.

If the owner process itself exits, no stop acknowledgement can be fabricated.
The coordinator retains ownership, rejects another navigation goal and requires
the complete managed backend to stop before restart. Direct flight-controller
landing remains available. The 2D obstacle supervisor is disabled only for this
strategy; the owner performs the 3D remaining-trajectory checks instead.

Validation and exclusions are recorded in
`docs/development/gcopter-simulation-acceptance.md`. Native Gazebo truth is an
independent offline witness and is never an input to planning or control.


## Explicit dynamic map and rolling navigation

Add `gcopter_dynamic_navigation:=true` to the managed `navigation_dev` command
above. It requires the GCOPTER strategy and cannot be combined with the FUEL
policy map. All switches are read-only and default to false. For separate
entrypoints, pair `gcopter.launch.py dynamic_navigation:=true` with
`navigate.launch.py strategy:=gcopter gcopter_dynamic_navigation:=true`.
Starting either entrypoint remains idle.

The mapper clears a previously hit cell only after three consecutive, distinct
source scans observe free space through that cell. Multiple rays in one scan
count once; a hit wins over any misses in the same scan; absent visibility
interrupts the clearing streak. Duplicate/old scans cannot advance evidence.
There is no occupancy timeout. Inflation is rebuilt from the remaining hits.
`clear_observation_scans` may be increased up to 20.

The default 12 × 8 × 3.6 m window rolls in XY on the original 0.2 m lattice,
preserving world coordinates and evidence in the overlap. Newly entered cells
are unknown. Rolling advances revisions without replacing the map UUID or
reset generation. Read-only `world_xy_min`/`world_xy_max` fence parameters default
to [-100,-100]/[100,100] m. The source-time sensor transform controls rolling;
implausible transform displacement or time reversal invalidates and latches the
mapper. Source time, producer and clock validity checks still apply.

The owner reserves the complete vehicle/tracking envelope beside unknown cells,
continuously checks the remaining polynomial against current maps, and accepts
only lattice-preserving window changes. Coordinate resets, fractional-cell
shifts, changed dimensions/resolution and stale maps require stopping/restart as
specified above. An observed future obstacle can trigger the existing exact
cancel/STOP/replan sequence; present-position collision aborts.

The coordinator keeps one global parent goal and selects observed local endpoints
up to 4 m ahead. Only a successful child terminal and exact planner STOP permit
the next segment. Actual distance to the global goal must decrease by at least
0.25 m after each completed partial segment. The initial 300 s wall deadline,
8 replan budget and 20 planner-session budget are shared across all segments.
A missing safe observed endpoint or exhausted budget fails the parent. This
bounded progress selector is not a global maze search and does not execute
through unobserved space. GCOPTER must independently certify every local route.

Sufficient three-dimensional ray coverage is required for the vehicle envelope.
A centreline observed by a sensor with vertical blind regions does not establish
this coverage. Simulation qualifications using an explicit spherical ray fixture
must not be described as MID-360 hardware coverage or hardware flight acceptance.


### Dynamic profile qualification (2026-10-29)

The native PX4/Gazebo run in
`artifacts/stages1-3-20261028/gcopter-dynamic-native-braking` completed a 30 m
parent goal outside the initial window, across nine native trajectories and one
obstacle-triggered replan. Endpoint error was 0.0781 m; maximum source-aligned
tracking error was 0.1336 m, below the unchanged 0.20 m stop. A physically removed
wall cleared only through fresh ray evidence. A wall inserted during flight
occupied the previous route in the exact source-stamped worker map; the new
route passed independent collision checks. Physical minimum wall clearance was
0.921 m. The mapper retained one UUID/generation while its origin shifted more
than 20 m.

Actual mapper suspension produced a failed terminal in 1.02 s; exact owned
cancellation completed in 0.11 s. Both cases rejected a foreign flight action,
held position without automatic resumption, and permitted explicit landing.
All native children exited normally after disarm. The run used an explicitly
configured 20,000-ray spherical Gazebo scanner with 8 m range / 0.6 m near limit;
it is not a MID-360 blind-region or hardware qualification. Actual peak speed
was 0.438 m/s for a requested upper bound of 0.8 m/s.

The initial surface-observation fixture error, the real reverse-planning failure
and the intermediate unsuccessful retiming attempt remain in the evidence.
The exact reverse request is a repository regression. Both installation modes,
all GCOPTER component tests, coordinator behavior/ownership tests, explicit
profile checks and the dependent SUPER regression were checked separately.

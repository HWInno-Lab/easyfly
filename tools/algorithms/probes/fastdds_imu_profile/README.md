# Gazebo input delivery diagnostics

This is a manual diagnostic tool for ROS Humble / Fast DDS 2.6.11.
`imu_recovery.xml` changes only the `/livox/imu` writer heartbeat period to 50 ms.
`guard_input_recovery.xml` additionally names `/camera/color/image_raw` and
`/camera/color/camera_info`, because the latter also exhibited delayed reception.
Neither supplies participant,
transport, default writer, history, reliability or application timeout overrides.
It does not change measurement timestamps or enable navigation.

The normal launch does not load this experimental profile. A clean run with it
is not sufficient evidence that the intermittent delivery stall is fixed.

Build against the active ROS installation:

```bash
cmake -S tools/algorithms/probes/fastdds_imu_profile \
  -B artifacts/new-algorithms/fastdds-imu-probe
cmake --build artifacts/new-algorithms/fastdds-imu-probe -j2
```

Verify the native writer settings in a private ROS domain:

```bash
ROS_DOMAIN_ID=210 ROS_LOCALHOST_ONLY=1 \
FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/tools/algorithms/probes/fastdds_imu_profile/imu_recovery.xml" \
FASTDDS_DEFAULT_PROFILES_FILE="$PWD/tools/algorithms/probes/fastdds_imu_profile/imu_recovery.xml" \
artifacts/new-algorithms/fastdds-imu-probe/verify_profile
```

The verifier creates two native ROS publishers, publishes no sensor messages,
and checks `/livox/imu` is 50 ms, `/unrelated/imu` remains the native 3 s default,
and both histories remain 1000. It checks the middleware identifier before
accessing its exported native writer handle. It is specific to the installed
Fast DDS ABI and is not a generic middleware tool.

For the three-input XML, set both XML variables to `guard_input_recovery.xml` and
pass `--ros-args -p extended_inputs:=true` to the verifier. It also checks actual
Image and CameraInfo writers have 50 ms heartbeat and unchanged history 32.

For a controlled simulation comparison, pass the same two XML environment
variables to the simulation command, record raw/reliable/best-effort/guarded
receipts with `openvins_input_trace.py`, and optionally collect producer timing
using `EASYFLY_IMU_TIMING_TRACE`. Compare with a fresh process using the ordinary
caller profile. Use separate outputs and Gazebo ports for each run. Preserve
both failures and clean runs; unchanged guard limits determine acceptance.

## Native and Python observer comparison

Source the repository's validated ROS/simulation overlay first. `imu_receipts`
is an independent C++ observer with reliable raw/guarded IMU and best-effort raw
IMU subscriptions. It records source stamps, publisher GIDs/sequences, six IMU
values, RMW source/reception metadata, and monotonic/realtime callback timestamps.
The optional native middleware notification hook records data availability,
which is not a wire arrival timestamp. Records are bounded (200,000 by default)
and flushed on normal shutdown. A `.ready` marker precedes the test producer.

The runner starts the native and Python observers before a managed static Gazebo
stack, uses the same verified private Fast DDS library for all participants, and
records commands, return codes, producer timing and actual merged Gazebo XML:

```bash
python3 tools/algorithms/probes/run_imu_observer_comparison.py \
  --native-executable artifacts/new-algorithms/fastdds-imu-probe/imu_receipts \
  --output artifacts/new-algorithms/native-baseline \
  --seconds 120 --domain-id 209 --master-port 11631 \
  --native-camera-load --no-native-notifications --audit-guard
```

Use a fresh output path for every case and an unused private domain/Gazebo port.
For the candidate, add `--profile` with the absolute path to
`guard_input_recovery.xml`. For a scheduling positive control, use `--bag` with a
recorded input bag and `--pause-after 200 --pause-ms 700`; the pause sleeps only
the independent native executor. Middleware threads and the real guard continue.
The runner never sends flight goals. It stops only its owned process groups.

`--audit-guard` explicitly adds the sibling `guard_trace_hook` directory to the
test process's `PYTHONPATH`. This diagnostic applies only to the installed
`openvins_guard` entry point. It preserves metadata from the existing single
Humble executor take and wraps callback durations, without adding a second take.
It checks the reviewed executor implementation, has a bounded trace and writes
on exit. This hook is not installed or enabled in ordinary launches. It changes
measurement overhead and is therefore accompanied by controls without the hook.

Analyze a case with its exact filenames:

```bash
python3 tools/algorithms/probes/analyze_native_imu.py \
  --native /absolute/case/native.csv \
  --python /absolute/case/python/samples.jsonl \
  --producer /absolute/case/producer.PID.imu_plugin.csv \
  --guard /absolute/case/guard-take.PID.json \
  --output /absolute/case/analysis.json
```

`--producer` and `--guard` are optional. Reception-to-take latency includes
middleware availability/ordering and executor scheduling; it must not be labeled
entirely Python cost. In this Fast DDS version, RMW metadata uses the native
SampleInfo source/reception times. It is not a packet capture. Per-sample native
notification association is reported only for complete, ordered single-writer
traces whose announced counts match. Six-value comparison is not an all-field
IMU comparison; the Python raw/guarded digest comparison checks all fields within
its observed admitted window. No observer proves another reader's receipt.

Results and limitations: [native receive report](../../../../docs/development/openvins-native-receive.md).

## Repeated contention and recovery comparison

An optional native Gazebo transport observer hashes the color camera pixels on
the independent `gazebo.msgs.ImageStamped` path. It does not modify the sensor or
ROS plugin. Enable its build explicitly (Gazebo 11 and OpenSSL development
packages are required):

```bash
cmake -S tools/algorithms/probes/fastdds_imu_profile \
  -B artifacts/new-algorithms/fastdds-imu-probe -DBUILD_GAZEBO_CAMERA_PROBE=ON
cmake --build artifacts/new-algorithms/fastdds-imu-probe -j2
python3 tools/algorithms/probes/run_imu_recovery_matrix.py \
  --native-executable artifacts/new-algorithms/fastdds-imu-probe/imu_receipts \
  --gazebo-camera-executable artifacts/new-algorithms/fastdds-imu-probe/gazebo_camera_receipts \
  --output artifacts/new-algorithms/recovery-matrix
```

The matrix runs seven sequential cold starts: ordinary/candidate at normal CPU
availability, two pairs of ordinary/candidate with contention in reversed order,
then a candidate 200 ms guard pause. Normal/contention cases last 120 s; the
pause case lasts 65 s. It reserves ROS domains 220–226 and Gazebo ports
11741–11747; use it only while these are unused. Each case records full commands.
It aborts on infrastructure failure, preserving failed and completed evidence.
Admission failure is an observed result, rather than an infrastructure exit.

The runner's `--cpu-limit 4 --cpu-workers 2` uses `taskset` on only its own child
processes, selecting the first four CPUs from its allowed set. Two bounded load
workers request 75 ms busy work per 100 ms wall period, and log actual CPU time.
This provides reproducible contention without changing system settings. Actual
CPU share depends on scheduling and must be reported from the worker logs.

`--guard-pause-after 35 --guard-pause-ms 800` adds a long-pause negative control
to an individual managed run (use `--seconds 65`). The runner finds exactly one
descendant guard, requires healthy input admission before injection, sends it
SIGSTOP and then SIGCONT, recording actual monotonic pause times. The entire
guard process, including its middleware threads, pauses. This is not packet-loss
injection; no host network settings are changed. A short pause may drain queued
inputs; an over-limit pause must preserve the guard's stale/fault behavior.

With the matrix complete and a separate long-pause output available:

```bash
python3 tools/algorithms/probes/summarize_imu_recovery.py \
  --matrix /absolute/recovery-matrix \
  --long-pause /absolute/long-pause \
  --output /absolute/recovery-summary.json
```

This preserves raw traces and derives native/guard/camera analyses. The camera
analysis joins timestamp, pixel SHA256 and geometry; observers' different start
and end coverage is retained. Identical pixels with duplicate timestamps in both
Gazebo and ROS establish the ambiguity existed upstream of DDS. SHA256 is not a
render exposure timestamp and cannot prove simulation-to-exposure correctness.
The Python comparison accepts any observed raw payload at an ambiguous stamp;
it still reports that ambiguity rather than treating the last duplicate as the
only valid original. Default launch, guard limits and navigation remain unchanged.

If a guard has already failed before the planned pause, preserve that failed
case. An explicit separate retry can be included with `--short-pause-retry`.
The aggregate preserves unsuccessful executions and distinguishes evidence
completeness from admission health; its exit 0 is not an acceptance verdict.
The initial interrupted observer cleanup failure required a separately labeled
`summary.recovered.json` from preserved JSONL. This historical artifact is not
treated as a clean execution. The observer now preserves its summary on SIGINT.

Repeated results: [pressure/recovery report](../../../../docs/development/openvins-recovery-pressure.md).

## Minimal source and rendering isolation

The sibling `run_vio_source_isolation.py` copies the specified iris RGB sensor
and installed MID-360 IMU into a static world with simple geometry. Sensor
parameters remain unchanged; only plugin filenames resolve to installed absolute
paths. No PX4, lidar, detectors or flight/control nodes run. This static source
fixture does not validate vehicle extrinsics or pose accuracy.

```bash
python3 tools/algorithms/probes/run_vio_source_isolation.py \
  --iris-sdf src/simulation/easyfly_px4_sitl/models/easyfly_iris/easyfly_iris.sdf \
  --native-executable artifacts/new-algorithms/fastdds-imu-probe/imu_receipts \
  --gazebo-camera-executable artifacts/new-algorithms/fastdds-imu-probe/gazebo_camera_receipts \
  --output /absolute/fresh-case --seconds 60 --domain-id 215 --master-port 11831
python3 tools/algorithms/probes/analyze_vio_source.py \
  --case /absolute/fresh-case --output /absolute/fresh-case/source-summary.json
```

For matched cases, explicitly add `--lockstep`, `--cpu-limit 4 --cpu-workers 2`,
or `--localization` (installed guard and actual OpenVINS only). The existing
`--profile /absolute/guard_input_recovery.xml` is a separate factor. Do not change
both rendering and middleware settings and attribute the result to only one.
Keep fresh paths and unused domains/ports. Source quality, localization admission
and evidence/exit completeness are separate reported fields.

Both runners now signal ROS launch parents rather than their entire process
groups, and check logs for abnormal nested child exits. The original group-wide
SIGINT path could interrupt child cleanup; parent exit 0 alone was insufficient.
Historical raw files remain unchanged, with a separate child-exit audit and
audited aggregate. See [source isolation report](../../../../docs/development/openvins-source-isolation.md).

## Managed rendering/recovery combination

The managed runner accepts `--gazebo-lockstep`. It creates an ament overlay only
inside the fresh output directory: package resources/libraries link to the real
installed package, while a tiny server entry point execs the same native server
with one additional `--lockstep` flag. Nothing is installed over a normal prefix.
The actual `/proc` executable, arguments and binary hash are recorded and checked
against the requested mode. This avoids changing normal profile startup for an
experiment. The staged wrapper and links remain as reproducibility artifacts.

```bash
python3 tools/algorithms/probes/run_imu_observer_comparison.py \
  --native-executable artifacts/new-algorithms/fastdds-imu-probe/imu_receipts \
  --gazebo-camera-executable artifacts/new-algorithms/fastdds-imu-probe/gazebo_camera_receipts \
  --profile "$PWD/tools/algorithms/probes/fastdds_imu_profile/guard_input_recovery.xml" \
  --output /absolute/fresh-managed-case --seconds 120 \
  --domain-id 210 --master-port 11931 --gazebo-lockstep \
  --native-camera-load --no-native-notifications --audit-guard
```

For the rendering control, omit only `--gazebo-lockstep`; keep the heartbeat XML
and observers identical. Add `--cpu-limit 4 --cpu-workers 2` for bounded contention,
or `--cpu-limit 4` alone to separate CPU affinity from the extra workers. These
are stationary source/admission experiments with the full managed profile;
they never send flight goals or certify dynamic localization.

After recording case commands in a matrix JSON (`cases` with `name`/`command`/
`returncode`, and `error`), aggregate the existing cases:

```bash
python3 tools/algorithms/probes/analyze_managed_sync.py \
  --matrix /absolute/managed-matrix \
  --extra-case /absolute/managed-matrix/four-cpu-combo \
  --output /absolute/managed-matrix/summary.json
```

The optional extra case supports separately recorded resource-boundary controls.
The aggregate includes original analyses, actual server configuration, source
production budgets in five-second bins and process startup times from ROS launch
logs. Coarse timeline correlation does not identify a specific blocking function.
Admission failures and abnormal exits remain visible; analysis exit 0 only means
the recorded evidence is complete, not that every case passed admission.

### Sensor load isolation

Add `--sensor-load all|no-lidar|no-depth|rgb-imu` to the lockstep command above,
with `--cpu-limit 4`, no extra CPU workers, and a 90-second observation window.
All four choices use the same explicit low-level diagnostic launch, rather than
registering modified resources as a normal managed scene. The launcher resolves
and validates the original scene first, then stages model copies under the fresh
output directory. It removes only the requested `laser_livox` and/or `depth`
sensor nodes; RGB, Livox IMU, model dynamics, world, spawn pose, clock, heartbeat,
and the remaining PX4/perception/platform components keep their configuration.
Original resources and production launch files are untouched. Every variant
keeps OpenVINS observation only and sends no flight goals.

The Python observer's optional `--sensor-topology` records read-only publisher
graph snapshots. Compare the last three snapshots against the requested sensor
set, in addition to checking the copied SDF. This proves graph presence/absence
at those snapshots, not continuous message delivery or exact function timing.
Removing a sensor also removes its downstream traffic and processing, so the
comparison measures its total stack effect, not just rendering or ray tracing.

Record two 90-second runs of each choice in a matrix, then run:

```bash
python3 tools/algorithms/probes/analyze_sensor_load.py \
  --matrix /absolute/sensor-load-matrix \
  --output /absolute/sensor-load-matrix/summary.json
```

The analysis checks original/staged fingerprints, exact sensor-node removal,
publisher snapshots, matching controls, trace completeness, nested process exits,
and cleanup. It reports source and wall-clock gaps separately. Exit 0 certifies
complete comparison evidence, including any recorded admission failures.

### Stack layer isolation

Use the same command with `--sensor-load rgb-imu --stack-layer LAYER
--process-budget`, a 75-second capture and `--cpu-limit 4`, without extra workers.
Available layers are `tiny`, `world`, `sitl`, `bridge`, `perception`, and `full`:

- `tiny`: static RGB/IMU mount, three simple visual boxes, original scene physics/light.
- `world`: same static mount in the original scene geometry.
- `sitl`: original airframe and PX4, with lidar/depth removed as above.
- `bridge`: add MAVROS/bridge/RC receiver and calibration/coordinate components.
- `perception`: add detection/tracking/pixel conversion and pointcloud/semantic processing.
- `full`: add the original idle execution/navigation and platform services.

Each layer starts the same OpenVINS guard, estimator and alignment observer at
the original 14-second launch timer. The generated launch copies change only
the reviewed `actions` and `p2_nodes` assignment sites in the installed launch;
an AST check rejects other edits. `full` retains the original launch actions.
The private static mount preserves the RGB sensor and nested MID-360 IMU mount.
No production files or normal presets are changed, and no goals are sent.

For these experiments the native image observer starts after the server's master
connection log. Native/ROS image identity must be evaluated in their common
source-time coverage, with excluded startup/tail frames reported separately.
Read-only CPU counters sample only the launched stack tree every five seconds;
independent observers are outside that accounting. Counters omit a process's
first partial sampling interval and cannot measure specific function/lock waits.
The kernel scheduling-statistics setting is recorded without changing it.

After the six first-pass layers, repeat both sides of the first adjacent change
in admission result in reverse order. If none changes, repeat endpoint controls.
Record that selection rule and exact commands in the matrix, then audit:

```bash
python3 tools/algorithms/probes/analyze_stack_layers.py \
  --matrix /absolute/layer-matrix \
  --output /absolute/layer-matrix/summary.json
```

The report retains every admission failure and verifies repeated outcomes
separately from complete evidence. Source-model/launch fingerprints, actual
process counts (including component containers), topology snapshots, shared
image identity, fixed runtime controls and process cleanup are audited.

### Bridge split and first STALE audit

The additional layers `communications` and `calibration` split `bridge` into
MAVROS/mavlink_bridge/RC receiver versus the two calibration/coordinate component
containers. Compare `sitl`, `communications`, `calibration`, and `bridge` twice,
in forward then reverse order, keeping all other layer controls identical.
Without communications, coordinate/alignment consumers lack their usual FCU
reference input; this is an interaction experiment, not equal downstream work.

Add `--audit-freshness` to the stack-layer command. Only the opt-in guard process
wraps the existing `Gate.status(now)` call once and records its actual result,
the six input-freshness conditions, current limits, source ages, receipt ages,
latest raw callback ages, pending dependencies and forward-completion markers.
The camera receipt used by the guard belongs to the admitted original image;
it is not the image's later publish time. The image marker measures completion
of a publish batch; the IMU marker is the end of a successful publishing callback.
Neither marker is a subscriber receipt or a new measurement timestamp. No extra
message take, status evaluation, payload copy or production configuration is added.

```bash
python3 tools/algorithms/probes/analyze_bridge_split.py \
  --matrix /absolute/bridge-split-matrix \
  --output /absolute/bridge-split-matrix/summary.json
```

The analysis independently recalculates each recorded input condition, retains
the first real STALE snapshot and joins matching guard RMW take/native image
receipt records. Native/RMW receipt times do not prove exposure or wire timing.
The output keeps mixed repeated outcomes visible and distinguishes complete
evidence from input admission. Default launch and guard thresholds are unchanged.

References:

- [MAVROS plugin/alignment and IMU wait isolation](../../../../docs/development/openvins-mavros-isolation.md)
- [Communication subgroups and actual guard Clock delivery](../../../../docs/development/openvins-communications-clock.md)
- [Bridge split, exact freshness conditions and clock visibility limits](../../../../docs/development/openvins-bridge-split.md)
- [Stack layer results and the repeatable admission boundary](../../../../docs/development/openvins-stack-layers.md)
- [Sensor load isolation results and remaining limits](../../../../docs/development/openvins-sensor-load.md)
- [Managed synchronization experiment results and resource limits](../../../../docs/development/openvins-managed-sync.md)
- [Fast DDS 2.6.11 writer profiles](https://fast-dds.docs.eprosima.com/en/v2.6.11/fastdds/xml_configuration/datawriter.html)
- [ROS Humble named writer profile loading](https://github.com/ros2/rmw_fastrtps/blob/humble/rmw_fastrtps_cpp/src/publisher.cpp)

### MAVROS versus project bridge/RC, and actual guard Clock delivery

Use `--stack-layer mavros` or `--stack-layer project_bridge` alongside `sitl`
and `communications`, twice in forward/reverse order with the same controls.
The explicit launch copies filter only the returned communication actions and
redirect the existing include to that copy. Their node parameters, delays and
other launch statements remain unchanged; independent AST checks verify this.
The project-only group has no MAVROS telemetry, so downstream reference work is
not equal across groups. Do not interpret this as an additive CPU benchmark.

Add `--audit-clock --audit-freshness --audit-guard`. The opt-in guard hook records
Clock payloads from its existing executor take and wraps the existing TimeSource
callback exactly once, recording cached/associated clocks after that callback.
It reads the actual subscription QoS and publisher endpoint graph every ten
seconds, with no extra subscription, take, clock write or production change.
Graph UNKNOWN history/depth 0 is retained as unknown, not a configured writer
depth. Metadata fields are reported explicitly; no sequence or availability
information is invented when the installed RMW does not expose it.

```bash
python3 tools/algorithms/probes/analyze_communications_clock.py \
  --matrix /absolute/communications-clock-matrix \
  --output /absolute/communications-clock-matrix/summary.json
```

Clock take/callback pairing and actual Gate clock snapshots must agree. Separate
readers' observed source stamps are compared only in their common time window;
an absent sample in one reader is not proof of network loss. The report's
complete-evidence result does not require every group to admit healthy inputs.

### MAVROS plugin/alignment factorial and native wait intervals

For the `communications` layer, add `--mavros-experiment` with `all_on`,
`all_off`, `reference_on`, or `reference_off`, plus `--step-trace-plugin` and
`--poll-trace-library` absolute paths. Clock/freshness/guard audits remain
required. Reference modes add exactly `imu`, `local_position`, `sys_status`,
`sys_time` to the MAVROS allowlist. On/off toggles the existing alignment launch
argument. Keep bridge/RC in every group and run forward/reverse repeats.

Build the private libraries under an artifact directory, without installing over
production files (exact commands are recorded in `build-commands.json`):

```bash
g++ -std=c++17 -O2 -fPIC -shared \
  tools/algorithms/probes/sim_wait_trace/step_trace.cpp \
  -o /absolute/artifacts/libeasyfly_step_trace.so $(pkg-config --cflags --libs gazebo)
g++ -std=c++17 -O2 -fPIC -shared \
  tools/algorithms/probes/sim_wait_trace/poll_trace.cpp \
  -o /absolute/artifacts/libeasyfly_poll_trace.so -ldl
```

The world copy adds one observer plugin. Only the owned server loads the poll
wrapper; its actual mapped libraries and fingerprints must be recorded. World
begin/end spans include callbacks and physics, not physics alone. Poll records
must be contained in those intervals; timestamps use the same host monotonic
clock as the guard and native observers. Traces record capacity and discards.

```bash
python3 tools/algorithms/probes/analyze_mavros_isolation.py \
  --matrix /absolute/mavros-isolation-matrix \
  --output /absolute/mavros-isolation-matrix/summary.json
```

For separately labelled focused runs, compile `condition_trace.cpp` alongside
`poll_trace.cpp` to another library. It adds original `pthread_cond_clockwait`
intervals from the MAVLink library and writes `poll.csv.conditions.csv`. Verify
the actual library SHA and return-address offset against its disassembly before
attributing calls to OnUpdate. Condition intervals include sleeping, scheduling,
and mutex reacquisition; their duration does not identify the upstream delay.
Analyse these two focused runs using `analyze_sim_conditions.py` with a separate
`conditions.json` matrix. Keep failed attempts and shutdown failures visible.

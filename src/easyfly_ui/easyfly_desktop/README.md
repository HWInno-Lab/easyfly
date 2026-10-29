# EasyFly desktop (read-only by default)

Native C++17 / Qt5 Widgets desktop. Uses `easyfly_ui_bridge` to observe an
already-running ROS graph. The standalone desktop never launches a backend; the
optional integrated connection manager provides explicit simulation/communications startup. Optional independent
viewing tools launch only after explicit user clicks. Managed simulation mission
operations require separate explicit startup opt-in; no standalone flight buttons exist.

See [OPERATIONS.md](OPERATIONS.md) for the development delivery boundary, opt-in
matrix, startup/close procedure, camera-source distinction and known limitations.

The spatial page also provides an offline geographic map with explicit WGS84/ENU
datum and an optional complete aircraft display. See [GEOGRAPHY.md](GEOGRAPHY.md)
for map import, source attribution, coordinate and TF requirements.

## Build and run

From the workspace with compatible ROS Humble/Qt/system dependencies:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to easyfly_desktop --executor sequential --cmake-args \
  -DBUILD_TESTING=ON -DEASYFLY_ENABLE_RVIZ_EMBED=OFF -DEASYFLY_ENABLE_MEDIA=OFF
source install/local_setup.bash
ros2 run easyfly_desktop easyfly_desktop
```

The workspace dependency closure is now six packages, including the pure
easyfly_mission_format library instead of the mission/flight runtime. For a clean
prefix, do not source the old workspace; use the independent build/install/log
directories shown in OPERATIONS.md. This is not a clean-machine dependency installer.

The default uses simulation time. With no `/clock`, state/images/geometry remain
unavailable; the window still opens. Use `--system-time` only for a system-time
backend. `ROS_DOMAIN_ID` and `ROS_LOCALHOST_ONLY` are inherited at startup; the
window does not change them. A domain is discovery isolation, not authentication.

Options: `--profile` (selection label only), `--front-topic`, `--down-topic`
(empty disables that camera), `--system-time`, `--help`.
In standalone mode, changing profile, topics or time source requires disconnecting
and completing local cleanup before reconnecting. In a managed workspace, the first
connection page owns editing and rebuilds the workspace with a frozen environment;
diagnostics shows effective settings read-only. Backend profile is confirmed
only by an explicit whitelisted read, not by selection or capabilities.

## Pages and boundaries

Fixed navigation: **连接与仿真 → 运行总览 → 任务中心 → 相机与感知 → 空间视图 →
记录与回放 → 诊断与设置**. The task center groups catalog, structured draft editor
(with advanced YAML tab), import and execution/query. Diagnostics/settings contains
advanced independent tools. With the optional manager installed, connection editing
and QGC executable selection live on the first page. Before connection, local draft,
record and settings pages remain usable without creating a ROS context.

The light blue-gray workspace uses a pale blue sidebar, white cards and blue primary
actions; only the narrow menu strip is dark slate. Blue icons, selection markers and
primary actions provide accents; warnings retain readable amber text/borders.
The supplied EasyFly logo is compiled into Qt resources, so runtime loading does
not depend on the ignored docs directory or the original source location.
Qt palette and widget styles are scoped to this window in `src/theme.cpp`, not
the entire application or ROS configuration. Tests check surfaces and contrast;
data validity and control admission still depend on the unchanged observation
contract, never on the color of a field.

- Overview: icon status cards, an artificial horizon and local heading compass,
  local XY map/actual trajectory and selected camera in independent panels.
  Position and attitude have separate validity: invalid attitude removes the
  compass/attitude needles and map heading arrow; valid position can remain a
  point. Stale/paused/reset/disconnected observation never displays neutral
  instruments as live data. Heading = (90 - ENU yaw in degrees) modulo 360:
  local +Y/north=0, +X/east=90; this is not magnetic or geographic true north.
  Roll/pitch follow ROS ENU/FLU (positive pitch is nose-down). Local Z is **not**
  ground clearance or height above the takeoff point. Independent validity,
  clock/receipt/source evidence and unbound global phase text are expandable;
  the event dock is available via View or the overview button, hidden initially.
  The 3D shortcut selects **空间视图 → 内嵌 RViz**, where the single renderer is
  manually opened when the installed build supports it. Hiding the page releases
  rendering; it never auto-reopens. The page remains present with an explanation
  when embedding was not compiled. Local grid/XY is not a SLAM or satellite basemap;
  RViz layers require actual publishers/TF and remain manually enabled/read-only.
- Missions: list catalog basenames, static validation, inspect validated catalog
  content/steps/defaults, query an explicitly known 32-character service
  `execution_id`. No active-ID/history discovery, arbitrary file access,
  mission start or cancel in the default mode. Optional managed controls below
  do not use the manually entered query ID. Service success is not execution
  success; `execution_pending` is shown separately. `not_found` proves no release.
- Perception: selectable front/down image and measured target/obstacle tables.
  Default is the latest raw image. Optional same-frame detection/tracking views
  use exact image stamp/frame matching, not guessed overlays on newer images.
  Tracking labels are raw SORT IDs, not stable/reidentified or physical identities;
  upstream tracking may include predicted boxes without a per-box flag. Depth is
  upstream fused optical-axis depth, not slant range. Missing/invalid/ambiguous
  matches display no depth instead of zero. Independent predictions, semantic
  obstacle copies and landing observation adapters are not included.
- Local XY: actual trajectory and current odom geometry only, not a planned
  route, occupied map, global geographic map or navigation input.
- Diagnostics: capability details, four whitelisted parameter reads, local
  observer evidence and bounded events. Default mode has no writes or fault
  clearing. The separately opted-in metadata editor is described below.
- Tools: explicit RViz viewing presets and offline-isolated QGC preview.

All upstream strings are plain text; tables are non-editable and do not enable
rich-text tooltips. Global phase events lack execution identity/heartbeat and
are not attached to the selected catalog file. Historical `command_uncertain`
warnings survive reconnect; new telemetry does not establish fault release.

Qt owns all widgets on the main thread. ROS callbacks never touch Qt. A single
background converter owns copied QImage pixels and at most one active, one
latest pending and one latest output frame. Camera/session/time-epoch binding
prevents stale converted output from reappearing. The main thread polls at 10Hz;
it never waits for a ROS service response. Source age is never advanced using
steady time. Existing freshness policy and original image/query budgets remain unchanged.

Selecting a same-frame view lazily adds one separate bounded converter. It saves
the exact metadata with the image conversion, checks freshness again against
the current session/clock, and never uses a newer result to label an older frame.
Overview and recording continue to use the original raw converter, even when
the perception page shows a delayed matched image. Painting does not modify
source or recording pixels. Labels are placed without overlap where space permits;
long labels are elided and crowded labels may be omitted, with details in the table.

In default read-only mode, disconnect, close and SIGINT/SIGTERM request nonblocking local stop. The window
stays responsive until its own observer and converter report stopped, then
releases them. No global ROS shutdown, external goal cancellation, process
search/kill or cleanup of user-owned nodes/tools occurs. Direct C++ destruction
without the normal close protocol can join local threads; application entry
and tests use the close protocol.

## Isolated checks

Desktop-only checks below do not include the moved platform-service suites.
Build/run easyfly_desktop_tests explicitly in an updated full developer workspace
for import, parameter/media and managed-task integration coverage. No suite was
deleted. Run packages/feature variants sequentially in their fixed test domains.

```bash
ctest --test-dir build/easyfly_desktop --output-on-failure
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 \
  ctest --test-dir build/easyfly_desktop -R '^cppcheck$' --output-on-failure
QT_QPA_PLATFORM=offscreen ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=1 \
  ros2 run easyfly_desktop easyfly_desktop --quit-after-ms 1000
```

GUI tests use offscreen Qt and domain 93 with isolated read-only service mocks.
They spy on all nine action endpoints, control messages, mode/arming services,
task operations and parameter writes. Image tests check format/layout ownership,
latest-slot overload, rebinding and nonblocking shutdown. Widget tests cover
offline/stale/paused/invalid data, IDs, pending ownership, pure text, limited
tables, reconnect and small-window navigation. Local screenshots generated by
the widget test carry an explicit mock-data watermark; they are not flight or
real-camera evidence. Screenshot locations are `docs/ui/d6-2/` and the updated
dashboard's `docs/ui/overview-refresh/` (ignored by Git).

These checks do not establish SITL/hardware readiness, desktop window-manager
behavior on every machine, real camera throughput or long-duration performance.
D6-4 enables only explicitly opted-in managed simulation missions; D6-3 does not
authorize QGC networking. Hardware operations remain unavailable in the UI.

One bridge regression run terminated with SIGSEGV in its image/global-event
test. Targeted and whole-suite reruns did not reproduce it. The test now retains
the same immutable snapshot it validated rather than dereferencing a separately
loaded snapshot, but without a crash stack the original runtime cause remains
unconfirmed. Passing reruns are not evidence of a certified crash fix.

Correction to the earlier independent retest attribution: the captured gdb
run had no ROS_DOMAIN_ID, so SetUp failed before creating the executor or the
production Observer. Its unconditional null-executor cleanup then crashed.
This is a confirmed test-harness defect, not evidence of a production teardown
race. Cleanup now shuts down only the owned context and tolerates partial
initialization; missing-domain and post-context fault-injection probes verify
normal assertion failure rather than a crash. A different, correctly configured
CTest run failed without that SetUp assertion and without a stack; its cause
remains unconfirmed. Ten correctly configured whole-suite repetitions passed,
which does not certify a fix for that separate failure.

## Mission details and identity design (D6-4 first increment)

The task page explicitly reads `/platform/mission/details`
(`easyfly_msgs/srv/GetMissionDetails`) from the existing mission-service process.
The server reads only its constrained catalog; it returns original validated
bytes and their SHA-256 without rewriting defaults, creating a task or sending
an action. The bridge checks the exact requested filename, size, UTF-8, digest
and complete static validation on its worker. Normalized steps and format
metadata come from the shared mission parser, not a second GUI grammar.
The GUI shows steps/defaults/units/timeouts and a separate read-only raw-text tab.

Use the updated easyfly_msgs/easyfly_mission_format/easyfly_mission_fsm/easyfly_mission_srvs packages
together with the bridge/desktop. A backend without this additive service shows
unavailable; no local-file or arbitrary-server-path fallback exists.
ManageMission's wire contract is unchanged. Default read-only startup cannot
start/cancel missions; managed opt-in uses the separate protocol described below.
The format library now uses the installed OpenSSL Crypto library for SHA-256.
Its development dependency is declared as
[`libssl-dev`](https://index.ros.org/d/libssl-dev/) in the ROS dependency manifest.

Selection changes, pending requests, errors and reconnect cannot turn a previous
preview into a new selected task. A cache older than two seconds is explicitly
historical, not launch confirmation. ROS clock reset discards old replies/caches.
Content is capped at 64KiB and steps at 256, upstream text is plain text,
numeric parameters are displayed with six decimals; exact input remains in the
raw-content tab. Inspection is header-less: source age is unavailable, not zero.

Identity boundaries: basename selects a catalog entry, SHA-256 identifies
reviewed bytes, ManageMission execution_id scopes a service execution, and
Action UUID scopes an SDK action. Observer session/request/epoch only associate
local observations. Querying an ID does not establish ownership of that task.
Global phase events, empty lists or fresh telemetry prove neither identity nor
release. Task files have no profile field; selected UI profile and explicitly
queried backend profile are separate evidence, not an atomic mission binding.

The service package now provides a separate ManageReviewedMission protocol for
same-read reviewed-content binding, client-request association and exact cancel, described in
its README. The independent easyfly_ui_control package provides durable intent
storage and lookup-only restoration, separate from the passive Source/Observer.
Default startup does not create this controller or its buttons. There is no
legacy-start fallback, cancel-all or fault-clear operation. A public terminal
status with execution_pending=true still means pending ownership drainage.

## Opt-in managed simulation missions (D6-4 fourth increment)

Use a dedicated persistent absolute directory whose parent already exists:

```bash
ROS_LOCALHOST_ONLY=1 ros2 run easyfly_desktop easyfly_desktop \
  --enable-mission-control --mission-journal /absolute/persistent/path/easyfly-intent
```

Both flags are required together; relative/dot/trailing-slash paths and
--system-time are rejected. A temporary directory is appropriate only in tests,
not for actual recovery. Keep the same directory/domain/localhost scope on restart.
The journal enforces private ownership and an exclusive lock. Different directories
do not provide mutual exclusion; do not switch directories to bypass a fault lock.

Invalid/occupied startup directories now fail before opening a window: the CLI
constructs one controller with a synchronously acquired journal lock, and exits2
with a diagnostic on local directory/lock failure. It transfers that same
controller into the window; there is no validation/unlock/reopen race. Existing
bytes and permissions are untouched. Let the application create mode0700 or
pre-create a private directory. Corrupt/unknown records, scope mismatch and runtime
faults still require the conservative unresolved exit choice; startup failure
does not prove a remote task stopped or authorize switching journals.

The manual simulation-only acknowledgement starts unchecked and resets on
observer session/time-epoch change. New starts require localhost-only discovery,
an advancing valid simulation clock, fresh valid state, explicit fault evidence,
queried boolean use_sim_time=true and an allowlisted queried simulation profile
matching the UI choice. Supported profiles: base_sim, full_perception,
navigation_dev and landing_dev. These are UI checks, not hardware authentication
or live flight admission; localhost can still host a hardware driver. The operator
must ensure the domain contains only simulation. The mission backend continues
to arbitrate each action. Ready does not authorize flight.

Read and review the selected task explicitly. Confirmations show name, normalized
step count, frame/units, original SHA-256, chosen profile and server instance;
default/escape is cancel. Human review can last more than two seconds: the old
copy is for review, never fresh flight evidence. After confirmation a bounded
read-only recheck obtains the exact same bytes/hash/name and new profile,
use_sim_time and capabilities in the original observer session/epoch. Config
must match the chosen simulation profile. State remains a live observation.

The existing Observer's endpoint/concurrency/rate limits still apply. Only
not-yet-queued reads may be deferred on Busy/RateLimited, within a five-second
overall recheck deadline; an accepted read is never reissued. New background
capability replies in the same session/epoch may supply fresh evidence. The
two parameter reads share one endpoint and are therefore naturally sequenced.
Reply errors, changes, superseding manual reads or timeout never dispatch start;
another explicit review/confirmation is required. A separate button cancels only
this local read-only recheck. Closing/session changes also revoke it; late replies
cannot send a goal. There is no automatic flight retry.

Only after exact-content recheck and all original admission checks pass is one
start enqueued. Rechecked bytes, configuration/capabilities and session still
have a two-second freshness budget, state one second; these thresholds were not
extended for human review. The server independently compares the same reviewed
bytes/hash again at admission. UI permission is not backend flight admission.

Only the locally journaled execution can be canceled, with another default-cancel
confirmation. The catalog selection and hand-entered query ID cannot change its
target. Owned cancel does not require fresh telemetry or a checked new-start permit.
It does not adopt legacy/external tasks. Cancel ACK and public terminal status are
not release; running, uncertain, cancel-pending, drain-pending and fault remain distinct.
Confirmed resolved means the intent is resolved, not necessarily successful.

Closing with unresolved/fault/local-queued state asks first (default: stay).
Explicit close stops local communication only, leaves remote work running and
retains the journal. Restart queries only the original request/instance; it never
replays start/cancel or silently clears a lock. Instance change and unknown state
remain blocked. Normal close waits for the control worker as well as the observer,
image worker and any owned tool cleanup.

test_mission_controls uses fake ports to verify confirmation/preflight/selection,
exact cancellation and close behavior. test_managed_stack connects real Qt,
MissionControl and the real mission service to mock ExecuteMission/ownership
providers in isolated domain115. Its healthy telemetry and profile evidence are
fabricated, not genuine flight observations. It verifies one submission, exact
cancel, terminal ownership drainage and lookup-only reopen even before admission
reply. A combined owned-tool test covers both exit prompts, default stay,
resetting a canceled exit choice, preserving unresolved intent and leaving a
user-started external tool untouched. These are fixtures, not real RViz/QGC
simultaneous-flight acceptance. Fixture teardown finishes only its mock action
after assertions. Screenshots
in the separately archived docs/ui/d6-4 evidence have explicit mock labels.
These fixtures establish no PX4/Gazebo/SITL/hardware readiness. Later real SITL
qualification is described below; server cross-restart persistence remains deferred.

A separate, manually confirmed SITL check on 2026-10-13 used localhost-only
ROS domain117, the `base_sim` profile and a private catalog containing only
1.5 m takeoff, 5 s hover and ordinary land. The recorded two ExecuteMission
goals and takeoff/land goals reached SUCCEEDED. The final managed request was
also queried as succeeded, execution_pending=false, with its exact Action UUID
reported RELEASED. PX4 confirmed landed and disarmed, and the UI exited normally.
This is a bounded simulation check, not hardware or full lifecycle acceptance.
Initial native estimator/GPS admission failure, one Gazebo shutdown -6 and a
repeated parameter-manager shutdown exception remain in the historical record.
The parameter-manager signal race has since been fixed and tested. On 2026-10-26,
two actual C08 SITL runs verified RViz/QGC, recording, closing with an unresolved
task, unchanged journal bytes and lookup-only reopen, then success, ownership
drain, landing/disarm and normal exit. The separate historical Gazebo/witness
timeouts still lack causal closure. See the installed
`easyfly_sim_launcher/ACCEPTANCE.md` for current scope. Server cross-restart
persistence and independent flight buttons remain deferred.

The additional opt-in `ManagedStack.DISABLED_RealToolsUnresolvedExitAndLookupOnlyReopen`
check uses actual RViz and offline-sandboxed QGC windows with real Qt/controller/
mission-service code, but a mock Action and fabricated telemetry. Two standalone
runs exited0: both exit prompts, canceled-exit reset, unchanged unresolved bytes,
one accepted goal/zero cancellations and lookup-only reopen were verified.
This is not SITL flying-task acceptance. Run it alone in domain115, not concurrently
with the default managed-stack tests. Set QT_QPA_PLATFORM=xcb and
EASYFLY_QGC_TEST_IMAGE to the trusted AppImage, then explicitly opt into the
disabled Google Test. Qt is stack-owned in the tool/managed-stack test mains;
an earlier static-destructor crash is retained separately from production issues.
A new domain118 SITL preparation hit the existing native GPS vertical-speed
check/invalid XY estimate; no flight request or production UI was started.
Its parameter manager now shut down normally under launch SIGINT.

## Optional tools (D6-3)

Install RViz2 and bubblewrap separately if desired. Missing tools or unsupported
namespaces fail visibly, without falling back to an unisolated process.
RViz2 is found on PATH; use `--rviz-path /absolute/path` to override it.
Choose the trusted QGC AppImage with the file picker or pass
`--qgc-path /home/jia/easyfly_ws/src/third_party/QGC.AppImage`.
Neither option starts a tool automatically.

RViz presets are generated from the current observation configuration at launch:
basic flight, navigation/SLAM, perception, and landing-camera viewing. The default
basic-flight preset displays the actual `/mavros/local_position/odom` pose as
vehicle axes in fixed frame `map`, matching the project's MAVROS configuration.
It keeps only one pose, needs neither FAST-LIO `/path` nor map/odom TF, and does
not add a robot model, transform publisher, or control plugin. The installed
`easyfly_rviz_plugins/Odometry` display preserves native RViz pose/axes behavior
and releases retired visual scene nodes in the tested 11.2.26 implementation.
The plugin is required by generated external presets as well as embedded
odometry; it does not replace a system RViz library. A differently
configured publisher needs a matching view; frame mismatch is not silently
relabelled or repaired with an identity transform.
The other presets retain fixed frame odom and require their actual publishers
and transforms. All presets use
inherited ROS_DOMAIN_ID/ROS_LOCALHOST_ONLY, explicit use_sim_time, best-effort
depth-one observations, and only MoveCamera/Select tools. The planned trajectory
and SLAM layers are disabled initially. All external RViz displays are explicitly
last-known/not freshness checked; paused ROS time can pause marker lifetimes.
They are not the desktop bridge's freshness-filtered evidence or flight admission.
The basic pose is also last-received evidence, not freshness checked or proof
of flight readiness. Missing or paused telemetry can leave the last axes visible.
The RViz wrapper preserves the parent DDS configuration and exposes host
`/dev/shm` read/write for normal POSIX DDS transport while retaining its owned
PID namespace and minimal device directory. In Humble, localhost Fast DDS
participants add SHM even with a UDP-only XML profile, so hiding this directory
can leave a running viewer with no observations. This trusted ROS viewer is not
a hostile-code sandbox. QGC never receives this SHM binding; its offline network,
device and temporary-settings isolation remain unchanged. No global transport,
domain, localhost setting or vehicle-control publisher is changed by this fix.
Prediction and semantic-copy displays are not adapted yet. Stop/restart RViz
after changing desktop configuration. A user who manually edits RViz can add
control plugins; this viewing preset is not a hostile-plugin security boundary.

QGC is a full-control application, not a read-only client. This entry permits
only offline preview: bwrap isolates network, IPC and PID namespaces, substitutes
minimal /dev, hides host /run and /tmp apart from the X11 transport/authority,
uses read-only host files and fresh temporary settings, and runs the AppImage
without FUSE. It does not modify MAVROS endpoints, PX4 or existing QGC settings.
Trusted binaries and the local X11 session are required; this is not a complete
malicious-software sandbox. No network connection, vehicle command or QGC
heartbeat/failsafe interaction is validated in this increment.

Status refers only to the owned wrapper process, never inferred vehicle
connection. Logs retain at most 32768 characters per tool. Stops are asynchronous,
then escalate only the owned wrapper after three seconds; PID namespaces contain
descendants. Closing with active tools asks first (default: keep everything).
Only an explicit stop-and-close choice stops these owned tools. User-started
tools and platform processes are never searched for or terminated.
Unexpected forced termination of EasyFly ends its contained tool processes.
Normal close waits for both tools and local observer/converter cleanup.

RViz uses Mesa software rendering in its child environment
(`LIBGL_ALWAYS_SOFTWARE=1`, `__GLX_VENDOR_LIBRARY_NAME=mesa`), without changing
the desktop's graphics settings or host drivers. Software-rendering behavior is
documented by [Mesa](https://docs.mesa3d.org/envvars.html); vendor selection is
implemented in [libglvnd](https://github.com/NVIDIA/libglvnd/blob/master/src/GLX/libglxmapping.c).
This was verified locally with RViz2 11.2.26; another graphics stack may fail
visibly rather than render. The three presets passed real-window startup checks
without a backend. The supplied QGC 5.0.8 AppImage also passed offline startup.
See `src/third_party/tools/qgc/README.md` for the inspected binary's identity and scope.

Default tool lifecycle/isolation tests require bwrap. Optional real-window
smoke checks never run in ordinary CTest:

```bash
QT_QPA_PLATFORM=xcb ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=1 \
  EASYFLY_QGC_TEST_IMAGE=/absolute/path/QGC.AppImage \
  build/easyfly_desktop/test_tools --gtest_also_run_disabled_tests \
  --gtest_filter='Tools.DISABLED_Real*'
```

## Managed start diagnostics

Each start prompt has an independent confirmation ID and a one-shot callback
bound to that exact prompt. A retired dialog cannot close or confirm its successor.
Accepted GUI confirmation IDs are consumed once by the controller, including
after a completed task; opening a new prompt creates a new ID. Process-local
retention is bounded at256 without eviction; this is not cross-restart recovery
or authentication. The durable intent schema and ROS service fields are unchanged.

ROS INFO audit records correlate prompt result/recheck with controller request
identity and service-side writer GUID/sequence. Only bounded identity metadata
is recorded, not mission bodies or paths. Keep the ROS log directory for diagnosis;
the log is not durable permission or proof of a physical user gesture. Default
startup remains read-only. A historical unexplained request is not declared fixed
merely because isolated confirmation/reopen tests pass.

## Offline mission drafts (D6-5 first increment)

The seventh page, `任务编辑（本地）`, works without ROS connectivity and is
independent of managed flight controls. It creates a sample, reads a local file,
edits v1 YAML or step forms, previews the step order/effective values, and exports
an explicitly chosen local draft. It never starts/cancels missions, writes ROS
parameters, imports a catalog, or changes the original task file automatically.

Use `新建示例` or `打开本地文件`, wait for background validation, then select a
step under `顺序与类型` and edit `当前步骤参数`. Append has its own type selector.
The eight existing v1 types are supported, without branches/retries or new
flight semantics. The existing mission library is the only schema validator;
passing static validation does not establish runtime readiness or flyability.
The view shows odom/m/rad/s, abort-on-failure, step count, summed timeout caps
(not estimated flight duration), and the exact content SHA-256.

Raw file bytes, including comments/CRLF, remain unchanged until an edit. Form
application and structural changes require a default-cancel normalization
confirmation: comments/formatting are removed and effective defaults are made
explicit. Invalid candidates leave the previous YAML intact. Unapplied form
changes disable raw editing/export until applied or reset. Legacy flat files,
including legacy files named `.yaml`, are identified but not converted or
exported as v1 automatically. Input/output is limited to64KiB, at most256 steps,
with one active worker job and one latest queued validation.

For export, choose a parent directory and a new child such as `easyfly-drafts`,
or enter that absolute path directly, then click `确认专用目录`. Only a newly
created private directory or a previously marked EasyFly draft directory is
accepted; existing ordinary directories are not adopted. Source/install trees
and the known installed task catalog are protected. The chosen root uses a
private `.easyfly-drafts-v1` marker; the file name must be a `.yaml/.yml` basename.
This is a local workflow constraint, not a sandbox against another process of
the same user or against subsequently configuring a task service to read drafts.

Existing drafts require a separate default-cancel overwrite confirmation bound
to their file/directory identity and content fingerprint. Changed targets are
rejected. Writes are fsynced in private temporary files and committed using
no-replace rename. Overwrite first retains the original as `.backup-<id>`; the
result shows its path. These two renames are not one filesystem transaction:
interruption can leave only the backup under its hidden name. Do not delete it
until the new draft is checked. Failed commits attempt non-overwriting restore;
unconfirmed writes and retained backups are reported, not labeled successful.
Concurrent mutation by another same-user process is not supported as a locked
multi-writer workflow.

Closing with unexported changes defaults to keeping the window, even after the
ROS observer disconnects. Explicit file work must finish/be dismissed first;
normal shutdown stops the local worker without issuing flight cancellation.
There is no draft autosave/recovery after a process crash in this increment.
Small windows use page/form scrolling. Task service import and RViz embedding
remain separate work. Draft geometry and metadata editing are described below.

## Local XY geometry preview (D6-5 second increment)

`二维几何示意` in the draft editor displays only the latest validated v1 draft:
waypoint/navigate targets in blue, search rectangles in teal, and selected
features in orange. Coordinates are odom XY in meters (X right, Y up), with
altitude labels; this is not GPS/north-up mapping or a live aircraft overlay.
Dashed arrows show the order of explicit point targets, not EGO trajectories,
collision checks, flyability, actual travel time, or sensor coverage.

No starting position is invented for takeoff/hover/land/track/precision_land.
Hover can preserve the ordering between explicit points; search, track,
precision_land, land and takeoff break that line sequence. Search is a rectangle
only: its center is not asserted to be the entry/exit point. No stripes are
generated because spacing and the executed route belong to the backend.
Tasks with no explicit XY geometry display an explanation, not a fabricated
origin or landing location. Unknown/nonfinite/excessive geometry fails closed;
the renderer's defensive size bounds are not new flight limits.

Click a point/rectangle to select its existing step (no task text mutation).
Overlapping features can always be selected from the step list. Wheel/buttons
zoom, middle drag pans, and `适配全部` restores the view. Aspect ratio is equal
on both axes; stepping selection preserves zoom. For more than24 features of
one kind, only the selected feature has a label to bound clutter; all shapes
remain present. Long labels are bounded/elided and the selected step details
remain available below the view/in the form. Small windows use page scrolling.

Any raw edit or unapplied form change immediately clears the previous geometry.
It returns only after validation/application or an explicit form reset, and old
validation revisions cannot restore an outdated diagram. Invalid or legacy
documents are not plotted. Loading a new revision refits automatically.
Geometry has no Source/ControlPort, live telemetry, planner, ROS client,
publisher, task import, drag-to-edit or navigation goal capability.

The local developer fixture `docs/ui/d6-5/geometry/geometry_example.yaml` has
three targets, one8×6m rectangle at(12,-6), and one point-order arrow. It is not
installed in the trusted mission catalog and must not be executed as a flight
test. This completes coarse draft geometry, not planner route preview, embedded
RViz or service-directory import.

## Opt-in platform metadata editing (D6-5 third increment)

```bash
ros2 run easyfly_desktop easyfly_desktop --enable-parameter-edit
```

This flag is independent of managed missions and requires no mission journal.
Default startup constructs no parameter writer. Source/Observer remain read-only;
an independent `ParameterControl` owns only the fixed `/platform/params/query`
client. The key selector contains exactly `operator_note` (valid UTF-8 text,
no NUL, at most512 **bytes**, including multibyte characters) and `export_pretty`
(boolean). The latter affects platform configuration JSON export indentation,
not draft YAML. This frontend subset is stricter than the existing backend's
string validation. No arbitrary node/key/path, flight boundary, profile/frame,
extrinsic, device parameter, mode, arming or setpoint operation is added.

In `配置与诊断`, explicitly read the selected key. The current JSON area stays
empty until a successful typed read; the proposed value is separate and is never
silently populated by subsequent reads. `复制读取值到待写值` is explicit. An empty
note is serialized as JSON `""` for a write; an empty wire value is reserved for
a read. Oversized/invalid text is rejected, never truncated into a valid write.

Each default-cancel confirmation binds its dialog, key, reviewed JSON, proposed
typed value and one32-hex ID. Changed/retired dialogs cannot submit. After Yes,
the worker first reads and compares the reviewed old value; a detected change
rejects the write. It then submits at most one application-level write and
independently reads back. A successful reply **and** matching readback are needed
for `写入已回读确认`; a reply alone is not success. The last observation's age is
shown; it is not a continuously current value. The existing read-only parameter
table has its own cache and is not silently refreshed by this editor.

There is no atomic compare-and-set, server-instance identity, authentication or
concurrent-editor guarantee in QueryParams. A service restart or change between
precheck/write/readback cannot be excluded; use one operator/client. The writer
does not guarantee exactly-once transport. It retains up to256 confirmation IDs
in-process without eviction and refuses new writes at capacity.

Timeout/transport failure after dispatch, observation disconnect while a write
is unresolved, or readback mismatch/invalidity keeps a process-local fault lock.
Reads remain possible after reconnect, but even a matching late read does not
unlock. No automatic retry, rollback or force-clear exists. Before dispatch,
failed precheck/disconnect rejects without claiming a remote write. Unconfirmed
results do not establish that the backend rejected or did not apply the change.
There is no persistent intent or automatic recovery across UI restart; do not
restart to bypass uncertainty. Independently verify the backend first.

Normal exit requires finishing/dismissing the confirmation or local operation.
A remaining fault requires a separate default-stay warning, even after observer
disconnect. Closing only stops local communication; it neither rolls back nor
replays a remote write. Direct process termination/crash cannot use this warning.

`test_parameters` uses only a parameter-service mock in domain123.
`test_parameter_controls` uses domain124, including a real ParameterManager
process with no flight nodes and a fabricated read-only observation source.
It checks typed writes, independent readback and configuration export, not
production connectivity or flight. Screenshots are under the ignored
`docs/ui/d6-5/parameters/`, marked as isolated metadata-only checks.

## Opt-in task-file import (D6-5 fourth increment)

```bash
ROS_LOCALHOST_ONLY=1 ros2 run easyfly_desktop easyfly_desktop \
  --enable-mission-import --import-journal /absolute/persistent/private/import-intent
```

Both flags are required; the journal is separate from mission execution intent.
Invalid/occupied directories fail before the window opens, using the same held
controller/lock through construction. Default startup constructs no importer.
The server separately needs enable_mission_import=true and a dedicated writable
private catalog (see easyfly_mission_srvs README). The desktop never starts that
server or writes its directory directly. No installed/source task catalog is
made writable. Existing mission execution opt-in is not implied by import.

In `任务编辑/导入`, validate and export the current formal v1 document to a local
dedicated draft directory. The separate import panel accepts only the exact
latest successfully exported revision/name/content/hash, with applied forms and
unchanged selected draft root/name. Raw/form edits immediately invalidate it.
It imports those reviewed cached bytes, not a later reread of a potentially
changed disk file. The offline editor itself still has no ROS/flight clients.

Explicitly read the configured server destination. The confirmation shows its
directory, server instance, catalog identity, basename and exact SHA-256; default
is cancel. Old dialogs or a changed live editor snapshot cannot submit, including
changes before the polling timer updates the panel. Confirmation first rechecks
the same enabled/nonfull destination, journals intent, then submits one create.
No existing name is overwritten. Success requires independent content/hash
readback and original-instance recheck, not just a reply.

Lost replies and UI restart recover only by querying the original request. A
changed server/catalog, unknown request or changed file retains the fault lock;
no retry, overwrite, rollback, journal deletion or automatic new-instance adoption
occurs. Import confirmation IDs are also one-use/in-process (max256, no eviction).
Observe the same domain/localhost scope and persistent directory on restart.
Do not move/replace the journal, change directories to bypass faults or pretend
an empty list proves an uncertain import never happened. The server's retained
ledger does not survive restart. Readback is an observation, not ongoing protection.

Normal close requires dismissing/finishing an active create operation. An
unresolved import has a separate default-stay warning and retains its journal,
even after observation disconnect. Closing only stops local communication;
it does not delete an imported file or cancel/execute a task. Source/Observer
remain read-only. Metadata import recovery queries may continue after observation
disconnect, but no new import is permitted. Local draft dirty-exit protection
and mission/parameter/tool exit choices still apply independently.

After import, manually refresh the task catalog and read the file details from
`任务查询`. This does not execute it. Flight requires separate managed-simulation
startup flags and a new human execution review; no execute button is added by
this increment. Current production catalogs/services are not restarted by UI.
The real Qt/worker/service test uses domain129, temporary draft/catalog/journal
directories, the production read-only Observer and no flight backend. It confirms
exact bytes, independent readback and catalog/details display, not SITL flying.
Screenshots/logs are under ignored `docs/ui/d6-5/import/`.

## Experimental read-only RViz embedding (D6-5)

The installed build controls availability; renderer startup is always manual. CMake defaults to
`EASYFLY_ENABLE_RVIZ_EMBED=OFF`: that binary does not link RViz/Ogre and rejects
`--enable-rviz-embed` with exit2. Package metadata declares the superset of
dependencies needed by the optional build. An enabled build requires the tested
Humble `rviz_common`, `rviz_rendering` and `rviz_default_plugins` version11.2.26,
plus the workspace `easyfly_rviz_plugins` package:

```bash
colcon build --packages-up-to easyfly_desktop --cmake-args -DEASYFLY_ENABLE_RVIZ_EMBED=ON
source install/setup.bash
ros2 run easyfly_desktop easyfly_desktop --enable-rviz-embed
```

The fixed `空间视图` page contains an embedded RViz tab, but creates no rendering surface or RViz node until
the user clicks `打开只读三维`. Only X11/xcb is supported in this increment;
unsupported platform, runtime version, plugin or foreign default ROS context
refuses initialization. Keep standalone RViz and the local XY canvas as fallback.
An enabled binary still has RViz shared-library dependencies even when the CLI
flag is absent; rebuild with OFF if those libraries are unavailable. There is no
automatic download, system-library patch or backend launch.

The page uses an RViz-style horizontal splitter: presets, layer switches, view
controls, warnings and status stay in a scrollable left sidebar. A separate
right-hand canvas contains only the render panel (or its idle placeholder),
never controls overlaid on the grid. Drag the divider to adjust sidebar width;
small windows scroll the sidebar vertically. The render panel is created with
this final canvas parent before its native window is initialized.

Presets are fixed: map + `/mavros/local_position/odom` (one pose axes), or odom
navigation observations. Grid is a reference, not an occupied map. TF is an
optional initially-disabled layer. No synthetic transforms or geographic map
are introduced. The pose-only preset retains its existing best-effort/volatile/
depth1/filter-size1 subscriber and same-reader header/TF diagnostic. That pose
and the standard TF layer are last received, not freshness-validated geometry.
They may persist when publishers stop or the simulation clock pauses.
Render/update count is not measured FPS; no display is a flight admission check.

Navigation has six fixed typed read-only geometry layers. Actual path starts
enabled; every additional data layer is initially disabled and has no data
subscription until checked. Colors intentionally distinguish types:

| Layer | Exact topic/type | Meaning |
| --- | --- | --- |
| Blue actual path | `/path`, nav_msgs/Path | SLAM observation history, not a commanded route |
| Orange planned line | `/optimal_list`, visualization_msgs/Marker | LINE_STRIP visualization, not an accepted trajectory or session proof |
| Green SLAM cloud | `/cloud_registered`, sensor_msgs/PointCloud2 | Sampled XYZ geometry |
| Green filtered cloud | `/pointcloud_filtered`, sensor_msgs/PointCloud2 | Sampled filtered XYZ, not a trusted obstacle map |
| Purple occupied centers | `/occupancy_grid`, nav_msgs/OccupancyGrid | Only cells >=50; upstream inflation retained, free/unknown cells not drawn |
| Red geometry boxes | `/obstacles`, easyfly_msgs/ObstacleInfoArray | Geometric edges only, not `/obstacles_classified` semantics or predictions |

The adapter honors each source frame and timestamp. EGO currently labels its
visualization `world`; FAST-LIO typically uses `camera_init`; semantic-map config
uses `odom`. Missing real TF clears that layer and reports the source->fixed frame.
No labels are rewritten to make geometry appear. Actual path takes observation
time from its last pose because FAST-LIO leaves the container stamp at startup;
mixed frames, non-monotonic/zero pose stamps and invalid coordinates are rejected.

Navigation layers check both source age (ROS clock) and original receipt age
(steady clock). Either exceeding 2s clears geometry on the next render update;
source stamps over 0.2s ahead, zero stamps and malformed inputs also clear it.
Duplicate/older source stamps never renew receipt time or revive cleared data.
Clock reversal or a >10s forward jump clears caches; in-flight conversions use a
mailbox epoch to reject results started before reset. A new valid observation is
required. These are UI-only limits, not backend safety or liveness guarantees.

Each layer has an explicit callback group excluded from Humble RViz's GUI
executor, an independent bounded conversion worker, and one replaceable result
slot. No per-message Qt event queue or raw cloud history is retained. Only GPU
geometry updates occur on the GUI thread, at the existing nominal 10Hz. Close
stops/joins owned workers before node/scene release; a stop flag covers startup
cancellation. Best-effort/volatile/depth1 is inspected on the local subscription.
Volatile subscriptions intentionally wait for a new publication rather than
treating transient retained maps as automatically current.

Input limits: cloud <=4MiB, 200000 points, 32 fields, point_step<=256; only unique
FLOAT32 XYZ is supported (both endian orders, organized row padding). Display
samples <=20000 finite cloud points and reports omissions. Path/line <=20000
input points, <=2000 rendered points with endpoints retained. Map <=262144 cells
and <=20000 occupied centers; boxes <=128. Excess map/box geometry is rejected,
not silently truncated. Counts, source/receipt ages, missing TF and replacement
counts appear in the left status pane. DDS deserialization/transport allocation
is outside these application-cache limits; this is not hostile-input isolation.

Only MoveCamera is instantiated; Orbit/TopDownOrtho, layer switches and view reset
are the available interactions. No arbitrary `.rviz` loading, property editor,
plugin chooser, Goal Pose, Initial Pose, PublishPoint, interactive markers,
navigation client, mode/arm or flight-command publisher exists in this component.
The version-pinned manager adapter avoids the upstream initializer entirely,
not merely removing control tools after they created publishers. ROS arguments
are not inherited/remapped; domain/localhost and the chosen simulation-time
setting remain the connection's startup scope. Only one surface per process
is supported; observation connection is not a statement of flight readiness.

Page hide, observation disconnect, time-source change and normal window close
release this surface's displays/subscriptions/timer/node; re-entry or reconnect
never automatically opens it. Observer/control/import contexts are not shut down.
The adapter establishes the native Qt parent before Ogre binds its window,
retains plugin libraries through callback destruction and explicitly releases
the owned ViewManager, scene and newly-created material handles. Materials
present before manager construction are not removed; this component requires
exclusive use of the rviz_rendering resource group inside the process. Ogre's
process-global renderer may remain cached;
this is not complete GPU-memory release. In-process plugin/graphics faults can
still terminate the UI: no crash isolation or real-flight safety claim is made.

`test_rviz_geometry` has eleven pure conversion/validity checks.
`test_rviz_view` has four headless checks and fourteen actual X11 checks. Layout
checks cover 1024×720 and 1440×900, with active-render splitter resizing and
non-overlapping canvas/control geometry. Native checks additionally cover typed
layers, default-off subscriptions, actual local QoS, real fixture TF, paused-clock
duplicate expiry, invalid/future data, reset recovery and reception/replacement
while GUI processing is withheld. The odometry lifetime regression includes
retired parents and detached covariance roots across repeated real rendered
updates; it checks exact scene-node retention after pending updates settle. Ordinary
CTest uses offscreen Qt and **skips** real rendering checks. Run those separately
in an isolated localhost domain with xcb (the fixtures publish only synthetic
observations and clock, never flight actions):

```bash
QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 ROS_DOMAIN_ID=131 ROS_LOCALHOST_ONLY=1 \
  build/easyfly_desktop/test_rviz_view
```

Use the local validated DDS transport profile when required by the host/container
setup; do not change production middleware configuration to run the tests.

Run the ON/OFF desktop CTest suites sequentially: their ROS fixtures use fixed
domains (including managed-stack domain115) and are not isolated from another
copy of the same suite. Concurrent copies can discover each other's mock service
and action servers; passing/failing results then do not establish production
request behavior. Keep fixture domains separate from live flight graphs.
Screenshots and diagnostics are under ignored `docs/ui/d6-5/rviz/`.

## Optional local camera recording (D6-6)

Build with `-DEASYFLY_ENABLE_MEDIA=ON`; its fixed recording page requires no CLI
opt-in. A directory and recording are created only after manual selection and Start.
The legacy `--enable-recording` flag remains accepted. Requesting it in a build without the backend exits
with code 2. No flight-control switch is needed to record observations.

Select the camera on the perception page, enter a **new independent absolute
directory** on the recording page, and confirm Start. Existing directories are
accepted only with the EasyFly recordings marker, matching ownership and mode
0700; ordinary directories are not adopted/chmod-repaired. Source/install trees,
mission catalog and configured UI journals are protected. Each recording creates
an exclusive UUID subdirectory and mode0600 files. No overwrite, deletion,
automatic cleanup/resume, cloud/audio/desktop capture, or remote task cancellation.
The file picker can select an existing dedicated directory; for the first recording
type a new leaf path under an existing parent.

The selected already-decoded UI image is sampled at at most 10fps, reduced without
upscaling to fit 1280×720. The worker has one pending frame and one latest metadata
snapshot; replacement, rate and stale drops are counted separately. This is **not
complete sensor capture**: UI sampling/upstream losses cannot all be counted.
Encoding, filesystem work, index scanning and final validation are on a dedicated
worker, not ROS callbacks or the GUI. Existing camera subscriptions/QoS/TTLs and
control APIs are unchanged. Camera, observation session/epoch, clock-source or
profile change, disconnect and normal window close finish the current recording.
Stale images are not encoded or duplicated to fill gaps. Closing waits
asynchronously for real local finalization, without canceling the remote task.
Filesystem/codec stalls can delay finalization; forced process termination cannot
promise a finalized record. The codec is in process, not a crash-isolated service.

MJPEG/AVI uses OpenCV JPEG encoding and EasyFly's bounded single-stream RIFF
writer to the exact `O_EXCL`/`O_NOFOLLOW` descriptor (no path-based encoder truncate).
Segments rotate at 600 frames, approximately 56MiB, resolution changes, or stop;
each is below 64MiB. Maximum 128 segments/36000 frames per recording, 512MiB
file budget (with reserved finalization overhead), 8MiB frame/event metadata and
256MiB available-disk reserve. Errors/limits stop local recording and retain the
partial artifacts. These are media budgets, not flight safety parameters.

Files: `inprogress.json`, `config.json`, `frames.jsonl`, `events.jsonl`,
`segment-N.avi`, and a final `manifest.json`. Every completed segment is actually
decoded with OpenCV's built-in MJPEG reader and checked for dimensions/frame count
before it is marked verified. SHA-256 covers segments and metadata; index refresh
checks hashes, types, sizes, ownership and identities. A missing/bad final manifest
or changed artifact is incomplete/corrupt, never silently complete. Index scanning
is bounded to 1024 directory entries, 256 recordings and 512MiB integrity reads;
over-budget files/entries report errors, not unlimited scanning. Select a verified
row to open the separate local-history program described below; it revalidates
the record rather than trusting a prior index scan.

AVI playback is fixed 10fps and **does not preserve real inter-frame gaps**.
`frames.jsonl` retains exact source nanoseconds as strings, first-received and
encoded steady seconds, decoder sequence and UTC write indexes. UTC is not a
flight/source clock; steady timestamps are process/host-local, not cross-restart
time synchronization. `config.json` stores UI/codec version, topic, clock/session,
selected (unconfirmed) profile label and media budgets, not connection credentials
or full mission bodies. Queried backend profile retains its observation quality.
Known controller request/instance/execution/Action UUID and explicitly queried
execution results are attributed separately; global mission phases/observer events
are always `global_unattributed`. Event/metadata capture is bounded best-effort,
not a durable authoritative execution audit or a reconstruction of physical events.

Validated dependency here: system C++ OpenCV 4.5.4 (`core`, `imgproc`, `imgcodecs`,
`videoio`). No FFmpeg CLI installation, Python OpenCV or NumPy change is needed.
Only built-in MJPEG is selected; system OpenCV may have other linked optional
backends. EasyFly's new code is Apache-2.0. OpenCV's
[4.5.4 upstream license](https://github.com/opencv/opencv/blob/4.5.4/LICENSE) is
Apache-2.0, while the installed Debian package's copyright file lists remaining
BSD-3-Clause code too: retain all applicable notices when redistributing, including
transitive JPEG/video libraries. AVI layout follows the
[Microsoft RIFF reference](https://learn.microsoft.com/en-us/windows/win32/directshow/avi-riff-file-reference).

`test_recording` covers exclusive storage, segmented readback, bounded queues,
staleness, disk reserve and actual OS file-write failure, bad metadata/video,
confirmation expiry/cancel, camera/clock/disconnect boundaries and asynchronous
window close with no task control. The write-failure test lowers `RLIMIT_FSIZE`
only in its own child process; it does not fill the user's disk or alter global limits.

## Independent local history viewer (D6-7, first batch)

An enabled media build additionally installs `easyfly_recording_player`. It is a
separate Qt/OpenCV executable, deliberately **not linked to desktop_widgets or ROS**:
no observer, node, publisher, service/Action/parameter client, mission journal or
live-control page. It never launches a backend or republishes recorded messages.
ROS domain selection is not its isolation mechanism: it does not initialize ROS.
This is application-level separation, not an OS/network sandbox for untrusted codecs.

```bash
ros2 run easyfly_desktop easyfly_recording_player \
  --directory /absolute/path/to/private-recordings \
  --record 00000000-0000-4000-8000-000000000001
```

Replace the example UUID with a real record ID. The installed executable can also
be run directly without sourcing ROS. With no arguments, choose a UUID session
directory under an existing EasyFly private record root, then explicitly open it.
From the live recording page, refresh the local index, select a verified row and
click Independent History Playback. This creates a separate window that is closed
independently, not managed as a flight tool or closed automatically with live UI.
It starts **paused**, including when launched with a record argument.

The window is visibly historical: left-side record/frame details and playback
controls, right-side sampled image and read-only event table/raw JSON. Previous,
next and the slider select exact recorded frames. Play advances at at most 10fps
without catch-up/duplicate frames; slow decoding slows playback. At end it pauses.
This fixed playback rate does not recreate source or received time gaps. Exact
source nanoseconds, process-local receive/encode steady times and write UTC remain
separate. Events retain original log order and all available fields; they are
**not automatically synchronized** to source-frame time by their write UTC.
Missing event attribution/identity displays Unrecorded, never inferred from a
global phase. Known identities are shown only when actually present in the log.

Only finalized verified v1 single-camera MJPEG records are accepted. Reopening
checks UUID/marker/permissions/type/hardlink constraints, config/log/video hashes,
bounded classic AVI headers/index/chunks, frame-log mappings and monotonic source/
receive times. Incomplete/failed/missing/corrupt/over-budget records refuse playback
and clear the old display. Video descriptors are pinned; each requested JPEG is
checked against its validated digest before decode, not reopened by a mutable path.
The reader accepts only this recorder's baseline JPEG and bounded 1280x720 format,
not generic third-party AVI, URLs, rosbag or arbitrary codecs.

All reads, verification and decode run on one worker with one pending selection
and one decoded image. Budgets: 512MiB integrity reads per record, 64MiB per segment,
128 segments, 36000 frames, combined 8MiB config/frame/event logs and 36000 entries
per log. The GUI event model is virtual rather than constructing all table widgets.
Normal close requests nonblocking worker shutdown and waits asynchronously;
filesystem/device stalls can still delay shutdown. No source files are modified,
deleted or chmod-repaired. This reader is for trusted local records, not a media
security sandbox. Its optional Qt/OpenCV dependencies retain their own notices.

There is **no recorded trajectory, point cloud or complete aircraft telemetry**
in the D6-6 format, so these historical data are explicitly unavailable. This batch
does not claim physical reconstruction, synchronized event/video replay, complete
task audit, full D6-7 delivery or real-flight acceptance. Trajectory acquisition,
isolated rosbag workflows and broader combination acceptance remain separate work.
The media-OFF build has no player target or OpenCV dependency in the base window.
`test_playback` covers file integrity/limits, large exact timestamps, multi-segment
selection, malformed mappings/dimensions/timing, unsafe files, post-load changes,
request supersession, GUI pause/seek/events/close and standalone linkage/CLI gates.

工作区的页面安排、配置入口、专注观察与分栏偏好见 [布局与使用](LAYOUT.md)。

### Unified FUEL exploration steps

The task editor supports YAML v1 `explore`: XYZ min/max bounds, an explicit
return-to-exploration-start checkbox, timeout and up to 16 forbidden boxes. Each
forbidden-box editor line is `xmin ymin zmin xmax ymax zmax` in meters. Invalid
input stays in the editor; applying or normalizing a step preserves all boxes.
The geometry tab shows XY projections with vertical ranges and labeled forbidden
boxes; it does not invent an exploration path or certify traversability.

Execution requires navigation_dev, FAST-LIO, GCOPTER and the FUEL backend. The
backend still checks fresh map, localization and flight state. Task execution uses
the managed Explore action and waits for recursive ownership release. The known
B-01 FUEL map issue remains separately deferred; format/SDK/UI integration does
not close that algorithm defect.

### Specialized read-only observations

The camera/perception tabs now show bounded AprilTag observations, the selected
optical landing target, odom landing guidance, owned PrecisionLand/Explore feedback,
model predictions and independent classified obstacle copies. Optical Z and yaw
are not relabeled as altitude/ENU heading. The executor relays real child feedback
once per SDK receipt with parent/child Goal IDs; the UI displays action phase/error
only for its matching unresolved local mission, never from global phase alone.

Each stream uses depth 1, bounded records/strings, one source and a 0.5-second
freshness deadline. Duplicates do not renew freshness. Conflicting/older messages,
clock changes, stopped streams and malformed input clear the current valid display.
Prediction reference and original observation times remain separate; variance is
model-based. Expired class association becomes unknown while valid geometry remains.

The navigation RViz preset adds opt-in landing-normal, model-prediction-cross and
classified-copy geometry layers. They have no flight publishers and clear after
0.5 seconds. A normal is not measured board corners; a cross is not a measured
trajectory; the copy never replaces original obstacle occupancy. Native Qt/ROS
and X11 rendering acceptance does not certify physical detection or landing accuracy.

### Explicit ROI and simulation gimbal operations

`--enable-peripheral-control` is independent of mission control and does not need
or enable a mission journal. It is rejected with system time or a managed QGC
maintenance session. The connection manager has a separate opt-in for local SITL;
legacy profiles omit the disabled flag and retain their existing canonical hash.
The perception tab's “单目标／云台” panel first connects read-only observations.
Initialize/reset/angle/tracking require the page's explicit session permission.
Query is read-only and works without mutable permission. No backend is auto-started.

For a separately launched simulation using the canonical front camera, start the
existing backends explicitly in the same isolated ROS domain and environment:

```bash
ros2 run easyfly_single_object_tracking single_object_tracking_node --ros-args \
  -p use_sim_time:=true -r /camera/image_raw:=/camera/color/image_raw
ros2 launch easyfly_gimbal_control gimbal.launch.py use_sim_time:=true
ros2 run easyfly_desktop easyfly_desktop --enable-peripheral-control
```

Drag within the image to select a normalized ROI; letterboxing is excluded. Clicking
initialize binds the exact image last painted in that panel, its optical frame,
source stamp, session and epoch. It never substitutes a newer received frame.
The frontend requires a fresh canonical front image (0.5 seconds) and at least
8×8 pixels. The backend matches that header in its bounded recent-image cache,
then performs CSRT work. Admission is not tracking success. Independent IDs,
initializing/tracking/lost/idle and unknown confidence remain explicit. A lost
tracker needs explicit reinitialization; reset does not affect the multi-object
tracker. Only a frame-matching actual CSRT box is drawn.

Gimbal controls show body FLU directions and degrees, check the simulation model
identity and connection, and display actual observed angles independently of the
service response. Target selection never silently switches to another ID. Target
loss or disconnection does not restart an old tracking request. These operations
never command the aircraft or change the fixed downward-camera transform.

There is one in-flight peripheral request. Timeouts remain unknown and block further
submissions; there is no automatic retry. A real late response reports admission
separately from current telemetry. Stopping observation sends no reset, angle or
tracking command. These services have no durable request identity: process restart
cannot recover attribution for an old unknown response; inspect backend state before
any new explicit operation. The physical gimbal backend remains unimplemented.

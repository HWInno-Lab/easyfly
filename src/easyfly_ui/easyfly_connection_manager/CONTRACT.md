# Managed connection contract

Configuration schema2; newline IPC envelope v=1; desktop capability protocol2;
UiSnapshot ABI / bridge SONAME3. An installed desktop protocol manifest is required.
The optional Qt connection-page plugin has C ABI1. The application entry execs the
desktop in connection-lobby mode; the lobby creates no observer or ROS context.
The configured workspace loads the same page inside MainWindow navigation. A lobby
closes only after a fresh integrated workspace heartbeat and no unsaved lobby draft, preserving domain isolation
without leaving a separate management window. Legacy workspace heartbeats cannot
claim an integrated handoff. The independent desktop still needs no manager package.
Legacy schema1 observe profiles are normalized with all new operations disabled,
without overwriting their file until an explicit save. Unknown fields/types fail closed.

Modes are observe, communication and sitl. Profiles freeze DDS domain/scope/time,
UI profile/topics, simulation scene/PX4/master/display settings, typed FCU transport,
GCS endpoint/tool and independently enabled desktop features. URLs are assembled as
QProcess arguments, never shell text. Private files, atomic writes, UUID identity,
32-profile limit and bounded parsing retain the initial CM-1 contract.

The Qt manager and supervisor initialize no ROS contexts. A backend helper uses its
own explicit ROS context/executor. Verified simulation compatibility is activated
before DDS; its controlled library/XML environment is propagated to the whole desktop.
Each workspace receives one fixed environment for SDK, Observer, control factories,
RViz contexts and tools. No in-place domain mutation or mixed old contexts occurs.

The supervisor uses a private Unix socket/UID credentials, flock and bounded request
replay. Every request has integer v=1, id<=64 characters and op<=32; frames<=16KiB,
input buffers<=32KiB, writes stop when outgoing backlog exceeds64KiB, at most8
local-user clients and64 cached
mutation IDs. Reusing an ID with different content is rejected. Status contains no token.

Operations: status, attach_manager(config_file), start(profile), close(session),
workspace(session/token/status), show_manager, stop_backend(session), start_qgc(session),
stop_qgc(session), exit_service. No operation accepts arbitrary shell, executable
arguments, environment or PID. QGC's executable is the explicitly selected saved path.

A live backend can only reopen the same frozen profile/hash. Different configuration
requires closing the old workspace and normally stopping the old owned backend.
Backend, workspace, QGC process/session and fresh actual data are distinct states.
Workspace/backend heartbeats expire after2.5s; old snapshots never become a flight permit.
A failed or missing heartbeat never starts a second process or triggers an automatic kill.
Backend observations/reporting errors retain ownership rather than abandon the launch.

The backend holds a per-user communications flock. Existing status publishers and
UDP/serial errors are rejected before launch. SITL also uses the existing single-vehicle
reservation, scene preflight and verified private PX4 rootfs; draft/incompatible/missing
resources fail before simulator creation. An already-running external backend is observed
through observe mode, never adopted or stopped by a matching name/port.

The managed SITL path defaults to background Gazebo. Optional native Gazebo GUI
uses the corrected project-local transport, including registered-node finalization
before clearing the topic table. New profiles still default gazebo_gui=false.
Workspace camera/RViz views remain available independently of that native GUI option.

Managed mutable flags require the configured session policy. Mission execution is
restricted to explicitly enabled local simulation. All mutable flags default off;
GCS maintenance rejects mission/import/parameter features. Recording and RViz pages
follow installed desktop build capabilities and remain idle until manual start. Legacy
schema2 recording/rviz_embed flags are preserved in serialization and frozen hashes,
but are no longer required to expose these pages; explicit unavailable flags still fail validation.
Journal directories are persistent per-profile private data, never temporary session logs.

Managed workspace close requires fresh unresolved=false and invokes the old normal
window-close rules. An unresolved observation is retained on workspace exit and prevents
backend stop until recovery is confirmed. No goal cancellation is sent on manager/window exit.

stop_backend first requires no owned workspace/QGC and no unresolved local intent;
the helper independently checks fresh single-source MAVROS connected, disarmed and
ON_GROUND observations plus no observed active action. Once an FCU has connected, unknown,
stale, armed or airborne state cannot permit a stop. Before first connection a queued
stop may cancel startup. Only the exact owned ros2 launch receives normal SIGINT.
Timeout keeps a still-running launch owned. The helper never escalates signals or cleans
up by broad process name. ROS launch retains its existing child timeout/escalation policy;
an abnormal child/launch exit is reported as failure, never normal completion.

QGC uses a private process session and per-run XDG/settings/save paths. Its auto serial/
other discovery is disabled and UDP is configured before start. Local SITL forwarding
uses a non14550 receiver to avoid PX4's separate native direct GCS stream. Diagnostics
match the exact GCS endpoint URL; aggregate router traffic does not prove GCS receipt.
Current verified application sample is QGC5.0.8. Remote reception and other builds need
separate acceptance; no one-way or read-only control claim is made.

QGC close sends WM_DELETE_WINDOW only to a main window whose PID belongs to the owned
process session and whose XDG_CONFIG_HOME matches the unique run. Confirmation is left
in QGC; no automatic Yes, wrapper-only termination or SIGKILL occurs. AppImage wrappers
and surviving owned descendants remain active until actual exit. X11/XWayland is required
for this managed native-window path. Existing offline-isolated QGC tools are unchanged.

Manager close preserves backend/workspace/QGC. exit_service rejects any live owned object.
In the integrated workspace, ending observation requests its normal close and opens
a new idle MainWindow only after cleanup. Draft/tool confirmation cancellation keeps
the current workspace. Each new close attempt records its own return intent; ordinary
window exit does not inherit a previously cancelled return-to-connections request.
The embedded page's show_manager request selects and raises the existing MainWindow.
Reopen queries the same running supervisor. Private atomic session records preserve the
frozen profile/hash, workspace credential, backend credential/identity, unresolved flag,
owned QGC group and bounded mutation receipts. A replacement supervisor must authenticate
the live helper with its credential, SO_PEERCRED user/PID and boot/start-time identity,
then obtain fresh status. Workspace reconnection additionally checks its token and peer
process identity. Cached telemetry never grants stop permission. A private authenticated
terminal receipt covers helper exit between polls, including preflight failure/cancellation.
Unknown/dead owners without a valid terminal receipt block duplicate startup and retain
logs/journals; no bare-PID adoption, signal escalation or automatic mutation retry occurs.
Recovery of a mutable workspace initially retains unresolved=true until its own fresh
snapshot/journal recovery says otherwise. A surviving QGC still closes through its original
group plus unique XDG_CONFIG_HOME window proof. Advanced discovery, hardware admission and
long-duration qualification remain separate work.

The first navigation item is connection/simulation, followed by overview, tasks,
camera/perception, space, records/playback and diagnostics/settings. One renderer is
owned by the spatial page; overview navigation does not duplicate it. Managed effective
configuration is read-only outside the connection page. Offline-isolated QGC and
networked QGC share only the selected executable, not startup authorization or isolation.
Scene preview reads a bounded, package-contained world only after catalog SHA validation;
it displays limited static primitive geometry, omitting models/meshes/includes. It is
not a simulator image or evidence of a running scene.

Algorithm options are an optional strict profile object. Defaults are omitted from
canonical JSON so unchanged legacy profile hashes remain stable. The shared schema
in easyfly_params builds the form and validates backend launch inputs. Only local
SITL creates the explicitly selected ellipse/color-line nodes; ordinary computation
libraries have no independent process. Options are frozen with the session and
applied on restart. No runtime plugin discovery or algorithm hot switching is provided.
A separate explicit TLS service and Qt client now manage allowlisted remote algorithm
presets with durable session/request identities and fresh ground interlocks; see REMOTE.md.
Ordinary DDS observation never starts or adopts remote processes. The desktop displays bounded, exact-frame geometry and
checks fresh task backend discovery before new confirmation; backend admission and
ownership remain authoritative.


SLAM 配置使用共享算法 schema：profile / fast_lio / openvins / disabled，默认随运行方案。
FAST-LIO 显式启用要求非 base_sim；输入限定绝对 ROS 话题，迭代/采样为整数、体素与距离为浮点数。
profile.launch → perception_sitl → scoped FAST-LIO include 传递同一份校验后的数据，关闭时不创建
FAST-LIO 或其 map→odom 组件；不发飞行请求。新增字段默认值省略，读取旧算法对象时仅为新字段
补默认值，保留既有完整旧字段约束与哈希；非默认 SLAM 配置仍纳入身份摘要。


OpenVINS is an opt-in `slam=openvins` startup selection for local SITL only. It
uses the fixed `gazebo_mono` calibration, excludes FAST-LIO, and rejects custom
FAST-LIO parameters. The `base_sim` profile cannot select it. Dependency checks
resolve `ov_msckf` and `easyfly_openvins_integration` only for this branch.
`EASYFLY_LOCALIZATION_BACKEND` carries the configured choice to the desktop;
it is a label, not DDS identity or flight admission. The desktop observes the
bounded `/localization/openvins/status` contract with wall and source freshness,
clock epochs, session/generation display and multiple-publisher rejection.
Navigation remains unavailable for OpenVINS until IMU/base and odom/map frame
integration is implemented. The coordinator rejects goals even with fresh
ESTIMATING status and refuses runtime changes of its read-only backend parameter.
The UI blocks new navigation/tracking/search mission confirmation for this choice.

# Managed connection contract

Configuration schema2; newline IPC envelope v=1; desktop capability protocol2;
UiSnapshot ABI / bridge SONAME1. An installed desktop protocol manifest is required.
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

The supported managed SITL path runs Gazebo in the background. Native Gazebo GUI
shutdown retained a transport owner in actual tests; gazebo_gui=true is currently
rejected before launch and disables the UI start action. New profiles default false.
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
Reopen queries the same running supervisor. Authenticated adoption after supervisor crash,
advanced DDS discovery presets, hardware admission and long-duration qualification remain
separate work; never trust a bare PID file or delete old journals to manufacture recovery.

The first navigation item is connection/simulation, followed by overview, tasks,
camera/perception, space, records/playback and diagnostics/settings. One renderer is
owned by the spatial page; overview navigation does not duplicate it. Managed effective
configuration is read-only outside the connection page. Offline-isolated QGC and
networked QGC share only the selected executable, not startup authorization or isolation.
Scene preview reads a bounded, package-contained world only after catalog SHA validation;
it displays limited static primitive geometry, omitting models/meshes/includes. It is
not a simulator image or evidence of a running scene.

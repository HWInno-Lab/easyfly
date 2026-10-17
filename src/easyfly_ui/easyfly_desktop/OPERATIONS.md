# EasyFly desktop development delivery

This is a Linux x86-64 development desktop, not a certified flight station,
self-contained distribution, hardware release or completed D6-7 acceptance.
Own code uses Apache-2.0. Dependency notices remain separate.

## What is available

Default: bounded read-only ROS observation, catalog/details/query, camera,
measured targets/obstacles, local XY view, diagnostics and local draft editor.
Catalog selection, static validation and a successful service reply do not
prove execution success, readiness or ownership release.

Explicit opt-ins: managed simulation missions, two non-flight metadata writes,
private-directory mission import, experimental read-only embedded RViz, local
single-camera recording and an independent local history viewer. The desktop
never starts a backend automatically. It has no standalone flight buttons.
QGC integration remains offline-isolated preview, not connected read-only QGC.

## Build and feature matrix

The optional easyfly_connection_manager package provides a graphical existing-
platform connection entry. Its installed easyfly_connections launcher loads its
own installation environment; select a domain, localhost/LAN scope and time
source, save, then connect. It opens this desktop as a new read-only process.
Return to connection manager is available in that workspace's menu. The managed
configuration is frozen; close the old workspace normally before selecting another.
All default draft/close protections still apply. This initial entry does not
start a backend, SITL, QGC, recording or mutable ports. Advanced network discovery
presets and real two-machine qualification remain outside the initial batch.

Graph-evidence snapshots require the versioned libeasyfly_ui_bridge.so.1 and the
matching installed connection-protocol manifest. Rebuild linked consumers
together. When using custom colcon build/install directories, pass both build-base
and install-base to colcon test as well, so old default-install libraries cannot
override the tested installation.

Assumes Ubuntu 22.04 x86-64 and compatible ROS Humble/Qt/system dependencies.
The read-only desktop's workspace closure is six packages: easyfly_msgs,
easyfly_core, easyfly_mission_format, easyfly_ui_control, easyfly_ui_bridge and
easyfly_desktop. The parser has no flight/ROS runtime. No mission runtime,
platform service or simulator package is required for this standalone build.
This does not install system dependencies on a clean machine or offline host.

From the source workspace, with a new build/install prefix and only the system
ROS underlay loaded (replace the example absolute directory):

```bash
source /opt/ros/humble/setup.bash
ui_build_root=/absolute/path/new-ui-build
colcon --log-base "$ui_build_root/log" build \
  --build-base "$ui_build_root/build" --install-base "$ui_build_root/install" \
  --packages-up-to easyfly_desktop --executor sequential --cmake-args -DBUILD_TESTING=ON \
  -DEASYFLY_ENABLE_RVIZ_EMBED=OFF -DEASYFLY_ENABLE_MEDIA=OFF
source "$ui_build_root/install/local_setup.bash"
ros2 run easyfly_desktop easyfly_desktop
```

The pure parser retains the existing C++ names/limits. Existing runtime callers
can still use the easyfly_mission_fsm forwarding headers, CMake target and CLI;
rebuild related runtime/service packages together when updating a full workspace.
An old backend can still expose unavailable/incompatible endpoints.

Desktop-only unit/ROS-mock checks remain in easyfly_desktop. The three suites
using platform services now live in the separate easyfly_desktop_tests package;
explicitly build/run it in an updated full developer workspace to retain import,
parameter/media and managed-task coverage. It uses the installed development
widgets target, with the desktop's actual feature flags; OFF builds do not prove
media coverage. The default UI still creates no mutable control port or renderer.
Do not run copies of these fixed-domain integration suites concurrently.

Always set **both** optional CMake switches explicitly when reusing a build tree.
Their defaults are OFF, but cached ON values remain ON if a later command omits
the switch. Use distinct build/install prefixes to compare variants. Installing
an OFF build over an old ON install does not necessarily remove the old optional
executable; file presence alone is not proof of the current build configuration.

| MEDIA | RVIZ_EMBED | Build products / dependencies |
| --- | --- | --- |
| OFF | OFF | Base desktop; no linked OpenCV/RViz/Ogre requirement |
| ON | OFF | Base + recorder/player; OpenCV; no embedded RViz/Ogre |
| OFF | ON | Base + embedded RViz; no recorder/player |
| ON | ON | Both optional backends |

ON builds retain linked optional shared-library dependencies even when their
runtime page is disabled. Runtime opt-out is not a way to run an ON binary with
missing shared libraries. Package metadata declares a dependency superset:
`rosdep` may install optional dependencies even for an OFF build.
The history executable is separate Qt/OpenCV code with no ROS linkage, nodes,
publishers or control clients. Its process does not use ROS domain isolation.
It is not an OS/network/untrusted-media sandbox.

After relocating an installation, source the environment from its new location;
do not retain old workspace overlays or library paths. Local draft and recording
storage rejects configured source/install trees and the currently indexed EasyFly
package prefixes. The executable's standard `PREFIX/lib/easyfly_desktop` layout
also protects that prefix when the package index is unavailable. Choose a separate
private data directory, not a child of an installed package. Relocation still
requires compatible system ROS/Qt libraries; this is not a self-contained bundle.

The tested embedded RViz build requires Humble `rviz_common`, `rviz_rendering`
and `rviz_default_plugins` **11.2.26 exactly**. A mismatch must be resolved by
using the supported environment or keeping embedding OFF; do not bypass the
version check. Independent viewing-tool availability is checked on explicit use.

## Runtime opt-ins

| Startup option | Boundary |
| --- | --- |
| None | Read-only observation/local draft work; no mission start/cancel, parameter write, import, recording or embedded renderer |
| `--enable-rviz-embed` | Requires RVIZ_EMBED build; manually open from the overview RViz tab or full experimental page; one shared renderer |
| `--enable-recording` | Requires MEDIA build; select private root and manually confirm Start; never auto-record/resume |
| `--enable-parameter-edit` | Only `operator_note` and `export_pretty`; each manual read/confirm/single write/readback |
| `--enable-mission-import --import-journal /absolute/private/child` | Separately enabled server/private destination needed; new files only, no execution |
| `--enable-mission-control --mission-journal /absolute/private/child` | Managed simulation only; human isolation permit, reviewed content and current profile/state checks; no hardware operation |
| `--qgc-path /absolute/trusted/QGC.AppImage` | Selects a binary, not startup/connect authorization; explicit offline preview only |
| `--rviz-path /absolute/rviz2` | Selects an independent viewer binary, not backend startup |

Flight/import journals are separate private persistent directories. Do not
reuse catalogs, draft/record roots, the workspace or one another as journals.
Do not delete, replace or chmod-repair unresolved journals to bypass a lock.
Unconfirmed parameter writes remain locked in the process; this API has no
instance/CAS identity and is not a cross-restart recovery protocol.
Unknown server instances/results keep their fault state, not an automatic retry.

## Connect and identify data correctly

Choose the backend's actual domain, localhost/transport settings and clock source
**before** startup. Default uses simulation time. Without valid `/clock`,
observations are unavailable; window startup is still expected to succeed.
Use `--system-time` only when the backend actually uses system timestamps.
ROS domain/localhost/profile labels are not authentication or flight readiness.

For an already-running simulation camera:

```bash
# Use the same explicitly selected environment/domain as that simulation.
ros2 run easyfly_desktop easyfly_desktop \
  --front-topic /camera/color/image_raw --down-topic ''
```

This command starts only the UI, not Gazebo/PX4, and enables no operation writes.
The current Iris model statically declares a 640x480 front RGB camera at this
topic. That declaration is not proof that a particular simulator/plugin is
running. A down-camera subscription without its actual publisher is unavailable,
not a synthetic substitute. Some viewing presets require actual odom/TF/map
publishers; missing frames are not repaired with invented transforms.

For the full `navigation_dev` simulation, explicitly add `--enable-rviz-embed`,
choose the overview's **RViz 三维** tab, select **导航观测**, then manually open
the renderer. This preset uses canonical `/odom` aircraft axes and fixed frame
`odom`; `/path` and `/cloud_registered` from FAST-LIO retain `camera_init` and
must use the existing TF chain. Enable cloud/map/obstacle layers individually.
The basic preset remains MAVROS `/mavros/local_position/odom` with its raw `map`
header; that header does not establish the same origin as a SLAM map.

Grounded, disarmed simulation can validate observation plumbing, but not a
flown trajectory, navigation clearance or control. A stationary path may be
short. Without a navigation task, no planned route should be expected. Local
Z remains relative to the estimator origin, not ground clearance. No command
port or recording operation is enabled by these viewing options.

### Same-frame detection and tracking views

In **相机与感知**, keep **实时原图** for the latest raw image, or choose
**同帧检测框** / **同帧跟踪框** explicitly. Blue boxes are detections; amber boxes
are upstream raw SORT tracks and may include predictions. The matched view is
delayed by processing and conversion; its caption shows the actual displayed
image stamp and age. No exact fresh match means no image/boxes in this mode,
not an overlay guessed onto the latest raw image. Overview and recordings stay raw.

Depth requires the same track event stamp, unique raw ID, class, expected world
frame and pixel center. Invalid/unknown depth is a dash. It is upstream fused
optical-axis depth, not slant range, ground clearance or flight safety evidence.
The upstream interface cannot establish actual depth-frame/TF synchronization,
per-track hit/prediction state or a physical person's identity. A valid empty list
is different from missing/stale/invalid observations. Changing camera, mode or
clock clears old overlay output. An unconfigured down camera is not replaced.

These additions change only readonly observation/display and local bounded
conversion/cache resources. No ROI initialization, target selection, tracking
reset, gimbal motion, flight command or parameter write is introduced. After
updating this C++ snapshot ABI, rebuild desktop and its linked consumers together.

### Why the media acceptance image has colored stripes

The first recording/replay acceptance used **synthetic test pixels**, not a
Gazebo scene or hardware camera:

| Item | Synthetic media acceptance | Project front-camera configuration |
| --- | --- | --- |
| Topic | `/easyfly_ui_media_demo/image` | `/camera/color/image_raw` |
| Source | Private observation fixture, domain134 | Gazebo camera plugin when actually running |
| Image | 320x180 RGB gradient + moving colored bar, 10Hz | Model configured 640x480 RGB scene, 30Hz |
| Source frame | `synthetic_camera_demo_not_aircraft` | `camera_color_optical_frame` |
| Profile label | `media_demo_not_aircraft`, explicitly unconfirmed | Must read the actual backend; UI selection alone proves nothing |

Colored stripes are **normal for that fixture and its history recording**.
They are not evidence of an encoding fault and not real-camera/scene acceptance.
The recorder persists the selected source topic/profile/session/clock in
`config.json`; it does not silently switch to a real camera on playback.
A scene must be separately verified against the actual publisher/topic and
fresh source stamps. Do not judge provenance from pixels alone.

If a known real scene unexpectedly shows stripes, check the selected source,
publisher, encoding/step/layout and stamps before declaring a camera fault.
Do not change production image/control QoS or safety freshness thresholds merely
to make the picture appear. No demo data should be labelled aircraft telemetry.

## Record and inspect history

The Camera and Perception page uses resizable left controls/status and a separate
right camera canvas. Scroll the left lists independently; drag the divider for
more image space. Images keep their aspect ratio, and source switching, freshness
and unavailable-source clearing are unchanged.

Choose a new independent absolute record-root leaf under an existing parent,
then manually confirm Start. Existing ordinary directories are not adopted;
existing EasyFly roots must retain their marker/owner/0700 permissions.
Each recording creates exclusive UUID files with mode0600. No overwrites,
auto-deletes, desktop/audio/cloud recording or remote task cancellation.
Do not use `/tmp` for durable operational records: system cleanup may remove it.

Only one selected camera, at most10fps/1280x720; dropped samples and three time
domains are separate. Normal stop performs real readback/hash finalization.
`inprogress.json` intentionally remains; completion is the final manifest plus
integrity check, not disappearance of the start marker. `verified` does not
certify an entire mission, real-time coverage or upstream loss accounting.
Disk reserve is a threshold check, not space reserved against other writers.

Confirmed paused/invalid clock, changed observation session/epoch or clock
evidence older than the existing1s validity bound finalizes the local record.
Recovery never resumes it: manually confirm a new Start for a new UUID.
An expired confirmation is discarded, not reused after clock recovery.
Image freshness remains a separate per-frame drop check; this does not change
camera QoS, observation freshness limits or any flight-control behavior.

Refresh the local index, select a verified row, open Independent History Playback.
Or run the player directly with a real UUID:

```bash
ros2 run easyfly_desktop easyfly_recording_player \
  --directory /absolute/private/records \
  --record 00000000-0000-4000-8000-000000000001
```

Replace both examples. The player defaults paused and revalidates files.
It shows historical images, exact frame times and the original event fields.
10fps playback does not reproduce source-time gaps or synchronize events by UTC.
Global events remain unattributed; missing optional fields are Unrecorded.
No historical trajectory/point cloud/complete aircraft telemetry was recorded.
The player has no live control connection; close it independently.

For recorded `control_snapshot` events, current phase codes are:1 Sent,3 Running,
4 CancelPending,5 DrainPending and6 Completed. Compare `instance_id`, `request_id`,
`execution_id`, `action_uuid` and `content_hash` within the same local intent.
Phase5 with `resolved=false` means a terminal result is still waiting for ownership
release; phase6 with `resolved=true` resolves that intent, not proof of mission success.
A result message is historical text, not fresh ownership evidence.
`mission_query_result.request_id` is an observer read sequence, not the managed
request or Action UUID. Global events do not inherit those identities.
`event_gap` reports a snapshot metadata budget truncation, not mission failure;
`metadata_replaced` in the finish event counts overwritten pending snapshots.
The recording is bounded sampled evidence, not a complete RPC/mission audit log.

## Normal close and uncertain outcomes

| Situation | Expected local behavior |
| --- | --- |
| Default read-only close/disconnect | Asynchronously stop only local observer/converters; no remote cancel |
| Recording active | Finalize locally on camera/session/clock change, confirmed paused/invalid clock or stale clock evidence, disconnect or accepted normal close; recovery requires another manual Start |
| Managed intent pending/running/unknown | Keep persistent identity; close communication only after the user's exit choice; restart queries the same original request |
| Owned external tool running | Prompt/default keep; explicitly stop only this UI's owned wrapper, never user-started tools |
| Local draft dirty | Require the existing save/discard/stay choice, including after disconnect |
| History viewer close | Stop its local worker asynchronously; independent of live UI/tasks |

Do not infer successful remote cancellation, landing or ownership release from
window exit, an ACK, fresh telemetry or a mode display. Results not known remain
unknown. Strong process kill/power loss/storage stalls do not guarantee a final
video manifest or bounded close time. Codec/render work in live UI is not a
process-crash/resource-isolation guarantee; the separate player does not alter that.
QGC run directly outside EasyFly is not subject to the offline preview isolation.

Choosing Keep in the owned-tool exit prompt keeps both EasyFly and its tools
open; it does not close the UI and detach those processes. Reopening EasyFly
does not automatically restart tools or recording. Same-instance mock joint
checks with local synthetic video, the real mission-control/service chain and
native RViz/offline QGC exercised these normal-close/recovery boundaries.
This is not a hardware, forced-kill, backend-restart or flight acceptance.

## Local dependency notices and delivery limitations

Verified machine baseline: ROS Humble / Qt5.15.3 / C++ OpenCV4.5.4 / RViz11.2.26;
external QGC sample5.0.8 with its own Qt6.8.3; bubblewrap0.6.1.
System packages and plugins keep their licenses; EasyFly's Apache-2.0 does not
relicense them. The detailed local inventory and complete source notices must
be retained if a later distribution includes those dependencies.

This is not a complete transitive SBOM, legal redistribution audit, standalone
AppImage/deb installer or clean-machine test. OpenCV's installed package contains
Apache-2.0 AND BSD-3-Clause notices; Qt's installed package declares alternatives
and separate third-party/tool licenses. QGC is independently licensed and not
redistributed here. Consult the actual installed/upstream notices, not only a
single package license field. See README's exact backend/environment references.

Acceptance evidence distinguishes mock graphs, synthetic images, limited prior
manual simulation work and native renderer tests. They are not interchangeable.
A camera-only Gazebo scene chain has now passed component checks and basic human
display/record/replay acceptance: the unchanged project front sensor was copied
into a fixed observation model in the project perception world, without PX4,
MAVROS or flight-control nodes. The actual scene recording's 224 frames were
independently read back at 640x480 and their hashes remained unchanged. This is
not full moving Iris/PX4 integration, a physical camera or flight acceptance.

A mixed-source check also exercised that fixed actual Gazebo camera while six
explicitly MOCK spatial layers rendered. Spatial loss/missing TF cleared geometry;
clock pause/reset finalized separate verified records without automatic resume.
These checks do not verify a genuine navigation/SLAM map or aircraft TF chain,
and short GUI timer observations are not long-run or real-time certification.

Pending: full aircraft sensor/flight-stack integration, fully combined
mission/media/tool close+reopen, sustained resource stress, clean install and
cross-platform delivery. Prediction/semantic/landing-specific displays, independent
flight controls, recorded trajectory and rosbag replay remain outside this increment.
The previously shelved production request-source issue is still unresolved.
No hardware connection or earlier paused flight acceptance is resumed by this guide.

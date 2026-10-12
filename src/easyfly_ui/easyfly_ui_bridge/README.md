# easyfly_ui_bridge

Qt-independent C++17 read-only observer for the EasyFly desktop. It connects to
an already running local ROS graph; it does not launch flight/simulation/tools.
No action submission, task start/cancel, flight command, parameter write or
generic service dispatcher is exposed. Existing backend, SDK and IDL are unchanged.

## Build and test

The commands below assume easyfly_msgs/easyfly_core are already built. For a clean
workspace, build with `--packages-up-to easyfly_ui_bridge` instead of packages-select.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select easyfly_ui_bridge
colcon test --packages-select easyfly_ui_bridge --ctest-args --output-on-failure
colcon test-result --test-result-base build/easyfly_ui_bridge/test_results --verbose
```

Component tests use localhost-only ROS domain 92, mock services/action servers
and two independent observer/SDK contexts. No PX4, Gazebo or real vehicle is needed.
Do not run another test graph in domain 92 concurrently. The library itself never
changes ROS_DOMAIN_ID or shuts down the default/global context.

Some Humble ament installations skip cppcheck 2.7 by default. Inspect skipped
results rather than treating a successful CTest launch as complete static analysis.
To explicitly run the small bridge-only analysis without installing/changing tools:

```bash
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 ctest --test-dir build/easyfly_ui_bridge -R '^cppcheck$' --output-on-failure
```

For consumers, `find_package(easyfly_ui_bridge REQUIRED)` and
`ament_target_dependencies(your_target easyfly_ui_bridge)` expose installed headers
and the shared library. There is no Qt dependency. JsonCpp is used privately for
bounded strict JSON parsing; its original MIT license is unaffected.

## API and lifecycle

Include `easyfly_ui_bridge/observer.hpp`. `Observer` starts observation in a worker,
so construction does not wait for a service or telemetry. `snapshot()` returns
`shared_ptr<const UiSnapshot>`; hold only the latest snapshot in a desktop model.
Old snapshots remain immutable, but a consumer retaining every old snapshot can
defeat the application's memory budget.

`BridgeConfig::use_sim_time` defaults to true and is explicitly applied to both
contexts. Choose false explicitly for a system-time backend; don't change domain
or time configuration while an observer is running. `selected_profile` is only
the UI selection, not proof of the actual backend profile. Camera bindings default
to separate front/down sources; missing cameras do not prevent startup.

Typed request methods return `RequestReceipt {id, status}` immediately. Queued is
not a server result. Observe the matching logical key's `evidence.request_id` in
subsequent snapshots. Busy, RateLimited, Invalid and Closed send no request.
Results are latest-per-logical-key, not an unbounded execution history.

- `capabilities`: four fixed platform endpoints; implemented/ready remain per-entry.
- `list_missions`, `validate_mission`: directory basenames only, no body/steps/active ID.
- `query_mission`: explicit 32-character lowercase service execution ID, not an Action UUID.
- `read_parameter`: profile/use_sim_time/operator_note/export_pretty only, with empty param_value.
- `perception`: targets/obstacles only, default range_limit=0.

Automatic polling queries the four capability endpoints at approximately 1Hz and
alternates targets/obstacles at a combined 1Hz. Mission/config reads are explicit.
Every endpoint is limited to one request per second, one outstanding logical key,
eight reserved requests overall and a one-second steady waiting budget. Timed-out
read requests are removed locally using rclcpp's pending-request cleanup; timeout,
unknown ID or task status alone never prove execution/ownership release.

`perception_reports` preserves the bounded upstream status/source/observed_at and
reported age even when geometry is invalid or too large to retain. Check display
quality before rendering a geometric layer; a missing geometry payload in an
invalid report must not be presented as a valid empty observation. Repeated reads
of the same geometry source preserve its original first-distinct receipt age,
including across failed/stale query attempts.

Call `stop()` on the GUI thread and poll `snapshot()->stopped`; stop itself does
not wait. Destroy only after cleanup completes, or on a non-GUI owner thread:
the destructor joins local cleanup and may wait for SDK cleanup. Do not treat
destruction as a GUI-safe instantaneous call. There are no owned SDK operations
in this facade, so close cannot cancel another client's action.

For reconnect, construct `Observer(new_config, stopped_previous_snapshot)`.
Only bounded event/warning history survives, labeled with its original session;
telemetry, geometry, service results, requests and trajectory are not reused.
Not passing history creates a separate observation history; a reconnecting desktop
must pass it so a prior command-uncertain warning does not disappear on disconnect.

## Display evidence, not a permission gate

`Quality::ValidAtObservation` means evidence at an observation/query, never flight
authorization. Retain upstream flags/raw task status and also use display quality.
Query success may accompany task failed and execution_pending=true. Global
MissionStatus is an unassociated phase event, not a heartbeat or terminal result.
An invalid/unavailable perception response is not a valid empty list.

Source ages use ROS time, while first-distinct receipt, waiting and cache ages use
steady time. Only show source_age when source_age_available is true; header-less
service results have query/receipt ages, not a fabricated zero source age. Repeated
source stamps do not renew first receipt or add trajectory points; same-stamp
conflicting payloads are invalid. Missing/zero/frozen simulated
clock cannot produce fresh display evidence. Backwards clock changes the epoch,
clears current geometry/images/trajectory/query results and invalidates old requests.
Historical phase events and observed command-uncertain warnings remain historical.

Budgets: two latest packed mono8/rgb8/bgr8 images, each at most 1920x1080 and 8MiB;
2000 geometry items; 64 capabilities, 4KiB per detail and 64KiB total details;
4096 trajectory points/300s; 1000 events/2048 bytes of text. Text byte limits are
conservative UTF-8 storage limits. A front camera image is never substituted for a
missing down camera. Conversion/overlays/widgets belong to the desktop stage.
Landing-specific observation panels, predictions and classified obstacles are
optional future adapters, not inferred from existing generic geometry.

These limits bound retained application data, not DDS deserialization, upstream
SDK memory, an untrusted ROS graph or arbitrary consumer-held old snapshots.
This local bridge is not authentication, a safety controller, a ground-height
estimator, or a substitute for backend flight admission.

## Known test instability

`test_observer` intermittently dies with SIGSEGV during harness teardown: an
independent retest on 2026-10-12 saw two failures in 57 native runs and one on
the first gdb run, with the stack inside `rclcpp::Executor::cancel()` called
from `ObserverTest::TearDown()` after the Observer under test had already been
destroyed inside the test body. The root cause is unconfirmed (no librclcpp
debug symbols, no AddressSanitizer run); a failing run only needs a rerun, but
repeated passes are not evidence of a certified crash fix.

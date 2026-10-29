# EasyFly owned C++ SDK

`Client` owns a separate ROS Context, Node, and one executor thread. It neither
borrows nor shuts down the application's ROS context. It publishes no control
commands. Ten typed action methods are provided: `takeoff`, `land`, `precision_land`, `go_to`,
`navigate`, `follow`, `track`, `search`, `explore`, `execute_mission`.

`running()` describes this SDK runtime, not vehicle readiness. `server_ready`
checks a particular endpoint using keys `takeoff`, `land`, `precision_land`, `goto`, `navigate`,
`follow`, `track`, `search`, `explore`, `execute_mission`. Discovery is asynchronous; an unavailable
server produces an explicit `Unavailable` operation, not success. Platform flight
admission remains the server's responsibility.

Operations expose independent admission, final status, latest typed feedback,
local status, cancellation response, and `remote_execution_pending`.
Only a real SUCCEEDED action result with `result.success=true` is success.
An explicit failed result prefixed `unsupported:` is reported Unsupported.
Rejected goals, aborted actions, canceled actions, and false-success results stay
distinct. `wait(seconds)` only observes: a short wait does NOT cancel. After
close, the last drained operation also waits for DDS cleanup within the same
budget; cleanup timeout can return false without changing the genuine result in
its snapshot. Other pending operations are not covered by one operation's wait.
`cancel()` queues cancellation of that exact goal handle. Accepted cancel
responses do NOT imply stopped execution; wait for the real result.

Managed navigation, search, exploration, precision landing and mission endpoints can finish their public
Action before internal children stop. Set `ClientOptions.require_managed_ownership=true`
for these endpoints: readiness also requires the matching UUID ownership service,
and drain waits for explicit RELEASED. `action_terminal_received` and `goal_id`
in snapshots distinguish the public result from `remote_execution_pending`.
UNKNOWN, missing peers and invalid identity echoes retain ownership. The option
defaults false for legacy standalone servers; default mode uses the protocol
only when discovered at send time. See [D3 contract](../../../../docs/development/d3-missions.md).

The independent admission response timeout defaults to five seconds. It records
`AdmissionTimedOut` while keeping remote ownership unresolved. Automatic exact
cancellation at that timeout is opt-in with `cancel_on_admission_timeout`.
There is no invented execution timeout or automatic global cancellation.

`close(seconds)` stops new admission, freezes/unsubscribes state, requests exact
cancellation of known and late-accepted owned goals, and waits up to the supplied
budget for drain. False means remote ownership or runtime cleanup is still
unresolved. The facade may
be destroyed promptly: its worker owns only C++ runtime data, never the facade or
Python objects. It continues draining until genuine terminal results and then
releases its executor, clients, Node and Context. If a peer never replies, bounded
pending data/resources remain retained (default capacity 32), rather than losing
late-cancel ownership. Process exit cannot guarantee that an external server
stopped. No force-stop or cancel-all escape hatch exists.

State exposes separate steady receipt freshness and source-time validity.
Combined `fresh` requires a positive valid odom stamp, source age in
[-0.2, freshness_seconds], and fresh receipt. Original pose/velocity/attitude/etc
valid flags remain independent; fresh is NOT permission to fly.
`ClientOptions.use_sim_time` defaults false; set true for a /clock-driven SITL
graph. Without valid /clock, simulated source time is not fresh.

Default example is query-only:

```bash
ros2 run easyfly_core sdk_state_example
```

ROS mock regressions use domain 91 and never connect to flight hardware.

`explore(Explore::Goal)` uses `/explore` and `/exploration/ownership`. Its result
`reason` maps to the same operation `message` field; feedback preserves phase,
coverage, completed viewpoints and next XYZ. A terminal action response does not
release a managed operation until the ownership service confirms release.
The Python binding accepts `explore(min_bound, max_bound, forbidden_boxes=[],
return_home=True, timeout_sec=120)`, where each box is a pair of XYZ triples.
Runtime bounds, current map, localization and flight admission remain server checks.

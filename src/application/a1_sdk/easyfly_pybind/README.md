<!--
  SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
  SPDX-FileCopyrightText: 2026 贾国源
  License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_pybind/licenses/.
  This revision offers no Apache-2.0 option for the listed original code.
-->
# Python binding of easyfly_core

This is the same C++ SDK, not a second flight state machine. ROS threads never
invoke Python callbacks. Poll `operation.snapshot()` or use
`operation.wait(seconds)`; wait and close release the GIL. `operation.cancel()`
only requests exact cancellation; check remote final status separately.

`Client` supports `state`, `takeoff`, `land`, `navigate`, `execute_mission`,
`server_ready`, `running`, `pending_operations`, `close` and a context manager.
Constructor options: max_pending_operations=32, admission_timeout_seconds=5,
cancel_on_admission_timeout=False, use_sim_time=False, require_managed_ownership=False.
Context manager exit initiates close without pretending unresolved goals stopped.
See easyfly_core's lifecycle and retained-drain limitations.

For D3 managed navigation/mission execution use `require_managed_ownership=True`.
An Action terminal result (`action_terminal_received`) is separate from recursive
drain (`remote_execution_pending`); only matching UUID RELEASED confirms drain.
Unknown or missing ownership services do not unlock an operation. Python still
binds the same C++ implementation, including these snapshot fields.
The minimal direct Python action set remains unchanged; track/search steps are
available through `execute_mission` rather than new direct Python methods.

Default example queries only:

```bash
ros2 run easyfly_pybind sdk_state_example
```

Explicit flight calls (ONLY in an appropriately prepared platform) return
Operation objects; request acceptance is never execution success.
Unavailable endpoints and unsupported features are explicit failed capabilities.
Python mock regressions use domain 92.

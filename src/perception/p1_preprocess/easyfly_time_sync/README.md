<!--
  SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
  SPDX-FileCopyrightText: 2026 贾国源
  License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_time_sync/licenses/.
  This revision offers no Apache-2.0 option for the listed original code.
-->
# Software image pairing

Explicit independent `time_sync_node` preserves both original image headers/data.
It publishes paired images and bounded queue/drop diagnostics, not hardware clock
synchronization or spatial registration. It does not remap production inputs.
Source ROS time and steady receipt/progress budgets are independent; missing,
paused or regressed clocks cannot keep old pairs ready. Readiness is not flight
permission. Application queue budgets do not cover DDS deserialization memory.

Default simulated launch: `ros2 launch easyfly_time_sync time_sync.launch.py`.
Use `use_sim_time:=false` for system-time streams. Parameters, interfaces,
verification and limitations: [D4 guide](../../../../docs/development/d4-perception.md).

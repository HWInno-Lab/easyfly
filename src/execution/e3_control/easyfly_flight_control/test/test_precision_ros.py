# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Real controller process with explicit fake flight feedback, not SITL."""

import os
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from easyfly_msgs.action import GoToWaypoint, Land, PrecisionLand, Takeoff
from easyfly_msgs.msg import FlightStatus, LandingGuidanceTarget, UAVCommand, UAVState
from easyfly_msgs.srv import (
    ArmMotors,
    GetActionOwnership,
    GetCapabilities,
    SetFlightMode,
)
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future


@pytest.mark.parametrize(
    "scenario",
    [
        "normal",
        "moving_normal",
        "moving_timeout",
        "moving_corridor",
        "cancel_guidance",
        "loss",
        "takeover",
        "cancel_land",
        "timeout_land",
        "cancel_mode_pending",
        "unknown_mode_late",
        "mode_wait",
        "mode_takeover_before_aggregate",
        "mode_wait_timeout",
        "cancel_mode_wait",
    ],
)
def test_precision_process(scenario):
    """Exercise parent ownership, child Land, cancel, expiry and takeover."""
    executable = os.path.join(
        get_package_prefix("easyfly_flight_control"),
        "lib/easyfly_flight_control/flight_control_node",
    )
    parameters = {
        "precision_enabled": "true",
        "precision_tag_id": "7",
        "precision_tag_size_m": "0.2",
        "precision_horizontal_speed": "0.2",
        "precision_descent_speed": "0.1",
        "precision_align_tolerance": "0.1",
        "precision_handoff_height": "0.3",
        "precision_descent_floor_height": "0.2",
        "precision_max_height": "3.0",
        "precision_max_radius": "2.0",
        "precision_plane_z": "0.0",
        "precision_target_max_age": "0.15",
        "precision_lost_timeout": "0.2",
        "precision_acquire_timeout": "0.3",
        "precision_settle_observations": "3",
        "land_timeout": "5.0",
    }
    if scenario.startswith('moving_'):
        parameters.update(precision_moving_target='true', precision_moving_land_timeout='2.0')
    controller_name = "precision_fixture_flight_" + scenario
    command = [executable, "--ros-args", "-r", "__node:=" + controller_name]
    for name, value in parameters.items():
        command.extend(["-p", name + ":=" + value])
    rclpy.init()
    node = rclpy.create_node("precision_fake_flight_peer_" + scenario)
    state = UAVState()
    state.header.frame_id = "odom"
    state.connected = state.pose_valid = state.attitude_valid = True
    state.flight_mode = 2
    land_started = None
    mode_requests = []
    arm_requests = []
    commands = []
    publish_target = True
    fixed_stamp = None
    mode_gate = Future()
    mode_pending_since = None
    mode_completed = False
    land_status = []
    status_stamp_override = None
    status_mode_override = None

    async def mode_callback(request, response):
        nonlocal land_started, mode_pending_since, mode_completed
        mode_requests.append(request.flight_mode)
        if request.flight_mode == 7 and scenario in [
            "cancel_mode_pending",
            "unknown_mode_late",
        ]:
            mode_pending_since = time.monotonic()
            await mode_gate
            mode_completed = True
            if scenario == "cancel_mode_pending":
                response.success = False
                response.message = "fixture: delayed explicit mode rejection"
                return response
        state.flight_mode = request.flight_mode
        if request.flight_mode == 7:
            land_started = time.monotonic()
        response.success = True
        return response

    def arm_callback(request, response):
        arm_requests.append(request.arm)
        state.armed = request.arm
        # Deliberate fixture dynamics only: no aircraft model or accuracy claim.
        state.position_z = 0.3 if request.arm else 0.0
        response.success = True
        return response

    node.create_service(
        SetFlightMode,
        "/set_flight_mode",
        mode_callback,
        callback_group=ReentrantCallbackGroup(),
    )
    node.create_service(ArmMotors, "/arm_motors", arm_callback)
    state_pub = node.create_publisher(UAVState, "/uav_state", 10)
    status_pub = node.create_publisher(FlightStatus, "/flight_status", 10)
    target_pub = node.create_publisher(
        LandingGuidanceTarget, "/landing/guidance_target", 1
    )
    node.create_subscription(UAVCommand, "/uav_command", commands.append, 10)
    node.create_subscription(
        GoalStatusArray,
        "/land/_action/status",
        lambda message: land_status.extend(message.status_list),
        10,
    )
    child_cancel_probe = node.create_client(
        CancelGoal, "/land/_action/cancel_goal"
    )
    takeoff = ActionClient(node, Takeoff, "/takeoff")
    precision = ActionClient(node, PrecisionLand, "/precision_land")
    ordinary_land = ActionClient(node, Land, "/land")
    waypoint = ActionClient(node, GoToWaypoint, "/goto_waypoint")
    ownership = node.create_client(
        GetActionOwnership, "/precision_landing/ownership"
    )
    capabilities = node.create_client(
        GetCapabilities, "/platform/precision_landing/capabilities"
    )
    process = subprocess.Popen(command)

    def pump(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if (
                scenario in ["normal", "mode_wait", "moving_normal"]
                and land_started is not None
                and time.monotonic() - land_started > 0.2
            ):
                state.armed = False
                state.pose_valid = False
            state.header.stamp = node.get_clock().now().to_msg()
            state_pub.publish(state)
            status = FlightStatus(
                valid=True, connected=state.connected, armed=state.armed,
                flight_mode=(state.flight_mode if status_mode_override is None
                             else status_mode_override),
                receipt_age_s=0.0)
            status.header.stamp = status_stamp_override or state.header.stamp
            status.flight_mode_name = {3: "OFFBOARD", 7: "AUTO.LAND"}.get(
                status.flight_mode, "OTHER")
            status_pub.publish(status)
            if publish_target:
                target = LandingGuidanceTarget()
                target.header.stamp = fixed_stamp or state.header.stamp
                target.header.frame_id = "odom"
                target.source_header.stamp = target.header.stamp
                target.source_header.frame_id = "camera_down_optical_frame"
                target.tag_id = 7
                target.tag_size_m = 0.2
                target.valid = True
                target.plane_normal.z = 1.0
                target.position.x = (
                    0.0
                    if scenario
                    in [
                        "normal",
                        "moving_normal",
                        "moving_timeout",
                        "moving_corridor",
                        "cancel_land",
                        "timeout_land",
                        "cancel_mode_pending",
                        "unknown_mode_late",
                        "mode_wait",
                        "mode_takeover_before_aggregate",
                        "mode_wait_timeout",
                        "cancel_mode_wait",
                    ]
                    else 1.0
                )
                if scenario == 'moving_corridor' and land_started is not None:
                    target.position.x = .5
                target_pub.publish(target)
            rclpy.spin_once(node, timeout_sec=0.01)

    def wait(future, timeout=4):
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            pump(0.02)
        assert future.done()
        return future.result()

    def owned(handle):
        request = GetActionOwnership.Request()
        request.goal_id = bytes(handle.goal_id.uuid).hex()
        return wait(ownership.call_async(request)).state

    def settle_fixture_graph():
        # wait_for_server may discover a request endpoint before the reverse
        # service response reader is matched. A failed response cannot be fixed
        # by waiting longer after sending, and retrying that goal is unsafe.
        # Match this unique process (not a prior case's stale graph participant)
        # and keep pumping feedback through a bounded discovery-settle window.
        deadline = time.monotonic() + 6
        stable_since = None
        while time.monotonic() < deadline:
            pump(0.02)
            own_state_reader = any(
                info.node_name == controller_name
                for info in node.get_subscriptions_info_by_topic("/uav_state")
            )
            own_command_writer = any(
                info.node_name == controller_name
                for info in node.get_publishers_info_by_topic("/uav_command")
            )
            matched = (
                own_state_reader
                and own_command_writer
                and state_pub.get_subscription_count() > 0
                and target_pub.get_subscription_count() > 0
                and all(
                    client.server_is_ready()
                    for client in [takeoff, precision, ordinary_land, waypoint]
                )
                and ownership.service_is_ready()
                and capabilities.service_is_ready()
            )
            if not matched:
                stable_since = None
            elif stable_since is None:
                stable_since = time.monotonic()
            elif time.monotonic() - stable_since >= 1.0:
                return
        pytest.fail(
            "new fixture DDS graph did not match and settle before any goal"
        )

    try:
        assert takeoff.wait_for_server(timeout_sec=5)
        assert precision.wait_for_server(timeout_sec=5)
        assert ownership.wait_for_service(timeout_sec=5)
        assert capabilities.wait_for_service(timeout_sec=5)
        settle_fixture_graph()
        # A discovered request endpoint does not prove the response reader has
        # matched. Only this read-only capability query is safe to retry before
        # flight; actions and mode/arming requests are still submitted once.
        deadline = time.monotonic() + 6
        observed = None
        while time.monotonic() < deadline and observed is None:
            probe = capabilities.call_async(GetCapabilities.Request())
            reply_deadline = min(deadline, time.monotonic() + .5)
            while not probe.done() and time.monotonic() < reply_deadline:
                pump(.02)
            if probe.done():
                observed = probe.result()
            else:
                capabilities.remove_pending_request(probe)
        assert observed is not None, 'read-only capability round trip never became ready'
        assert observed.names == ["precision_landing"]
        assert observed.implemented == [True]
        pump(0.2)
        takeoff_goal = Takeoff.Goal()
        takeoff_goal.target_altitude = 0.3
        takeoff_handle = wait(takeoff.send_goal_async(takeoff_goal))
        assert takeoff_handle.accepted
        assert (
            wait(takeoff_handle.get_result_async(), 6).status
            == GoalStatus.STATUS_SUCCEEDED
        )
        pump(0.05)
        if scenario in ["mode_wait", "mode_takeover_before_aggregate",
                        "mode_wait_timeout", "cancel_mode_wait"]:
            status_stamp_override = state.header.stamp
        goal = PrecisionLand.Goal()
        goal.tag_id = 7
        goal.timeout = (
            0.4
            if scenario == "timeout_land"
            else 0.8
            if scenario == "mode_wait_timeout"
            else 10.0
            if scenario == "unknown_mode_late"
            else 5.0
        )
        feedback = []
        handle = wait(
            precision.send_goal_async(
                goal,
                feedback_callback=lambda message: feedback.append(
                    message.feedback
                ),
            )
        )
        assert handle.accepted
        result = handle.get_result_async()
        assert owned(handle) == GetActionOwnership.Response.OWNED
        if scenario in ["cancel_guidance", "loss", "takeover"]:
            assert not wait(
                ordinary_land.send_goal_async(Land.Goal())
            ).accepted
            move = GoToWaypoint.Goal()
            move.target_z = 0.3
            assert not wait(waypoint.send_goal_async(move)).accepted
        if scenario in ["mode_wait", "mode_takeover_before_aggregate",
                        "mode_wait_timeout", "cancel_mode_wait"]:
            deadline = time.monotonic() + 3
            while not any(item.phase == "handoff_wait_mode" for item in feedback):
                assert time.monotonic() < deadline
                pump(0.02)
            assert mode_requests == [3]
            baseline = len(commands)
            state.position_z = .29
            pump(.1)
            state.position_z = .27
            pump(.1)
            held = commands[baseline:]
            assert len(held) >= 2 and all(abs(c.position_z - .3) < 1e-6 for c in held)
            assert mode_requests == [3] and not result.done()
            if scenario == "cancel_mode_wait":
                assert wait(handle.cancel_goal_async()).goals_canceling
            elif scenario != "mode_wait_timeout":
                status_stamp_override = None
            if scenario == "mode_takeover_before_aggregate":
                status_mode_override = 7
                assert state.flight_mode == 3  # Deliberately stale aggregate mode.
                pump(.1)
                stopped_count = len(commands)
                pump(.1)
                assert len(commands) == stopped_count
        elif scenario == "loss":
            # Replayed positive source stamps do not refresh receipt TTL/count.
            fixed_stamp = state.header.stamp
            pump(0.45)
        elif scenario == "takeover":
            state.flight_mode = 6
            pump(0.15)
            baseline = len(commands)
            pump(0.15)
            assert len(commands) == baseline
        elif scenario == "cancel_guidance":
            assert wait(handle.cancel_goal_async()).goals_canceling
        elif scenario in ["cancel_mode_pending", "unknown_mode_late"]:
            deadline = time.monotonic() + 3
            while mode_pending_since is None and time.monotonic() < deadline:
                pump(0.02)
            assert mode_pending_since is not None
            assert state.flight_mode == 3
            assert not result.done()
            assert owned(handle) == GetActionOwnership.Response.OWNED
            if scenario == "cancel_mode_pending":
                assert wait(handle.cancel_goal_async()).goals_canceling
                deadline = time.monotonic() + 2
                while not land_status and time.monotonic() < deadline:
                    pump(0.02)
                assert land_status
                child_uuid = land_status[-1].goal_info.goal_id
                probe = CancelGoal.Request()
                probe.goal_info.goal_id = child_uuid
                probe_future = child_cancel_probe.call_async(probe)
                baseline = len(commands)
                pump(0.15)
                assert not result.done()
                assert owned(handle) == GetActionOwnership.Response.OWNED
                assert all(
                    item.position_z >= state.position_z - 1e-6
                    for item in commands[baseline:]
                )
                assert mode_requests == [3, 7]
                assert not probe_future.done()
                mode_gate.set_result(True)
        elif scenario in ["cancel_land", "timeout_land"]:
            deadline = time.monotonic() + 3
            while state.flight_mode != 7 and time.monotonic() < deadline:
                pump(0.02)
            assert state.flight_mode == 7
            assert owned(handle) == GetActionOwnership.Response.OWNED
            assert any(item.phase == "land_handoff" for item in feedback)
            if scenario == "cancel_land":
                assert wait(handle.cancel_goal_async()).goals_canceling
        wrapped = wait(result, 6)
        if scenario in ['moving_timeout', 'moving_corridor']:
            assert 'moving pad' in wrapped.result.message
            assert 'child Land drained' in wrapped.result.message
            assert any(item.status == GoalStatus.STATUS_CANCELED for item in land_status)
        expected = (
            GoalStatus.STATUS_SUCCEEDED
            if scenario in ["normal", "mode_wait", "moving_normal"]
            else GoalStatus.STATUS_CANCELED
            if scenario.startswith("cancel")
            else GoalStatus.STATUS_ABORTED
        )
        assert wrapped.status == expected
        assert wrapped.result.success == (scenario in ["normal", "mode_wait", "moving_normal"])
        assert owned(handle) == GetActionOwnership.Response.RELEASED
        if scenario == "cancel_mode_pending":
            assert not wait(probe_future).goals_canceling
            pump(0.05)
            assert any(
                item.goal_info.goal_id == child_uuid
                and item.status == GoalStatus.STATUS_ABORTED
                for item in land_status
            )
        if scenario == "unknown_mode_late":
            assert wrapped.result.message.startswith("command_result_unknown:")
            assert (
                not mode_completed
            )  # Public terminal precedes late response.
            pump(
                0.05
            )  # Drain already queued output before checking stop-stream.
            stopped_count = len(commands)
            pump(0.1)
            assert len(commands) == stopped_count
            # Action drain is not proof the remote mode transaction is known.
            # Fresh disarmed baseline would otherwise allow a new Takeoff.
            state.armed = False
            state.flight_mode = 2
            pump(0.05)
            retry = Takeoff.Goal()
            retry.target_altitude = 1.0
            assert not wait(takeoff.send_goal_async(retry)).accepted
            assert not wait(precision.send_goal_async(goal)).accepted
            assert not wait(
                ordinary_land.send_goal_async(Land.Goal())
            ).accepted
            move = GoToWaypoint.Goal()
            move.target_z = 1.0
            assert not wait(waypoint.send_goal_async(move)).accepted
            assert time.monotonic() - mode_pending_since >= 4.9
            mode_gate.set_result(True)
            deadline = time.monotonic() + 2
            while not mode_completed and time.monotonic() < deadline:
                pump(0.02)
            assert mode_completed
            assert state.flight_mode == 7
            pump(0.05)
            assert not wait(takeoff.send_goal_async(retry)).accepted
            assert not wait(precision.send_goal_async(goal)).accepted
            assert owned(handle) == GetActionOwnership.Response.RELEASED
            stopped_count = len(commands)
            pump(0.1)
            assert len(commands) == stopped_count
        assert arm_requests == [True]  # No compensating disarm.
        if scenario in ["normal", "mode_wait", "moving_normal"]:
            assert mode_requests == [3, 7]
        elif scenario in [
            "cancel_land",
            "timeout_land",
            "moving_timeout",
            "moving_corridor",
        ]:
            assert mode_requests == [3, 7, 3]
            assert state.armed
        elif scenario in ["unknown_mode_late", "cancel_mode_pending"]:
            assert mode_requests == [3, 7]
        else:
            assert mode_requests == [3]
        process.send_signal(signal.SIGINT)
        assert process.wait(timeout=3) == 0
    finally:
        if not mode_gate.done():
            mode_gate.set_result(True)
        if mode_pending_since is not None and not mode_completed:
            pump(
                0.1
            )  # Drain the fake async service before destroying its handle.
        if process.poll() is None:
            process.kill()
            process.wait(timeout=3)
        node.destroy_node()
        rclpy.shutdown()

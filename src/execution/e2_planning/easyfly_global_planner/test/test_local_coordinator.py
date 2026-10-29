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

"""Coordinator with real ROS mock flight/planner: ordering, routes, failures."""
import os
import signal
import subprocess
import time
import unittest
import uuid
from unique_identifier_msgs.msg import UUID

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.qos import QoSProfile, DurabilityPolicy
from easyfly_msgs.action import FollowTrajectory, NavigateTo
from easyfly_msgs.msg import UAVState, NavigationSession, VoxelGrid
from easyfly_msgs.srv import (
    SetNavigationSession,
    StopNavigation,
    ReserveNavigation,
    GetActionOwnership,
)
from nav_msgs.msg import OccupancyGrid


class LocalCoordinatorTest(unittest.TestCase):
    def run_scenario(self, strategy, scenario):
        rclpy.init()
        node = rclpy.create_node("local_coordinator_mock")
        events, handles, start_times = [], {}, {}
        position = [0.0, 0.0, 1.0]
        goals = []
        blocked = False
        reset_stamp = False
        flight_mode = 3
        active = ""
        fault_pub = node.create_publisher(
            NavigationSession,
            "/planning/navigation_fault",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        replan_pub = node.create_publisher(
            NavigationSession,
            "/planning/" + strategy + "_replan",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        stops = node.create_client(StopNavigation, "/navigate/_stop_owned")

        async def execute(handle):
            session = handle.request.navigation_session_id
            handles[session] = handle
            goals.append(handle.request)
            events.append(("accept", session))
            complete = rclpy.task.Future()

            def poll():
                nonlocal blocked, reset_stamp, flight_mode
                if complete.done():
                    return
                if handle.is_cancel_requested:
                    complete.set_result(False)
                elif session in start_times:
                    age = time.monotonic() - start_times[session]
                    if scenario == "dynamic" and len(goals) == 1 and age > 0.1:
                        blocked = True
                    if scenario == "invalid" and age > 0.1:
                        reset_stamp = True
                    if scenario == "emergency" and age > 0.1:
                        fault_pub.publish(NavigationSession(session_id=session, active=False))
                    if (
                        scenario in ("gcopter-change", "gcopter-budget", "gcopter-hold-z")
                        and age > 0.1
                        and (len(goals) == 1 or scenario == "gcopter-budget")
                    ):
                        if scenario == "gcopter-hold-z":
                            position[2] = 1.5
                        replan_pub.publish(NavigationSession(session_id=session, active=False))
                    if scenario in ("rtl", "land") and age > 0.1:
                        flight_mode = 6 if scenario == "rtl" else 7
                    if scenario == "streamlost" and age > 0.1:
                        complete.set_result(False)
                    if scenario not in (
                        "dynamic",
                        "invalid",
                        "cancel",
                        "owned",
                        "emergency",
                        "rtl",
                        "land",
                        "streamlost",
                        "gcopter-change",
                        "gcopter-budget",
                        "gcopter-hold-z",
                    ) or (len(goals) > 1 and scenario != "gcopter-budget"):
                        if age > 0.15:
                            if scenario != "rolling-no-progress":
                                position[:] = [
                                    handle.request.target_x,
                                    handle.request.target_y,
                                    handle.request.target_z,
                                ]
                            complete.set_result(True)

            timer = node.create_timer(0.02, poll)
            reached = await complete
            node.destroy_timer(timer)
            result = FollowTrajectory.Result()
            result.success = reached
            result.message = (
                "trajectory stream lost"
                if scenario == "streamlost"
                else "arrived"
                if reached
                else "canceled"
            )
            if handle.is_cancel_requested:
                handle.canceled()
            elif scenario == "streamlost":
                handle.abort()
            else:
                handle.succeed()
            events.append(("terminal", session))
            return result

        def session(request, response):
            nonlocal active
            events.append(("start" if request.active else "stop", request.session_id))
            if request.active:
                self.assertIn(request.session_id, handles)  # Follow accepted before planner start.
                self.assertEqual(active, "")  # Previous STOP was acknowledged.
                active = request.session_id
                start_times[active] = time.monotonic()
                fault_pub.publish(NavigationSession(session_id="old-session", active=False))
                replan_pub.publish(NavigationSession(session_id="old-session", active=False))
                response.success = scenario != "reject"
            else:
                active = ""
                response.success = scenario != "lease-fault"
            return response

        flight = ActionServer(
            node,
            FollowTrajectory,
            "/follow_trajectory",
            execute,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        planner = node.create_service(
            SetNavigationSession,
            "/planning/" + strategy + "_session"
            if strategy in ("gcopter", "super")
            else "/planning/navigation_session",
            session,
        )
        state_pub = node.create_publisher(UAVState, "/uav_state", 10)
        map_pub = node.create_publisher(OccupancyGrid, "/occupancy_grid", 10)
        voxel_pub = node.create_publisher(
            VoxelGrid,
            "/planning/voxel_grid",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        def publish():
            stamp = node.get_clock().now().to_msg()
            state = UAVState()
            state.header.stamp, state.header.frame_id = stamp, "odom"
            state.connected = state.pose_valid = state.armed = True
            state.velocity_valid = True
            state.flight_mode = flight_mode
            state.position_x, state.position_y, state.position_z = position
            state_pub.publish(state)
            grid = OccupancyGrid()
            grid.header.frame_id = "odom"
            if not reset_stamp:
                grid.header.stamp = stamp
            grid.info.resolution = 0.2
            grid.info.width = grid.info.height = 60
            grid.info.origin.position.x = grid.info.origin.position.y = -4.0
            grid.info.origin.orientation.w = 1.0
            grid.data = [0] * 3600
            # A static wall requires a detour, but remains bounded and reachable.
            if scenario == "static":
                for row in range(15, 26):
                    grid.data[row * 60 + 30] = 100
            if blocked:
                for row in range(18, 23):
                    grid.data[row * 60 + 26] = 100
            map_pub.publish(grid)
            if strategy in ("gcopter", "super"):
                voxel = VoxelGrid()
                voxel.header.stamp, voxel.header.frame_id = stamp, "odom"
                voxel.session_id, voxel.generation, voxel.revision = "map", 1, 1
                voxel.valid, voxel.resolution, voxel.inflation_radius = True, 0.5, 0.65
                voxel.origin.x = voxel.origin.y = -4.0
                voxel.size_x = voxel.size_y = voxel.size_z = 20
                voxel.data = [0] * 8000
                if scenario.startswith("rolling"):
                    voxel.origin.x = position[0]-4.0
                    voxel.origin.z = -2.0
                    voxel.revision = stamp.sec*10**9+stamp.nanosec
                voxel_pub.publish(voxel)

        node.create_timer(0.05, publish)
        process = subprocess.Popen(
            [
                "ros2",
                "run",
                "easyfly_global_planner",
                "navigation_coordinator_node",
                "--ros-args",
                "-p",
                "strategy:=" + strategy,
                "-p",
                "local_goal_distance:=1.5",
                "-p",
                "cancel_timeout:=0.8",
                "-p",
                "session_service:="
                + (
                    "/planning/" + strategy + "_session"
                    if strategy in ("gcopter", "super")
                    else "/planning/navigation_session"
                ),
                "-p",
                "max_replans:=" + ("1" if strategy in ("gcopter", "super") else "3"),
                "-p", "gcopter_segmented_navigation:=" +
                str(scenario.startswith("rolling")).lower(),
                "-p", "gcopter_max_segments:=" + ("2" if scenario == "rolling-budget" else "20"),
            ],
            start_new_session=True,
        )
        client = ActionClient(node, NavigateTo, "/navigate")

        def wait(future, seconds=10):
            end = time.monotonic() + seconds
            while not future.done() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertTrue(future.done(), (scenario, events))
            return future.result()

        try:
            self.assertTrue(client.wait_for_server(timeout_sec=10))
            if strategy in ("gcopter", "super"):
                # GCOPTER admission requires actual fresh voxel receipts;
                # discovering the action server alone is not readiness.
                end = time.monotonic() + 3.0
                while voxel_pub.get_subscription_count() < 1 and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.02)
                self.assertGreaterEqual(voxel_pub.get_subscription_count(), 1)
                end = time.monotonic() + 0.2
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.02)
            goal = NavigateTo.Goal()
            goal.target_x, goal.target_z, goal.max_speed = 4.0, 1.0, 1.0
            if scenario.startswith("rolling"):
                goal.target_x = 10.0
            if scenario == "gcopter-hold-z":
                goal.target_z = float("nan")
            if scenario in ("lease", "lease-fault"):
                leases = node.create_client(ReserveNavigation, "/navigation/reserve")
                self.assertTrue(leases.wait_for_service(timeout_sec=3.0))
                mission = uuid.uuid4().hex
                child = uuid.uuid4().hex

                def lease(mission_id=mission, child_goal_id="", active=True):
                    return wait(
                        leases.call_async(
                            ReserveNavigation.Request(
                                mission_id=mission_id, child_goal_id=child_goal_id, active=active
                            )
                        )
                    )

                self.assertTrue(lease(child_goal_id=child).success)
                self.assertFalse(lease(mission_id=uuid.uuid4().hex).success)
                self.assertFalse(wait(client.send_goal_async(goal)).accepted)
                handle = wait(
                    client.send_goal_async(goal, goal_uuid=UUID(uuid=list(bytes.fromhex(child))))
                )
                self.assertFalse(lease(active=False).success)
            else:
                handle = wait(client.send_goal_async(goal))
            self.assertTrue(handle.accepted)
            if scenario == "lease-fault":
                end = time.monotonic() + 3.0
                while not active and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.02)
                self.assertTrue(active)
                wait(handle.cancel_goal_async())
            if scenario in ("cancel", "owned"):
                end = time.monotonic() + 3
                while not active and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.02)
                self.assertTrue(active)
                if scenario == "cancel":
                    wait(handle.cancel_goal_async())
                else:
                    request = StopNavigation.Request()
                    request.navigation_goal_id = "wrong-old-goal"
                    self.assertFalse(wait(stops.call_async(request)).success)
                    request.navigation_goal_id = bytes(handle.goal_id.uuid).hex()
                    self.assertTrue(wait(stops.call_async(request)).success)
            result = wait(handle.get_result_async())
            expected = (
                5
                if scenario == "cancel"
                else 6
                if scenario
                in (
                    "reject",
                    "invalid",
                    "owned",
                    "emergency",
                    "rtl",
                    "land",
                    "streamlost",
                    "gcopter-budget",
                    "rolling-budget",
                    "rolling-no-progress",
                    "lease-fault",
                )
                else 4
            )
            self.assertEqual(result.status, expected, events)
            self.assertEqual(result.result.success, expected == 4)
            if scenario == "emergency":
                self.assertEqual(result.result.message, "planner_emergency_stop")
            if scenario == "streamlost":
                self.assertEqual(result.result.message, "trajectory stream lost")
            if scenario in ("rtl", "land"):
                self.assertEqual(result.result.message, "navigation input invalid or stale")
            self.assertEqual(active, "")
            if scenario == "lease-fault":
                self.assertFalse(lease(active=False).success)
                self.assertFalse(wait(client.send_goal_async(goal)).accepted)
                ownership = node.create_client(GetActionOwnership, "/navigation/ownership")
                self.assertTrue(ownership.wait_for_service(timeout_sec=3.0))
                state = wait(
                    ownership.call_async(
                        GetActionOwnership.Request(goal_id=bytes(handle.goal_id.uuid).hex())
                    )
                )
                self.assertEqual(state.state, GetActionOwnership.Response.OWNED)
            if scenario == "lease":
                self.assertFalse(wait(client.send_goal_async(goal)).accepted)
                self.assertFalse(lease(mission_id=uuid.uuid4().hex, active=False).success)
                self.assertTrue(lease(active=False).success)
                following = wait(client.send_goal_async(goal))
                self.assertTrue(following.accepted)
                self.assertEqual(wait(following.get_result_async()).status, 4)
            self.assertTrue(goals)
            if scenario.startswith("rolling"):
                self.assertLess(goals[0].target_x, goal.target_x)
                if scenario == "rolling":
                    self.assertGreater(len(goals), 2)
                    self.assertEqual(goals[-1].target_x, goal.target_x)
                else:
                    self.assertIn("progress or session budget", result.result.message)
                    self.assertEqual(len(goals), 2 if scenario == "rolling-budget" else 1)
            if scenario in ("gcopter-change", "gcopter-budget", "gcopter-hold-z"):
                self.assertEqual(len(goals), 2)
                if scenario == "gcopter-hold-z":
                    self.assertEqual([g.target_z for g in goals], [1.0, 1.0])
                if scenario == "gcopter-budget":
                    self.assertIn("replan budget", result.result.message)
            if scenario in ("static", "dynamic"):
                self.assertGreater(len(goals), 1)
                self.assertTrue(any(abs(g.target_y) > 0.1 for g in goals))
            for i, g in enumerate(goals):
                self.assertLessEqual(len(g.navigation_session_id), 128)
                self.assertEqual(g.acceptance_radius, 0.1)
                if i:
                    sid = goals[i - 1].navigation_session_id
                    self.assertLess(
                        events.index(("terminal", sid)),
                        events.index(("accept", g.navigation_session_id)),
                    )
                    self.assertLess(
                        events.index(("stop", sid)),
                        events.index(("accept", g.navigation_session_id)),
                    )
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            flight.destroy()
            node.destroy_service(planner)
            client.destroy()
            node.destroy_node()
            rclpy.shutdown()

    def test_fuel_parent_lease_exact_child_and_intergoal_gap(self):
        self.run_scenario("gcopter", "lease")

    def test_fuel_lease_retained_when_stop_ack_missing(self):
        self.run_scenario("gcopter", "lease-fault")

    def test_rolling_targets_retain_parent_until_actual_global_arrival(self):
        self.run_scenario("gcopter", "rolling")

    def test_rolling_targets_stop_on_missing_measured_progress(self):
        self.run_scenario("gcopter", "rolling-no-progress")

    def test_rolling_targets_retain_original_session_budget(self):
        self.run_scenario("gcopter", "rolling-budget")

    def test_ego(self):
        self.run_scenario("ego", "clear")

    def test_super_stops_both_owners_before_new_session(self):
        self.run_scenario("super", "gcopter-change")

    def test_super_replanning_preserves_initial_hold_altitude(self):
        self.run_scenario("super", "gcopter-hold-z")

    def test_super_replanning_is_bounded(self):
        self.run_scenario("super", "gcopter-budget")

    def test_gcopter_stops_both_owners_before_new_session(self):
        self.run_scenario("gcopter", "gcopter-change")

    def test_gcopter_replanning_preserves_initial_hold_altitude(self):
        self.run_scenario("gcopter", "gcopter-hold-z")

    def test_gcopter_replanning_is_bounded(self):
        self.run_scenario("gcopter", "gcopter-budget")

    def test_static_combined(self):
        self.run_scenario("combined", "static")

    def test_dynamic_combined(self):
        self.run_scenario("combined", "dynamic")

    def test_invalid_map(self):
        self.run_scenario("combined", "invalid")

    def test_planner_rejection(self):
        self.run_scenario("ego", "reject")

    def test_cancel(self):
        self.run_scenario("ego", "cancel")

    def test_owned_stop(self):
        self.run_scenario("ego", "owned")

    def test_emergency_event_is_owned(self):
        self.run_scenario("ego", "emergency")

    def test_external_rtl(self):
        self.run_scenario("ego", "rtl")

    def test_external_land(self):
        self.run_scenario("ego", "land")

    def test_downstream_stream_failure(self):
        self.run_scenario("ego", "streamlost")


if __name__ == "__main__":
    unittest.main()

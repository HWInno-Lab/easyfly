# Copyright 2026 EasyFly Project
#
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

"""Real A* node: budgets, cancellation and invalid-map child ownership."""

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix
from easyfly_msgs.action import GoToWaypoint, NavigateTo
from easyfly_msgs.msg import UAVState
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse


class PlanningControlTest(unittest.TestCase):
    """Use synthetic maps/telemetry and an explicit mock waypoint server."""

    def setUp(self):
        """Set up isolated publishers and observable child actions."""
        rclpy.init()
        self.node = rclpy.create_node('alg3_planning_control_test')
        self.invalid = False
        self.calls = []
        self.cancels = 0
        self.process = None
        self.log = tempfile.TemporaryFile(mode='w+')
        self.states = self.node.create_publisher(UAVState, '/uav_state', 10)
        self.maps = self.node.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

        async def execute(handle):
            self.calls.append(handle.request)
            done = rclpy.task.Future()
            timer = self.node.create_timer(
                .01, lambda: done.set_result(True)
                if handle.is_cancel_requested and not done.done() else None)
            try:
                await done
                handle.canceled()
                return GoToWaypoint.Result(success=False, message='child canceled')
            finally:
                self.node.destroy_timer(timer)

        def cancel(_):
            self.cancels += 1
            return CancelResponse.ACCEPT

        self.server = ActionServer(self.node, GoToWaypoint, '/goto_waypoint', execute,
                                   cancel_callback=cancel)
        self.timer = self.node.create_timer(.03, self.publish)
        self.client = ActionClient(self.node, NavigateTo, '/navigate_astar')

    def publish(self):
        """Keep telemetry fresh; optionally issue semantic-map reset protocol."""
        state = UAVState(connected=True, armed=True, pose_valid=True, velocity_valid=True)
        state.header.frame_id = 'odom'
        state.header.stamp = self.node.get_clock().now().to_msg()
        state.position_x, state.position_y, state.position_z = -3., 0., 1.5
        state.flight_mode = 3
        self.states.publish(state)
        grid = OccupancyGrid()
        grid.header = state.header
        grid.info.width = grid.info.height = 50
        grid.info.resolution = .2
        grid.info.origin.position.x = grid.info.origin.position.y = -5.
        grid.info.origin.orientation.w = 1.
        grid.data = [0] * 2500
        for row in range(10, 40):
            grid.data[row * 50 + 25] = 100
        if self.invalid:
            grid.header.stamp.sec = grid.header.stamp.nanosec = 0
            grid.data = [-1] * 2500
        self.maps.publish(grid)

    def spin_until(self, predicate, timeout=10):
        """Bound every wait and include subprocess diagnostics on failure."""
        until = time.monotonic() + timeout
        while not predicate() and time.monotonic() < until:
            rclpy.spin_once(self.node, timeout_sec=.01)
        self.log.flush()
        self.log.seek(0)
        self.assertTrue(predicate(), self.log.read())

    def wait(self, future):
        """Resolve an action future while servicing the mock server."""
        self.spin_until(future.done)
        return future.result()

    def spin_for(self, duration):
        """Allow delayed child sends to become observable."""
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=.01)

    def start(self, parameters=()):
        """Start the installed real node in this test's nonzero DDS domain."""
        executable = Path(get_package_prefix('easyfly_global_planner')) / (
            'lib/easyfly_global_planner/global_planner_node')
        self.process = subprocess.Popen([str(executable), '--ros-args', *parameters],
                                        stdout=self.log, stderr=self.log,
                                        start_new_session=True)
        self.assertTrue(self.client.wait_for_server(timeout_sec=10))
        self.spin_for(.4)

    def stop(self):
        """Require clean shutdown instead of counting forced cleanup as success."""
        if self.process is not None:
            process, self.process = self.process, None
            os.killpg(process.pid, signal.SIGINT)
            try:
                self.assertEqual(process.wait(timeout=8), 0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
                self.fail('owned planner did not shut down cleanly')
            self.spin_for(.2)

    def goal(self):
        """Choose a blocked straight line that requires multiple path segments."""
        handle = self.wait(self.client.send_goal_async(
            NavigateTo.Goal(target_x=3., target_y=0., target_z=1.5)))
        self.assertTrue(handle.accepted)
        return handle

    def test_zero_expansions_and_expired_deadline_send_no_child(self):
        """Real node forwards both requested limits into the search core."""
        for parameters, message in (
                (['-p', 'planning_max_expansions:=0'], 'planning expansion budget exhausted'),
                (['-p', 'planning_timeout:=0.000000001'], 'planning time budget exhausted')):
            self.start(parameters)
            handle = self.goal()
            result = self.wait(handle.get_result_async())
            self.assertEqual(result.status, 6)
            self.assertFalse(result.result.success)
            self.assertEqual(result.result.message, message)
            self.spin_for(.2)
            self.assertEqual(self.calls, [])
            self.stop()

    def test_invalid_map_and_cancel_stop_before_next_segment(self):
        """Drain the active child and prevent every later waypoint send."""
        self.start()
        handle = self.goal()
        self.spin_until(lambda: len(self.calls) == 1)
        self.invalid = True
        result = self.wait(handle.get_result_async())
        self.assertEqual(result.status, 6)
        self.assertFalse(result.result.success)
        self.assertIn('occupancy grid invalidated', result.result.message)
        self.assertGreaterEqual(self.cancels, 1)
        self.spin_for(.3)
        self.assertEqual(len(self.calls), 1)
        self.invalid = False
        self.spin_for(.3)
        handle = self.goal()
        self.spin_until(lambda: len(self.calls) == 2)
        self.assertTrue(self.wait(handle.cancel_goal_async()).goals_canceling)
        result = self.wait(handle.get_result_async())
        self.assertEqual(result.status, 5)
        self.assertFalse(result.result.success)
        self.assertGreaterEqual(self.cancels, 2)
        self.spin_for(.3)
        self.assertEqual(len(self.calls), 2)
        self.stop()

    def tearDown(self):
        """Clean up only processes owned by this fixture."""
        try:
            self.stop()
        finally:
            self.client.destroy()
            self.server.destroy()
            self.node.destroy_node()
            rclpy.shutdown()
            self.log.close()

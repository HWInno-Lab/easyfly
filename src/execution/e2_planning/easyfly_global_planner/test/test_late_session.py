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

"""Late START is still owned after public abort; STOP must follow its response."""
import os
import signal
import subprocess
import time
import unittest

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from easyfly_msgs.action import FollowTrajectory, NavigateTo
from easyfly_msgs.msg import UAVState
from easyfly_msgs.srv import SetNavigationSession


class LateSessionTest(unittest.TestCase):
    def test_late_start_fault_latch(self):
        rclpy.init()
        node = rclpy.create_node("late_session_mock")
        events = []
        group = ReentrantCallbackGroup()

        async def execute(handle):
            ready = rclpy.task.Future()
            timer = node.create_timer(
                0.02,
                lambda: ready.set_result(True)
                if handle.is_cancel_requested and not ready.done()
                else None,
            )
            await ready
            node.destroy_timer(timer)
            handle.canceled()
            events.append("child_terminal")
            return FollowTrajectory.Result(success=False)

        async def session(request, response):
            if request.active:
                events.append("start_request")
                ready = rclpy.task.Future()
                timer = node.create_timer(
                    0.7, lambda: ready.set_result(True) if not ready.done() else None
                )
                await ready
                node.destroy_timer(timer)
                events.append("start_response")
            else:
                events.append("stop")
            response.success = True
            return response

        flight = ActionServer(
            node,
            FollowTrajectory,
            "/follow_trajectory",
            execute,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group,
        )
        service = node.create_service(
            SetNavigationSession, "/planning/navigation_session", session, callback_group=group
        )
        publisher = node.create_publisher(UAVState, "/uav_state", 10)

        def publish():
            msg = UAVState()
            msg.header.frame_id = "odom"
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.connected = msg.armed = msg.pose_valid = msg.velocity_valid = True
            msg.flight_mode, msg.position_z = 3, 1.0
            publisher.publish(msg)

        timer = node.create_timer(0.05, publish)
        process = subprocess.Popen(
            [
                "ros2",
                "run",
                "easyfly_global_planner",
                "navigation_coordinator_node",
                "--ros-args",
                "-p",
                "strategy:=ego",
                "-p",
                "response_timeout:=0.2",
                "-p",
                "cancel_timeout:=0.2",
            ],
            start_new_session=True,
        )
        client = ActionClient(node, NavigateTo, "/navigate")

        def wait(future):
            deadline = time.monotonic() + 5
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
            self.assertTrue(future.done(), events)
            return future.result()

        try:
            self.assertTrue(client.wait_for_server(timeout_sec=10))
            goal = NavigateTo.Goal(target_x=2.0, target_z=1.0)
            handle = wait(client.send_goal_async(goal))
            self.assertTrue(handle.accepted)
            result = wait(handle.get_result_async())
            self.assertEqual(result.status, 6)
            self.assertIn("child_terminal", events)
            self.assertNotIn("stop", events)
            self.assertFalse(wait(client.send_goal_async(goal)).accepted)
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
            self.assertLess(events.index("start_response"), events.index("stop"))
            next_handle = wait(client.send_goal_async(goal))
            self.assertTrue(next_handle.accepted)
            wait(next_handle.cancel_goal_async())
            wait(next_handle.get_result_async())
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            flight.destroy()
            node.destroy_service(service)
            node.destroy_timer(timer)
            client.destroy()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()

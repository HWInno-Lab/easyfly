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

"""Real ROS action proxy checks without a simulator or flight controller."""
import os
import math
import signal
import subprocess
import time
import unittest

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from easyfly_msgs.action import NavigateTo
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from easyfly_msgs.srv import GetActionOwnership


class CoordinatorTest(unittest.TestCase):
    def test_proxy(self):
        rclpy.init()
        node = rclpy.create_node("coordinator_mock")
        feedback_seen = []
        backend_goals = []

        async def execute(handle):
            backend_goals.append(handle.request)
            feedback = NavigateTo.Feedback()
            feedback.current_segment = 2
            handle.publish_feedback(feedback)
            if handle.request.target_x >= 3.0:
                ready = rclpy.task.Future()
                delay = 0.1 if handle.request.target_x == 5.0 else 0.6
                timer = node.create_timer(
                    delay, lambda: ready.set_result(True) if not ready.done() else None
                )
                await ready
                node.destroy_timer(timer)
            result = NavigateTo.Result()
            result.success = handle.request.target_x != 2.0
            result.message = "mock"
            if handle.is_cancel_requested:
                result.success = False
                handle.canceled()
            else:
                handle.succeed()
            return result

        server = ActionServer(
            node,
            NavigateTo,
            "/navigate_astar",
            execute,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        process = subprocess.Popen(
            [
                "ros2",
                "run",
                "easyfly_global_planner",
                "navigation_coordinator_node",
                "--ros-args",
                "-p",
                "cancel_timeout:=0.2",
            ],
            start_new_session=True,
        )
        client = ActionClient(node, NavigateTo, "/navigate")
        ownership = node.create_client(GetActionOwnership, "/navigation/ownership")

        def wait(future):
            deadline = time.monotonic() + 10
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertTrue(future.done())
            return future.result()

        try:
            self.assertTrue(client.wait_for_server(timeout_sec=10))
            self.assertTrue(ownership.wait_for_service(timeout_sec=5))
            self.assertEqual(
                wait(ownership.call_async(GetActionOwnership.Request(goal_id="unknown"))).state, 0
            )
            for target, expected_status in [(1.0, 4), (2.0, 6)]:
                goal = NavigateTo.Goal()
                goal.target_x = target
                goal.target_z = 1.0
                handle = wait(
                    client.send_goal_async(
                        goal,
                        feedback_callback=lambda f: feedback_seen.append(
                            f.feedback.current_segment
                        ),
                    )
                )
                self.assertTrue(handle.accepted)
                result = wait(handle.get_result_async())
                self.assertEqual(result.status, expected_status)
                self.assertEqual(result.result.success, target == 1.0)
            self.assertIn(2, feedback_seen)
            goal = NavigateTo.Goal()
            goal.target_x = 3.0
            goal.target_z = 1.0
            handle = wait(client.send_goal_async(goal))
            self.assertTrue(handle.accepted)
            rejected = wait(client.send_goal_async(goal))
            self.assertFalse(rejected.accepted)
            wait(handle.cancel_goal_async())
            result = wait(handle.get_result_async())
            self.assertEqual(result.status, 6)  # public abort before child terminal
            identity = bytes(handle.goal_id.uuid).hex()
            self.assertEqual(
                wait(ownership.call_async(GetActionOwnership.Request(goal_id=identity))).state, 1
            )
            rejected = wait(client.send_goal_async(goal))
            self.assertFalse(rejected.accepted)  # fault retains child ownership
            deadline = time.monotonic() + 0.8
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertEqual(
                wait(ownership.call_async(GetActionOwnership.Request(goal_id=identity))).state, 2
            )
            goal.target_x = 1.0
            handle = wait(client.send_goal_async(goal))
            self.assertTrue(handle.accepted)  # true child result releases the slot
            self.assertEqual(wait(handle.get_result_async()).status, 4)
            goal.target_x = 5.0
            handle = wait(client.send_goal_async(goal))
            self.assertTrue(handle.accepted)
            wait(handle.cancel_goal_async())
            result = wait(handle.get_result_async())
            self.assertEqual(result.status, 5)
            self.assertFalse(result.result.success)
            stop = node.create_client(Trigger, "/navigate/_stop")
            self.assertTrue(stop.wait_for_service(timeout_sec=3))
            self.assertFalse(wait(stop.call_async(Trigger.Request())).success)
            goal.target_x = 3.0
            handle = wait(client.send_goal_async(goal))
            self.assertTrue(handle.accepted)
            self.assertTrue(wait(stop.call_async(Trigger.Request())).success)
            result = wait(handle.get_result_async())
            self.assertEqual(result.status, 6)
            self.assertEqual(result.result.message, "supervisor_stop")
            deadline = time.monotonic() + 0.8
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            node.destroy_client(stop)
            # RViz cannot bypass admission; z=0 preserves altitude via NaN.
            rviz = node.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
            deadline = time.monotonic() + 2
            while not rviz.get_subscription_count() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.header.stamp.sec = -1
            pose.pose.position.x = 8.0
            rviz.publish(pose)  # malformed time is rejected without crashing
            deadline = time.monotonic() + 0.2
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertFalse(any(g.target_x == 8.0 for g in backend_goals))
            pose.header.stamp = node.get_clock().now().to_msg()
            rviz.publish(pose)
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            forwarded = [g for g in backend_goals if g.target_x == 8.0]
            self.assertEqual(len(forwarded), 1)
            self.assertTrue(math.isnan(forwarded[0].target_z))
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            server.destroy()
            client.destroy()
            node.destroy_client(ownership)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()

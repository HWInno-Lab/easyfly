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

"""Real ROS constraints/stop service checks; no setpoint producer or simulator."""
import os
import signal
import subprocess
import time
import unittest

import rclpy
from easyfly_msgs.msg import ObstacleInfo, ObstacleInfoArray, UAVState
from easyfly_msgs.srv import GetObstacles, StopNavigation
from std_msgs.msg import String
from std_srvs.srv import Trigger


class SupervisorTest(unittest.TestCase):
    def test_auto_stop_retries_transient_failure(self):
        rclpy.init()
        node = rclpy.create_node("supervisor_retry_mock")
        calls = []

        def stop(request, response):
            calls.append(request.navigation_goal_id)
            response.success = len(calls) >= 2
            return response

        server = node.create_service(StopNavigation, "/navigate/_stop_owned", stop)
        publisher = node.create_publisher(String, "/navigation/goal_id", 10)
        timer = node.create_timer(0.05, lambda: publisher.publish(String(data="b" * 32)))
        process = subprocess.Popen(
            [
                "ros2",
                "run",
                "easyfly_obstacle_avoidance",
                "obstacle_supervisor_node",
                "--ros-args",
                "-p",
                "auto_stop:=true",
            ],
            start_new_session=True,
        )
        try:
            until = time.monotonic() + 5
            while len(calls) < 2 and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertEqual(calls, ["b" * 32, "b" * 32])
            until = time.monotonic() + 1.3
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertEqual(len(calls), 2)  # Success latches, failures do not.
        finally:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5)
            node.destroy_timer(timer)
            node.destroy_service(server)
            node.destroy_node()
            rclpy.shutdown()

    def test_constraints_and_scoped_stop(self):
        rclpy.init()
        node = rclpy.create_node("supervisor_mock")
        calls = []

        def stop(request, response):
            calls.append(request.navigation_goal_id)
            response.success = request.navigation_goal_id == "a" * 32
            response.message = "stop requested"
            return response

        server = node.create_service(StopNavigation, "/navigate/_stop_owned", stop)
        state_pub = node.create_publisher(UAVState, "/uav_state", 10)
        obstacles_pub = node.create_publisher(ObstacleInfoArray, "/obstacles", 10)
        goal_pub = node.create_publisher(String, "/navigation/goal_id", 10)
        valid = True
        empty = False

        def publish():
            stamp = node.get_clock().now().to_msg()
            state = UAVState()
            state.header.frame_id = "odom"
            state.header.stamp = stamp
            state.connected = state.pose_valid = True
            state.position_z = 1.0
            state_pub.publish(state)
            msg = ObstacleInfoArray()
            msg.header.frame_id = "odom"
            if valid:
                msg.header.stamp = stamp
            if not empty:
                o = ObstacleInfo()
                o.header = msg.header
                o.center_x, o.center_z = 1.0, 1.0
                o.size_x = o.size_y = o.size_z = 0.4
                msg.obstacles = [o]
            obstacles_pub.publish(msg)
            goal = String()
            goal.data = "a" * 32
            goal_pub.publish(goal)

        timer = node.create_timer(0.05, publish)
        process = subprocess.Popen(
            ["ros2", "run", "easyfly_obstacle_avoidance", "obstacle_supervisor_node"],
            start_new_session=True,
        )
        query = node.create_client(GetObstacles, "/avoidance/get_near_obstacles")
        stop_client = node.create_client(Trigger, "/avoidance/stop_navigation")

        def spin(duration):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)

        def wait(future):
            end = time.monotonic() + 5
            while not future.done() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertTrue(future.done())
            return future.result()

        try:
            self.assertTrue(query.wait_for_service(timeout_sec=10))
            self.assertTrue(stop_client.wait_for_service(timeout_sec=3))
            spin(0.3)
            request = GetObstacles.Request()
            request.range_limit = 0.6  # center-to-AABB minus UAV radius = 0.5
            response = wait(query.call_async(request))
            self.assertEqual(len(response.obstacles.obstacles), 1)
            request.range_limit = 0.4
            self.assertEqual(len(wait(query.call_async(request)).obstacles.obstacles), 0)
            empty = True
            spin(0.2)
            self.assertGreater(wait(query.call_async(request)).obstacles.header.stamp.sec, 0)
            valid = False
            spin(0.2)
            response = wait(query.call_async(request))
            self.assertEqual(response.obstacles.header.stamp.sec, 0)
            self.assertEqual(response.obstacles.header.stamp.nanosec, 0)
            self.assertTrue(wait(stop_client.call_async(Trigger.Request())).success)
            self.assertEqual(calls, ["a" * 32])
            request.range_limit = float("nan")
            self.assertEqual(wait(query.call_async(request)).obstacles.header.stamp.sec, 0)
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            node.destroy_timer(timer)
            node.destroy_service(server)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()

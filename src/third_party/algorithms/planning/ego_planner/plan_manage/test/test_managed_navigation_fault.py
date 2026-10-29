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

"""Actual EGO regression for managed preset configuration and emergency faults."""
import os
import signal
import subprocess
import tempfile
import time
import unittest

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from easyfly_msgs.msg import NavigationSession
from easyfly_msgs.srv import SetNavigationSession
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header


class ManagedNavigationFaultTest(unittest.TestCase):
    def test_preset_configuration_uses_managed_goal_and_emergency_is_reported(self):
        os.environ["ROS_DOMAIN_ID"] = "183"
        rclpy.init()
        node = rclpy.create_node("managed_navigation_fault_regression")
        logfile = tempfile.TemporaryFile(mode="w+")
        process = subprocess.Popen(
            [
                "ros2",
                "launch",
                "easyfly_local_planner",
                "local_planner.launch.py",
                "use_sim_time:=false",
                "flight_type:=2",
            ],
            stdout=logfile,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        odom_pub = node.create_publisher(Odometry, "/odom", 10)
        cloud_pub = node.create_publisher(PointCloud2, "/pointcloud_nonground", 10)
        commands, faults = [], []
        _ = [
            node.create_subscription(PositionCommand, "/position_cmd", commands.append, 100),
            node.create_subscription(
                NavigationSession,
                "/planning/navigation_fault",
                faults.append,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            ),
        ]
        client = node.create_client(SetNavigationSession, "/planning/navigation_session")
        publish_cloud = True

        def inputs():
            header = Header(frame_id="odom", stamp=node.get_clock().now().to_msg())
            odom = Odometry()
            odom.header = header
            odom.child_frame_id = "base_link"
            odom.pose.pose.orientation.w = 1.0
            odom.pose.pose.position.z = 1.0
            odom_pub.publish(odom)
            if publish_cloud:
                cloud_pub.publish(create_cloud_xyz32(header, [(0.0, 5.0, 1.0)]))

        node.create_timer(0.02, inputs)

        def until(predicate, timeout):
            end = time.monotonic() + timeout
            while not predicate() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertTrue(predicate())

        def call(active):
            request = SetNavigationSession.Request()
            request.session_id, request.active = "preset-regression", active
            request.target_x, request.target_z, request.max_speed = 3.0, 1.0, 2.0
            future = client.call_async(request)
            until(future.done, 10.0)
            return future.result()

        try:
            until(client.service_is_ready, 15.0)
            end = time.monotonic() + 2.0
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)
            response = call(True)
            self.assertTrue(response.success, response.message)
            until(lambda: bool(commands), 10.0)
            # Exercise actual EXEC_TRAJ with flight_type=2, not just init.
            end = time.monotonic() + 0.3
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertIsNone(process.poll())
            self.assertEqual(commands[-1].navigation_session_id, "preset-regression")
            publish_cloud = False
            until(lambda: bool(faults), 5.0)
            self.assertEqual(faults[-1].session_id, "preset-regression")
            self.assertFalse(faults[-1].active)
            response = call(False)
            self.assertTrue(response.success, response.message)
            logfile.seek(0)
            self.assertIn("managed navigation overrides legacy flight_type=2", logfile.read())
        finally:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            logfile.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()

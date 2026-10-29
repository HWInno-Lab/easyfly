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

"""Actual ROS trajectory output: held measured yaw, stale odometry and session reset."""
import math
import os
import signal
import subprocess
import time
import unittest

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.qos import QoSProfile, DurabilityPolicy
from easyfly_msgs.msg import NavigationSession
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from rosgraph_msgs.msg import Clock
from traj_utils.msg import Bspline


class HeldNavigationYawTest(unittest.TestCase):
    def test_held_yaw_rejects_stale_odometry_and_resets_only_for_new_session(self):
        os.environ["ROS_DOMAIN_ID"] = "228"
        rclpy.init()
        node = rclpy.create_node("held_yaw_contract_fixture")
        executable = os.path.join(get_package_prefix("ego_planner"), "lib/ego_planner/traj_server")
        process = subprocess.Popen(
            [
                executable,
                "--ros-args",
                "-p",
                "use_sim_time:=true",
                "-p",
                "navigation_managed:=true",
                "-p",
                "hold_navigation_yaw:=true",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        commands = []
        node.create_subscription(PositionCommand, "/position_cmd", commands.append, 100)
        lifecycle = node.create_publisher(
            NavigationSession,
            "/planning/navigation_session_state",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        splines = node.create_publisher(Bspline, "/planning/bspline", 10)
        odometry = node.create_publisher(Odometry, "/odom", 10)
        clock = node.create_publisher(Clock, "/clock", 10)
        measured_yaw, publish_odometry = [0.7], [True]

        def publish():
            stamp = node.get_clock().now().to_msg()
            clock.publish(Clock(clock=stamp))
            if publish_odometry[0]:
                odom = Odometry(child_frame_id="base_link")
                odom.header.stamp, odom.header.frame_id = stamp, "odom"
                odom.pose.pose.position.z = 1.0
                odom.pose.pose.orientation.z = math.sin(measured_yaw[0] / 2)
                odom.pose.pose.orientation.w = math.cos(measured_yaw[0] / 2)
                odometry.publish(odom)

        timer = node.create_timer(0.02, publish)

        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.01)

        def trajectory(session, identifier):
            value = Bspline(navigation_session_id=session, order=3, traj_id=identifier)
            value.start_time = node.get_clock().now().to_msg()
            value.knots = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
            value.pos_pts = [Point(x=float(3 - i), z=1.0) for i in range(4)]
            splines.publish(value)

        try:
            spin(1.5)
            lifecycle.publish(NavigationSession(session_id="first", active=True))
            spin(0.1)
            trajectory("first", 1)
            spin(0.2)
            self.assertTrue(commands)
            self.assertTrue(all(abs(c.yaw - 0.7) < 1e-6 for c in commands))
            self.assertTrue(all(abs(c.yaw_dot) < 1e-6 for c in commands))
            measured_yaw[0] = -1.0
            spin(0.1)
            commands.clear()
            trajectory("first", 2)
            spin(0.2)
            self.assertTrue(commands)
            self.assertTrue(all(abs(c.yaw - 0.7) < 1e-6 for c in commands))
            publish_odometry[0] = False
            spin(0.7)
            commands.clear()
            trajectory("first", 3)
            spin(0.2)
            self.assertFalse(commands)
            lifecycle.publish(NavigationSession(session_id="first", active=False))
            publish_odometry[0] = True
            spin(0.1)
            lifecycle.publish(NavigationSession(session_id="second", active=True))
            spin(0.1)
            trajectory("second", 4)
            spin(0.2)
            self.assertTrue(commands)
            self.assertTrue(all(abs(c.yaw + 1.0) < 1e-6 for c in commands))
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
                self.fail("trajectory server required forced shutdown")
            node.destroy_timer(timer)
            node.destroy_node()
            rclpy.shutdown()
        self.assertEqual(process.returncode, 0)


if __name__ == "__main__":
    unittest.main()

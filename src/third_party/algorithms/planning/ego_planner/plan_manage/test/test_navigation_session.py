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

"""Real ROS ingress test: stopped/foreign trajectories cannot revive a session."""
import os
import subprocess
import time
import unittest

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.qos import QoSProfile, DurabilityPolicy
from easyfly_msgs.msg import NavigationSession
from geometry_msgs.msg import Point
from quadrotor_msgs.msg import PositionCommand
from traj_utils.msg import Bspline, TrajectoryAck


class NavigationSessionTest(unittest.TestCase):
    def test_session_activation_stop_and_foreign_traffic(self):
        os.environ["ROS_DOMAIN_ID"] = "182"
        rclpy.init()
        node = rclpy.create_node("navigation_session_regression")
        process = subprocess.Popen(
            [
                os.path.join(
                    get_package_prefix("ego_planner"), "lib", "ego_planner", "traj_server"
                ),
                "--ros-args",
                "-p",
                "navigation_managed:=true",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        commands, acknowledgments = [], []
        _ = [
            node.create_subscription(PositionCommand, "/position_cmd", commands.append, 50),
            node.create_subscription(
                TrajectoryAck,
                "/trajectory_ack",
                acknowledgments.append,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            ),
        ]
        lifecycle = node.create_publisher(
            NavigationSession,
            "/planning/navigation_session_state",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        splines = node.create_publisher(Bspline, "/planning/bspline", 10)

        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)

        def trajectory(session, identifier):
            candidate = Bspline()
            candidate.navigation_session_id = session
            candidate.order = 3
            candidate.traj_id = identifier
            candidate.start_time = node.get_clock().now().to_msg()
            candidate.knots = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
            candidate.pos_pts = [Point(x=float(i), z=1.0) for i in range(4)]
            splines.publish(candidate)

        try:
            spin(2.0)
            lifecycle.publish(NavigationSession(session_id="first", active=True))
            spin(0.2)
            trajectory("first", 1)
            spin(0.3)
            self.assertTrue(commands)
            self.assertEqual(commands[-1].navigation_session_id, "first")
            self.assertEqual(acknowledgments[-1].navigation_session_id, "first")
            lifecycle.publish(NavigationSession(session_id="first", active=False))
            spin(0.2)
            commands.clear()
            trajectory("first", 2)
            spin(0.3)
            self.assertFalse(commands)
            lifecycle.publish(NavigationSession(session_id="second", active=True))
            spin(0.2)
            trajectory("second", 3)
            spin(0.2)
            self.assertEqual(commands[-1].navigation_session_id, "second")
            commands.clear()
            lifecycle.publish(NavigationSession(session_id="first", active=False))
            trajectory("first", 4)
            trajectory("x" * 129, 5)
            spin(0.3)
            self.assertTrue(commands)
            self.assertTrue(all(c.navigation_session_id == "second" for c in commands))
            lifecycle.publish(NavigationSession(session_id="third", active=True))
            spin(0.2)
            commands.clear()
            trajectory("second", 6)
            spin(0.2)
            self.assertFalse(commands)
            trajectory("third", 7)
            spin(0.2)
            self.assertTrue(commands)
            self.assertTrue(all(c.navigation_session_id == "third" for c in commands))
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()

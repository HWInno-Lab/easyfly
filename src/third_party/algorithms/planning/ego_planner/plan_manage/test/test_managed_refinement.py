#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Exercise real managed EGO refinement with valid synthetic ROS inputs, not flight."""

import math
import os
import signal
import subprocess
import tempfile
import time
import unittest

from easyfly_msgs.msg import NavigationSession
from easyfly_msgs.srv import SetNavigationSession
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from traj_utils.msg import Bspline


class ManagedRefinementTest(unittest.TestCase):
    """Validate strict final bounds after real optimization and time refinement."""

    def test_climbing_start_produces_bounded_trajectory(self):
        """A short diagonal path must not repeatedly reject successful refinement."""
        self.run_case()

    def test_occupied_start_still_fails_closed(self):
        """An obstacle covering the seed is not cleared to make refinement succeed."""
        self.run_case(blocked=True)

    def test_excessive_boundary_speed_is_not_scaled_away(self):
        """Keep the measured boundary derivative rather than fabricating a slow start."""
        self.run_case(overspeed=True)

    def test_short_exact_endpoint_produces_bounded_trajectory(self):
        """A 14 cm final approach still needs to reach the exact action target."""
        self.run_case(short=True)

    def test_short_occupied_seed_still_fails_closed(self):
        """Short motion keeps the original collision rejection."""
        self.run_case(short=True, blocked=True)

    def run_case(self, blocked=False, overspeed=False, short=False):
        """Run an isolated real planner with valid component-owned input publishers."""
        os.environ["ROS_DOMAIN_ID"] = "185"
        rclpy.init()
        node = rclpy.create_node("managed_refinement_regression")
        logfile = tempfile.TemporaryFile(mode="w+")
        process = subprocess.Popen(
            [
                "ros2",
                "launch",
                "easyfly_local_planner",
                "local_planner.launch.py",
                "use_sim_time:=false",
            ],
            stdout=logfile,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        odom_pub = node.create_publisher(Odometry, "/odom", 10)
        cloud_pub = node.create_publisher(PointCloud2, "/pointcloud_nonground", 10)
        trajectories, faults = [], []
        subscriptions = [
            node.create_subscription(Bspline, "/planning/bspline", trajectories.append, 10),
            node.create_subscription(
                NavigationSession,
                "/planning/navigation_fault",
                faults.append,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            ),
        ]
        client = node.create_client(SetNavigationSession, "/planning/navigation_session")

        def inputs():
            header = Header(frame_id="odom", stamp=node.get_clock().now().to_msg())
            odom = Odometry(header=header, child_frame_id="base_link")
            odom.pose.pose.orientation.w = 1.0
            # Nearest stored UAVState to the original first planning attempt.
            odom.pose.pose.position.x = -0.03239358589053154
            odom.pose.pose.position.y = -0.02704579196870327
            odom.pose.pose.position.z = 1.325653076171875
            odom.twist.twist.linear.x = 0.8 if overspeed else -0.033138200640678406
            odom.twist.twist.linear.y = 0.08463609963655472
            odom.twist.twist.linear.z = 0.07423700392246246
            if short:
                odom.twist.twist.linear.x = 0.0
                odom.twist.twist.linear.y = 0.0
                odom.twist.twist.linear.z = 0.0
            odom_pub.publish(odom)
            obstacle = (0.0, 0.0, 1.325) if blocked else (5.0, 5.0, 1.5)
            cloud_pub.publish(create_cloud_xyz32(header, [obstacle]))

        timer = node.create_timer(0.02, inputs)

        def until(predicate, seconds):
            deadline = time.monotonic() + seconds
            while not predicate() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertTrue(predicate())

        try:
            until(client.service_is_ready, 15.0)
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            request = SetNavigationSession.Request(
                session_id="climbing-refinement",
                active=True,
                target_x=0.5,
                target_y=-0.55,
                target_z=1.5,
                max_speed=0.6,
            )
            if short:
                request.target_x = -0.03239358589053154 + 0.1
                request.target_y = -0.02704579196870327 + 0.1
                request.target_z = 1.325653076171875
            future = client.call_async(request)
            until(future.done, 5.0)
            self.assertTrue(future.result().success, future.result().message)
            until(
                lambda: bool(faults)
                if blocked or overspeed
                else bool(trajectories) or bool(faults),
                5.0,
            )
            if blocked or overspeed:
                self.assertTrue(faults)
                for trajectory in trajectories:
                    points = [(p.x, p.y, p.z) for p in trajectory.pos_pts]
                    self.assertTrue(points)
                    self.assertTrue(
                        all(p == points[0] for p in points),
                        "rejected seed published a moving trajectory",
                    )
                return
            self.assertFalse(faults, "valid unobstructed seed generated a planner fault")
            self.assertTrue(trajectories)
            trajectory = trajectories[0]
            self.assertEqual(trajectory.navigation_session_id, request.session_id)
            self.assertEqual(trajectory.order, 3)
            points = [(p.x, p.y, p.z) for p in trajectory.pos_pts]
            self.assertGreaterEqual(len(points), 6)
            self.assertTrue(all(math.isfinite(v) for p in points for v in p))
            bounds = []
            for i in range(len(points) - 1):
                interval = trajectory.knots[i + 4] - trajectory.knots[i + 1]
                self.assertGreater(interval, 0.0)
                bounds.append(3 * math.dist(points[i + 1], points[i]) / interval)
            self.assertLessEqual(max(bounds), request.max_speed + 1e-9)
            duration = trajectory.knots[len(points)] - trajectory.knots[3]
            self.assertGreater(duration, 0.0)
            self.assertLessEqual(duration, 300.0)
            endpoint = tuple(
                (points[-3][a] + 4 * points[-2][a] + points[-1][a]) / 6 for a in range(3)
            )
            self.assertLess(
                math.dist(endpoint, [request.target_x, request.target_y, request.target_z]), 1e-9
            )
            print("strict final speed bound:", max(bounds), flush=True)
            stop = SetNavigationSession.Request(session_id=request.session_id, active=False)
            future = client.call_async(stop)
            until(future.done, 5.0)
            self.assertTrue(future.result().success)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
            logfile.seek(0)
            print(logfile.read(), flush=True)
            logfile.close()
            node.destroy_timer(timer)
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()

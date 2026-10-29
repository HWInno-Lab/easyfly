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

"""
Short software plant with REAL coordinator/EGO/traj_server/flight_control.

Synthetic odom/cloud/grid bypass perception and PX4. This is not SITL or flight
certification; it verifies the new session chain and bounded obstacle routing.
"""
import math
import os
import signal
import subprocess
import tempfile
import time
import unittest
from types import SimpleNamespace

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from easyfly_msgs.action import NavigateTo, Takeoff
from easyfly_msgs.msg import NavigationSession, UAVCommand, UAVState
from easyfly_msgs.srv import ArmMotors, SetFlightMode
from nav_msgs.msg import OccupancyGrid, Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from traj_utils.msg import TrajectoryAck


class NavigationChainTest(unittest.TestCase):
    def run_chain(self, dynamic, sdk_mode=False):
        rclpy.init()
        node = rclpy.create_node("navigation_software_plant")
        processes, logs = [], []
        position, velocity, target = [0.0, 0.0, 0.5], [0.0] * 3, [0.0, 0.0, 0.5]
        history, sessions, acks, commands, trajectories = [], [], [], [], []
        obstacle = not dynamic
        map_warning = not dynamic
        cloud_enabled = True
        flight_mode = 3
        armed = False
        command_time = time.monotonic()

        def command(msg):
            nonlocal command_time
            target[:] = [msg.position_x, msg.position_y, msg.position_z]
            command_time = time.monotonic()
            commands.append((command_time, msg.control_mode))

        state_pub = node.create_publisher(UAVState, "/uav_state", 10)
        odom_pub = node.create_publisher(Odometry, "/odom", 10)
        cloud_pub = node.create_publisher(PointCloud2, "/pointcloud_nonground", 10)
        map_pub = node.create_publisher(OccupancyGrid, "/occupancy_grid", 10)
        _ = [
            node.create_subscription(UAVCommand, "/uav_command", command, 20),
            node.create_subscription(
                NavigationSession,
                "/planning/navigation_session_state",
                sessions.append,
                QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            ),
            node.create_subscription(
                TrajectoryAck,
                "/trajectory_ack",
                acks.append,
                QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            ),
            node.create_subscription(PositionCommand, "/position_cmd", trajectories.append, 100),
        ]

        def arm_motors(request, response):
            nonlocal armed
            armed = request.arm
            response.success = True
            return response

        def set_mode(request, response):
            nonlocal flight_mode
            flight_mode = request.flight_mode
            response.success = True
            return response

        node.create_service(ArmMotors, "/arm_motors", arm_motors)
        node.create_service(SetFlightMode, "/set_flight_mode", set_mode)

        def plant():
            if time.monotonic() - command_time < 0.3:
                for i in range(3):
                    delta = max(-0.04, min(0.04, (target[i] - position[i]) * 0.35))
                    position[i] += delta
                    velocity[i] = delta / 0.02
            else:
                velocity[:] = [0.0] * 3
            header = Header(frame_id="odom", stamp=node.get_clock().now().to_msg())
            state = UAVState()
            state.header = header
            state.connected = state.pose_valid = state.velocity_valid = state.attitude_valid = True
            state.armed = armed
            state.flight_mode = flight_mode
            state.position_x, state.position_y, state.position_z = position
            state.velocity_x, state.velocity_y, state.velocity_z = velocity
            state_pub.publish(state)
            odom = Odometry()
            odom.header, odom.child_frame_id = header, "base_link"
            odom.pose.pose.orientation.w = 1.0
            (
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                odom.pose.pose.position.z,
            ) = position
            (
                odom.twist.twist.linear.x,
                odom.twist.twist.linear.y,
                odom.twist.twist.linear.z,
            ) = velocity
            odom_pub.publish(odom)
            history.append(tuple(position))

        def perception():
            header = Header(frame_id="odom", stamp=node.get_clock().now().to_msg())
            points = [
                (0.0, 5.0, 1.0)
            ]  # A nonempty trusted distant return, not an empty-cloud bypass.
            if obstacle:
                for height in range(6, 31, 6):
                    for angle in range(0, 360, 45):
                        points.append(
                            (
                                2.5 + 0.25 * math.cos(math.radians(angle)),
                                0.25 * math.sin(math.radians(angle)),
                                height / 10.0,
                            )
                        )
            cloud_pub.publish(create_cloud_xyz32(header, points if cloud_enabled else []))
            grid = OccupancyGrid()
            grid.header = header
            grid.info.width = grid.info.height = 80
            grid.info.resolution = 0.2
            grid.info.origin.position.x = grid.info.origin.position.y = -6.0
            grid.info.origin.orientation.w = 1.0
            grid.data = [0] * 6400
            if map_warning:
                for row in range(80):
                    for col in range(80):
                        if math.hypot(-6 + (col + 0.5) * 0.2 - 2.5, -6 + (row + 0.5) * 0.2) <= 1.3:
                            grid.data[row * 80 + col] = 100
            map_pub.publish(grid)

        plant_timer = node.create_timer(0.02, plant)
        perception_timer = node.create_timer(0.2, perception)

        def launch(argv):
            log = tempfile.TemporaryFile(mode="w+")
            logs.append(log)
            processes.append(
                subprocess.Popen(argv, stdout=log, stderr=log, start_new_session=True)
            )

        launch(
            [
                "ros2",
                "launch",
                "easyfly_local_planner",
                "local_planner.launch.py",
                "use_sim_time:=false",
                "virtual_ceil_height:=2.0",
            ]
        )
        launch(["ros2", "run", "easyfly_flight_control", "flight_control_node"])
        launch(
            [
                "ros2",
                "run",
                "easyfly_global_planner",
                "navigation_coordinator_node",
                "--ros-args",
                "-p",
                "strategy:=combined",
                "-p",
                "goal_timeout:=60.0",
            ]
        )
        takeoff = ActionClient(node, Takeoff, "/takeoff")
        navigate = ActionClient(node, NavigateTo, "/navigate")
        sdk_client = None
        scenario_finished = False

        def spin(seconds):
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.01)

        def wait(future, seconds=10):
            until = time.monotonic() + seconds
            while not future.done() and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.01)
                self.assertTrue(all(p.poll() is None for p in processes), "child process exited")
            self.assertTrue(future.done(), "future deadline expired")
            return future.result()

        def send(goal):
            if not sdk_mode:
                return wait(navigate.send_goal_async(goal))
            op = sdk_client.navigate(goal.target_x, goal.target_y, goal.target_z, goal.max_speed)
            until = time.monotonic() + 10
            while op.snapshot().admission == sdk.Admission.PENDING and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.01)
            return SimpleNamespace(
                accepted=op.snapshot().admission == sdk.Admission.ACCEPTED, operation=op
            )

        def result(handle, seconds=10):
            if not sdk_mode:
                return wait(handle.get_result_async(), seconds)
            until = time.monotonic() + seconds
            while (
                handle.operation.snapshot().remote_execution_pending and time.monotonic() < until
            ):
                rclpy.spin_once(node, timeout_sec=0.01)
            state = handle.operation.snapshot()
            self.assertFalse(state.remote_execution_pending, state.local_message)
            status = {sdk.Status.SUCCEEDED: 4, sdk.Status.CANCELED: 5, sdk.Status.FAILED: 6}.get(
                state.status
            )
            return SimpleNamespace(
                status=status, result=SimpleNamespace(success=state.success, message=state.message)
            )

        try:
            self.assertTrue(takeoff.wait_for_server(timeout_sec=15))
            self.assertTrue(navigate.wait_for_server(timeout_sec=10))
            if sdk_mode:
                import easyfly_pybind as sdk

                sdk_client = sdk.Client()
                until = time.monotonic() + 5
                while not sdk_client.server_ready("navigate") and time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=0.01)
                self.assertTrue(sdk_client.server_ready("navigate"))
            spin(2.0)
            th = wait(takeoff.send_goal_async(Takeoff.Goal(target_altitude=1.5, climb_speed=1.0)))
            self.assertTrue(th.accepted)
            self.assertTrue(wait(th.get_result_async(), 15).result.success)
            goal = NavigateTo.Goal(target_x=5.0, target_z=1.5, max_speed=1.0)
            handle = send(goal)
            self.assertTrue(handle.accepted)
            if dynamic:
                until = time.monotonic() + 10
                while position[0] < 0.25 and time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=0.01)
                self.assertGreater(position[0], 0.2)
                # A new obstacle appears sufficiently far ahead for replanning.
                # Occupancy and trusted cloud change together: no forecast input.
                previous = next(s.session_id for s in reversed(sessions) if s.active)
                map_warning = obstacle = True
                until = time.monotonic() + 5
                while (
                    not any(s.active and s.session_id != previous for s in sessions)
                    and time.monotonic() < until
                ):
                    rclpy.spin_once(node, timeout_sec=0.01)
                self.assertTrue(any(s.active and s.session_id != previous for s in sessions))
            completed = result(handle, 65)
            self.assertEqual(completed.status, 4, completed.result.message)
            self.assertTrue(completed.result.success)
            self.assertLess(math.hypot(position[0] - 5, position[1]), 0.15)
            self.assertGreater(max(abs(p[1]) for p in history), 0.8)
            self.assertGreater(min(math.hypot(p[0] - 2.5, p[1]) - 0.25 for p in history), 0.8)
            active_ids = [s.session_id for s in sessions if s.active]
            self.assertGreaterEqual(len(set(active_ids)), 2)
            self.assertTrue(acks)
            self.assertTrue(all(a.navigation_session_id in active_ids for a in acks))
            self.assertTrue(trajectories)
            self.assertLessEqual(
                max(
                    math.sqrt(c.velocity.x ** 2 + c.velocity.y ** 2 + c.velocity.z ** 2)
                    for c in trajectories
                ),
                1.001,
            )
            spin(0.4)
            # Cancel a real new goal and verify no further trajectory output.
            h2 = send(NavigateTo.Goal(target_x=5.0, target_y=2.0, target_z=1.5, max_speed=1.0))
            self.assertTrue(h2.accepted)
            spin(0.5)
            if sdk_mode:
                self.assertTrue(h2.operation.cancel())
            else:
                wait(h2.cancel_goal_async())
            canceled = result(h2)
            self.assertEqual(canceled.status, 5)
            spin(0.4)
            tail_start = time.monotonic()
            spin(0.4)
            self.assertFalse(
                any(t > tail_start and mode == UAVCommand.TRAJECTORY for t, mode in commands)
            )
            spin(0.4)
            h3 = send(NavigateTo.Goal(target_x=5.0, target_y=-2.0, target_z=1.5, max_speed=1.0))
            self.assertTrue(h3.accepted)
            started_at = len(trajectories)
            until = time.monotonic() + 5
            while len(trajectories) == started_at and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.01)
            self.assertGreater(len(trajectories), started_at)
            cloud_enabled = False  # EGO's production freshness/empty-cloud safety path.
            emergency = result(h3, 5)
            self.assertEqual(emergency.status, 6)
            self.assertEqual(emergency.result.message, "planner_emergency_stop")
            scenario_finished = True
        except Exception:
            for log in logs:
                log.seek(0)
                print(log.read()[-14000:])
            raise
        finally:
            sdk_drained = sdk_client.close(2) if sdk_client else True
            for process in processes:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
            for process in processes:
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
            for log in logs:
                log.close()
            node.destroy_timer(plant_timer)
            node.destroy_timer(perception_timer)
            takeoff.destroy()
            navigate.destroy()
            node.destroy_node()
            rclpy.shutdown()
            if scenario_finished:
                self.assertTrue(
                    sdk_drained, "SDK drain did not complete after observed real terminals"
                )

    def test_static_obstacle_real_chain(self):
        self.run_chain(False)

    def test_dynamic_obstacle_real_chain(self):
        self.run_chain(True)

    def test_cpp_sdk_python_real_chain(self):
        self.run_chain(False, sdk_mode=True)


if __name__ == "__main__":
    unittest.main()

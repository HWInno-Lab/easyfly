# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_perception_srvs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
#

"""Read-only queries distinguish absence, empty observations and invalid data."""
import copy
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from easyfly_msgs.msg import (
    Target3DArray,
    Target3D,
    ObstacleInfoArray,
    ObstacleInfo,
    UAVState,
    LandingTarget,
)
from rosgraph_msgs.msg import Clock
from easyfly_msgs.srv import GetPerception, GetCapabilities
from easyfly_perception_srvs.node import PerceptionServices


class ServiceTest(unittest.TestCase):
    def test_health_range_and_capabilities(self):
        rclpy.init()
        service = PerceptionServices()
        node = rclpy.create_node('perception_service_test')
        executor = SingleThreadedExecutor()
        executor.add_node(service)
        executor.add_node(node)
        targets_pub = node.create_publisher(
            Target3DArray, '/target_positions', 10
        )
        obstacles_pub = node.create_publisher(
            ObstacleInfoArray,
            '/obstacles',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        state_pub = node.create_publisher(UAVState, '/uav_state', 10)
        landing_pub = node.create_publisher(
            LandingTarget, '/landing_target', 10
        )
        query = node.create_client(GetPerception, '/platform/perception/query')
        cap = node.create_client(
            GetCapabilities, '/platform/perception/capabilities'
        )

        def spin(seconds):
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=0.01)

        def call(client, request):
            self.assertTrue(client.wait_for_service(timeout_sec=3))
            future = client.call_async(request)
            until = time.monotonic() + 3
            while not future.done() and time.monotonic() < until:
                executor.spin_once(timeout_sec=0.01)
            self.assertTrue(future.done())
            return future.result()

        def stamp(message):
            message.header.frame_id = 'odom'
            message.header.stamp = node.get_clock().now().to_msg()
            return message

        def ask(kind='targets', distance=0.0):
            return call(
                query, GetPerception.Request(kind=kind, range_limit=distance)
            )

        try:
            spin(0.1)
            self.assertEqual(ask().status, GetPerception.Response.NOT_READY)
            self.assertEqual(
                ask('unknown').status, GetPerception.Response.UNSUPPORTED
            )
            self.assertEqual(
                ask(distance=float('nan')).status,
                GetPerception.Response.INVALID,
            )
            empty = stamp(Target3DArray())
            targets_pub.publish(empty)
            obstacles_pub.publish(stamp(ObstacleInfoArray()))
            spin(0.1)
            result = ask()
            self.assertEqual(result.status, result.VALID)
            self.assertFalse(result.targets.targets)
            self.assertEqual(result.targets.header, empty.header)
            self.assertEqual(ask('obstacles').status, result.VALID)
            self.assertEqual(ask(distance=1.0).status, result.NOT_READY)
            invalid = Target3DArray()
            invalid.header.frame_id = 'odom'
            targets_pub.publish(invalid)
            spin(0.05)
            self.assertEqual(ask().status, result.INVALID)
            old = stamp(Target3DArray())
            old.header.stamp.sec -= 10
            targets_pub.publish(old)
            spin(0.05)
            self.assertEqual(ask().status, result.STALE)
            items = stamp(Target3DArray())
            for x in (0.5, 3.0):
                item = stamp(
                    Target3D(position_x=x, position_valid=True, confidence=0.9)
                )
                items.targets.append(item)
            targets_pub.publish(items)
            state = stamp(UAVState(connected=True, pose_valid=True))
            state_pub.publish(state)
            spin(0.05)
            result = ask(distance=1.0)
            self.assertEqual(result.status, result.VALID)
            self.assertEqual(len(result.targets.targets), 1)
            broken = copy.deepcopy(items)
            broken.targets[0].position_valid = False
            targets_pub.publish(broken)
            spin(0.05)
            self.assertEqual(ask().status, result.INVALID)
            bad = stamp(ObstacleInfoArray())
            bad.obstacles = [stamp(ObstacleInfo(size_x=-1.0))]
            obstacles_pub.publish(bad)
            spin(0.05)
            self.assertEqual(ask('obstacles').status, result.INVALID)
            landing = LandingTarget(valid=True, tag_id=7, optical_depth_m=1.0)
            landing.header.frame_id = 'camera_color_optical_frame'
            landing.header.stamp = node.get_clock().now().to_msg()
            landing.pose.position.z = landing.pose.orientation.w = 1.0
            landing_pub.publish(landing)
            spin(0.1)
            self.assertTrue(call(cap, GetCapabilities.Request()).ready[2])
            landing.valid = False
            landing_pub.publish(landing)
            spin(0.05)
            self.assertFalse(call(cap, GetCapabilities.Request()).ready[2])
            landing.valid = True
            landing.header.stamp = node.get_clock().now().to_msg()
            landing_pub.publish(landing)
            targets_pub.publish(stamp(Target3DArray()))
            spin(0.05)
            self.assertTrue(
                service.set_parameters_atomically(
                    [Parameter('max_age', value=0.05)]
                ).successful
            )
            self.assertFalse(
                service.set_parameters_atomically(
                    [Parameter('max_age', value=-1.0)]
                ).successful
            )
            self.assertEqual(service.get_parameter('max_age').value, 0.05)
            spin(0.1)
            self.assertEqual(
                ask().status, result.STALE
            )  # Monotonic receipt also enforces age.
            response = call(cap, GetCapabilities.Request())
            self.assertEqual(response.implemented, [True, True, True, True])
            self.assertFalse(any(response.ready))
            self.assertEqual(len(response.names), len(response.details))
        finally:
            executor.shutdown()
            node.destroy_node()
            service.destroy_node()
            rclpy.shutdown()

    def test_landing_clock_zero_pause_rollback_require_new_observation(self):
        rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
        service = PerceptionServices()
        node = rclpy.create_node('landing_capability_clock_test')
        executor = SingleThreadedExecutor()
        executor.add_node(service)
        executor.add_node(node)
        clock = node.create_publisher(Clock, '/clock', 10)
        landing = node.create_publisher(LandingTarget, '/landing_target', 10)
        cap = node.create_client(
            GetCapabilities, '/platform/perception/capabilities'
        )

        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=0.01)

        def ready():
            self.assertTrue(cap.wait_for_service(timeout_sec=3))
            future = cap.call_async(GetCapabilities.Request())
            end = time.monotonic() + 3
            while not future.done() and time.monotonic() < end:
                executor.spin_once(timeout_sec=0.01)
            self.assertTrue(future.done())
            return future.result().ready[2]

        def advance(seconds, observe=True):
            message = Clock()
            message.clock.sec = seconds
            clock.publish(message)
            spin(
                0.08
            )  # Timer detects the new clock epoch before receiving observations.
            if observe:
                target = LandingTarget(
                    valid=True, tag_id=7, optical_depth_m=1.0
                )
                target.header.frame_id = 'camera_color_optical_frame'
                target.header.stamp.sec = seconds
                target.pose.position.z = target.pose.orientation.w = 1.0
                landing.publish(target)
                spin(0.08)

        try:
            spin(0.1)
            self.assertFalse(ready())
            advance(100)
            self.assertTrue(ready())
            spin(1.1)
            self.assertFalse(ready())
            advance(101)
            self.assertTrue(ready())
            advance(50, observe=False)
            self.assertFalse(ready())
            advance(51)
            self.assertTrue(ready())
        finally:
            executor.shutdown()
            node.destroy_node()
            service.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()

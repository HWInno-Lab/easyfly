# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_lidar_camera_fusion/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
#

"""Real ROS clock/executor regression for independent fusion and classification."""
import struct
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster
from easyfly_msgs.msg import (
    ObstacleInfo,
    ObstacleInfoArray,
    Target3D,
    Target3DArray,
)
from easyfly_msgs.srv import GetCapabilities
from easyfly_lidar_camera_fusion.node import FusionNode
from easyfly_obstacle_classification.node import ObstacleClassifier
from test_projection import fixtures


class ClockROS(unittest.TestCase):
    def test_zero_pause_replayed_source_rollback_and_new_epoch(self):
        rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
        fusion, classifier = FusionNode(), ObstacleClassifier()
        node = rclpy.create_node('d4_fusion_classification_clock_test')
        executor = SingleThreadedExecutor()
        for item in (node, fusion, classifier):
            executor.add_node(item)
        clock = node.create_publisher(Clock, '/clock', 10)
        images = node.create_publisher(
            Image, '/camera/color/image_raw', qos_profile_sensor_data
        )
        infos = node.create_publisher(
            CameraInfo, '/camera/color/camera_info', qos_profile_sensor_data
        )
        clouds = node.create_publisher(
            PointCloud2, '/pointcloud_filtered', qos_profile_sensor_data
        )
        geometry = node.create_publisher(
            ObstacleInfoArray,
            '/obstacles',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        targets = node.create_publisher(Target3DArray, '/target_positions', 10)
        capabilities = [
            node.create_client(GetCapabilities, path)
            for path in (
                '/platform/lidar_camera/capabilities',
                '/platform/classification/capabilities',
            )
        ]
        broadcaster = StaticTransformBroadcaster(node)

        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=0.005)

        def ready():
            futures = [
                client.call_async(GetCapabilities.Request())
                for client in capabilities
            ]
            end = time.monotonic() + 3
            while (
                not all(future.done() for future in futures)
                and time.monotonic() < end
            ):
                executor.spin_once(timeout_sec=0.005)
            self.assertTrue(all(future.done() for future in futures))
            return [future.result().ready[0] for future in futures]

        def send(seconds, advance=True):
            if advance:
                message = Clock()
                message.clock.sec = seconds
                clock.publish(message)
                spin(0.08)
            transforms = []
            for child in ('camera_color_optical_frame', 'lidar_test'):
                tf = TransformStamped()
                tf.header.frame_id, tf.child_frame_id = 'odom', child
                tf.header.stamp.sec = seconds
                tf.transform.rotation.w = 1.0
                transforms.append(tf)
            broadcaster.sendTransform(transforms)
            image, info, _ = fixtures()
            image.header.frame_id = (
                info.header.frame_id
            ) = 'camera_color_optical_frame'
            image.header.stamp.sec = info.header.stamp.sec = seconds
            images.publish(image)
            infos.publish(info)
            spin(0.05)
            cloud = PointCloud2(
                height=1,
                width=1,
                point_step=12,
                row_step=12,
                data=struct.pack('<fff', 0.0, 0.0, 2.0),
                fields=[
                    PointField(name=n, offset=i * 4, datatype=7, count=1)
                    for i, n in enumerate(('x', 'y', 'z'))
                ],
            )
            cloud.header.frame_id, cloud.header.stamp.sec = (
                'lidar_test',
                seconds,
            )
            clouds.publish(cloud)
            obstacles = ObstacleInfoArray()
            obstacles.header.frame_id, obstacles.header.stamp.sec = (
                'odom',
                seconds,
            )
            obstacle = ObstacleInfo(
                header=obstacles.header,
                obstacle_id=7,
                center_z=1.0,
                size_x=2.0,
                size_y=2.0,
                size_z=2.0,
            )
            obstacles.obstacles = [obstacle]
            measured = Target3DArray(
                header=obstacles.header,
                targets=[
                    Target3D(
                        header=obstacles.header,
                        track_id=9,
                        class_name='person',
                        confidence=0.9,
                        position_z=1.0,
                        position_valid=True,
                    )
                ],
            )
            targets.publish(measured)
            geometry.publish(obstacles)
            spin(0.15)

        try:
            spin(0.2)
            for client in capabilities:
                self.assertTrue(client.wait_for_service(timeout_sec=3))
            self.assertEqual(ready(), [False, False])
            send(100)
            self.assertEqual(ready(), [True, True])
            spin(0.65)
            self.assertEqual(ready(), [False, False])
            send(
                100, advance=False
            )  # Receipt refresh cannot revive frozen ROS time.
            self.assertEqual(ready(), [False, False])
            send(101)
            self.assertEqual(ready(), [True, True])
            rollback = Clock()
            rollback.clock.sec = 50
            clock.publish(rollback)
            spin(0.1)
            self.assertEqual(ready(), [False, False])
            send(51)
            self.assertEqual(ready(), [True, True])
            self.assertEqual(node.count_publishers('/obstacles'), 1)
            self.assertEqual(node.count_publishers('/occupancy_grid'), 0)
        finally:
            fusion.close()
            executor.shutdown()
            for item in (node, classifier, fusion):
                item.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()

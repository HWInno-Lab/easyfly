# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_lidar_camera_fusion/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
#

import os
import signal
import struct
import subprocess
import tempfile
import threading
import time
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from diagnostic_msgs.msg import DiagnosticArray
from tf2_ros import StaticTransformBroadcaster
from easyfly_msgs.srv import GetCapabilities
from test_projection import fixtures


class FusionROS(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('fusion_test')
        cls.output = cls.depth_output = None
        cls.status = ''
        cls.node.create_subscription(
            PointCloud2,
            '/lidar_camera/colored_points',
            lambda m: setattr(cls, 'output', m),
            10,
        )
        cls.node.create_subscription(
            Image,
            '/lidar_camera/depth',
            lambda m: setattr(cls, 'depth_output', m),
            qos_profile_sensor_data,
        )
        cls.node.create_subscription(
            DiagnosticArray,
            '/lidar_camera/status',
            lambda m: setattr(cls, 'status', m.status[0].message),
            10,
        )
        cls.images = cls.node.create_publisher(
            Image, '/camera/color/image_raw', qos_profile_sensor_data
        )
        cls.infos = cls.node.create_publisher(
            CameraInfo, '/camera/color/camera_info', qos_profile_sensor_data
        )
        cls.clouds = cls.node.create_publisher(
            PointCloud2, '/pointcloud_filtered', qos_profile_sensor_data
        )
        cls.cap = cls.node.create_client(
            GetCapabilities, '/platform/lidar_camera/capabilities'
        )
        cls.tf = StaticTransformBroadcaster(cls.node)
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin)
        cls.thread.start()
        cls.log = tempfile.TemporaryFile(mode='w+')
        cls.process = subprocess.Popen(
            [
                'ros2',
                'run',
                'easyfly_lidar_camera_fusion',
                'lidar_camera_fusion',
                '--ros-args',
                '-p',
                'use_sim_time:=false',
            ],
            stdout=cls.log,
            stderr=cls.log,
            start_new_session=True,
        )
        cls.addClassCleanup(cls.cleanup)
        if not cls.cap.wait_for_service(timeout_sec=8):
            cls.log.seek(0)
            raise RuntimeError(cls.log.read())
        transforms = []
        for child in ('camera_color_optical_frame', 'lidar_test'):
            tf = TransformStamped()
            tf.header.stamp = cls.node.get_clock().now().to_msg()
            tf.header.frame_id, tf.child_frame_id = 'odom', child
            tf.transform.rotation.w = 1.0
            transforms.append(tf)
        cls.tf.sendTransform(transforms)

    @classmethod
    def cleanup(cls):
        if cls.process.poll() is None:
            os.killpg(cls.process.pid, signal.SIGINT)
        try:
            cls.process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            os.killpg(cls.process.pid, signal.SIGKILL)
            cls.process.wait(timeout=2)
        cls.executor.shutdown()
        cls.thread.join()
        cls.node.destroy_node()
        rclpy.shutdown()
        cls.log.close()

    def until(self, predicate, pulse=None, timeout=4):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if pulse:
                pulse()
            if predicate():
                return
            time.sleep(0.08)
        self.log.seek(0)
        self.fail(f'{self.status}: {self.log.read()}')

    def send(self, frame='lidar_test', offset=0, bad_camera=False):
        image, info, _ = fixtures()
        image.header.frame_id = (
            info.header.frame_id
        ) = 'camera_color_optical_frame'
        image.header.stamp = info.header.stamp = (
            self.node.get_clock().now().to_msg()
        )
        if bad_camera:
            info.k[0] = float('nan')
        self.images.publish(image)
        self.infos.publish(info)
        time.sleep(0.025)
        stamp = image.header.stamp
        source_ns = (
            stamp.sec * 1000000000 + stamp.nanosec + round(offset * 1e9)
        )
        cloud = PointCloud2(
            height=1,
            width=2,
            point_step=12,
            row_step=24,
            data=struct.pack('<ffffff', 0.0, 0.0, 4.0, 0.0, 0.0, 2.0),
            fields=[
                PointField(name=n, offset=i * 4, datatype=7, count=1)
                for i, n in enumerate(('x', 'y', 'z'))
            ],
        )
        cloud.header.frame_id = frame
        cloud.header.stamp.sec, cloud.header.stamp.nanosec = divmod(
            source_ns, 1000000000
        )
        self.clouds.publish(cloud)

    def test_known_projection_preserves_source_coordinates_and_headers(self):
        type(self).output = type(self).depth_output = None
        self.until(
            lambda: self.output is not None
            and self.depth_output is not None
            and self.output.width == 1,
            pulse=self.send,
        )
        self.assertEqual(self.output.header.frame_id, 'lidar_test')
        self.assertEqual(
            struct.unpack('<fffI', self.output.data), (0.0, 0.0, 2.0, 0x0A141E)
        )
        self.assertEqual(
            self.depth_output.header.frame_id, 'camera_color_optical_frame'
        )
        self.assertEqual(struct.unpack('<9f', self.depth_output.data)[4], 2.0)
        self.assertGreater(self.output.header.stamp.sec, 0)

    def test_no_tf_bad_camera_and_timing_fail_explicitly(self):
        self.until(
            lambda: 'projection_failed:' in self.status,
            pulse=lambda: self.send('missing_tf'),
        )
        self.until(
            lambda: 'CameraInfo unavailable' in self.status,
            pulse=lambda: self.send(bad_camera=True),
        )
        self.until(
            lambda: 'outside tolerance' in self.status,
            pulse=lambda: self.send(offset=0.1),
        )
        future = self.cap.call_async(GetCapabilities.Request())
        self.until(future.done)
        self.assertFalse(future.result().ready[0])

    def test_stale_success_cannot_keep_capability_ready(self):
        self.until(lambda: self.status == 'valid_projection', pulse=self.send)
        time.sleep(0.65)
        future = self.cap.call_async(GetCapabilities.Request())
        self.until(future.done)
        self.assertFalse(future.result().ready[0])


if __name__ == '__main__':
    unittest.main()

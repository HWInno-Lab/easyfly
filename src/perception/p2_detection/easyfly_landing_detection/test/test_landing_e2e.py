# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_landing_detection/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
#

"""Real generated marker image -> real tag pose -> real landing adapter."""
import os
import signal
import subprocess
import tempfile
import time
import unittest
import rclpy
from sensor_msgs.msg import Image, CameraInfo
from easyfly_msgs.msg import LandingTarget


class LandingEndToEnd(unittest.TestCase):
    def test_image_to_selected_optical_target_and_blank_loss(self):
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        node = rclpy.create_node('landing_e2e')
        self.addCleanup(node.destroy_node)
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        log = tempfile.TemporaryFile(mode='w+')
        self.addCleanup(log.close)
        subprocess.run(
            [
                'ros2',
                'run',
                'easyfly_apriltag_detection',
                'tag_fixture',
                tmp.name + '/marker',
            ],
            check=True,
        )
        with open(tmp.name + '/marker', 'rb') as file:
            pixels = file.read()
        processes = []

        def stop():
            for process in processes:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
            for process in processes:
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)

        self.addCleanup(stop)
        for package, exe, param in [
            (
                'easyfly_apriltag_detection',
                'apriltag_detection_node',
                'tag_size_m:=0.2',
            ),
            (
                'easyfly_landing_detection',
                'landing_detection_node',
                'selected_tag_id:=7',
            ),
        ]:
            processes.append(
                subprocess.Popen(
                    ['ros2', 'run', package, exe, '--ros-args', '-p', param],
                    stdout=log,
                    stderr=log,
                    start_new_session=True,
                )
            )
        pub = node.create_publisher(Image, '/camera/image_raw', 10)
        calibration = node.create_publisher(
            CameraInfo, '/camera/camera_info', 10
        )
        outputs = []
        node.create_subscription(
            LandingTarget, '/landing_target', outputs.append, 10
        )
        stamps = set()

        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.005)

        def feed(data):
            outputs.clear()
            for _ in range(16):
                image = Image(
                    height=480,
                    width=640,
                    encoding='mono8',
                    step=640,
                    data=data,
                )
                image.header.frame_id = 'camera_color_optical_frame'
                image.header.stamp = node.get_clock().now().to_msg()
                stamps.add(
                    (image.header.stamp.sec, image.header.stamp.nanosec)
                )
                info = CameraInfo(
                    height=480,
                    width=640,
                    distortion_model='plumb_bob',
                    k=[600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0],
                    d=[0.0] * 5,
                )
                info.header = image.header
                calibration.publish(info)
                pub.publish(image)
                spin(0.06)

        spin(0.4)
        feed(pixels)
        valid = [output for output in outputs if output.valid]
        if not valid:
            log.seek(0)
            print(log.read())
        self.assertTrue(valid)
        target = valid[-1]
        self.assertEqual(target.tag_id, 7)
        self.assertAlmostEqual(target.optical_depth_m, 1.0, delta=0.06)
        self.assertEqual(target.header.frame_id, 'camera_color_optical_frame')
        self.assertIn(
            (target.header.stamp.sec, target.header.stamp.nanosec), stamps
        )
        feed(bytes([255]) * (640 * 480))
        self.assertTrue(outputs)
        self.assertFalse(outputs[-1].valid)
        self.assertEqual(outputs[-1].reason, 'selected_tag_missing')


if __name__ == '__main__':
    unittest.main()

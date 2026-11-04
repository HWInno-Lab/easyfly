# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_apriltag_detection/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
#

import os
import signal
import subprocess
import tempfile
import time
import unittest
import rclpy
from sensor_msgs.msg import Image, CameraInfo
from easyfly_msgs.msg import TagObservationArray
from easyfly_msgs.srv import GetCapabilities


class TagRos(unittest.TestCase):
    def test_real_dictionary_pose_and_invalid_inputs(self):
        rclpy.init()
        node = rclpy.create_node('tag_test')
        outputs = []
        self.addCleanup(rclpy.shutdown)
        self.addCleanup(node.destroy_node)
        image_pub = node.create_publisher(Image, '/camera/image_raw', 10)
        info_pub = node.create_publisher(CameraInfo, '/camera/camera_info', 10)
        node.create_subscription(
            TagObservationArray, '/tag_observations', outputs.append, 10
        )
        capabilities = node.create_client(
            GetCapabilities, '/platform/apriltag/capabilities'
        )
        log = tempfile.TemporaryFile(mode='w+')
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(log.close)
        self.addCleanup(tmp.cleanup)
        subprocess.run(
            [
                'ros2',
                'run',
                'easyfly_apriltag_detection',
                'tag_fixture',
                tmp.name + '/image',
            ],
            check=True,
        )
        with open(tmp.name + '/image', 'rb') as file:
            pixels = file.read()
        process = None

        def start(size):
            return subprocess.Popen(
                [
                    'ros2',
                    'run',
                    'easyfly_apriltag_detection',
                    'apriltag_detection_node',
                    '--ros-args',
                    '-p',
                    'tag_size_m:=' + str(float(size)),
                ],
                stdout=log,
                stderr=log,
                start_new_session=True,
            )

        def stop():
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)

        self.addCleanup(lambda: stop() if process else None)

        def feed(kind='good', size=True):
            outputs.clear()
            for _ in range(12):
                h = node.get_clock().now().to_msg()
                image = Image(
                    height=480,
                    width=640,
                    encoding='mono8',
                    step=640,
                    data=pixels,
                )
                image.header.frame_id = 'camera_color_optical_frame'
                image.header.stamp = h
                info = CameraInfo(
                    height=480,
                    width=640,
                    distortion_model='plumb_bob',
                    k=[600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0],
                    d=[0.0] * 5,
                )
                info.header.frame_id = image.header.frame_id
                info.header.stamp = h
                if kind == 'badframe':
                    image.header.frame_id = 'map'
                elif kind == 'stale':
                    image.header.stamp.sec -= 2
                elif kind == 'payload':
                    image.data = b'bad'
                elif kind == 'empty':
                    image.data = bytes([255]) * (640 * 480)
                elif kind == 'badK':
                    info.k[0] = 0.0
                elif kind == 'badInfoFrame':
                    info.header.frame_id = 'map'
                if kind != 'noK':
                    info_pub.publish(info)
                image_pub.publish(image)
                end = time.monotonic() + 0.06
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.005)
            self.assertTrue(outputs)
            return outputs[-1]

        def ready():
            future = capabilities.call_async(GetCapabilities.Request())
            end = time.monotonic() + 3
            while not future.done() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.005)
            self.assertTrue(future.done())
            return future.result().ready[0]

        def info_only(focal=600.0):
            info = CameraInfo(
                height=480,
                width=640,
                distortion_model='plumb_bob',
                k=[focal, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0],
                d=[0.0] * 5,
            )
            info.header.frame_id = 'camera_color_optical_frame'
            info.header.stamp = node.get_clock().now().to_msg()
            outputs.clear()
            info_pub.publish(info)
            end = time.monotonic() + 0.08
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.005)

        try:
            process = start(0.2)
            time.sleep(0.3)
            self.assertFalse(feed('noK').valid)
            observed = feed()
            self.assertTrue(observed.valid, observed.reason)
            valid = [o for o in observed.observations if o.valid]
            self.assertTrue(
                valid, [(o.tag_id, o.reason) for o in observed.observations]
            )
            self.assertEqual(valid[0].tag_id, 7)
            self.assertAlmostEqual(valid[0].pose.position.z, 1.0, delta=0.06)
            self.assertTrue(ready())
            info_only()
            self.assertTrue(
                ready()
            )  # same fresh model must not revoke an existing fresh result
            info_only(0.0)
            self.assertFalse(
                ready()
            )  # bad CameraInfo revokes without any new image
            self.assertTrue(outputs)
            self.assertFalse(outputs[-1].valid)
            self.assertTrue(feed().valid)
            info_only(650.0)
            self.assertFalse(
                ready()
            )  # model changed, old result is not silently relabeled
            self.assertTrue(feed().valid)
            self.assertFalse(
                feed('noK').valid
            )  # new image receipts cannot refresh old calibration time
            self.assertFalse(ready())
            self.assertTrue(feed().valid)
            empty = feed('empty')
            self.assertTrue(empty.valid)
            self.assertFalse(empty.observations)
            for kind in (
                'badK',
                'badInfoFrame',
                'badframe',
                'stale',
                'payload',
            ):
                self.assertFalse(feed(kind).valid, kind)
            stop()
            process = start(0)
            time.sleep(0.3)
            self.assertFalse(feed().valid)
        finally:
            if process:
                stop()
            log.seek(0)
            if self._outcome and not self._outcome.success:
                print(log.read())
            log.close()
            tmp.cleanup()


if __name__ == '__main__':
    unittest.main()

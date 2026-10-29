# Copyright 2026 EasyFly Project
#
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

import os
import signal
import subprocess
import tempfile
import time
import unittest
import rclpy
from sensor_msgs.msg import Image
from easyfly_msgs.msg import SingleTrackState
from easyfly_msgs.srv import ManageSingleTrack


class SingleRos(unittest.TestCase):
    def test_roi_identity_translation_reset_loss_and_stale_cache(self):
        rclpy.init()
        node = rclpy.create_node('single_test')
        states = []
        self.addCleanup(rclpy.shutdown)
        self.addCleanup(node.destroy_node)
        pub = node.create_publisher(Image, '/camera/image_raw', 10)
        node.create_subscription(
            SingleTrackState, '/single_track/state', states.append, 10
        )
        client = node.create_client(ManageSingleTrack, '/single_track/manage')
        log = tempfile.TemporaryFile(mode='w+')
        self.addCleanup(log.close)
        process = subprocess.Popen(
            [
                'ros2',
                'run',
                'easyfly_single_object_tracking',
                'single_object_tracking_node',
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

        self.addCleanup(stop)

        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.005)

        def image(dx=0):
            data = bytearray([30]) * (200 * 160)
            for y in range(40):
                for x in range(40):
                    data[(50 + y) * 200 + 60 + dx + x] = (
                        (x * 127 + y * 53) ^ (x * y * 13)
                    ) % 256
            m = Image(
                height=160, width=200, encoding='mono8', step=200, data=data
            )
            m.header.frame_id = 'camera_color_optical_frame'
            m.header.stamp = node.get_clock().now().to_msg()
            pub.publish(m)
            spin(0.04)
            return m

        def call(operation, m=None, x=0.3, width=0.2):
            req = ManageSingleTrack.Request(
                operation=operation,
                roi_x=x,
                roi_y=0.3125,
                roi_width=width,
                roi_height=0.25,
                class_name='texture',
            )
            if m:
                req.image_header = m.header
            future = client.call_async(req)
            end = time.monotonic() + 3
            while not future.done() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.005)
            self.assertTrue(future.done())
            return future.result()

        try:
            self.assertTrue(client.wait_for_service(timeout_sec=5))
            spin(0.2)
            m = image()
            self.assertFalse(call('initialize', m, x=-0.1).accepted)
            newer = image(1)
            self.assertNotEqual(newer.header.stamp, m.header.stamp)
            result = call('initialize', m)
            self.assertEqual(result.state.header, m.header)
            self.assertTrue(result.accepted)
            first = result.state.track_id
            spin(0.12)
            self.assertTrue(
                any(s.valid and s.track_id == first for s in states)
            )
            image(8)
            spin(0.12)
            latest = [s for s in states if s.valid][-1]
            self.assertAlmostEqual(
                latest.track.bbox_center_x, 0.44, delta=0.04
            )
            self.assertEqual(latest.track.confidence, 0.0)
            self.assertTrue(call('reset').accepted)
            spin(0.04)
            self.assertFalse(states[-1].valid)
            m = image()
            second = call('initialize', m)
            self.assertTrue(second.accepted)
            self.assertGreater(second.state.track_id, first)
            spin(0.65)
            self.assertFalse(call('query').state.valid)
            self.assertEqual(call('query').state.phase, 'lost')
            self.assertFalse(call('initialize', m).accepted)
            m = image()
            self.assertTrue(call('initialize', m).accepted)
            self.assertTrue(call('reset').accepted)
            spin(0.15)
            self.assertEqual(call('query').state.phase, 'idle')
            self.assertFalse(states[-1].valid)
        finally:
            stop()
            log.close()


if __name__ == '__main__':
    unittest.main()

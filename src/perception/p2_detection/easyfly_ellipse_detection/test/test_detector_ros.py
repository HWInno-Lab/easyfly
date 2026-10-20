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

import math
import os
import signal
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix
import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from easyfly_msgs.msg import EllipseObservationArray
from easyfly_msgs.srv import GetCapabilities


KIND = 'ellipse'
PACKAGE = 'easyfly_' + KIND + '_detection'


class DetectorRos(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        self.node = rclpy.create_node(KIND + '_test')
        self.addCleanup(self.node.destroy_node)
        self.outputs = []
        self.node.create_subscription(
            EllipseObservationArray, '/' + KIND + '_observations', self.outputs.append, 10)
        self.publisher = self.node.create_publisher(
            Image, '/camera/image_raw', qos_profile_sensor_data)
        self.clock = self.node.create_publisher(Clock, '/clock', 10)
        self.capability = self.node.create_client(
            GetCapabilities, '/platform/' + KIND + '/capabilities')
        self.process = None
        self.log = tempfile.TemporaryFile(mode='w+')
        self.addCleanup(self.log.close)
        self.addCleanup(self.stop)
        self.sim_seconds = None

    def stop(self):
        if self.process is not None:
            if self.process.poll() is None:
                os.killpg(self.process.pid, signal.SIGINT)
            try:
                status = self.process.wait(timeout=7)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=5)
                self.fail('detector did not stop cleanly')
            self.log.seek(0)
            output = self.log.read()
            self.process = None
            self.assertEqual(status, 0, output)
            self.assertNotIn('process has died', output)

    def spin(self, seconds=0.1):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def wait(self, predicate, seconds=6.0):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if predicate():
                return
            self.spin(0.02)
        self.log.seek(0)
        self.fail('timed out: ' + self.log.read())

    def start(self, sim=False, green=False):
        # Exercise the installed launch file and installed default configuration.
        command = ['ros2', 'launch', PACKAGE, 'detection.launch.py',
                   'use_sim_time:=' + str(sim).lower()]
        if green:
            self.config = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml')
            self.addCleanup(self.config.close)
            self.config.write('color_line_detection:\n  ros__parameters:\n'
                              '    hue_min: 50\n    hue_max: 70\n')
            self.config.flush()
            command.append('config_file:=' + self.config.name)
        self.process = subprocess.Popen(
            command, stdout=self.log, stderr=self.log, start_new_session=True)
        self.wait(lambda: self.capability.service_is_ready())
        self.wait(lambda: self.publisher.get_subscription_count() > 0)
        self.wait(lambda: self.node.count_publishers('/' + KIND + '_observations') > 0)
        self.spin(0.2)
        self.assertFalse(self.ready())

    def ready(self):
        future = self.capability.call_async(GetCapabilities.Request())
        self.wait(future.done)
        self.assertTrue(future.result().implemented[0])
        return future.result().ready[0]

    def fixture(self, green=False):
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        if KIND == 'ellipse':
            cv2.ellipse(image, (160, 120), (65, 30), 30, 0, 360, (255, 255, 255), -1)
        else:
            cv2.rectangle(image, (210, 20), (230, 220),
                          (0, 255, 0) if green else (0, 0, 255), -1)
        return image

    def message(self, image=None, encoding='bgr8', padding=0):
        if image is None:
            image = self.fixture()
        if encoding == 'rgb8':
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        elif encoding == 'mono8':
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        channels = 1 if encoding == 'mono8' else 3
        width = image.shape[1]
        rows = image.reshape(image.shape[0], width * channels)
        if padding:
            rows = np.pad(rows, ((0, 0), (0, padding)), constant_values=17)
        message = Image(height=image.shape[0], width=width, encoding=encoding,
                        step=width * channels + padding, data=rows.tobytes())
        message.header.frame_id = 'camera_color_optical_frame'
        if self.sim_seconds is None:
            message.header.stamp = self.node.get_clock().now().to_msg()
        else:
            message.header.stamp.sec = self.sim_seconds
        return message

    def send(self, message):
        before = len(self.outputs)
        self.publisher.publish(message)
        self.wait(lambda: len(self.outputs) > before)
        self.wait(lambda: any(item.header == message.header for item in self.outputs[before:]))
        matches = [item for item in self.outputs[before:] if item.header == message.header]
        return matches[-1]

    def clock_to(self, seconds):
        self.sim_seconds = seconds
        clock = Clock()
        clock.clock.sec = seconds
        # Repeat only clock samples for DDS discovery; images are never replayed as fresh.
        for _ in range(4):
            self.clock.publish(clock)
            self.spin(0.03)

    def test_geometry_layout_invalid_input_and_expiry(self):
        self.start()
        for encoding in ['bgr8', 'rgb8']:
            message = self.message(encoding=encoding, padding=11)
            result = self.send(message)
            self.assertTrue(result.valid, result.reason)
            self.assertEqual(len(result.observations), 1)
            self.assertEqual(result.header, message.header)
            self.assertEqual((result.image_width, result.image_height), (320, 240))
            observation = result.observations[0]
            if KIND == 'ellipse':
                self.assertAlmostEqual(observation.center_x, 160, delta=1)
                self.assertAlmostEqual(observation.major_axis_px, 130, delta=3)
                self.assertAlmostEqual(observation.angle_rad, math.pi / 6, delta=0.04)
            else:
                self.assertAlmostEqual(observation.center_x, 220, delta=1)
                self.assertAlmostEqual(observation.length_px, 200, delta=2)
                self.assertGreater(observation.offset_x_normalized, 0.3)
            self.assertTrue(self.ready())
        mono = self.send(self.message(encoding='mono8'))
        self.assertEqual(mono.valid, KIND == 'ellipse')
        self.assertEqual(len(mono.observations), 1 if KIND == 'ellipse' else 0)
        for kind in [
                'frame', 'stale', 'future', 'payload', 'step', 'encoding', 'oversize', 'zero']:
            bad = self.message()
            if kind == 'frame':
                bad.header.frame_id = 'map'
            elif kind == 'stale':
                bad.header.stamp.sec -= 2
            elif kind == 'future':
                bad.header.stamp.sec += 2
            elif kind == 'payload':
                bad.data = b'bad'
            elif kind == 'step':
                bad.step = 1
            elif kind == 'encoding':
                bad.encoding = '32FC1'
            elif kind == 'oversize':
                bad.width = 2**32 - 1
            elif kind == 'zero':
                bad.header.stamp.sec = 0
                bad.header.stamp.nanosec = 0
            result = self.send(bad)
            self.assertFalse(result.valid, kind)
            self.assertFalse(result.observations, kind)
            self.assertFalse(self.ready(), kind)
        good = self.message()
        self.assertTrue(self.send(good).valid)
        self.assertFalse(self.send(good).valid)  # Duplicate cannot renew the source.
        good = self.message()
        self.assertTrue(self.send(good).valid)
        old = self.message()
        old.header.stamp = good.header.stamp
        old.header.stamp.nanosec = max(0, old.header.stamp.nanosec - 1)
        self.assertFalse(self.send(old).valid)
        blank = np.zeros((240, 320, 3), dtype=np.uint8)
        result = self.send(self.message(blank))
        self.assertTrue(result.valid)
        self.assertEqual(result.reason, 'no_detection')
        self.assertFalse(result.observations)
        self.assertTrue(self.ready())
        self.outputs.clear()
        self.wait(lambda: any(not item.valid for item in self.outputs), seconds=2)
        self.assertFalse(self.ready())
        self.assertTrue(self.send(self.message()).valid)

    def test_simulated_clock_pause_rewind_and_recovery(self):
        self.start(sim=True)
        self.clock_to(100)
        self.assertTrue(self.send(self.message()).valid)
        original = self.outputs[-1].header
        self.outputs.clear()
        self.wait(lambda: any(not item.valid for item in self.outputs), seconds=2)
        self.assertEqual(self.outputs[-1].header, original)
        self.assertFalse(self.ready())
        self.clock_to(101)
        self.assertTrue(self.send(self.message()).valid)
        self.outputs.clear()
        self.clock_to(50)
        self.wait(lambda: any(item.reason == 'clock_rewind' for item in self.outputs))
        self.assertFalse(self.ready())
        self.assertTrue(self.send(self.message()).valid)

    def test_rejects_invalid_startup_parameters(self):
        executable = os.path.join(get_package_prefix(PACKAGE), 'lib', PACKAGE,
                                  KIND + '_detection_node')
        invalid = 'min_axis_px:=0.0' if KIND == 'ellipse' else 'hue_max:=180'
        result = subprocess.run(
            [executable, '--ros-args', '-p', invalid], capture_output=True, timeout=8)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(b'invalid', result.stderr)

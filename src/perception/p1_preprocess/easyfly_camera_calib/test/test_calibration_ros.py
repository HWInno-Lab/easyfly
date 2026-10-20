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

"""Actual TF publisher and continuous CameraInfo checker, no flight stack."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import CameraInfo
from rosgraph_msgs.msg import Clock
from diagnostic_msgs.msg import DiagnosticArray
from tf2_msgs.msg import TFMessage
from ament_index_python.packages import get_package_share_directory


class CalibrationROS(unittest.TestCase):
    def setUp(self):
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('calibration_test', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.status = None
        self.transforms = []
        self.processes = []
        self.addCleanup(self.cleanup)
        self.node.create_subscription(
            DiagnosticArray,
            '/camera/calibration/status',
            lambda m: setattr(self, 'status', m.status[0].message),
            10,
        )
        self.node.create_subscription(
            TFMessage,
            '/tf_static',
            lambda m: self.transforms.extend(m.transforms),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.publisher = self.node.create_publisher(
            CameraInfo, '/camera/color/camera_info', qos_profile_sensor_data
        )

    def cleanup(self):
        for process in self.processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2)
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.shutdown()

    def start(self, executable, *arguments, sim=False):
        process = subprocess.Popen(
            [
                'ros2',
                'run',
                'easyfly_camera_calib',
                executable,
                '--ros-args',
                '-p',
                f'use_sim_time:={str(sim).lower()}',
                *arguments,
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.processes.append(process)
        return process

    def until(self, predicate, timeout=3, tick=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if tick:
                tick()
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        self.fail('calibration condition not reached')

    def info(self):
        message = CameraInfo()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = 'camera_color_optical_frame'
        message.width, message.height = 640, 480
        message.k = [337.36, 0.0, 320.5, 0.0, 337.36, 240.5, 0.0, 0.0, 1.0]
        message.distortion_model = 'plumb_bob'
        message.d = [0.0] * 5
        return message

    def test_valid_then_mismatch_then_stale_recovery_is_continuous(self):
        self.start('camera_info_checker', '-p', 'max_age:=0.25')
        self.until(lambda: self.publisher.get_subscription_count() > 0, 6)
        self.until(
            lambda: self.status == 'valid',
            tick=lambda: self.publisher.publish(self.info()),
        )

        def mismatch():
            message = self.info()
            message.k[0] = 100.0
            self.publisher.publish(message)

        self.until(lambda: self.status == 'intrinsics_mismatch', tick=mismatch)
        self.until(
            lambda: self.status == 'valid',
            tick=lambda: self.publisher.publish(self.info()),
        )
        self.until(lambda: self.status == 'stale_camera_info')

        def invalid():
            message = self.info()
            message.k[0] = float('nan')
            self.publisher.publish(message)

        self.until(lambda: self.status == 'invalid_K', tick=invalid)

    def test_valid_complete_static_tree_and_invalid_config_no_partial_tf(self):
        self.start('calib_tf_publisher')
        self.until(lambda: len(self.transforms) >= 5, 6)
        children = {t.child_frame_id for t in self.transforms}
        self.assertEqual(
            children,
            {
                'camera_link',
                'camera_color_optical_frame',
                'camera_depth_optical_frame',
                'livox_frame',
                'livox_imu_frame',
            },
        )
        imu = next(t for t in self.transforms if t.child_frame_id == 'livox_imu_frame')
        self.assertEqual(imu.header.frame_id, 'livox_frame')
        self.assertAlmostEqual(imu.transform.translation.z, -0.00588)
        source = (
            Path(get_package_share_directory('easyfly_camera_calib'))
            / 'config/sensors.yaml'
        )
        text = source.read_text()
        with tempfile.TemporaryDirectory() as directory:
            invalid = Path(directory) / 'invalid.yaml'
            # Valid camera first, corrupt lidar later: no early camera publication allowed.
            invalid.write_text(
                text.replace('child: livox_frame', 'child: base_link')
            )
            before = len(self.transforms)
            failed = self.start(
                'calib_tf_publisher', '-p', f'calib_file:={invalid}'
            )
            self.until(lambda: failed.poll() is not None, 6)
            self.assertNotEqual(failed.returncode, 0)
            deadline = time.monotonic() + 0.2
            while time.monotonic() < deadline:
                self.executor.spin_once(timeout_sec=0.02)
            self.assertEqual(len(self.transforms), before)

    def test_sim_clock_missing_pause_repeated_receipt_and_rollback(self):
        clock = self.node.create_publisher(Clock, '/clock', 10)
        self.start('camera_info_checker', '-p', 'max_age:=0.25', sim=True)
        self.until(lambda: self.publisher.get_subscription_count() > 0, 6)

        def observed(seconds):
            message = self.info()
            message.header.stamp.sec = int(seconds)
            message.header.stamp.nanosec = int((seconds - int(seconds)) * 1e9)
            self.publisher.publish(message)

        self.until(
            lambda: self.status == 'clock_not_ready',
            tick=lambda: observed(0.1),
        )
        self.until(lambda: clock.get_subscription_count() > 0)
        message = Clock()
        message.clock.sec = 1
        clock.publish(message)
        self.until(lambda: self.status == 'valid', tick=lambda: observed(1))
        # Repeated receipts of the same sample cannot mask a paused source clock.
        self.until(
            lambda: self.status == 'clock_paused', tick=lambda: observed(1)
        )
        message.clock.sec, message.clock.nanosec = 0, 500000000
        clock.publish(message)
        self.until(lambda: self.status in ('clock_reset', 'not_received'))
        self.until(lambda: self.status == 'valid', tick=lambda: observed(0.5))


if __name__ == '__main__':
    unittest.main()

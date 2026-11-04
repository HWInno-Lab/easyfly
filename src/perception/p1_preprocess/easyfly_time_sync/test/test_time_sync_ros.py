# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_time_sync/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
#

"""Actual software pairer, isolated graph, no aircraft nodes."""
import os
import signal
import subprocess
import time
import unittest

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock
from easyfly_msgs.msg import TimeSyncStatus
from easyfly_msgs.srv import GetCapabilities


class TimeSyncROS(unittest.TestCase):
    def setUp(self):
        self.sim = 'clock' in self._testMethodName
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node(
            'time_sync_test',
            context=self.context,
            parameter_overrides=[Parameter('use_sim_time', value=self.sim)],
        )
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.addCleanup(self.cleanup)
        self.status = None
        self.images = [[], []]
        self.node.create_subscription(
            TimeSyncStatus,
            '/time_sync/status',
            lambda m: setattr(self, 'status', m),
            10,
        )
        self.node.create_subscription(
            Image,
            '/time_sync/color/image_raw',
            lambda m: self.images[0].append(m),
            qos_profile_sensor_data,
        )
        self.node.create_subscription(
            Image,
            '/time_sync/depth/image_raw',
            lambda m: self.images[1].append(m),
            qos_profile_sensor_data,
        )
        self.inputs = [
            self.node.create_publisher(Image, name, qos_profile_sensor_data)
            for name in (
                '/camera/color/image_raw',
                '/camera/aligned_depth_to_color/depth/image_raw',
            )
        ]
        self.clock = self.node.create_publisher(Clock, '/clock', 10)
        self.cap = self.node.create_client(
            GetCapabilities, '/platform/time_sync/capabilities'
        )
        self.process = subprocess.Popen(
            [
                'ros2',
                'run',
                'easyfly_time_sync',
                'time_sync_node',
                '--ros-args',
                '-p',
                f'use_sim_time:={str(self.sim).lower()}',
                '-p',
                'max_age:=0.25',
                '-p',
                'queue_size:=2',
                '-p',
                'max_image_bytes:=1024',
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.until(lambda: self.cap.service_is_ready(), 6)
        self.until(
            lambda: all(p.get_subscription_count() for p in self.inputs), 3
        )
        if self.sim:
            self.until(lambda: self.clock.get_subscription_count() >= 2, 3)
            self.set_clock(10)
            self.until(
                lambda: self.node.get_clock().now().nanoseconds == 10000000000,
                2,
                lambda: self.set_clock(10),
            )

    def cleanup(self):
        if hasattr(self, 'process') and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=2)
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.shutdown()

    def until(self, predicate, timeout=2, tick=None):
        deadline = time.monotonic() + timeout
        last_tick = 0
        while time.monotonic() < deadline:
            if tick and time.monotonic() - last_tick >= 0.02:
                tick()
                last_tick = time.monotonic()
            self.executor.spin_once(timeout_sec=0.015)
            if predicate():
                return
        self.fail(
            f'pairer condition not reached: status={self.status}, '
            f'outputs={[len(v) for v in self.images]}, '
            f'clock={self.node.get_clock().now().nanoseconds}, process={self.process.poll()}'
        )

    def set_clock(self, seconds):
        message = Clock()
        message.clock.sec, message.clock.nanosec = divmod(
            round(seconds * 1e9), 1000000000
        )
        self.clock.publish(message)

    def image(self, offset=0):
        message = Image()
        stamp = self.node.get_clock().now().nanoseconds + int(offset * 1e9)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(
            stamp, 1000000000
        )
        message.header.frame_id = 'camera_color_optical_frame'
        message.width, message.height, message.step = 2, 2, 2
        message.encoding = 'mono8'
        message.data = bytes([1, 2, 3, 4])
        return message

    def test_pair_keeps_both_source_headers_and_signed_difference(self):
        sources = [dict(), dict()]

        def transmit():
            first = self.image()
            second = self.image(0.02)
            for side, image in enumerate((first, second)):
                sources[side][
                    (image.header.stamp.sec, image.header.stamp.nanosec)
                ] = image
                self.inputs[side].publish(image)

        self.until(
            lambda: self.status
            and self.status.pair_valid
            and all(self.images),
            tick=transmit,
        )
        for side in (0, 1):
            received = self.images[side][0]
            original = sources[side][
                (received.header.stamp.sec, received.header.stamp.nanosec)
            ]
            self.assertEqual(received.header, original.header)
            self.assertEqual(received.data, original.data)
        first, second = self.status.first_header, self.status.second_header
        expected = (second.stamp.sec - first.stamp.sec) + (
            second.stamp.nanosec - first.stamp.nanosec
        ) / 1e9
        self.assertAlmostEqual(self.status.delta_seconds, expected)
        future = self.cap.call_async(GetCapabilities.Request())
        self.until(future.done)
        self.assertEqual(future.result().implemented, [True, False])
        self.until(lambda: self.status and not self.status.pair_valid, 2)

    def test_invalid_layout_stale_and_queue_overflow_report_drops(self):
        broken = self.image()
        broken.data = bytes([1])

        def invalid_frames():
            self.inputs[0].publish(broken)
            self.inputs[1].publish(self.image(-2))

        self.until(
            lambda: self.status
            and self.status.dropped_first
            and self.status.dropped_second,
            tick=invalid_frames,
        )
        for _ in range(4):
            self.inputs[0].publish(self.image())
            self.executor.spin_once(timeout_sec=0.02)
        self.until(lambda: self.status and self.status.dropped_first >= 3)
        self.assertLessEqual(self.status.queued_first, 2)
        self.assertEqual(self.status.paired_count, 0)
        self.assertFalse(any(self.images))

    def test_clock_pause_and_rollback_cannot_keep_old_pair_ready(self):
        reference = [10.1]
        sent = []

        def pulse():
            reference[0] += 0.01
            self.set_clock(reference[0])
            first = self.image()
            first.header.stamp.sec, first.header.stamp.nanosec = divmod(
                round(reference[0] * 1e9), 1000000000
            )
            sent.append(first)
            self.inputs[0].publish(first)
            self.inputs[1].publish(first)

        self.until(lambda: self.status and self.status.pair_valid, tick=pulse)
        first = sent[-1]
        self.until(lambda: self.status and not self.status.pair_valid, 2)
        old_count = self.status.paired_count
        self.inputs[0].publish(first)
        self.inputs[1].publish(first)
        self.until(
            lambda: self.status
            and self.status.dropped_first
            and self.status.dropped_second
        )
        self.assertEqual(self.status.paired_count, old_count)
        self.set_clock(5)
        self.until(
            lambda: self.status
            and self.status.header.stamp.sec == 5
            and not self.status.pair_valid,
            tick=lambda: self.set_clock(5),
        )
        reference[0] = 5
        self.until(
            lambda: self.status
            and self.status.pair_valid
            and self.status.paired_count > old_count,
            tick=pulse,
        )


if __name__ == '__main__':
    unittest.main()

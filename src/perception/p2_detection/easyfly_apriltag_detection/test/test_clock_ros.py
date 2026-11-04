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
from pathlib import Path
import rclpy
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, CameraInfo
from easyfly_msgs.msg import (
    TagObservationArray,
    TagObservation,
    LandingTarget,
    SingleTrackState,
)
from easyfly_msgs.srv import ManageSingleTrack


class ClockRos(unittest.TestCase):
    def test_zero_paused_and_regressed_clock_then_recovery(self):
        package = Path(__file__).resolve().parents[1].name
        tag = package == 'easyfly_apriltag_detection'
        landing = package == 'easyfly_landing_detection'
        name = (
            'apriltag_detection'
            if tag
            else 'landing_detection'
            if landing
            else 'single_object_tracking'
        )
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        node = rclpy.create_node('clock_test')
        self.addCleanup(node.destroy_node)
        clock = node.create_publisher(Clock, '/clock', 10)
        outputs = []
        kind = (
            TagObservationArray
            if tag
            else LandingTarget
            if landing
            else SingleTrackState
        )
        topic = (
            '/tag_observations'
            if tag
            else '/landing_target'
            if landing
            else '/single_track/state'
        )
        node.create_subscription(kind, topic, outputs.append, 10)
        pub = node.create_publisher(
            TagObservationArray if landing else Image,
            '/tag_observations' if landing else '/camera/image_raw',
            10,
        )
        info = (
            node.create_publisher(CameraInfo, '/camera/camera_info', 10)
            if tag
            else None
        )
        manage = (
            node.create_client(ManageSingleTrack, '/single_track/manage')
            if not tag and not landing
            else None
        )
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        log = tempfile.TemporaryFile(mode='w+')
        self.addCleanup(log.close)
        if tag:
            subprocess.run(
                ['ros2', 'run', package, 'tag_fixture', tmp.name + '/image'],
                check=True,
            )
            with open(tmp.name + '/image', 'rb') as file:
                pixels = file.read()
        elif not landing:
            pixels = bytearray([30]) * (200 * 160)
            for y in range(40):
                for x in range(40):
                    pixels[(50 + y) * 200 + 60 + x] = (
                        (x * 127 + y * 53) ^ (x * y * 13)
                    ) % 256
        args = [
            'ros2',
            'run',
            package,
            name + '_node',
            '--ros-args',
            '-p',
            'use_sim_time:=true',
        ]
        if tag:
            args += ['-p', 'tag_size_m:=0.2']
        if landing:
            args += ['-p', 'selected_tag_id:=7']
        process = subprocess.Popen(
            args, stdout=log, stderr=log, start_new_session=True
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

        def stamp(t):
            n = round(t * 1e9)
            return Time(sec=n // 1000000000, nanosec=n % 1000000000)

        def valid():
            return bool(
                outputs
                and outputs[-1].valid
                and (not tag or any(o.valid for o in outputs[-1].observations))
            )

        def feed(t, ahead=0.0, info_ahead=0.0, expected=True):
            clock.publish(Clock(clock=stamp(t)))
            spin(0.08)
            if landing:
                m = TagObservationArray(valid=True)
                m.header.frame_id = 'camera_color_optical_frame'
                m.header.stamp = stamp(t + ahead)
                o = TagObservation(tag_id=7, tag_size_m=0.2, valid=True)
                o.header = m.header
                o.pose.position.z = 2.0
                o.pose.orientation.w = 1.0
                m.observations = [o]
            else:
                m = Image(
                    height=480 if tag else 160,
                    width=640 if tag else 200,
                    encoding='mono8',
                    step=640 if tag else 200,
                    data=pixels,
                )
                m.header.frame_id = 'camera_color_optical_frame'
                m.header.stamp = stamp(t + ahead)
                if tag:
                    c = CameraInfo(
                        height=480,
                        width=640,
                        distortion_model='plumb_bob',
                        k=[
                            600.0,
                            0.0,
                            320.0,
                            0.0,
                            600.0,
                            240.0,
                            0.0,
                            0.0,
                            1.0,
                        ],
                        d=[0.0] * 5,
                    )
                    c.header.frame_id = m.header.frame_id
                    c.header.stamp = stamp(t + info_ahead)
                    info.publish(c)
            pub.publish(m)
            spin(0.06)
            if manage:
                request = ManageSingleTrack.Request(
                    operation='initialize',
                    image_header=m.header,
                    roi_x=0.3,
                    roi_y=0.3125,
                    roi_width=0.2,
                    roi_height=0.25,
                )
                future = manage.call_async(request)
                end = time.monotonic() + 3
                while not future.done() and time.monotonic() < end:
                    spin(0.01)
                self.assertTrue(future.done())
                self.assertEqual(future.result().accepted, expected)
            spin(0.15)

        spin(0.4)
        self.assertFalse(valid())  # no ROS clock supplied
        if manage:
            self.assertTrue(manage.wait_for_service(timeout_sec=3))
        feed(100.0)
        self.assertTrue(valid())
        spin(0.65)
        self.assertFalse(valid())  # wall receipt + ROS-progress watchdog
        feed(100.1)
        self.assertTrue(valid())
        clock.publish(Clock(clock=stamp(50.0)))
        spin(0.15)
        self.assertFalse(valid())
        feed(50.1)
        self.assertTrue(valid())  # old high timestamp watermark was cleared
        feed(50.2, ahead=0.069, info_ahead=0.036)
        self.assertTrue(valid())
        feed(50.3, ahead=0.201, expected=False)
        self.assertFalse(valid())
        if tag:
            feed(50.4, ahead=0.069, info_ahead=0.201, expected=False)
            self.assertFalse(valid())


if __name__ == '__main__':
    unittest.main()

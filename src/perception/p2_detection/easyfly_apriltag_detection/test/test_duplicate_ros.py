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

import copy
import os
import signal
import subprocess
import tempfile
import time

import pytest
import rclpy
from builtin_interfaces.msg import Time
from easyfly_msgs.msg import TagObservationArray
from easyfly_msgs.srv import GetCapabilities
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image


@pytest.mark.parametrize(
    'case',
    [
        'identical',
        'ttl',
        'frame',
        'width',
        'height',
        'encoding',
        'endian',
        'step',
        'pixel',
        'reverse',
        'calibration',
        'model',
        'frozen',
        'oversize',
    ],
)
def test_exact_duplicate_requires_original_live_evidence(case):
    rclpy.init()
    node = rclpy.create_node('tag_duplicate_peer')
    images = node.create_publisher(Image, '/camera/image_raw', 10)
    infos = node.create_publisher(CameraInfo, '/camera/camera_info', 10)
    clocks = node.create_publisher(Clock, '/clock', 10)
    outputs = []
    node.create_subscription(
        TagObservationArray, '/tag_observations', outputs.append, 10
    )
    capability = node.create_client(
        GetCapabilities, '/platform/apriltag/capabilities'
    )
    with tempfile.TemporaryDirectory() as directory, tempfile.TemporaryFile(
        mode='w+'
    ) as log:
        subprocess.run(
            [
                'ros2',
                'run',
                'easyfly_apriltag_detection',
                'tag_fixture',
                directory + '/image',
            ],
            check=True,
        )
        with open(directory + '/image', 'rb') as stream:
            pixels = stream.read()
        process = subprocess.Popen(
            [
                'ros2',
                'run',
                'easyfly_apriltag_detection',
                'apriltag_detection_node',
                '--ros-args',
                '-p',
                'tag_size_m:=0.2',
                '-p',
                'use_sim_time:=true',
            ],
            stdout=log,
            stderr=log,
            start_new_session=True,
        )

        def pump(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.005)

        def clock(nanoseconds):
            clocks.publish(Clock(clock=Time(sec=100, nanosec=nanoseconds)))
            pump(0.025)

        def capabilities():
            future = capability.call_async(GetCapabilities.Request())
            end = time.monotonic() + 2
            while not future.done() and time.monotonic() < end:
                pump(0.01)
            assert future.done()
            return future.result()

        def ready():
            return capabilities().ready[0]

        try:
            assert capability.wait_for_service(timeout_sec=5)
            end = time.monotonic() + 5

            # All inputs are volatile: a clock/calibration sent before matching
            # is lost even when the image subscription is already discovered.
            def inputs_matched():
                return (
                    images.get_subscription_count() > 0
                    and infos.get_subscription_count() > 0
                    and clocks.get_subscription_count() > 0
                    and node.count_publishers('/tag_observations') > 0
                )

            while not inputs_matched() and time.monotonic() < end:
                pump(0.01)
            assert inputs_matched(), 'detector input/output discovery incomplete'
            # Matching is not delivery. A reply stamped by the real detector
            # proves its TimeSource received an input before the one-shot frame.
            end = time.monotonic() + 5
            while True:
                clocks.publish(Clock(clock=Time(sec=99)))
                pump(0.025)
                if capabilities().header.stamp.sec == 99:
                    break
                assert time.monotonic() < end, 'detector did not receive startup clock'
            clock(0)
            assert capabilities().header.stamp.sec == 100
            image = Image(
                height=480, width=640, encoding='mono8', step=640, data=pixels
            )
            image.header.frame_id = 'camera_color_optical_frame'
            image.header.stamp = Time(sec=100)
            info = CameraInfo(
                height=480,
                width=640,
                distortion_model='plumb_bob',
                k=[600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0],
                d=[0.0] * 5,
            )
            info.header = copy.deepcopy(image.header)
            infos.publish(info)
            pump(0.05)
            outputs.clear()
            images.publish(image)
            end = time.monotonic() + 0.25
            while not any(m.valid for m in outputs) and time.monotonic() < end:
                pump(0.005)
            assert outputs and outputs[-1].valid
            assert any(o.valid for o in outputs[-1].observations)
            assert ready()
            outputs.clear()
            changed = copy.deepcopy(image)
            if case in ('identical', 'ttl'):
                clock(10_000_000)
                images.publish(changed)
                pump(0.05)
                assert not outputs, 'exact live duplicate must publish nothing'
                assert ready()
                if case == 'ttl':
                    end = time.monotonic() + 0.65
                    tick = 10_000_000
                    while time.monotonic() < end:
                        # Keep ROS time progressing and the current calibration fresh;
                        # only the original image receipt/result evidence may expire.
                        tick += 1_000_000
                        clock(tick)
                        info.header.stamp = Time(sec=100, nanosec=tick)
                        infos.publish(info)
                        images.publish(image)
                        pump(0.025)
                    assert outputs and not outputs[-1].valid
                    assert (
                        not ready()
                    ), 'duplicate receipts must not extend the original TTL'
                return
            if case == 'frame':
                changed.header.frame_id = 'map'
            elif case == 'width':
                changed.width -= 1
            elif case == 'height':
                changed.height -= 1
            elif case == 'encoding':
                changed.encoding = 'rgb8'
            elif case == 'endian':
                changed.is_bigendian = 1
            elif case == 'step':
                changed.step += 1
            elif case == 'pixel':
                changed.data[0] ^= 1
            elif case == 'reverse':
                changed.header.stamp.sec -= 1
            elif case in ('calibration', 'model'):
                info.k[0] = 0.0 if case == 'calibration' else 650.0
                infos.publish(info)
                pump(0.04)
                info.k[0] = 600.0
                infos.publish(info)
                pump(0.04)
            elif case == 'frozen':
                pump(0.6)
                clock(10_000_000)
            elif case == 'oversize':
                changed.width = 4_000_000_000
            images.publish(changed)
            pump(0.07)
            assert outputs and not outputs[-1].valid
            assert not ready()
            # Returning to the exact old input cannot resurrect an invalid generation.
            outputs.clear()
            images.publish(image)
            pump(0.07)
            assert outputs and not outputs[-1].valid
            assert not ready()
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            log.seek(0)
            print(log.read())
            node.destroy_node()
            rclpy.shutdown()

# Copyright 2026 EasyFly Project
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

"""Exercise real two-domain native relays with exact source data and invalid-time inputs."""

import copy
import json
import os
import signal
import subprocess
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu


def test_native_sensor_identity_source_expiry_and_clock_rollback(tmp_path):
    """Relay original fields while rejecting foreign frames, duplicate/stale time and rollback."""
    contexts, nodes, executors = [], [], []
    for domain, name in [(214, 'sensor_source'), (215, 'sensor_destination')]:
        context = Context()
        context.init(domain_id=domain)
        node = rclpy.create_node(name, context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        contexts.append(context)
        nodes.append(node)
        executors.append(executor)
    clocks, images, imus = [], [], []
    source, destination = nodes
    clock = source.create_publisher(Clock, '/clock', qos_profile_sensor_data)
    imu = source.create_publisher(Imu, '/fleet/alpha/livox/imu', qos_profile_sensor_data)
    image = source.create_publisher(
        Image, '/fleet/alpha/camera/color/image_raw', qos_profile_sensor_data)
    destination.create_subscription(
        Clock, '/clock', lambda m: clocks.append(m.clock), qos_profile_sensor_data)
    destination.create_subscription(
        Imu, '/livox/imu', lambda m: imus.append(m), qos_profile_sensor_data)
    destination.create_subscription(
        Image, '/camera/color/image_raw', lambda m: images.append(m), qos_profile_sensor_data)
    result = tmp_path / 'statistics.json'
    log = (tmp_path / 'native.log').open('w')
    process = subprocess.Popen([os.environ['NATIVE_SENSOR_BOUNDARY'], 'alpha', '214', '215',
                                str(result)], stdout=log, stderr=subprocess.STDOUT)

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            for executor in executors:
                executor.spin_once(timeout_sec=.001)
            assert process.poll() is None

    def wait(predicate):
        deadline = time.monotonic() + 5
        while not predicate():
            assert time.monotonic() < deadline
            pump(.01)

    def clock_to(seconds, nanos=0):
        message = Clock()
        message.clock.sec, message.clock.nanosec = seconds, nanos
        clock.publish(message)
        wait(lambda: clocks and clocks[-1] == message.clock)

    try:
        wait(lambda: imu.get_subscription_count() == 1 and image.get_subscription_count() == 1 and
             clock.get_subscription_count() == 1 and
             destination.count_publishers('/clock') == 1)
        pump(.3)
        clock_to(10, 20000000)
        message = Imu()
        message.header.frame_id = 'alpha/livox_imu_frame'
        message.header.stamp.sec, message.header.stamp.nanosec = 10, 10000000
        message.orientation.w = 1.
        message.angular_velocity.x = .12345
        message.linear_acceleration.z = 9.81
        message.orientation_covariance[0] = .004
        imu.publish(message)
        frame = Image()
        frame.header = copy.deepcopy(message.header)
        frame.header.frame_id = 'alpha/camera_color_optical_frame'
        frame.width, frame.height, frame.step, frame.encoding = 2, 2, 6, 'rgb8'
        frame.data = list(range(12))
        image.publish(frame)
        wait(lambda: len(imus) == 1 and len(images) == 1)
        expected = copy.deepcopy(message)
        expected.header.frame_id = 'livox_imu_frame'
        assert imus[0] == expected
        expected_image = copy.deepcopy(frame)
        expected_image.header.frame_id = 'camera_color_optical_frame'
        assert images[0] == expected_image
        imu.publish(message)
        foreign = copy.deepcopy(message)
        foreign.header.stamp.nanosec = 15000000
        foreign.header.frame_id = 'bravo/livox_imu_frame'
        imu.publish(foreign)
        future = copy.deepcopy(message)
        future.header.stamp.sec = 12
        imu.publish(future)
        invalid = copy.deepcopy(message)
        invalid.header.stamp.nanosec = 1000000000
        imu.publish(invalid)
        pump(.2)
        assert len(imus) == 1
        pump(1.05)
        stale_clock = copy.deepcopy(message)
        stale_clock.header.stamp.nanosec = 19000000
        imu.publish(stale_clock)
        duplicate_clock = Clock()
        duplicate_clock.clock.sec, duplicate_clock.clock.nanosec = 10, 20000000
        clock.publish(duplicate_clock)
        pump(.05)
        imu.publish(stale_clock)
        pump(.15)
        assert len(imus) == 1
        clock_to(10, 40000000)
        recovered = copy.deepcopy(message)
        recovered.header.stamp.nanosec = 30000000
        imu.publish(recovered)
        wait(lambda: len(imus) == 2)
        clock_to(9)
        clock_to(11)
        after_rollback = copy.deepcopy(message)
        after_rollback.header.stamp.sec, after_rollback.header.stamp.nanosec = 10, 990000000
        imu.publish(after_rollback)
        pump(.2)
        assert len(imus) == 2
        process.send_signal(signal.SIGINT)
        assert process.wait(timeout=5) == 0
        stats = json.loads(result.read_text())
        assert stats['clock_fault'] and stats['sensors']['/livox/imu']['rejected'] >= 6
        assert stats['sensors']['/livox/imu']['forwarded'] == 2
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=5)
        log.close()
        for executor, node, context in zip(executors, nodes, contexts):
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
            context.shutdown()

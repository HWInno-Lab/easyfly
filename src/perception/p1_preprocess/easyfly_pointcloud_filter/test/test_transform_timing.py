# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Real node regression: delayed TF must not rotate a scan at another time."""
import os
import signal
import struct
import subprocess
import time

from geometry_msgs.msg import TransformStamped
from livox_ros_driver2.msg import CustomMsg, CustomPoint
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage


def test_measured_transform_and_bounded_wait(tmp_path):
    domain = 160 + os.getpid() % 60
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node('cloud_transform_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    clock = node.create_publisher(Clock, '/clock', 10)
    transforms = node.create_publisher(TFMessage, '/tf', 10)
    clouds = node.create_publisher(CustomMsg, '/livox/lidar', 10)
    observed = []
    node.create_subscription(PointCloud2, '/pointcloud_filtered', observed.append, 10)
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
    log = (tmp_path / 'filter.log').open('w')
    process = subprocess.Popen(
        [os.environ['POINTCLOUD_FILTER_EXECUTABLE'], '--ros-args', '-p', 'use_sim_time:=true'],
        env=env, stdout=log, stderr=subprocess.STDOUT)
    current = [10.0]

    def stamp(message, value):
        ns = round(value * 1e9)
        message.sec, message.nanosec = divmod(ns, 10**9)

    def pump(seconds, advance=False):
        deadline = time.monotonic() + seconds
        previous = time.monotonic()
        while time.monotonic() < deadline:
            now = time.monotonic()
            if advance:
                current[0] += (now - previous) * 0.5
            previous = now
            msg = Clock()
            stamp(msg.clock, current[0])
            clock.publish(msg)
            executor.spin_once(timeout_sec=0.01)
            assert process.poll() is None, (tmp_path / 'filter.log').read_text()

    def transform(value, quarter_turn=False):
        msg = TransformStamped()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'livox_frame'
        stamp(msg.header.stamp, value)
        msg.transform.rotation.w = 2**-0.5 if quarter_turn else 1.0
        msg.transform.rotation.z = 2**-0.5 if quarter_turn else 0.0
        transforms.publish(TFMessage(transforms=[msg]))

    def cloud(value):
        msg = CustomMsg()
        msg.header.frame_id = 'livox_frame'
        stamp(msg.header.stamp, value)
        msg.timebase = round(value * 1e9)
        point = CustomPoint()
        point.x, point.reflectivity = 1.0, 10
        msg.points, msg.point_num = [point], 1
        clouds.publish(msg)

    try:
        deadline = time.monotonic() + 8
        while (clouds.get_subscription_count() == 0 or
               transforms.get_subscription_count() == 0 or
               node.count_publishers('/pointcloud_filtered') == 0):
            assert time.monotonic() < deadline
            pump(0.05)
        pump(0.2)
        for _ in range(4):
            transform(9.9)
            pump(0.03)
        cloud(10.0)
        pump(0.25, advance=True)
        assert observed == [], 'An old TF was incorrectly used for a newer scan'
        transform(10.0, quarter_turn=True)
        pump(0.2)
        assert len(observed) == 1
        assert observed[0].header.frame_id == 'odom'
        assert observed[0].header.stamp.sec == 10
        assert observed[0].header.stamp.nanosec == 0
        assert struct.unpack_from('<ffff', observed[0].data) == pytest.approx((0, 1, 0, 10))
        assert [(f.name, f.offset, f.datatype, f.count) for f in observed[0].fields] == [
            ('x', 0, 7, 1), ('y', 4, 7, 1), ('z', 8, 7, 1), ('intensity', 12, 7, 1)]
        assert observed[0].point_step == observed[0].row_step == 16
        assert observed[0].width == observed[0].height == 1

        # A frozen simulation clock must not keep a missing-transform scan forever.
        current[0] = 10.5
        pump(0.1)
        cloud(10.5)
        pump(1.15)
        transform(10.5)
        pump(0.15)
        assert len(observed) == 1
        cloud(10.5)  # Replayed expired scan must not acquire a fresh receipt TTL.
        pump(0.1)
        assert len(observed) == 1
        current[0] = 10.6
        pump(0.05)
        transform(10.6)
        cloud(10.6)
        pump(0.2)
        assert len(observed) == 2
        assert struct.unpack_from('<fff', observed[-1].data) == pytest.approx((1, 0, 0))

        cloud(10.6)  # Duplicate and out-of-order scans cannot republish.
        cloud(10.55)
        pump(0.1)
        assert len(observed) == 2

        # Zero time must never mean "latest", and invalid-age data stays unpublished.
        for value in (0.0, 8.0, 12.0):
            cloud(value)
        pump(0.15)
        assert len(observed) == 2

        # Drop pending data from the old epoch and admit fresh data after a rewind.
        current[0] = 10.7
        pump(0.05)
        cloud(10.7)
        pump(0.05)
        current[0] = 5.0
        pump(0.2)
        for _ in range(3):
            transform(5.0, quarter_turn=True)
            pump(0.03)
        cloud(5.0)
        pump(0.2)
        assert len(observed) == 3
        assert observed[-1].header.stamp.sec == 5
        assert struct.unpack_from('<fff', observed[-1].data) == pytest.approx((0, 1, 0))
    finally:
        process.send_signal(signal.SIGINT)
        try:
            code = process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
            code = -9
        log.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown(context=context)
        assert code == 0, (tmp_path / 'filter.log').read_text()

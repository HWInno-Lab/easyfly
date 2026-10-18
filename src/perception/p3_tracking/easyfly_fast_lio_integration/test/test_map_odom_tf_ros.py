# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""Actual integration node TF composition, smoothing, source stamp and frame settings."""
import math
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage


@pytest.mark.parametrize('alpha,custom_frames', [(.1, False), (1., True)])
def test_map_odom_tf_composition_and_smoothing(tmp_path, alpha, custom_frames):
    rclpy.init()
    peer = rclpy.create_node('alg81_map_odom_tf_peer')
    odom_pub = peer.create_publisher(Odometry, '/odom', 10)
    slam_pub = peer.create_publisher(Odometry, '/Odometry', 10)
    dynamic, static = [], []
    peer.create_subscription(TFMessage, '/tf', lambda m: dynamic.extend(m.transforms), 10)
    peer.create_subscription(TFMessage, '/tf_static', lambda m: static.extend(m.transforms),
                             QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    map_frame, odom_frame, slam_frame = ('test_map', 'test_odom', 'test_slam') if custom_frames else (
        'map', 'odom', 'camera_init')
    log_path = tmp_path / 'tf.log'
    process = None

    def spin(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            assert process.poll() is None, log_path.read_text()
            rclpy.spin_once(peer, timeout_sec=.01)

    def wait(predicate):
        until = time.monotonic() + 6.
        while not predicate():
            assert time.monotonic() < until, log_path.read_text()
            spin(.02)

    def pose(stamp, xyz, yaw):
        msg = Odometry()
        msg.header.stamp.sec = stamp
        msg.header.frame_id = slam_frame
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = xyz
        msg.pose.pose.orientation.z = math.sin(yaw/2)
        msg.pose.pose.orientation.w = math.cos(yaw/2)
        return msg

    def receive(stamp):
        wait(lambda: any(t.header.stamp.sec == stamp for t in dynamic))
        tf = next(t for t in dynamic if t.header.stamp.sec == stamp)
        assert tf.header.frame_id == map_frame and tf.child_frame_id == odom_frame
        assert tf.header.stamp.nanosec == 0
        return tf.transform

    try:
        with log_path.open('w') as log:
            executable = Path(get_package_prefix('easyfly_fast_lio_integration')) / (
                'lib/easyfly_fast_lio_integration/map_odom_tf')
            command = [str(executable), '--ros-args', '-p', 'use_sim_time:=false']
            if custom_frames:
                command += ['-p', 'smooth_alpha:=1.0', '-p', 'map_frame:='+map_frame,
                            '-p', 'odom_frame:='+odom_frame, '-p', 'slam_frame:='+slam_frame]
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            wait(lambda: odom_pub.get_subscription_count() and slam_pub.get_subscription_count() and static)
            identity = next(t for t in static if t.child_frame_id == slam_frame)
            assert identity.header.frame_id == map_frame
            assert identity.transform.rotation.w == 1.
            assert identity.transform.translation.x == 0.
            slam_pub.publish(pose(101, (4., 6., 8.), .2))
            spin(.15)
            assert not dynamic  # No map->odom estimate before odom evidence.
            odom_pub.publish(pose(101, (1., 2., 3.), .2))
            spin(.1)
            slam_pub.publish(pose(102, (4., 6., 8.), .2))
            first = receive(102)
            assert (first.translation.x, first.translation.y, first.translation.z) == pytest.approx((3., 4., 5.))
            slam_pub.publish(pose(103, (6., 10., 12.), .2))
            second = receive(103)
            expected = (3.+alpha*2., 4.+alpha*4., 5.+alpha*4.)
            assert (second.translation.x, second.translation.y, second.translation.z) == pytest.approx(expected)
            slam_pub.publish(pose(104, (6., 10., 12.), 1.2))
            third = receive(104)
            # Analytic R(1 rad) composition and slerp from identity.
            target = (6.-math.cos(1.)+2.*math.sin(1.), 10.-math.sin(1.)-2.*math.cos(1.), 9.)
            xyz = tuple(a+alpha*(b-a) for a, b in zip(expected, target))
            assert (third.translation.x, third.translation.y, third.translation.z) == pytest.approx(xyz)
            assert third.rotation.z == pytest.approx(math.sin(alpha/2))
            assert third.rotation.w == pytest.approx(math.cos(alpha/2))
            process.send_signal(signal.SIGINT)
            assert process.wait(timeout=4.) == 0, log_path.read_text()
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait(timeout=2.)
        peer.destroy_node()
        rclpy.shutdown()

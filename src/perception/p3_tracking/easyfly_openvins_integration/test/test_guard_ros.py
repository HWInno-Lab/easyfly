# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Actual guard-process fault tests. Synthetic poses test the contract, not VIO."""

from pathlib import Path
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from easyfly_msgs.msg import LocalizationStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu


def timestamp(message, seconds):
    value = round(seconds * 1e9)
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(value, 10**9)
    return message


class Harness:
    def __init__(self, directory):
        self.directory = directory
        rclpy.init()
        self.node = rclpy.create_node('guard_contract_observer')
        clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.clock = self.node.create_publisher(Clock, '/clock', clock_qos)
        self.image = self.node.create_publisher(Image, '/camera/color/image_raw', 32)
        self.info = self.node.create_publisher(CameraInfo, '/camera/color/camera_info', 32)
        self.imu = self.node.create_publisher(Imu, '/livox/imu', 1000)
        self.raw_pose = self.node.create_publisher(
            PoseWithCovarianceStamped, '/openvins/raw/poseimu', 10)
        self.status, self.images, self.poses = None, [], []
        self.node.create_subscription(
            LocalizationStatus, '/localization/openvins/status',
            lambda m: setattr(self, 'status', m),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.guarded_image = self.node.create_subscription(
            Image, '/openvins/input/camera', self.images.append, 32)
        self.node.create_subscription(PoseWithCovarianceStamped, '/localization/openvins/pose',
                                      self.poses.append, 100)
        self.process = None

    def start(self, suffix='first', receipt_timeout=2.0, pause_executor=False,
              defer_calibration=False):
        prefix = Path(get_package_prefix('easyfly_openvins_integration'))
        config = Path(get_package_share_directory('easyfly_openvins_integration')) / \
            'config/guard.yaml'
        executable = [str(prefix / 'lib/easyfly_openvins_integration/openvins_guard')]
        if pause_executor:
            # Pause only Python's main executor. SIGSTOP also freezes DDS receive
            # threads and tests transport retransmission, not delivered backlog.
            self.pause_marker = self.directory / 'executor-paused'
            wrapper = self.directory / 'pause_executor.py'
            wrapper.write_text(
                'import signal, time\nfrom pathlib import Path\n'
                'from easyfly_openvins_integration.node import main\n'
                'def pause(*args):\n'
                f'    Path({str(self.pause_marker)!r}).write_text("paused")\n'
                '    time.sleep(.35)\n'
                'signal.signal(signal.SIGUSR1, pause)\nmain()\n')
            executable = [sys.executable, str(wrapper)]
        with (self.directory / (suffix + '.log')).open('w') as log:
            self.process = subprocess.Popen(executable + [
                '--ros-args',
                '--params-file', str(config), '-p', 'use_sim_time:=true',
                '-p', 'defer_camera_until_calibration:='+str(defer_calibration).lower(),
                '-p', f'receipt_timeout:={receipt_timeout}',
                '-r', '__ns:=/openvins'], stdout=log, stderr=subprocess.STDOUT)
        self.until(lambda: self.clock.get_subscription_count() and
                   (defer_calibration or self.image.get_subscription_count()) and
                   self.imu.get_subscription_count() and
                   self.info.get_subscription_count() and self.raw_pose.get_subscription_count())
        self.until(lambda: self.node.count_publishers('/openvins/input/camera') > 0 and
                   self.status is not None)

    def spin(self, seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            assert self.process.poll() is None
            rclpy.spin_once(self.node, timeout_sec=.01)

    def until(self, predicate):
        until = time.monotonic() + 8
        while not predicate():
            assert time.monotonic() < until, (
                self.status, [m.header.stamp.nanosec for m in self.images])
            self.spin(.02)

    def set_clock(self, seconds):
        msg = Clock()
        msg.clock.sec, msg.clock.nanosec = divmod(round(seconds * 1e9), 10**9)
        self.clock.publish(msg)
        self.spin(.08)

    def image_at(self, seconds, marker=17, include_info=True):
        image = timestamp(Image(), seconds)
        image.header.frame_id = 'camera_color_optical_frame'
        image.width, image.height, image.step = 640, 480, 1920
        image.encoding = 'rgb8'
        image.data = bytes([marker]) * (1920 * 480)
        info = timestamp(CameraInfo(), seconds)
        info.header.frame_id = image.header.frame_id
        info.width, info.height = 640, 480
        info.distortion_model, info.d = 'plumb_bob', [0.] * 5
        f = 337.3570508552852
        info.k = [f, 0., 320.5, 0., f, 240.5, 0., 0., 1.]
        if include_info:
            self.info.publish(info)
        self.image.publish(image)
        self.spin(.025)
        return image

    def feed(self):
        self.set_clock(10.15)
        for i in range(31):
            msg = timestamp(Imu(), 10 + .005 * i)
            msg.header.frame_id = 'livox_imu_frame'
            msg.linear_acceleration.z = 9.81
            self.imu.publish(msg)
        self.spin(.1)
        self.image_at(10.02)
        self.image_at(10.053)
        self.image_at(10.053, 18)
        self.image_at(10.053, 19)
        self.image_at(10.087, 20)
        self.image_at(10.12, 21)
        self.until(lambda: len(self.images) == 2)
        assert [m.header.stamp.nanosec for m in self.images] == [20_000_000, 87_000_000]
        assert bytes(self.images[0].data) == bytes([17]) * (1920 * 480)
        assert bytes(self.images[1].data) == bytes([20]) * (1920 * 480)
        self.until(lambda: self.status and self.status.state == LocalizationStatus.INITIALIZING)
        assert self.status.camera_rejected == 3

    def valid_pose(self):
        msg = timestamp(PoseWithCovarianceStamped(), 10.087)
        msg.header.frame_id = 'global'
        msg.pose.pose.position.x = 1.0
        msg.pose.pose.orientation.w = 1.0
        for i in range(6):
            msg.pose.covariance[i * 7] = .01
        self.raw_pose.publish(msg)
        self.until(lambda: self.status and self.status.pose_valid)
        return msg

    def stop(self):
        if self.process and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            assert self.process.wait(timeout=5) == 0

    def close(self):
        try:
            self.stop()
        finally:
            if self.process and self.process.poll() is None:
                self.process.kill()
                self.process.wait(timeout=3)
            self.node.destroy_node()
            rclpy.shutdown()


@pytest.fixture
def harness(tmp_path, request):
    h = Harness(tmp_path)
    try:
        h.start(**getattr(request, 'param', {}))
        yield h
    finally:
        h.close()


def test_duplicate_groups_paused_clock_stale_and_new_session(harness):
    h = harness
    h.feed()
    h.valid_pose()
    h.until(lambda: len(h.poses) == 1)
    assert h.poses[0].header.frame_id == 'openvins_odom'
    assert h.poses[0].header.stamp.nanosec == 87_000_000
    assert not h.status.velocity_valid and not h.status.tracking_quality_available
    session = h.status.session_id
    h.until(lambda: h.status.state == LocalizationStatus.STALE)
    assert not h.status.pose_valid
    h.set_clock(9.)
    h.until(lambda: h.status.state == LocalizationStatus.RESET_REQUIRED)
    assert h.status.generation == 1
    h.set_clock(12.)
    h.spin(.1)
    assert not h.status.pose_valid and len(h.poses) == 1
    h.stop()
    h.start('restarted')
    h.until(lambda: h.status.session_id != session)
    assert h.status.generation == 0 and not h.status.pose_valid


@pytest.mark.parametrize('harness', [dict(receipt_timeout=.5, pause_executor=True)], indirect=True)
def test_delivered_dependency_backlog_is_drained_before_image_burst(harness):
    h = harness
    h.set_clock(10.45)
    h.process.send_signal(signal.SIGUSR1)
    h.until(h.pause_marker.exists)
    for i in range(91):
        msg = timestamp(Imu(), 10.0 + .005 * i)
        msg.header.frame_id = 'livox_imu_frame'
        msg.linear_acceleration.z = 9.81
        h.imu.publish(msg)
    for i in range(8):
        h.image_at(10.153 + .033 * i, marker=30+i)
    h.until(lambda: h.status.camera_forwarded == 7 or h.status.generation > 0)
    assert h.status.generation == 0, h.status
    h.until(lambda: len(h.images) == 7)
    assert h.status.imu_forwarded == 91
    assert h.status.camera_rejected == 0
    assert h.status.state == LocalizationStatus.INITIALIZING
    assert [m.data[0] for m in h.images[-7:]] == list(range(30, 37))


@pytest.mark.parametrize('harness', [dict(receipt_timeout=.5)], indirect=True)
def test_missing_dependencies_expire_while_image_admission_is_paused(harness):
    h = harness
    h.set_clock(10.45)
    for i in range(91):
        msg = timestamp(Imu(), 10.0 + .005 * i)
        msg.header.frame_id = 'livox_imu_frame'
        msg.linear_acceleration.z = 9.81
        h.imu.publish(msg)
    for i in range(8):
        h.image_at(10.153 + .033 * i, include_info=False)
    h.until(lambda: h.status.generation > 0)
    assert h.status.state == LocalizationStatus.RESET_REQUIRED
    assert h.status.reason == 'pending_camera_expired'
    assert not h.status.pose_valid
    assert h.status.camera_forwarded == 0


@pytest.mark.parametrize('harness', [dict(receipt_timeout=.5, defer_calibration=True)],
                         indirect=True)
def test_metadata_reader_precedes_images_but_runtime_missing_pairs_still_expire(harness):
    h = harness
    h.set_clock(10.15)
    session = h.status.session_id
    h.image_at(10.02, include_info=False)
    h.spin(.65)
    assert h.image.get_subscription_count() == 0
    assert h.status.state == LocalizationStatus.WAITING_INPUT
    assert h.status.generation == 0 and h.status.camera_forwarded == 0
    h.image_at(10.0)  # Metadata may be discovered before the first IMU reader match.
    h.spin(.1)
    for i in range(4):
        h.image_at(10.02 + .033*i)
    h.spin(.65)  # Longer than the unchanged .5 s runtime dependency timeout.
    assert h.status.generation == 0 and h.status.camera_forwarded == 0, h.status
    assert h.status.state == LocalizationStatus.WAITING_INPUT
    assert h.image.get_subscription_count() == 0
    for i in range(31):
        msg = timestamp(Imu(), 10 + .005*i)
        msg.header.frame_id = 'livox_imu_frame'
        msg.linear_acceleration.z = 9.81
        h.imu.publish(msg)
    h.until(lambda: h.image.get_subscription_count() > 0)
    h.spin(.1)
    for i in range(4):
        h.image_at(10.02 + .033*i)
    h.until(lambda: len(h.images) >= 2)
    assert h.status.session_id == session and h.status.generation == 0
    h.set_clock(10.45)
    for i in range(60):
        msg = timestamp(Imu(), 10.155 + .005*i)
        msg.header.frame_id = 'livox_imu_frame'
        msg.linear_acceleration.z = 9.81
        h.imu.publish(msg)
    for i in range(8):
        h.image_at(10.153 + .033*i, include_info=False)
    h.until(lambda: h.status.generation > 0)
    assert h.status.state == LocalizationStatus.RESET_REQUIRED
    assert h.status.reason == 'pending_camera_expired'


@pytest.mark.parametrize('fault,expected', [
    ('imu_gap', LocalizationStatus.RESET_REQUIRED),
    ('nan_pose', LocalizationStatus.FAILED),
    ('calibration', LocalizationStatus.FAILED)])
def test_faults_latch_and_never_reuse_last_output(harness, fault, expected):
    h = harness
    h.feed()
    pose = h.valid_pose()
    if fault == 'imu_gap':
        h.set_clock(10.20)
        msg = timestamp(Imu(), 10.20)
        msg.header.frame_id = 'livox_imu_frame'
        h.imu.publish(msg)
    elif fault == 'nan_pose':
        pose.pose.pose.position.x = float('nan')
        h.raw_pose.publish(pose)
    else:
        msg = timestamp(CameraInfo(), 10.15)
        msg.header.frame_id = 'camera_color_optical_frame'
        h.info.publish(msg)
    h.until(lambda: h.status.state == expected)
    assert not h.status.pose_valid and h.status.generation == 1
    previous = len(h.poses)
    pose.pose.pose.position.x = 1.
    h.raw_pose.publish(pose)
    h.spin(.12)
    assert not h.status.pose_valid and len(h.poses) == previous

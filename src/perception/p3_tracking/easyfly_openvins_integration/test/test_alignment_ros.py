# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Actual ROS executable: source matching, stale sessions and absence of global TF."""
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from easyfly_msgs.msg import LocalizationStatus, UAVState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
import numpy as np
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu

from easyfly_openvins_integration.geometry import imu_to_base, multiply, rotation, yaw_quaternion


def timestamp(message, seconds):
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(round(seconds*1e9), 10**9)
    return message


def set_pose(pose, p, q):
    pose.position.x, pose.position.y, pose.position.z = map(float, p)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)


class Harness:
    def __init__(self, directory, initial_mode='PASSTHROUGH', backend='openvins', orb_inertial=False):
        self.backend = backend
        self.require_covariance = backend != 'orbslam3' or orb_inertial
        self.frame = {'openvins':'openvins_odom', 'fast_livo2':'fastlivo_odom', 'orbslam3':'orbslam3_map'}[backend]
        self.remaps = []
        if backend != 'openvins':
            for topic in ['pose', 'status', 'base_pose', 'aligned_pose', 'alignment_status']:
                self.remaps += ['-r', '/localization/openvins/'+topic+':=/localization/fast_livo2/'+topic]
        rclpy.init(args=['--ros-args', *self.remaps])
        self.node = rclpy.create_node('alignment_contract_fixture')
        self.clock = self.node.create_publisher(Clock, '/clock', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.pose = self.node.create_publisher(PoseWithCovarianceStamped,
                                               '/localization/openvins/pose', 32)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.node.create_publisher(
            LocalizationStatus, '/localization/openvins/status', qos)
        self.reference = self.node.create_publisher(
            PoseStamped, '/mavros/local_position/pose', 100)
        self.health = self.node.create_publisher(UAVState, '/uav_state', 32)
        self.attitude = self.node.create_publisher(Imu, '/mavros/imu/data', 200)
        self.clock_mode = initial_mode
        self.clock_requests = 0

        def clock_parameters(request, response):
            assert request.names == ['timesync_mode', 'use_sim_time']
            self.clock_requests += 1
            response.values = [ParameterValue(type=4, string_value=self.clock_mode),
                               ParameterValue(type=1, bool_value=True)]
            return response

        self.service = self.node.create_service(
            GetParameters, '/mavros/time/get_parameters', clock_parameters)
        self.observed = None
        self.base, self.aligned = [], []
        self.node.create_subscription(
            LocalizationStatus, '/localization/openvins/alignment_status',
            lambda m: setattr(self, 'observed', m), qos)
        self.node.create_subscription(PoseWithCovarianceStamped,
                                      '/localization/openvins/base_pose', self.base.append, 32)
        self.node.create_subscription(PoseStamped, '/localization/openvins/aligned_pose',
                                      self.aligned.append, 32)
        prefix = Path(get_package_prefix('easyfly_openvins_integration'))
        self.log = (directory / 'alignment.log').open('w')
        self.process = subprocess.Popen([
            str(prefix / 'lib/easyfly_openvins_integration/openvins_alignment'), '--ros-args',
            '-p', 'use_sim_time:=true', '-p', 'source_backend:='+backend,
            '-p', 'source_frame:='+self.frame,
            '-p', 'require_source_covariance:='+('true' if self.require_covariance else 'false'), *self.remaps], stdout=self.log, stderr=subprocess.STDOUT)
        self.until(lambda: all(pub.get_subscription_count() for pub in (
            self.clock, self.pose, self.status, self.reference, self.attitude, self.health)))
        self.until(lambda: self.observed is not None and self.clock_requests > 0)
        self.spin(.15)

    def spin(self, seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            assert self.process.poll() is None
            rclpy.spin_once(self.node, timeout_sec=.01)

    def until(self, predicate, seconds=5):
        deadline = time.monotonic()+seconds
        while not predicate():
            assert time.monotonic() < deadline, self.observed
            self.spin(.02)

    def send(self, t, session='fixture-a', ref_shift=0., generation=0):
        clock = Clock()
        clock.clock.sec, clock.clock.nanosec = divmod(round((t+.02)*1e9), 10**9)
        self.clock.publish(clock)
        self.spin(.03)
        p, q = np.array([.1*t, .2*t, 1.]), yaw_quaternion(.1*t)
        bp, bq, _ = imu_to_base(p, q, np.eye(6)*.01, [.011, .02329, -.12412], [0, 0, 0, 1])
        aq, at = yaw_quaternion(.5), np.array([1.+ref_shift, -2., .3])
        for offset in (-.01, .01):
            ref = timestamp(PoseStamped(), t+offset)
            ref.header.frame_id = 'map'
            # Intentionally incorrect cached attitude must never enter alignment.
            set_pose(ref.pose, at+rotation(aq)@bp, [0., 0., 0., 1.])
            self.reference.publish(ref)
        self.expected_orientation = multiply(aq, bq)
        for offset in (-.02, 0., .02):
            attitude = timestamp(Imu(), t+offset)
            attitude.header.frame_id = 'base_link'
            reference_q = self.expected_orientation
            attitude.orientation.x = float(reference_q[0])
            attitude.orientation.y = float(reference_q[1])
            attitude.orientation.z = float(reference_q[2])
            attitude.orientation.w = float(reference_q[3])
            self.attitude.publish(attitude)
        health = timestamp(UAVState(), t+.02)
        health.header.frame_id = 'odom'
        health.connected = health.pose_valid = health.attitude_valid = True
        self.health.publish(health)
        msg = timestamp(PoseWithCovarianceStamped(), t)
        msg.header.frame_id = self.frame
        set_pose(msg.pose.pose, p, q)
        msg.pose.covariance = (np.eye(6)*.01).ravel().tolist()
        if not self.require_covariance:
            msg.pose.covariance = [-1.]+[0.]*35
        self.pose.publish(msg)
        status = timestamp(LocalizationStatus(), t+.02)
        status.header.frame_id, status.tracked_frame = self.frame, 'livox_imu_frame'
        status.backend_id, status.session_id = self.backend, session
        status.state, status.pose_valid, status.covariance_available = 2, True, self.require_covariance
        status.generation = generation
        status.pose_stamp = msg.header.stamp
        self.status.publish(status)
        self.spin(.08)
        return at+rotation(aq)@bp

    def ready(self):
        for i in range(7):
            expected = self.send(1+i*.08)
        self.until(lambda: self.observed.state == 2 and self.aligned)
        return expected

    def close(self):
        self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=5)
            assert self.process.returncode == 0
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
            pytest.fail('alignment process did not exit on SIGINT')
        finally:
            self.log.close()
            self.node.destroy_node()
            rclpy.shutdown()


@pytest.mark.parametrize('fault', ['session', 'generation', 'reference_jump', 'clock_rollback', 'clock_policy'])
@pytest.mark.parametrize('backend', ['openvins', 'fast_livo2', 'orbslam3'])
def test_aligned_observation_is_session_pinned_and_never_publishes_tf(tmp_path, fault, backend):
    fixture = Harness(tmp_path, backend=backend)
    try:
        expected = fixture.ready()
        msg = fixture.aligned[-1]
        np.testing.assert_allclose([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                                   expected, atol=1e-8)
        from easyfly_openvins_integration.geometry import angle_between
        q = msg.pose.orientation
        assert angle_between([q.x, q.y, q.z, q.w], fixture.expected_orientation) < 1e-7
        assert msg.header.frame_id == 'odom'
        assert fixture.base[-1].header.frame_id == fixture.frame
        if backend == 'orbslam3':
            assert fixture.base[-1].pose.covariance[0] == -1.
        assert not fixture.observed.covariance_available
        assert not fixture.observed.velocity_valid
        assert fixture.node.count_publishers('/tf') == 0
        assert fixture.node.count_publishers('/tf_static') == 0
        assert fixture.node.count_publishers('/odom') == 0
        count = len(fixture.aligned)
        fixture.spin(.65)
        assert fixture.observed.state == 3
        assert len(fixture.aligned) == count
        if fault == 'generation':
            fixture.send(1.56, generation=1)
        elif fault == 'session':
            fixture.send(1.56, session='fixture-b')
        elif fault == 'reference_jump':
            fixture.send(1.56, ref_shift=5.)
        elif fault == 'clock_policy':
            fixture.clock_mode = 'NONE'
        else:
            msg = Clock()
            msg.clock.sec = 1
            fixture.clock.publish(msg)
        fixture.until(lambda: fixture.observed.state == 4)
        fixture.send(1.64)
        assert fixture.observed.state == 4
        assert len(fixture.aligned) == count
    finally:
        fixture.close()


def test_receipt_time_reference_policy_cannot_authorize_outputs(tmp_path):
    fixture = Harness(tmp_path, initial_mode='NONE')
    try:
        for i in range(3):
            fixture.send(1+i*.08)
        assert not fixture.base and not fixture.aligned
        assert fixture.observed.reason == 'waiting_for_mavros_passthrough_clock'
        fixture.clock_mode = 'PASSTHROUGH'
        fixture.spin(.3)
        for i in range(8):
            fixture.send(1.24+i*.08)
        fixture.until(lambda: fixture.observed.state == 2 and fixture.aligned)
    finally:
        fixture.close()


def test_inertial_orb_requires_actual_covariance_and_latches_source_epoch(tmp_path):
    fixture = Harness(tmp_path, backend='orbslam3', orb_inertial=True)
    try:
        fixture.ready()
        covariance = np.array(fixture.base[-1].pose.covariance).reshape(6, 6)
        assert np.linalg.eigvalsh(covariance).min() > 0
        assert not fixture.observed.covariance_available  # Alignment uncertainty remains unknown.
        fixture.send(1.56, generation=1)
        fixture.until(lambda: fixture.observed.state == 4)
        count = len(fixture.aligned)
        fixture.send(1.64, generation=1)
        assert len(fixture.aligned) == count
    finally:
        fixture.close()

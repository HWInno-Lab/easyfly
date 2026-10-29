# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Actual ROS adapter wiring with synthetic contract fixtures; not VIO accuracy."""
import time
import pytest

from easyfly_msgs.msg import LocalizationStatus, UAVState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

from easyfly_openvins_integration.navigation_node import OpenVinsNavigation


@pytest.mark.parametrize('fault', ['session', 'publisher'])
@pytest.mark.parametrize('backend', ['openvins', 'fast_livo2', 'orbslam3'])
def test_measurement_output_stale_and_session_change(fault, backend):
    args = ['--ros-args', '-p', 'use_sim_time:=true', '-p', 'source_backend:='+backend]
    if backend != 'openvins':
        for topic in ['aligned_pose', 'alignment_status', 'navigation_status', 'navigation_state']:
            args += ['-r', '/localization/openvins/'+topic+':=/localization/fast_livo2/'+topic]
        args += ['-r', '/openvins/raw/visual_tracking:='+('/fastlivo/raw/visual_tracking' if backend=='fast_livo2' else '/orbslam3/raw/visual_tracking')]
    rclpy.init(args=args)
    adapter = OpenVinsNavigation()
    harness = rclpy.create_node('navigation_contract_inputs')
    executor = SingleThreadedExecutor()
    executor.add_node(adapter)
    executor.add_node(harness)
    odometry, statuses = [], []
    clock = harness.create_publisher(Clock, '/clock', QoSProfile(
        depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    health = harness.create_publisher(UAVState, '/uav_state', 32)
    pose = harness.create_publisher(PoseStamped, '/localization/openvins/aligned_pose', 32)
    tracks = harness.create_publisher(DiagnosticArray, '/openvins/raw/visual_tracking', 32)
    status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    source = harness.create_publisher(
        LocalizationStatus, '/localization/openvins/alignment_status', status_qos)
    harness.create_subscription(Odometry, '/odom', odometry.append, 100)
    harness.create_subscription(LocalizationStatus, '/localization/openvins/navigation_status',
                                statuses.append, status_qos)

    def spin(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)

    try:
        spin(.5)
        # Empty initial status must wait, not permanently fail before a UUID exists.
        initial = LocalizationStatus(backend_id=backend+'_alignment', tracked_frame='base_link')
        initial.header.frame_id = 'odom'
        source.publish(initial)
        spin(.1)
        assert not adapter.engine.failure
        for i in range(8):
            value = 10_000_000_000+50_000_000*i
            stamp = rclpy.time.Time(nanoseconds=value).to_msg()
            c = Clock(clock=stamp)
            clock.publish(c)
            spin(.025)
            h = UAVState(connected=True, pose_valid=True, attitude_valid=True)
            h.header.frame_id, h.header.stamp = 'odom', stamp
            health.publish(h)
            diagnostic = DiagnosticArray()
            diagnostic.header.stamp = stamp
            diagnostic.status = [DiagnosticStatus(name=backend+'/visual_tracking', values=[
                KeyValue(key='tracked_features', value='4' if i == 4 else '12'),
                KeyValue(key='initialized', value='1')])]
            tracks.publish(diagnostic)
            p = PoseStamped()
            p.header.frame_id, p.header.stamp = 'odom', stamp
            p.pose.position.x, p.pose.position.z = i*.01, 1.
            p.pose.orientation.w = 1.
            pose.publish(p)
            s = LocalizationStatus(backend_id=backend+'_alignment', session_id='fixture-session',
                                   tracked_frame='base_link', state=2, pose_valid=True)
            s.header.frame_id, s.header.stamp, s.pose_stamp = 'odom', stamp, stamp
            source.publish(s)
            spin(.05)
        assert odometry and statuses[-1].state == 2
        assert odometry[-1].header.stamp == stamp
        assert abs(odometry[-1].pose.pose.position.x-.07) < 1e-12
        assert not statuses[-1].covariance_available
        spin(.65)
        assert statuses[-1].state == LocalizationStatus.STALE
        count = len(odometry)
        spin(.15)
        assert len(odometry) == count
        if fault == 'session':
            s.session_id = 'another-session'
            source.publish(s)
            spin(.15)
            assert adapter.engine.failure == 'navigation_session_changed'
        else:
            duplicate = harness.create_publisher(
                LocalizationStatus, '/localization/openvins/navigation_status', status_qos)
            spin(.7)
            assert adapter.engine.failure == 'multiple_navigation_publishers'
            harness.destroy_publisher(duplicate)
        assert statuses[-1].state == LocalizationStatus.RESET_REQUIRED
    finally:
        executor.shutdown()
        adapter.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()

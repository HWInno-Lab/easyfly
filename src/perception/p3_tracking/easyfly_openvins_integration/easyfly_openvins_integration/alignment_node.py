# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Observation-only body pose and frozen alignment; never owns global TF or /odom."""

import math
import re
import time

from easyfly_msgs.msg import LocalizationStatus, UAVState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from .alignment import Alignment, AlignmentLimits, Pose
from .gate import Gate
from .node import ros_stamp, stamp_ns
from .reference import ReferencePairer


def values(message, now, with_covariance=False):
    if message.header.stamp.sec < 0 or message.header.stamp.nanosec >= 10**9:
        raise ValueError('malformed source timestamp')
    pose = message.pose.pose if with_covariance else message.pose
    p, q = pose.position, pose.orientation
    return Pose(stamp_ns(message), [p.x, p.y, p.z], [q.x, q.y, q.z, q.w], now,
                list(message.pose.covariance) if with_covariance else None)


def fill_pose(message, pose):
    message.position.x, message.position.y, message.position.z = map(float, pose.position)
    message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w = \
        map(float, pose.orientation)


class OpenVinsAlignment(Node):
    def __init__(self):
        super().__init__('openvins_alignment')
        fixed = ParameterDescriptor(read_only=True)
        defaults = dict(
            source_pose_topic='/localization/openvins/pose',
            source_status_topic='/localization/openvins/status',
            reference_pose_topic='/mavros/local_position/pose',
            reference_attitude_topic='/mavros/imu/data', reference_attitude_frame='base_link',
            reference_health_topic='/uav_state', reference_pose_frame='map',
            reference_clock_service='/mavros/time/get_parameters',
            source_frame='openvins_odom', source_body='livox_imu_frame',
            output_frame='odom', tracked_frame='base_link',
            body_to_base_translation=[.011, .02329, -.12412],
            body_to_base_orientation=[0., 0., 0., 1.])
        defaults.update({k: f.default for k, f in AlignmentLimits.__dataclass_fields__.items()})
        self.p = {name: self.declare_parameter(name, default, fixed).value
                  for name, default in defaults.items()}
        topics = [value for name, value in self.p.items() if name.endswith('_topic')]
        outputs = {'/localization/openvins/base_pose', '/localization/openvins/aligned_pose',
                   '/localization/openvins/alignment_status'}
        if len(set(topics)) != len(topics) or outputs.intersection(topics) or any(
                len(topic) > 256 or not re.fullmatch(
                    r'/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', topic)
                for topic in topics):
            raise ValueError('alignment topics must be distinct absolute ROS names')
        for name in ('source_frame', 'source_body', 'reference_pose_frame',
                     'reference_attitude_frame', 'output_frame', 'tracked_frame'):
            if len(self.p[name]) > 256 or not re.fullmatch(
                    r'[A-Za-z_][A-Za-z0-9_/]*', self.p[name]):
                raise ValueError('invalid alignment frame: ' + name)
        for name in AlignmentLimits.__dataclass_fields__:
            value = self.p[name]
            if not math.isfinite(value) or value <= 0 or value > 1000:
                raise ValueError('invalid alignment limit: ' + name)
        if self.p['initialization_samples'] > 100 or self.p['reference_samples'] < 2 or \
                self.p['pending_samples'] > 100 or self.p['freshness'] > 5 or \
                self.p['reference_gap'] > self.p['freshness']:
            raise ValueError('invalid bounded alignment window')
        if len(self.p['body_to_base_translation']) != 3 or any(
                abs(v) > 100 for v in self.p['body_to_base_translation']):
            raise ValueError('invalid alignment extrinsic')
        self.engine = Alignment(self.p['body_to_base_translation'],
                                self.p['body_to_base_orientation'], AlignmentLimits(**{
                                    k: self.p[k] for k in AlignmentLimits.__dataclass_fields__}))
        self.pairer = ReferencePairer(self.engine)
        self.add_on_set_parameters_callback(lambda params: SetParametersResult(
            successful=False, reason='restart the whole localization session'))
        reliable = QoSProfile(depth=32)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.base_pub = self.create_publisher(PoseWithCovarianceStamped,
                                              '/localization/openvins/base_pose', 10)
        self.aligned_pub = self.create_publisher(PoseStamped,
                                                 '/localization/openvins/aligned_pose', 10)
        self.status_pub = self.create_publisher(
            LocalizationStatus, '/localization/openvins/alignment_status', status_qos)
        self.create_subscription(PoseWithCovarianceStamped, self.p['source_pose_topic'],
                                 self.on_pose, reliable)
        self.create_subscription(LocalizationStatus, self.p['source_status_topic'],
                                 self.on_status, status_qos)
        # LOCAL_POSITION_NED stamps position but embeds cached attitude. Obtain
        # attitude from its own stamped IMU stream, never from that cached field.
        self.create_subscription(PoseStamped, self.p['reference_pose_topic'],
                                 self.on_reference, QoSProfile(
                                     depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Imu, self.p['reference_attitude_topic'], self.on_attitude,
                                 QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(UAVState, self.p['reference_health_topic'],
                                 self.on_health, reliable)
        self.clock_client = self.create_client(GetParameters, self.p['reference_clock_service'])
        self.clock_query = None
        self.clock_query_started = self.next_clock_query = 0.
        self.clock_verified_at = float('-inf')
        self.clock_ever_verified = False
        self.health_fields_valid = False
        self.last_state = None
        self.last_graph = float('-inf')
        self.create_timer(.05, self.report, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def update_clock(self):
        self.engine.update_clock(self.get_clock().now().nanoseconds)

    def on_pose(self, msg):
        self.update_clock()
        try:
            if msg.header.frame_id != self.p['source_frame']:
                raise ValueError('source pose frame mismatch')
            self.engine.pose(values(msg, time.monotonic(), True))
        except ValueError:
            self.engine.fail('invalid_source_pose_for_alignment', reset=False)
        self.publish_outputs()

    def on_status(self, msg):
        self.update_clock()
        if msg.backend_id != 'openvins' or msg.header.frame_id != self.p['source_frame'] or \
                msg.tracked_frame != self.p['source_body'] or \
                msg.pose_valid != (msg.state == Gate.ESTIMATING) or \
                len(msg.reason) > 2048 or len(msg.last_rejection) > 2048 or \
                msg.header.stamp.sec < 0 or msg.header.stamp.nanosec >= 10**9 or \
                msg.pose_stamp.sec < 0 or msg.pose_stamp.nanosec >= 10**9 or \
                (msg.pose_valid and (not msg.covariance_available or msg.velocity_valid)):
            self.engine.fail('invalid_localization_status_for_alignment', reset=False)
        else:
            self.engine.confirm(msg.session_id, msg.generation, msg.state,
                                msg.pose_stamp.sec*10**9+msg.pose_stamp.nanosec,
                                stamp_ns(msg), time.monotonic())
        self.publish_outputs()

    def on_reference(self, msg):
        self.update_clock()
        if not self.clock_policy_ready(time.monotonic()):
            return  # Simulation NONE mode stamps ROS receipt time, not FCU measurements.
        try:
            if msg.header.frame_id != self.p['reference_pose_frame']:
                raise ValueError('reference frame mismatch')
            now = time.monotonic()
            if msg.header.stamp.sec < 0 or msg.header.stamp.nanosec >= 10**9:
                raise ValueError('malformed reference position time')
            p = msg.pose.position
            pose = Pose(stamp_ns(msg), [p.x, p.y, p.z], [0., 0., 0., 1.], now)
            if not self.engine.fresh(pose.stamp, now, now):
                if self.engine.references:
                    self.engine.fail('invalid_reference_source_time')
                return
            self.pairer.add_position(pose)
        except ValueError:
            self.engine.fail('invalid_reference_pose_for_alignment')
        self.publish_outputs()

    def on_attitude(self, msg):
        self.update_clock()
        if not self.clock_policy_ready(time.monotonic()):
            return
        try:
            if msg.header.frame_id != self.p['reference_attitude_frame'] or \
                    msg.header.stamp.sec < 0 or msg.header.stamp.nanosec >= 10**9 or \
                    not math.isfinite(msg.orientation_covariance[0]) or \
                    msg.orientation_covariance[0] < 0:
                raise ValueError('reference attitude unavailable or wrong frame/time')
            q = msg.orientation
            now = time.monotonic()
            sample = Pose(stamp_ns(msg), [0., 0., 0.], [q.x, q.y, q.z, q.w], now)
            if not self.engine.fresh(sample.stamp, now, now):
                if self.engine.references:
                    self.engine.fail('invalid_reference_attitude_time')
                return
            self.pairer.add_attitude(sample)
        except ValueError:
            self.engine.fail('invalid_reference_attitude_for_alignment')
        self.publish_outputs()

    def on_health(self, msg):
        self.update_clock()
        self.health_fields_valid = msg.header.frame_id == self.p['output_frame'] and \
            msg.connected and msg.pose_valid and msg.attitude_valid
        self.engine.health(self.health_fields_valid and self.clock_policy_ready(time.monotonic()),
                           stamp_ns(msg), time.monotonic())
        self.publish_outputs()

    def clock_policy_ready(self, now):
        return 0 <= now-self.clock_verified_at <= .5

    def observe_clock_policy(self, now):
        if self.clock_query is not None:
            if self.clock_query.done():
                try:
                    values = self.clock_query.result().values
                    valid = len(values) == 2 and values[0].type == 4 and \
                        values[0].string_value == 'PASSTHROUGH' and \
                        values[1].type == 1 and values[1].bool_value
                except (RuntimeError, AttributeError):
                    valid = False
                self.clock_query = None
                if valid:
                    self.clock_verified_at, self.clock_ever_verified = now, True
                else:
                    self.clock_verified_at = float('-inf')
                    if self.clock_ever_verified:
                        self.engine.fail('reference_clock_policy_changed')
            elif now-self.clock_query_started > .5:
                self.clock_client.remove_pending_request(self.clock_query)
                self.clock_query.cancel()
                self.clock_query = None
                self.clock_verified_at = float('-inf')
        if self.clock_query is None and now >= self.next_clock_query:
            self.next_clock_query = now+.2
            if self.clock_client.service_is_ready():
                self.clock_query_started = now
                self.clock_query = self.clock_client.call_async(GetParameters.Request(
                    names=['timesync_mode', 'use_sim_time']))

    def publish_outputs(self):
        self.engine.health_valid = self.health_fields_valid and self.clock_policy_ready(
            time.monotonic())
        self.pairer.drain(time.monotonic())
        for base, aligned in self.engine.drain(time.monotonic()):
            msg = PoseWithCovarianceStamped()
            msg.header.stamp, msg.header.frame_id = ros_stamp(base.stamp), self.p['source_frame']
            fill_pose(msg.pose.pose, base)
            msg.pose.covariance = base.covariance.ravel().tolist()
            self.base_pub.publish(msg)
            if aligned is not None:
                msg = PoseStamped()
                msg.header.stamp = ros_stamp(aligned.stamp)
                msg.header.frame_id = self.p['output_frame']
                fill_pose(msg.pose, aligned)
                self.aligned_pub.publish(msg)

    def report(self):
        self.update_clock()
        now = time.monotonic()
        self.observe_clock_policy(now)
        if now-self.last_graph >= .5:
            self.last_graph = now
            for key in ('source_pose_topic', 'source_status_topic', 'reference_pose_topic',
                        'reference_attitude_topic', 'reference_health_topic'):
                if self.count_publishers(self.p[key]) > 1:
                    self.engine.fail('multiple_publishers_for_alignment_input')
        self.publish_outputs()
        state, reason = self.engine.status(now)
        if self.engine.failure is None and not self.clock_policy_ready(now):
            state, reason = Gate.WAITING_INPUT, 'waiting_for_mavros_passthrough_clock'
        msg = LocalizationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p['output_frame']
        msg.backend_id, msg.session_id = 'openvins_alignment', self.engine.session_id
        msg.generation, msg.state, msg.reason = self.engine.generation, state, reason
        msg.tracked_frame = self.p['tracked_frame']
        msg.pose_valid = state == Gate.ESTIMATING
        msg.velocity_valid = msg.covariance_available = msg.tracking_quality_available = False
        msg.pose_stamp = ros_stamp(self.engine.last_output_stamp)
        msg.pose_rejected, msg.last_rejection = self.engine.rejected, self.engine.last_rejection
        self.status_pub.publish(msg)
        if state != self.last_state:
            self.get_logger().info(f'alignment state={state}: {reason}')
            self.last_state = state


def main():
    from .shutdown import init_owned_context
    init_owned_context()
    node = None
    try:
        node = OpenVinsAlignment()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

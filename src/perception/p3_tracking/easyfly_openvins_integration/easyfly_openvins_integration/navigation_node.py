# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Simulation navigation state and TF from confirmed aligned OpenVINS measurements."""
import math
import time

from easyfly_msgs.msg import LocalizationStatus, UAVState
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformBroadcaster

from .alignment_node import fill_pose, values
from .gate import Gate
from .geometry import rotation
from .navigation import Navigation
from .node import ros_stamp, stamp_ns
from .shutdown import init_owned_context, spin_owned_executor


class OpenVinsNavigation(Node):
    def __init__(self):
        super().__init__('openvins_navigation')
        if not self.get_parameter('use_sim_time').value:
            raise ValueError('OpenVINS navigation adapter currently requires simulation time')
        self.backend = self.declare_parameter('source_backend', 'openvins', ParameterDescriptor(read_only=True)).value
        if self.backend not in ('openvins', 'fast_livo2', 'orbslam3'):
            raise ValueError('unsupported navigation source')
        self.engine = Navigation()
        self.minimum_tracks = self.declare_parameter(
            'minimum_visual_tracks', 8, ParameterDescriptor(read_only=True)).value
        if not 8 <= self.minimum_tracks <= 500:
            raise ValueError('minimum_visual_tracks must be 8..500')
        self.add_on_set_parameters_callback(lambda params: SetParametersResult(
            successful=False, reason='restart the entire simulation navigation session'))
        self.tracking_stamp = 0
        self.tracking_receipt = float('-inf')
        self.tracking_count = 0
        self.tracking_initialized = False
        self.last_good_tracking_stamp = 0
        self.health = None
        self.last_state = None
        self.last_graph = float('-inf')
        self.state_pub = self.create_publisher(
            UAVState, '/localization/openvins/navigation_state', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(
            LocalizationStatus, '/localization/openvins/navigation_status', status_qos)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(PoseStamped, '/localization/openvins/aligned_pose',
                                 self.on_pose, 32)
        self.create_subscription(LocalizationStatus, '/localization/openvins/alignment_status',
                                 self.on_status, status_qos)
        self.create_subscription(UAVState, '/uav_state', self.on_health, 32)
        self.create_subscription(DiagnosticArray, '/openvins/raw/visual_tracking',
                                 self.on_tracking, 32)
        self.create_timer(.05, self.report, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def update_clock(self):
        self.engine.update_clock(self.get_clock().now().nanoseconds)

    def health_valid(self, now):
        return self.health is not None and self.health.header.frame_id == 'odom' and \
            self.health.connected and self.health.pose_valid and self.health.attitude_valid and \
            self.engine.fresh(self.engine.health_stamp, self.engine.health_receipt, now) and \
            self.tracking_fresh(now)

    def tracking_fresh(self, now):
        return self.tracking_initialized and self.last_good_tracking_stamp > 0 and \
            0 <= self.tracking_stamp-self.last_good_tracking_stamp <= .15e9 and \
            self.engine.fresh(self.tracking_stamp, self.tracking_receipt, now)

    def on_tracking(self, msg):
        self.update_clock()
        try:
            if len(msg.status) != 1 or msg.status[0].name != self.backend+'/visual_tracking':
                raise ValueError('unexpected visual diagnostic identity')
            pairs = [(v.key, v.value) for v in msg.status[0].values]
            data = dict(pairs)
            if len(data) != len(pairs) or set(data) != {'tracked_features', 'initialized'} or \
                    data['initialized'] not in ('0', '1'):
                raise ValueError('malformed visual diagnostic')
            count = int(data['tracked_features'])
            stamp = stamp_ns(msg)
            if not 0 <= count <= 10000 or stamp <= 0 or stamp < self.tracking_stamp:
                raise ValueError('invalid visual track count/time')
            if stamp != self.tracking_stamp:
                self.tracking_receipt = time.monotonic()
            self.tracking_stamp, self.tracking_count = stamp, count
            self.tracking_initialized = data['initialized'] == '1'
            if self.tracking_initialized and count >= self.minimum_tracks:
                self.last_good_tracking_stamp = stamp
            if self.engine.last_output and count == 0:
                self.engine.fail('visual_tracking_lost')
        except (ValueError, TypeError):
            self.engine.fail('invalid_visual_tracking_diagnostic')
        self.publish_outputs()

    def on_health(self, msg):
        self.update_clock()
        self.health = msg
        self.engine.health(stamp_ns(msg), time.monotonic())
        self.publish_outputs()

    def on_status(self, msg):
        self.update_clock()
        if msg.backend_id != self.backend+'_alignment' or msg.header.frame_id != 'odom' or \
                msg.tracked_frame != 'base_link' or msg.velocity_valid or \
                msg.covariance_available or msg.pose_valid != (msg.state == Gate.ESTIMATING):
            self.engine.fail('invalid_alignment_contract')
        else:
            self.engine.confirm(msg.session_id, msg.generation, msg.state,
                                msg.pose_stamp.sec*10**9+msg.pose_stamp.nanosec,
                                stamp_ns(msg), time.monotonic())
        self.publish_outputs()

    def on_pose(self, msg):
        self.update_clock()
        try:
            if msg.header.frame_id != 'odom':
                raise ValueError('navigation pose frame mismatch')
            self.engine.pose(values(msg, time.monotonic()))
        except ValueError:
            self.engine.fail('invalid_navigation_pose')
        self.publish_outputs()

    def publish_outputs(self):
        now = time.monotonic()
        for pose, velocity in self.engine.drain(now, self.health_valid(now)):
            # A fixed local mapping gauge, certified only at admitted pose stamps.
            # Publish both edges together; loss of VIO expires the mapping basis.
            basis = TransformStamped()
            basis.header.stamp = ros_stamp(pose.stamp)
            basis.header.frame_id, basis.child_frame_id = 'map', 'odom'
            basis.transform.rotation.w = 1.
            transform = TransformStamped()
            transform.header.stamp = ros_stamp(pose.stamp)
            transform.header.frame_id, transform.child_frame_id = 'odom', 'base_link'
            transform.transform.translation.x, transform.transform.translation.y, \
                transform.transform.translation.z = map(float, pose.position)
            q = pose.orientation
            transform.transform.rotation.x, transform.transform.rotation.y, \
                transform.transform.rotation.z, transform.transform.rotation.w = map(float, q)
            self.tf.sendTransform([basis, transform])
            state = UAVState()
            state.header = transform.header
            state.connected, state.armed = self.health.connected, self.health.armed
            state.flight_mode, state.flight_mode_name = (
                self.health.flight_mode, self.health.flight_mode_name)
            state.rc_connected = self.health.rc_connected
            state.position_x, state.position_y, state.position_z = map(float, pose.position)
            x, y, z, w = q
            state.roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
            state.pitch = math.asin(max(-1., min(1., 2*(w*y-z*x))))
            state.yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
            state.pose_valid = state.attitude_valid = True
            if velocity is not None:
                state.velocity_x, state.velocity_y, state.velocity_z = map(float, velocity)
                state.velocity_valid = True
            self.state_pub.publish(state)
            odom = Odometry()
            odom.header, odom.child_frame_id = transform.header, 'base_link'
            fill_pose(odom.pose.pose, pose)
            if velocity is not None:
                body_velocity = rotation(q).T@velocity
                odom.twist.twist.linear.x, odom.twist.twist.linear.y, \
                    odom.twist.twist.linear.z = map(float, body_velocity)
            # Covariance is unavailable after alignment to a correlated FCU reference.
            # Consumers must use navigation_status.covariance_available (false).
            self.odom_pub.publish(odom)

    def report(self):
        self.update_clock()
        now = time.monotonic()
        if now-self.last_graph >= .5:
            self.last_graph = now
            for topic in ['/localization/openvins/aligned_pose',
                          '/localization/openvins/alignment_status', '/uav_state', '/odom',
                          '/localization/openvins/navigation_state',
                          '/localization/openvins/navigation_status']:
                if self.count_publishers(self.resolve_topic_name(topic)) > 1:
                    self.engine.fail('multiple_navigation_publishers')
            if self.count_publishers(self.resolve_topic_name('/openvins/raw/visual_tracking')) > 1:
                self.engine.fail('multiple_visual_tracking_publishers')
        self.publish_outputs()
        state, reason = self.engine.status(now, self.health_valid(now))
        status = LocalizationStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = 'odom'
        status.backend_id, status.session_id = self.backend+'_navigation', self.engine.session
        status.generation, status.state, status.reason = self.engine.generation, state, reason
        status.tracked_frame = 'base_link'
        status.pose_valid = state == Gate.ESTIMATING
        status.tracking_quality_available = self.tracking_fresh(now)
        status.pose_stamp = ros_stamp(self.engine.last_output)
        self.status_pub.publish(status)
        if state != Gate.ESTIMATING:
            invalid = UAVState()
            invalid.header.frame_id = 'odom'
            invalid.header.stamp = ros_stamp(self.engine.last_output)
            self.state_pub.publish(invalid)
        if state != self.last_state:
            self.get_logger().info(f'navigation localization state={state}: {reason}')
            self.last_state = state


def main():
    from rclpy.executors import SingleThreadedExecutor
    init_owned_context()
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = OpenVinsNavigation()
        executor.add_node(node)
        spin_owned_executor(executor)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

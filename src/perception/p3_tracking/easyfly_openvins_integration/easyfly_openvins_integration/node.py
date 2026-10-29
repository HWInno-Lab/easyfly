# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""ROS input ownership, immutable profile validation and honest output status."""

import copy
import math
import re
import time
import uuid

from builtin_interfaces.msg import Time
from easyfly_msgs.msg import LocalizationStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu

from .gate import Gate, Limits


def stamp_ns(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


def ros_stamp(value):
    return Time(sec=value // 10**9, nanosec=value % 10**9)


class CameraAdmissionGroup(MutuallyExclusiveCallbackGroup):
    """Leave images in their bounded DDS reader while dependencies catch up."""

    def __init__(self, available):
        super().__init__()
        self.available = available

    def can_execute(self, entity):
        return self.available() and super().can_execute(entity)

    def beginning_execution(self, entity):
        return self.available() and super().beginning_execution(entity)


class OpenVinsGuard(Node):
    def __init__(self, observe_only=False):
        super().__init__('openvins_startup' if observe_only else 'openvins_guard')
        self.observe_only = observe_only
        descriptor = ParameterDescriptor(read_only=True)
        defaults = dict(
            camera_topic='/camera/color/image_raw', camera_info_topic='/camera/color/camera_info',
            imu_topic='/livox/imu', guarded_camera_topic='/openvins/input/camera',
            guarded_imu_topic='/openvins/input/imu', raw_pose_topic='/openvins/raw/poseimu',
            pose_topic='/localization/openvins/pose', status_topic='/localization/openvins/status',
            camera_frame='camera_color_optical_frame', imu_frame='livox_imu_frame',
            output_frame='openvins_odom', raw_pose_frame='global',
            image_encoding='rgb8', width=640, height=480,
            defer_camera_until_calibration=False,
            intrinsics=[337.3570508552852, 337.3570508552852, 320.5, 240.5],
            max_imu_gap=.025, max_camera_gap=.25, receipt_timeout=.5, pose_timeout=.5,
            max_source_age=.5, future_tolerance=.2, max_pending=4)
        self.p = {name: self.declare_parameter(name, value, descriptor).value
                  for name, value in defaults.items()}
        topics = [v for k, v in self.p.items() if k.endswith('_topic')]
        pattern = r'/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*'
        if len(topics) != len(set(topics)) or any(not re.fullmatch(pattern, v) for v in topics):
            raise ValueError('topics must be distinct absolute ROS names')
        for key in ['camera_frame', 'imu_frame', 'output_frame', 'raw_pose_frame']:
            if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_/]*', self.p[key]):
                raise ValueError('invalid frame: ' + key)
        if self.p['image_encoding'] not in ['rgb8', 'mono8'] or not \
                (1 <= self.p['width'] <= 4096 and 1 <= self.p['height'] <= 4096):
            raise ValueError('unsupported image profile')
        intrinsics = self.p['intrinsics']
        if len(intrinsics) != 4 or not all(math.isfinite(v) for v in intrinsics) or \
                min(intrinsics[:2]) <= 0 or not (0 <= intrinsics[2] <= self.p['width']) or \
                not (0 <= intrinsics[3] <= self.p['height']):
            raise ValueError('invalid intrinsics')
        limit_keys = list(Limits.__dataclass_fields__)
        if any(not math.isfinite(self.p[k]) or not 0 < self.p[k] <= 5
               for k in limit_keys if k != 'max_pending') or not 2 <= self.p['max_pending'] <= 16:
            raise ValueError('invalid admission limits')
        self.gate = Gate(Limits(**{k: self.p[k] for k in limit_keys}))
        self.session_id = str(uuid.uuid4())
        self.add_on_set_parameters_callback(lambda params: SetParametersResult(
            successful=False, reason='restart the whole localization launch to change parameters'))
        reliable = QoSProfile(depth=32, reliability=ReliabilityPolicy.RELIABLE)
        imu_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
        if not observe_only:
            self.camera_pub = self.create_publisher(Image, self.p['guarded_camera_topic'], reliable)
            self.imu_pub = self.create_publisher(Imu, self.p['guarded_imu_topic'], imu_qos)
            self.pose_pub = self.create_publisher(
                PoseWithCovarianceStamped, self.p['pose_topic'], 10)
            status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.status_pub = self.create_publisher(
                LocalizationStatus, self.p['status_topic'], status_qos)
        self.camera_group = CameraAdmissionGroup(
            lambda: self.gate.failure is not None or
            len(self.gate.pending) < self.p['max_pending'])
        self.image_qos, self.camera_subscription = reliable, None
        if not self.p['defer_camera_until_calibration']:
            self.subscribe_camera()
        self.create_subscription(Imu, self.p['imu_topic'], self.on_imu, imu_qos)
        self.create_subscription(CameraInfo, self.p['camera_info_topic'], self.on_info, reliable)
        if not observe_only:
            self.create_subscription(PoseWithCovarianceStamped, self.p['raw_pose_topic'],
                                     self.on_pose, 100)
        self.last_state = None
        self.create_timer(.05, self.report, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def subscribe_camera(self):
        if self.camera_subscription is None:
            self.camera_subscription = self.create_subscription(
                Image, self.p['camera_topic'], self.on_camera, self.image_qos,
                callback_group=self.camera_group)

    def update_clock(self):
        stamp = self.get_clock().now().nanoseconds
        if self.gate.clock and stamp < self.gate.clock and self.gate.failure is None:
            self.get_logger().warning(
                f'ROS clock moved back {self.gate.clock} -> {stamp}; '
                f'admitted IMU={self.gate.imu_forwarded}, camera_seen={self.gate.camera_seen}')
        self.gate.clock_update(stamp)

    def forward(self, images):
        if not self.observe_only and self.gate.failure is None:
            for image in images:
                self.camera_pub.publish(image)  # Original image, header and data are untouched.

    def on_camera(self, msg):
        received = time.monotonic()
        self.update_clock()
        channels = 3 if self.p['image_encoding'] == 'rgb8' else 1
        if msg.header.frame_id != self.p['camera_frame'] or \
                msg.width != self.p['width'] or msg.height != self.p['height'] or \
                msg.encoding != self.p['image_encoding'] or \
                msg.step < msg.width * channels or len(msg.data) != msg.step * msg.height:
            self.gate.camera_rejected += 1
            self.gate.fail('camera_layout_or_frame_mismatch', reset=False)
            return
        self.forward(self.gate.camera(stamp_ns(msg), msg, received))

    def on_imu(self, msg):
        self.update_clock()
        values = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                  msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        if msg.header.frame_id != self.p['imu_frame'] or not all(
                math.isfinite(v) for v in values) or max(abs(v) for v in values[:3]) > 100 or \
                max(abs(v) for v in values[3:]) > 200:
            self.gate.imu_rejected += 1
            self.gate.fail('invalid_imu_measurement_or_frame')
            return
        accepted, images = self.gate.imu(stamp_ns(msg), time.monotonic())
        if accepted and self.gate.failure is None and not self.observe_only:
            self.imu_pub.publish(msg)
            self.forward(images)
        self.subscribe_camera_when_dependencies_ready()

    def on_info(self, msg):
        self.update_clock()
        k = self.p['intrinsics']
        expected = [k[0], 0., k[2], 0., k[1], k[3], 0., 0., 1.]
        if msg.header.frame_id != self.p['camera_frame'] or \
                msg.width != self.p['width'] or msg.height != self.p['height'] or \
                msg.distortion_model != 'plumb_bob' or len(msg.d) != 5 or \
                not all(math.isfinite(v) and abs(v - e) < 1e-5
                        for v, e in zip(msg.k, expected)) or \
                not all(math.isfinite(v) and abs(v) < 1e-12 for v in msg.d):
            self.gate.fail('camera_calibration_mismatch', reset=False)
            return
        self.forward(self.gate.info(stamp_ns(msg), time.monotonic()))
        self.subscribe_camera_when_dependencies_ready()

    def subscribe_camera_when_dependencies_ready(self):
        # Both dependencies must reach this reader before opening the camera
        # epoch. DDS can discover CameraInfo before IMU in a new guard process,
        # even after a separate startup observer has confirmed healthy inputs.
        # Once subscribed, the original timing/pairing and failure latch apply.
        if self.p['defer_camera_until_calibration'] and self.gate.failure is None and \
                self.gate.imu_stamp and self.gate.infos and self.camera_subscription is None:
            stamp = self.gate.infos[-1]
            imu_fresh = (time.monotonic() - self.gate.imu_receipt <= self.p['receipt_timeout']
                         and self.gate.clock - self.gate.imu_stamp <=
                         self.p['max_source_age'] * 1e9)
            if imu_fresh and 0 < stamp <= self.gate.clock + self.p['future_tolerance'] * 1e9 and \
                    self.gate.clock - stamp <= self.p['max_source_age'] * 1e9:
                self.subscribe_camera()

    def on_pose(self, msg):
        self.update_clock()
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w, *msg.pose.covariance]
        valid = msg.header.frame_id == self.p['raw_pose_frame'] and \
            all(math.isfinite(v) for v in values) and \
            abs(sum(v*v for v in [q.x, q.y, q.z, q.w]) - 1) < 1e-3
        if valid:
            covariance = np.array(msg.pose.covariance).reshape(6, 6)
            try:
                valid = np.allclose(covariance, covariance.T, rtol=1e-6, atol=1e-8) and \
                    np.min(np.linalg.eigvalsh(covariance)) >= -1e-9
            except np.linalg.LinAlgError:
                valid = False
        if not valid:
            self.gate.pose_rejected += 1
            self.gate.fail('invalid_estimator_pose_or_covariance', reset=False)
            return
        if self.gate.pose(stamp_ns(msg), time.monotonic()):
            result = copy.deepcopy(msg)
            result.header.frame_id = self.p['output_frame']  # Name this backend's local gauge.
            self.pose_pub.publish(result)

    def report(self):
        self.update_clock()
        state, reason = self.gate.status(time.monotonic())
        msg = LocalizationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p['output_frame']
        msg.backend_id, msg.session_id = 'openvins', self.session_id
        msg.generation, msg.state, msg.reason = self.gate.generation, state, reason
        msg.tracked_frame = self.p['imu_frame']
        msg.pose_valid = state == Gate.ESTIMATING
        msg.velocity_valid = False  # This adapter exposes pose only.
        msg.covariance_available = self.gate.pose_stamp > 0
        msg.tracking_quality_available = False  # Upstream poses do not certify feature tracking.
        msg.pose_stamp = ros_stamp(self.gate.pose_stamp)
        msg.camera_stamp = ros_stamp(self.gate.camera_stamp)
        msg.imu_stamp = ros_stamp(self.gate.imu_stamp)
        for key in ['camera_forwarded', 'camera_rejected', 'imu_forwarded', 'imu_rejected',
                    'pose_rejected', 'last_rejection']:
            setattr(msg, key, getattr(self.gate, key))
        self.status_pub.publish(msg)
        if state != self.last_state:
            detail = ('; ' + self.gate.failure_detail) if self.gate.failure_detail else ''
            self.get_logger().info(f'localization state={state}: {reason}{detail}')
            self.last_state = state


def main():
    from .shutdown import init_owned_context, spin_owned_executor
    init_owned_context()
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = OpenVinsGuard()
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

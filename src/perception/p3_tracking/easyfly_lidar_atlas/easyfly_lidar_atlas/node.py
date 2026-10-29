# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Explicit private atlas observer. Never publishes control topics or /tf."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import signal
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor, ParameterEvent
from rcl_interfaces.srv import GetParameters, ListParameters
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation

from .engine import Engine
from .geometry import GeometryRejected, inverse
from .input import (SourceGate, cloud_points, cloud_fingerprint, parameter_identity,
                    odometry_pose, odometry_fingerprint, stamp_ns)

PREFIX = '/localization/lidar_atlas'


class AtlasNode(Node):
    def __init__(self):
        super().__init__('lidar_atlas')
        def parameter(name, default):
            return self.declare_parameter(name, default,
                ParameterDescriptor(read_only=True)).value
        self.backend = parameter('backend', 'fast_lio')
        if self.backend not in ('fast_lio', 'fast_livo2'):
            raise ValueError('unsupported native atlas backend')
        self.load_path = parameter('load_atlas', '')
        self.save_path = parameter('save_atlas', '')
        for path in (self.load_path, self.save_path):
            if path and not Path(path).is_absolute():
                raise ValueError('atlas paths must be absolute')
        if self.save_path and Path(self.save_path).exists():
            raise ValueError('atlas output must be a new directory')
        self.native_node = '/laser_mapping' if self.backend == 'fast_lio' else '/fastlivo/raw/laserMapping'
        self.cloud_topic = '/cloud_registered_body' if self.backend == 'fast_lio' else '/fastlivo/raw/atlas_cloud'
        self.odom_topic = '/Odometry' if self.backend == 'fast_lio' else '/fastlivo/raw/atlas_odometry'
        self.gate = SourceGate()
        self.engine = None
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='lidar-atlas-geometry')
        self.job = None
        self.poses = deque(maxlen=128)
        self.pending = None
        self.state = {}
        self.correction = None
        self.reason = 'waiting_for_native_parameter_identity'
        self.last_submitted = 0
        self.last_published = 0
        self.dropped = 0
        self.admitted = False
        self.last_clock = 0
        self.saved_identity = ''
        self.parameter_names = None
        self.identity_future = None
        self.identity_started = 0.
        self.list_parameters_client = self.create_client(ListParameters, self.native_node+'/list_parameters')
        self.get_parameters_client = self.create_client(GetParameters, self.native_node+'/get_parameters')
        qos = QoSProfile(depth=8, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, self.odom_topic, self.on_odometry, qos)
        self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, qos)
        self.create_subscription(ParameterEvent, '/parameter_events', self.on_parameters, 10)
        self.pose_publisher = self.create_publisher(PoseStamped, PREFIX+'/pose', 5)
        self.transform_publisher = self.create_publisher(TransformStamped, PREFIX+'/local_to_atlas', 5)
        self.status_publisher = self.create_publisher(DiagnosticArray, PREFIX+'/status', 5)
        self.create_service(Trigger, PREFIX+'/save', self.on_save)
        # Source silence must expire even when /clock itself stops advancing.
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_parameters(self, event):
        if self.engine is not None and event.node == self.native_node and \
                (event.changed_parameters or event.new_parameters or event.deleted_parameters):
            self.gate.reject('native backend configuration changed; new atlas session required')

    def publisher_identity(self, topic):
        # Humble's executor discards message metadata and its Python RMW info
        # does not expose the publisher GID. Pin the unique discovered native
        # endpoint, as the existing fleet source admission does.
        sources = self.get_publishers_info_by_topic(topic)
        if len(sources) != 1:
            raise GeometryRejected('native publisher discovery is not unique')
        source = sources[0]
        name = ('/' + source.node_namespace.strip('/') + '/' + source.node_name).replace('//', '/')
        if name != self.native_node:
            raise GeometryRejected('unexpected native source node')
        return source.endpoint_gid

    def on_odometry(self, message):
        if self.engine is None or self.gate.failure:
            return
        try:
            pose = odometry_pose(message, self.backend)
            stamp = stamp_ns(message)
            self.gate.admit('odometry', stamp, self.get_clock().now().nanoseconds,
                            time.monotonic(), self.publisher_identity(self.odom_topic), pose=pose)
            self.poses.append((stamp, pose, message.header.stamp,
                odometry_fingerprint(message)))
        except (GeometryRejected, ValueError, OverflowError) as error:
            self.gate.reject(str(error))

    def on_cloud(self, message):
        if self.engine is None or self.gate.failure:
            return
        try:
            stamp = stamp_ns(message)
            # Byte/layout checks happen before retaining a queued input.
            if len(message.data) > 16*1024*1024 or message.width*message.height > 200000:
                raise GeometryRejected('native cloud exceeds resource bounds')
            self.gate.admit('cloud', stamp, self.get_clock().now().nanoseconds,
                            time.monotonic(), self.publisher_identity(self.cloud_topic))
            if self.pending is not None:
                self.dropped += 1
            self.pending = message
        except (GeometryRejected, ValueError, OverflowError) as error:
            self.gate.reject(str(error))

    def initialize_identity(self):
        if not self.list_parameters_client.service_is_ready() or not self.get_parameters_client.service_is_ready():
            return
        if self.identity_future is None:
            self.identity_started = time.monotonic()
            self.identity_future = self.list_parameters_client.call_async(ListParameters.Request())
            return
        if not self.identity_future.done():
            if time.monotonic()-self.identity_started > 5:
                raise GeometryRejected('native calibration service deadline')
            return
        if self.parameter_names is None:
            names = self.identity_future.result().result.names
            if not 5 <= len(names) <= 256 or len(set(names)) != len(names):
                raise GeometryRejected('invalid native parameter inventory')
            self.parameter_names = sorted(names)
            self.identity_started = time.monotonic()
            self.identity_future = self.get_parameters_client.call_async(
                GetParameters.Request(names=self.parameter_names))
            return
        values = self.identity_future.result().values
        if len(values) != len(self.parameter_names) or any(value.type == 0 for value in values):
            raise GeometryRejected('native parameters changed during identity capture')
        configuration = dict(zip(self.parameter_names, values))
        required = ['publish.scan_publish_en', 'publish.scan_bodyframe_pub_en'] if self.backend == 'fast_lio' else ['publish.atlas_keyframes']
        if any(name not in configuration or configuration[name].type != 1 or
               not configuration[name].bool_value for name in required):
            raise GeometryRejected('native complete-scan output must be explicitly enabled')
        identity = parameter_identity(self.backend, self.parameter_names, values)
        self.engine = Engine(self.backend, identity, atlas_path=self.load_path or None)
        self.state = self.engine.status()
        self.state['calibration_sha256'] = identity
        self.reason = 'waiting_for_native_scans'

    def process_scan(self, message, matched):
        stamp, native, _, odometry_hash = matched
        scan = cloud_points(message, expected_frame='body' if self.backend == 'fast_lio' else 'camera_init')
        digest = cloud_fingerprint(message)
        pose, new_keyframe = self.engine.process(scan, native, stamp, digest,
            scan_is_imu=self.backend == 'fast_lio')
        state = self.engine.status()
        state.update(scan_source_sha256=digest, odometry_source_sha256=odometry_hash,
                     keyframe_committed=new_keyframe, input_stamp_ns=stamp,
                     calibration_sha256=self.engine.calibration)
        return pose @ inverse(native), state

    def tick(self):
        now, wall = self.get_clock().now().nanoseconds, time.monotonic()
        if now < self.last_clock:
            self.gate.reject('clock regressed; explicit new session required')
        self.last_clock = now
        try:
            names = [('/' + namespace.strip('/') + '/' + name).replace('//', '/')
                     for name, namespace in self.get_node_names_and_namespaces()]
            if names.count(self.native_node) > 1:
                raise GeometryRejected('ambiguous native backend node')
            for topic in (self.cloud_topic, self.odom_topic):
                count = len(self.get_publishers_info_by_topic(topic))
                if count > 1 or (self.admitted and count != 1):
                    raise GeometryRejected('native source publisher missing or ambiguous')
            if self.engine is None and not self.gate.failure:
                self.initialize_identity()
            if self.job is not None and self.job.done():
                try:
                    self.correction, self.state = self.job.result()
                    self.reason = 'private_atlas_localized'
                except GeometryRejected as error:
                    self.reason = str(error)
                    if self.correction is not None:
                        self.gate.reject(str(error))
                finally:
                    self.job = None
            fresh = self.gate.fresh(now, wall)
            if self.admitted and not fresh:
                self.gate.reject('native input freshness lost; explicit new session required')
            if not self.gate.failure and self.engine is not None and self.pending is not None and self.job is None:
                stamp = stamp_ns(self.pending)
                # No interpolation or restamping: use the actual exact native
                # scan-end odometry sample produced by this same frontend.
                match = next((entry for entry in reversed(self.poses) if entry[0] == stamp), None)
                if match is not None and stamp-self.last_submitted >= 500_000_000:
                    self.job = self.pool.submit(self.process_scan, self.pending, match)
                    self.pending = None
                    self.last_submitted = stamp
            if fresh and self.correction is not None and not self.gate.failure and self.poses:
                self.admitted = True
                stamp, native, ros_stamp, _ = self.poses[-1]
                if stamp != self.last_published:
                    pose = self.correction @ native
                    quaternion = Rotation.from_matrix(pose[:3, :3]).as_quat()
                    message = PoseStamped()
                    message.header.stamp = ros_stamp
                    message.header.frame_id = 'lidar_atlas'
                    message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, pose[:3, 3])
                    message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w = map(float, quaternion)
                    self.pose_publisher.publish(message)
                    correction = TransformStamped()
                    correction.header = message.header
                    correction.child_frame_id = 'camera_init'
                    correction.transform.translation.x, correction.transform.translation.y, correction.transform.translation.z = map(float, self.correction[:3, 3])
                    q = Rotation.from_matrix(self.correction[:3, :3]).as_quat()
                    correction.transform.rotation.x, correction.transform.rotation.y, correction.transform.rotation.z, correction.transform.rotation.w = map(float, q)
                    self.transform_publisher.publish(correction)
                    self.last_published = stamp
        except (GeometryRejected, ValueError, RuntimeError, OSError) as error:
            self.gate.reject(str(error))
        record = dict(self.state, reason=self.gate.failure or self.reason,
                      valid=self.admitted and self.gate.fresh(now, wall),
                      latched=bool(self.gate.failure), dropped_pending_scans=self.dropped,
                      covariance='unknown; geometric scores are not covariance',
                      control_frame_unchanged=True, saved_identity=self.saved_identity)
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(name='lidar_atlas', hardware_id=self.backend,
                                  level=DiagnosticStatus.OK if record['valid'] else DiagnosticStatus.ERROR,
                                  message=record['reason'])
        status.values = [KeyValue(key='state', value=json.dumps(record, sort_keys=True))]
        message.status = [status]
        self.status_publisher.publish(message)

    def on_save(self, request, response):
        if not self.save_path or self.engine is None or self.job is not None or \
                not self.gate.fresh(self.get_clock().now().nanoseconds, time.monotonic()):
            response.message = 'save requires configured new path, fresh inputs and idle geometry worker'
            return response
        try:
            self.saved_identity = self.engine.persist(self.save_path)
            response.success = True
            response.message = self.saved_identity
        except (GeometryRejected, OSError, ValueError) as error:
            response.message = str(error)
        return response

    def close(self):
        self.gate.reject('shutdown')
        self.pool.shutdown(wait=True, cancel_futures=True)


def main():
    stopping = False
    def stop(signum, frame):
        nonlocal stopping
        stopping = True
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node = None
    try:
        node = AtlasNode()
        while not stopping and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()

# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Opt-in simulation EV writer; source control and ground activation are explicit."""
from collections import deque
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import stat
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from diagnostic_msgs.msg import DiagnosticArray
from easyfly_msgs.msg import LocalizationStatus, UAVState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, ParameterEvent
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation

from .policy import Policy, Sample, at

PREFIX = '/localization/failover'
EV_PARAMETERS = {'EKF2_EV_CTRL': 3, 'EKF2_EV_NOISE_MD': 1,
                 'EKF2_EVP_NOISE': .3, 'EKF2_EVA_NOISE': .15, 'EKF2_EV_DELAY': 0.,
                 'EKF2_EV_QMIN': 0, 'EKF2_EV_POS_X': 0.,
                 'EKF2_EV_POS_Y': 0., 'EKF2_EV_POS_Z': 0.}
LEVER = np.array([.011, .02329, -.12412])  # calibrated gazebo_mono IMU -> base origin


def stamp(m):
    h = m.header.stamp
    if h.sec < 0 or not 0 <= h.nanosec < 10**9:
        raise ValueError('malformed timestamp')
    return h.sec*10**9+h.nanosec


def sample(m, pose, body=False):
    p, q = pose.position, pose.orientation
    value = Sample.make(stamp(m), time.monotonic(), [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
    if body:
        value = Sample.make(value.stamp, value.receipt,
                            value.p+Rotation.from_quat(value.q).apply(LEVER), value.q)
    return value


def endpoint_key(topic, endpoint):
    # Fast DDS discovers an endpoint GID before ROS node metadata arrives.
    # The actual GID is the identity; _NODE_NAME_UNKNOWN_ becoming the native
    # name is not a process restart. Never pin those mutable discovery labels.
    gid = bytes(endpoint.endpoint_gid)
    if not 16 <= len(gid) <= 32 or not any(gid):
        raise ValueError('invalid discovered publisher GID')
    return topic, gid.hex()


def lock(name):
    root = Path('/tmp') / ('easyfly-failover-'+str(os.getuid()))
    root.mkdir(mode=0o700, exist_ok=True)
    info = root.lstat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or stat.S_IMODE(info.st_mode) != 0o700:
        raise RuntimeError('private owned failover directory required')
    path = root / (str(int(os.environ.get('ROS_DOMAIN_ID', '0')))+'-'+name+'.lock')
    fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BaseException:
        os.close(fd)
        raise
    return fd, path.with_suffix('.json')


class Manager(Node):
    def __init__(self):
        super().__init__('localization_failover')
        if not self.get_parameter('use_sim_time').value:
            raise ValueError('this reviewed profile is simulation-only')
        if not self.declare_parameter('enable_external_vision', False,
                                      ParameterDescriptor(read_only=True)).value:
            raise ValueError('explicit enable_external_vision:=true required')
        self.lock_fd, _ = lock('ev-writer')
        self.policy = Policy()
        self.references, self.positions, self.attitudes = deque(maxlen=512), deque(maxlen=64), deque(maxlen=512)
        self.reference_last = 0
        self.ground_since = 0
        self.health = None
        self.health_receipt = float('-inf')
        self.inputs = {}
        self.vio_status = None
        self.vio_receipt = float('-inf')
        self.vio_identity = None
        self.pending_vio = deque(maxlen=64)
        self.tracking = None
        self.writers = {}
        self.graph_at = float('-inf')
        self.params = {}
        self.param_at = float('-inf')
        self.clock_verified = False
        self.native_clock = None
        self.native_clock_at = float('-inf')
        self.native_clock_session = None
        self.queries = {}
        self.query_started = self.next_query = 0.
        self.parameters = self.create_client(GetParameters, '/mavros/param/get_parameters')
        self.timesync = self.create_client(GetParameters, '/mavros/time/get_parameters')
        self.vision = self.create_client(GetParameters, '/mavros/vision_pose/get_parameters')
        self.ev = self.create_publisher(PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov', 10)
        self.status_pub = self.create_publisher(String, PREFIX+'/status', 10)
        self.create_service(Trigger, PREFIX+'/activate', self.activate)
        self.create_subscription(Odometry, '/Odometry', self.lio, 32)
        self.create_subscription(PoseWithCovarianceStamped, '/localization/openvins/pose', self.vio, 32)
        self.create_subscription(LocalizationStatus, '/localization/openvins/status', self.confirm,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(DiagnosticArray, '/openvins/raw/visual_tracking', self.tracks, 32)
        self.create_subscription(Imu, '/livox/imu', lambda m:self.input('imu', m), qos_profile_sensor_data)
        self.create_subscription(CustomMsg, '/livox/lidar', lambda m:self.input('lidar', m), qos_profile_sensor_data)
        self.create_subscription(UAVState, '/uav_state', self.state, 32)
        self.create_subscription(String, PREFIX+'/clock_status', self.on_native_clock, 10)
        self.create_subscription(ParameterEvent, '/parameter_events', self.parameter_event, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.reference_position, qos_profile_sensor_data)
        self.create_subscription(Imu, '/mavros/imu/data', self.reference_attitude, qos_profile_sensor_data)
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.last_status = 0.

    def clock_update(self):
        self.policy.clock_update(self.get_clock().now().nanoseconds)

    def input(self, name, m):
        try:
            ns = stamp(m)
            old = self.inputs.get(name)
            if old and ns <= old[0]:
                if ns < old[0]:
                    self.policy.fail('original_'+name+'_clock_rollback')
                return
            if (name == 'imu' and (m.header.frame_id != 'livox_imu_frame' or
                    not np.isfinite([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                                     m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]).all())):
                raise ValueError('malformed original IMU')
            if name == 'lidar' and (m.point_num != len(m.points) or not 100 <= m.point_num <= 500000):
                raise ValueError('malformed original LiDAR')
            self.inputs[name] = ns, time.monotonic()
        except ValueError as error:
            self.policy.fail(str(error))

    def lio(self, m):
        self.clock_update()
        try:
            if m.header.frame_id != 'camera_init' or m.child_frame_id != 'body':
                raise ValueError('FAST-LIO native frame mismatch')
            self.policy.ingest('fast_lio', sample(m, m.pose.pose, body=True))
        except ValueError as error:
            self.policy.sources['fast_lio'].failure = str(error)

    def vio(self, m):
        try:
            if m.header.frame_id != 'openvins_odom':
                raise ValueError('OpenVINS native frame mismatch')
            self.pending_vio.append(sample(m, m.pose.pose, body=True))
        except ValueError as error:
            self.policy.sources['openvins'].failure = str(error)

    def confirm(self, m):
        try:
            if (m.backend_id != 'openvins' or m.tracked_frame != 'livox_imu_frame' or
                    m.header.frame_id != 'openvins_odom' or not m.session_id or
                    m.pose_valid != (m.state == LocalizationStatus.ESTIMATING)):
                raise ValueError('invalid OpenVINS health contract')
            identity = m.session_id, m.generation
            if self.vio_identity is not None and self.vio_identity != identity:
                raise ValueError('OpenVINS session reset requires new ground activation')
            self.vio_identity = identity
            if m.state in (LocalizationStatus.RESET_REQUIRED, LocalizationStatus.FAILED):
                raise ValueError('native OpenVINS guard latched: '+m.reason)
            self.vio_status, self.vio_receipt = m, time.monotonic()
        except ValueError as error:
            self.policy.sources['openvins'].failure = str(error)

    def tracks(self, m):
        try:
            if len(m.status) != 1 or m.status[0].name != 'openvins/visual_tracking':
                raise ValueError('invalid visual tracking diagnostic identity')
            data = {v.key:v.value for v in m.status[0].values}
            if (len(data) != len(m.status[0].values) or set(data) != {'initialized', 'tracked_features'} or
                    data['initialized'] not in ('0', '1') or not 0 <= int(data['tracked_features']) <= 10000):
                raise ValueError('invalid visual tracking diagnostic values')
            ns = stamp(m)
            if self.tracking and ns <= self.tracking[0]:
                if ns < self.tracking[0]:
                    raise ValueError('visual tracking time rollback')
                return
            self.tracking = ns, time.monotonic(), data['initialized'] == '1' and int(data['tracked_features']) >= 8
        except ValueError as error:
            self.policy.sources['openvins'].failure = str(error)

    def state(self, m):
        self.health, self.health_receipt = m, time.monotonic()

    def on_native_clock(self, m):
        try:
            if len(m.data) > 8192:
                raise ValueError('oversized native clock status')
            value = json.loads(m.data)
            if (not isinstance(value, dict) or type(value.get('verified')) is not bool or
                    type(value.get('count')) is not int or not 0 <= value['count'] < 2**63 or
                    type(value.get('clock_ns')) is not int or not 0 <= value['clock_ns'] < 2**63 or
                    not isinstance(value.get('failure'), str)):
                raise ValueError('invalid native clock status types')
            session = value['identity']['session']
            if not isinstance(session, str) or not 1 <= len(session) <= 128:
                raise ValueError('invalid native clock identity')
            if self.native_clock_session is not None and session != self.native_clock_session:
                raise ValueError('native clock witness session changed')
            self.native_clock_session = session
            self.native_clock, self.native_clock_at = value, time.monotonic()
        except (ValueError, TypeError, KeyError) as error:
            self.policy.fail(str(error))

    def parameter_event(self, event):
        if not (event.changed_parameters or event.new_parameters or event.deleted_parameters):
            return
        name = ('fast_lio' if event.node == '/laser_mapping' else
                'openvins' if event.node in ('/openvins/raw/run_subscribe_msckf', '/openvins/openvins_guard') else None)
        if name:
            source = self.policy.sources[name]
            event_ns = event.stamp.sec*10**9+event.stamp.nanosec
            if source.samples and event_ns >= source.samples[0].stamp:
                source.failure = 'native_configuration_changed_new_ground_session_required'

    def native_clock_valid(self, now):
        value = self.native_clock
        return bool(value and value.get('verified') is True and not value.get('failure') and
                    value.get('count', 0) >= 510 and
                    self.time_fresh(value.get('clock_ns', 0), self.native_clock_at, now, wall=.4))

    def reference_position(self, m):
        if self.policy.active or not self.clock_verified:
            return
        try:
            if m.header.frame_id != 'map':
                raise ValueError('FCU position frame mismatch')
            p = m.pose.position
            value = Sample.make(stamp(m), time.monotonic(), [p.x, p.y, p.z], [0., 0., 0., 1.])
            if not self.positions or value.stamp > self.positions[-1].stamp:
                self.positions.append(value)
        except ValueError as error:
            self.policy.fail(str(error))

    def reference_attitude(self, m):
        if self.policy.active or not self.clock_verified:
            return
        try:
            if m.header.frame_id != 'base_link' or not math.isfinite(m.orientation_covariance[0]) or m.orientation_covariance[0] < 0:
                raise ValueError('FCU attitude frame/availability mismatch')
            q = m.orientation
            value = Sample.make(stamp(m), time.monotonic(), [0., 0., 0.], [q.x, q.y, q.z, q.w])
            if not self.attitudes or value.stamp > self.attitudes[-1].stamp:
                self.attitudes.append(value)
        except ValueError as error:
            self.policy.fail(str(error))

    def time_fresh(self, ns, receipt, now, source=.35, wall=.75):
        # The reviewed native Gazebo /clock is published at 10 Hz. The live
        # sensor health watermark can lead that quantized clock by one period.
        # Pose admission/output retains Policy's stricter 20 ms future bound;
        # this allowance cannot manufacture or advance an EV measurement.
        return -.15e9 <= self.policy.clock-ns <= source*1e9 and 0 <= now-receipt <= wall

    def fcu_fresh(self, now):
        m = self.health
        return bool(m is not None and m.connected and m.pose_valid and m.attitude_valid and
                    m.velocity_valid and m.header.frame_id == 'odom' and self.time_fresh(stamp(m), self.health_receipt, now) and
                    np.isfinite([m.position_x, m.position_y, m.position_z, m.roll, m.pitch, m.yaw,
                                 m.velocity_x, m.velocity_y, m.velocity_z]).all())

    def healthy(self, now):
        imu = 'imu' in self.inputs and self.time_fresh(*self.inputs['imu'], now)
        lidar = 'lidar' in self.inputs and self.time_fresh(*self.inputs['lidar'], now)
        m = self.vio_status
        vio = bool(m is not None and m.pose_valid and self.time_fresh(stamp(m), self.vio_receipt, now) and
                   self.time_fresh(m.pose_stamp.sec*10**9+m.pose_stamp.nanosec, self.vio_receipt, now) and
                   self.tracking and self.time_fresh(self.tracking[0], self.tracking[1], now) and self.tracking[2])
        return {'fast_lio': imu and lidar, 'openvins': imu and vio}

    def graph(self, now):
        sources = {'fast_lio': ['/Odometry', '/livox/lidar', '/livox/imu'],
                   'openvins': ['/localization/openvins/pose', '/localization/openvins/status',
                                '/openvins/raw/visual_tracking', '/livox/imu',
                                '/camera/color/image_raw', '/camera/color/camera_info']}
        for name, topics in sources.items():
            identities = []
            for topic in topics:
                entries = self.get_publishers_info_by_topic(topic)
                if len(entries) > 1:
                    self.policy.sources[name].failure = 'ambiguous_source_publishers:'+topic
                if len(entries) != 1:
                    identities = []
                    break
                e = entries[0]
                try:
                    identities.append(endpoint_key(topic, e))
                except ValueError:
                    identities = []
                    break
            if identities:
                self.policy.identity(name, tuple(identities))
        for topic, count in [('/mavros/vision_pose/pose_cov', 1), ('/mavros/vision_pose/pose', 0),
                             ('/mavros/odometry/out', 0), ('/mavros/vision_speed/speed_twist_cov', 0),
                             ('/mavros/vision_speed/speed_twist', 0), ('/mavros/vision_speed/speed_vector', 0)]:
            if self.count_publishers(topic) != count:
                self.policy.fail('external_vision_writer_conflict:'+topic)
        for topic in ['/uav_state', '/mavros/local_position/pose', '/mavros/imu/data', PREFIX+'/clock_status']:
            entries = self.get_publishers_info_by_topic(topic)
            if len(entries) > 1:
                self.policy.fail('ambiguous_FCU_reference:'+topic)
            if len(entries) == 1:
                identity = bytes(entries[0].endpoint_gid).hex()
                if topic in self.writers and self.writers[topic] != identity:
                    self.policy.fail('FCU_reference_identity_changed')
                self.writers[topic] = identity
        self.graph_at = now

    @staticmethod
    def parameter_value(value):
        return {1: lambda:value.bool_value, 2:lambda:value.integer_value,
                3:lambda:value.double_value, 4:lambda:value.string_value}.get(value.type, lambda:None)()

    def parameters_update(self, now):
        if self.queries:
            if all(f.done() for f in self.queries.values()):
                try:
                    result = {name:[self.parameter_value(v) for v in f.result().values] for name, f in self.queries.items()}
                    self.params = dict(zip(EV_PARAMETERS, result['px4']))
                    valid = (len(result['px4']) == len(EV_PARAMETERS) and
                             all(isinstance(self.params.get(k), (int, float)) and
                                 abs(self.params[k]-v) < 1e-5 for k, v in EV_PARAMETERS.items()) and
                             result['clock'] == ['PASSTHROUGH'] and result['vision'] == [False])
                    self.clock_verified = result['clock'] == ['PASSTHROUGH']
                    if valid:
                        self.param_at = now
                    else:
                        self.param_at = float('-inf')
                        if self.policy.active:
                            self.policy.fail('PX4_EV_parameters_or_clock_policy_changed')
                except Exception as error:
                    self.get_logger().warning('parameter verification failed: '+str(error))
                self.queries = {}
                self.next_query = now+1.
            elif now-self.query_started > 4.:
                for f in self.queries.values():
                    f.cancel()
                self.queries = {}
                self.next_query = now+1.
        elif now >= self.next_query and all(c.service_is_ready() for c in [self.parameters, self.timesync, self.vision]):
            self.queries = {
                'px4':self.parameters.call_async(GetParameters.Request(names=list(EV_PARAMETERS))),
                'clock':self.timesync.call_async(GetParameters.Request(names=['timesync_mode'])),
                'vision':self.vision.call_async(GetParameters.Request(names=['tf/listen']))}
            self.query_started = now

    def activate(self, request, response):
        self.tick()
        now = time.monotonic()
        try:
            if not self.fcu_fresh(now) or self.health.armed or now-self.param_at > 3. or not self.native_clock_valid(now):
                raise ValueError('fresh disarmed FCU, verified PX4 EV profile and native TIMESYNC convergence required')
            self.graph(now)
            evidence = self.policy.activate(self.references, now, self.ground_since, self.healthy(now))
            # Drop all live reference buffers permanently. No feedback refitting.
            self.references.clear(); self.positions.clear(); self.attitudes.clear()
            response.success = True
            response.message = json.dumps(evidence)
        except ValueError as error:
            response.message = str(error)
        return response

    def tick(self):
        self.clock_update()
        now = time.monotonic()
        if now-self.graph_at >= .1:
            self.graph(now)
        self.parameters_update(now)
        if self.vio_status:
            confirmed = self.vio_status.pose_stamp.sec*10**9+self.vio_status.pose_stamp.nanosec
            while self.pending_vio and self.pending_vio[0].stamp <= confirmed:
                self.policy.ingest('openvins', self.pending_vio.popleft())
        if not self.policy.active:
            grounded = (self.fcu_fresh(now) and not self.health.armed and
                        np.linalg.norm([self.health.velocity_x, self.health.velocity_y, self.health.velocity_z]) < .08 and
                        max(abs(self.health.roll), abs(self.health.pitch)) < .08)
            if not grounded:
                self.ground_since = 0
            elif not self.ground_since:
                self.ground_since = self.policy.clock
            while self.positions:
                p = self.positions[0]
                q = at(self.attitudes, p.stamp)
                if q is None:
                    if self.attitudes and self.attitudes[-1].stamp >= p.stamp:
                        self.positions.popleft()
                        continue
                    break
                self.positions.popleft()
                if p.stamp > self.reference_last:
                    self.references.append(Sample.make(p.stamp, min(p.receipt, q.receipt), p.p, q.q))
                    self.reference_last = p.stamp
        if self.policy.active and (now-self.param_at > 5. or not self.fcu_fresh(now)):
            self.policy.fail('PX4_configuration_or_FCU_health_expired')
        if self.policy.active and not self.native_clock_valid(now):
            self.policy.fail('native_PX4_timesync_proof_expired')
        value = self.policy.step(now, self.healthy(now))
        if value:
            message = PoseWithCovarianceStamped()
            message.header.frame_id = 'map'
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(value.stamp, 10**9)
            message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = map(float, value.p)
            q = message.pose.pose.orientation
            q.x, q.y, q.z, q.w = map(float, value.q)
            # MAVROS forwards the upper triangle; unknown covariance stays NaN.
            # PX4 must be explicitly verified in parameter-noise mode before use.
            message.pose.covariance = [float('nan')]*36
            self.ev.publish(message)
        if now-self.last_status >= .05:
            result = self.policy.status()
            result.update(parameters=self.params, parameters_verified=now-self.param_at <= 5.,
                          clock_verified=self.clock_verified, ground_since=self.ground_since,
                          native_timesync_verified=self.native_clock_valid(now),
                          native_timesync_count=self.native_clock.get('count', 0) if self.native_clock else 0,
                          source_health=self.healthy(now), profile='gazebo_mono_fixed_noise_position_only',
                          max_solo_seconds=self.policy.max_solo, original_source_timestamps=True)
            self.status_pub.publish(String(data=json.dumps(result, allow_nan=False)))
            self.last_status = now

    def destroy_node(self):
        result = super().destroy_node()
        if getattr(self, 'lock_fd', None) is not None:
            os.close(self.lock_fd)
            self.lock_fd = None
        return result


def main():
    stopping = False
    def stop(*args):
        nonlocal stopping
        stopping = True
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, stop); signal.signal(signal.SIGTERM, stop)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = Manager(); executor.add_node(node)
        while not stopping:
            executor.spin_once(timeout_sec=.05)
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

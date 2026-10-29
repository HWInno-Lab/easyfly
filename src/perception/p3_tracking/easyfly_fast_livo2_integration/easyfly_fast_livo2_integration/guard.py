# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Bounded sensor admission for real FAST-LIVO2, preserving source data."""
from collections import OrderedDict
import math
import re
import time
import uuid

import numpy as np
from diagnostic_msgs.msg import DiagnosticArray
from easyfly_msgs.msg import LocalizationStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu
from easyfly_openvins_integration.geometry import covariance, quaternion
from easyfly_openvins_integration.node import ros_stamp, stamp_ns
from easyfly_openvins_integration.shutdown import init_owned_context, spin_owned_executor


class Admission:
    def __init__(self):
        self.clock = 0
        self.seen = {}
        self.receipts = {}
        self.failure = ""
        self.forwarded = dict(camera=0, imu=0, lidar=0)
        self.rejected = dict(camera=0, imu=0, lidar=0, pose=0)
        self.last_rejection = ""

    def fail(self, reason):
        if not self.failure:
            self.failure = reason

    def update_clock(self, stamp):
        if self.clock and stamp < self.clock:
            self.fail("clock_rollback")
        self.clock = stamp

    def fresh(self, stamp, receipt, now):
        return (
            self.clock > 0
            and stamp > 0
            and -0.2e9 <= self.clock - stamp <= 0.5e9
            and 0 <= now - receipt <= 0.5
        )

    def accept(self, kind, stamp, now):
        if self.failure:
            return False
        if not self.fresh(stamp, now, now):
            self.rejected[kind] += 1
            self.last_rejection = kind + "_stale_or_future"
            return False
        previous = self.seen.get(kind, 0)
        if stamp <= previous:
            self.rejected[kind] += 1
            self.fail(kind + "_rollback_or_duplicate")
            return False
        if kind == "imu" and previous and stamp - previous > 20_000_000:
            self.rejected[kind] += 1
            self.fail("imu_source_gap")
            return False
        self.seen[kind], self.receipts[kind] = stamp, now
        return True

    def ready(self, now):
        return not self.failure and all(
            self.fresh(self.seen.get(k, 0), self.receipts.get(k, float("-inf")), now)
            for k in ("camera", "imu", "lidar")
        )


class FastLivoGuard(Node):
    def __init__(self):
        super().__init__("fastlivo_guard")
        if not self.get_parameter("use_sim_time").value:
            raise ValueError("FAST-LIVO2 currently requires calibrated Gazebo time")
        self.engine = Admission()
        self.session = uuid.uuid4().hex
        self.calibration = False
        self.pose_stamp = 0
        self.tracking = {}
        self.pending = OrderedDict()
        self.camera_stamps = OrderedDict()
        self.last_state = None
        self.last_graph = 0.0
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(
            LocalizationStatus, "/localization/fast_livo2/status", qos
        )
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/localization/fast_livo2/pose", 16
        )
        self.pubs = {}
        self.input_topics = {}
        fixed = ParameterDescriptor(read_only=True)
        for kind, cls, topic in [
            ("camera", Image, "/camera/color/image_raw"),
            ("imu", Imu, "/livox/imu"),
            ("lidar", CustomMsg, "/livox/lidar"),
        ]:
            selected = self.declare_parameter(kind + "_topic", topic, fixed).value
            if (
                not isinstance(selected, str)
                or len(selected) > 256
                or not re.fullmatch(r"/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*", selected)
                or selected.startswith("/fastlivo/")
            ):
                raise ValueError("invalid or feedback sensor topic")
            self.input_topics[kind] = selected
            self.pubs[kind] = self.create_publisher(
                cls, "/fastlivo/input/" + kind, 16 if kind != "imu" else 512
            )
            # Point-cloud callbacks must not evict intervening original IMU samples.
            # Queue capacity is bounded; source/receipt admission remains unchanged.
            input_qos = QoSProfile(
                depth=512 if kind == "imu" else 8,
                reliability=(ReliabilityPolicy.RELIABLE if kind == "imu"
                             else ReliabilityPolicy.BEST_EFFORT)
            )
            self.create_subscription(cls, selected, lambda m, k=kind: self.sensor(k, m), input_qos)
        if len(set(self.input_topics.values())) != 3:
            raise ValueError("sensor input topics must be distinct")
        self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self.info, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, "/fastlivo/raw/odometry", self.pose, 32)
        self.create_subscription(DiagnosticArray, "/fastlivo/raw/visual_tracking", self.visual, 32)
        self.create_timer(0.05, self.report, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def clock_update(self):
        self.engine.update_clock(self.get_clock().now().nanoseconds)

    def info(self, m):
        expected = [337.3570508552852, 0.0, 320.5, 0.0, 337.3570508552852, 240.5, 0.0, 0.0, 1.0]
        if (
            m.width != 640
            or m.height != 480
            or m.header.frame_id != "camera_color_optical_frame"
            or not np.allclose(m.k, expected, rtol=0, atol=1e-6)
            or any(abs(v) > 1e-9 for v in m.d)
        ):
            self.engine.fail("camera_calibration_mismatch")
        else:
            self.calibration = True

    def sensor(self, kind, m):
        self.clock_update()
        if not self.calibration:
            return
        try:
            frame = {
                "camera": "camera_color_optical_frame",
                "imu": "livox_imu_frame",
                "lidar": "livox_frame",
            }[kind]
            if m.header.frame_id != frame:
                raise ValueError(kind + "_frame_mismatch")
            if kind == "camera":
                if (
                    m.width != 640
                    or m.height != 480
                    or m.encoding not in ("rgb8", "bgr8")
                    or m.step < m.width * 3
                    or len(m.data) != m.step * m.height
                ):
                    raise ValueError("invalid_camera_layout")
            elif kind == "imu":
                vals = [
                    getattr(v, a)
                    for v in (m.angular_velocity, m.linear_acceleration)
                    for a in ("x", "y", "z")
                ]
                if not all(math.isfinite(v) for v in vals) or max(abs(v) for v in vals) > 1000:
                    raise ValueError("invalid_imu")
            else:
                if (
                    not 3 <= m.point_num <= 200000
                    or m.point_num != len(m.points)
                    or any(not math.isfinite(v) for p in m.points for v in (p.x, p.y, p.z))
                ):
                    raise ValueError("invalid_lidar")
            stamp = stamp_ns(m)
            if self.engine.accept(kind, stamp, time.monotonic()):
                # Stop the entire estimator stream when one required sensor expires.
                # Other live measurements are observed but never accumulate behind a missing input.
                if self.engine.ready(time.monotonic()):
                    self.pubs[kind].publish(m)
                    self.engine.forwarded[kind] += 1
                if kind == "camera" and self.engine.ready(time.monotonic()):
                    self.camera_stamps[stamp] = None
                    while len(self.camera_stamps) > 128:
                        self.camera_stamps.popitem(last=False)
        except (ValueError, TypeError) as e:
            self.engine.rejected[kind] += 1
            self.engine.fail(str(e))

    def visual(self, m):
        try:
            if len(m.status) != 1 or m.status[0].name != "fast_livo2/visual_tracking":
                raise ValueError("visual_identity")
            data = {v.key: v.value for v in m.status[0].values}
            count = int(data["tracked_features"])
            if (
                set(data) != {"tracked_features", "initialized"}
                or data["initialized"] not in ("0", "1")
                or not 0 <= count <= 10000
            ):
                raise ValueError("visual_payload")
            self.tracking[stamp_ns(m)] = (count, data["initialized"] == "1", time.monotonic())
            if len(self.tracking) > 64:
                self.tracking.pop(next(iter(self.tracking)))
            self.drain()
        except (ValueError, KeyError):
            self.engine.fail("invalid_visual_diagnostic")

    def pose(self, m):
        self.clock_update()
        try:
            p, q = m.pose.pose.position, m.pose.pose.orientation
            if (
                m.header.frame_id != "camera_init"
                or m.child_frame_id != "aft_mapped"
                or not all(math.isfinite(v) for v in (p.x, p.y, p.z))
            ):
                raise ValueError("raw_pose_contract")
            quaternion([q.x, q.y, q.z, q.w])
            covariance(m.pose.covariance)
            t = stamp_ns(m)
            if t <= self.pose_stamp or t in self.pending:
                raise ValueError("raw_pose_rollback")
            self.pending[t] = (m, time.monotonic())
            if len(self.pending) > 32:
                raise ValueError("raw_pose_queue_budget")
            self.drain()
        except ValueError as e:
            self.engine.rejected["pose"] += 1
            self.engine.fail(str(e))

    def drain(self):
        now = time.monotonic()
        for t, (m, receipt) in list(self.pending.items()):
            if t not in self.tracking:
                if now - receipt > 0.5:
                    self.pending.pop(t)
                continue
            tracks, initialized, tr = self.tracking[t]
            self.pending.pop(t)
            if (
                not self.engine.ready(now)
                or tracks < 8
                or not initialized
                or not self.engine.fresh(t, receipt, now)
                or not self.engine.fresh(t, tr, now)
            ):
                continue
            # Floating seconds in upstream preserve the actual camera update instant
            # within one nanosecond; no time fitting or delayed-current replacement.
            if not any(abs(t - c) <= 1 for c in self.camera_stamps):
                self.engine.fail("output_not_at_forwarded_camera_time")
                continue
            if t <= self.pose_stamp:
                self.engine.fail("admitted_pose_rollback")
                continue
            out = PoseWithCovarianceStamped()
            out.header = m.header
            out.header.frame_id = "fastlivo_odom"
            out.pose = m.pose
            self.pose_pub.publish(out)
            self.pose_stamp = t
            self.report()

    def report(self):
        self.clock_update()
        now = time.monotonic()
        if now - self.last_graph > 0.5:
            self.last_graph = now
            for topic in [
                *self.input_topics.values(),
                "/camera/color/camera_info",
                "/fastlivo/raw/odometry",
                "/fastlivo/raw/visual_tracking",
                "/fastlivo/input/camera",
                "/fastlivo/input/imu",
                "/fastlivo/input/lidar",
            ]:
                if self.count_publishers(topic) > 1:
                    self.engine.fail("duplicate_producer:" + topic)
        e = self.engine
        state = LocalizationStatus()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = "fastlivo_odom"
        state.backend_id = "fast_livo2"
        state.session_id = self.session
        state.tracked_frame = "livox_imu_frame"
        fresh_pose = self.pose_stamp and e.fresh(
            self.pose_stamp, self.tracking.get(self.pose_stamp, (0, 0, float("-inf")))[2], now
        )
        state.state = (
            state.RESET_REQUIRED
            if e.failure
            else state.ESTIMATING
            if e.ready(now) and fresh_pose
            else state.STALE
            if self.pose_stamp
            else state.INITIALIZING
        )
        state.reason = e.failure or (
            "tracking"
            if state.state == state.ESTIMATING
            else "waiting for fresh three-sensor visual estimate"
        )
        state.pose_valid = state.state == state.ESTIMATING
        state.covariance_available = True
        state.tracking_quality_available = True
        state.pose_stamp = ros_stamp(self.pose_stamp)
        state.camera_stamp = ros_stamp(e.seen.get("camera", 0))
        state.imu_stamp = ros_stamp(e.seen.get("imu", 0))
        state.camera_forwarded = e.forwarded["camera"]
        state.imu_forwarded = e.forwarded["imu"]
        state.camera_rejected = e.rejected["camera"]
        state.imu_rejected = e.rejected["imu"]
        state.pose_rejected = e.rejected["pose"]
        state.last_rejection = e.last_rejection
        self.status_pub.publish(state)
        if state.state != self.last_state:
            self.get_logger().info(f"FAST-LIVO2 state={state.state}: {state.reason}")
            self.last_state = state.state


def main(args=None):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    init_owned_context()
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = FastLivoGuard()
        executor.add_node(node)
        spin_owned_executor(executor)
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

# Copyright 2026 EasyFly Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Validate original-time landing observations; never publish flight commands."""

from collections import Counter, deque
from functools import wraps
import json
import math
import signal
import time

from easyfly_msgs.msg import LandingGuidanceTarget, LandingTarget, UAVState
from easyfly_msgs.srv import GetCapabilities
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from .geometry import finite, rotation, transform
from .extrinsics import Mount, MountHistory


def stamp(header):
    """Return positive nanoseconds, or zero for malformed timestamps."""
    value = header.stamp
    if value.sec < 0 or not 0 <= value.nanosec < 1000000000:
        return 0
    return value.sec * 1000000000 + value.nanosec


def timed_callback(function):
    """Measure opt-in wall duration without changing callback safety logic."""

    @wraps(function)
    def measured(self, *args, **kwargs):
        if not self.diagnostics:
            return function(self, *args, **kwargs)
        started = time.perf_counter()
        self.diag_counts[function.__name__] += 1
        try:
            return function(self, *args, **kwargs)
        finally:
            self.diag_max_ms[function.__name__] = max(
                self.diag_max_ms.get(function.__name__, 0.0),
                (time.perf_counter() - started) * 1000,
            )

    return measured


class DiagnosticExecutor(SingleThreadedExecutor):
    """Keep Humble's take semantics, additionally retaining native RMW timing."""

    def __init__(self, node, recorder=None):
        super().__init__()
        self.recorder = recorder or node.diagnostic_transport

    def _take_subscription(self, subscription):
        # Installed Humble's ordinary executor returns item[0] and discards info.
        # No second reader, QoS change, queue drain or message substitution.
        with subscription.handle:
            item = subscription.handle.take_message(
                subscription.msg_type, subscription.raw
            )
            if item is not None:
                metadata = dict(item[1])
                if hasattr(item[0], "header"):
                    metadata["header_source_ns"] = stamp(item[0].header)
                elif hasattr(item[0], "clock"):
                    clock = item[0].clock
                    metadata["header_source_ns"] = (
                        clock.sec * 1000000000 + clock.nanosec
                    )
                self.recorder(
                    subscription.topic_name, metadata
                )
                return item[0]
        return None


class LandingTransform(Node):
    """Pair source-time observations with bounded, trusted vehicle snapshots."""

    def __init__(self):
        super().__init__("landing_transform")
        descriptor = ParameterDescriptor(read_only=True)

        def parameter(name, default):
            return self.declare_parameter(name, default, descriptor).value

        self.enabled = parameter("configured", False)
        self.diagnostics = parameter("diagnostics", False)
        self.diag_counts = Counter()
        self.diag_max_ms = {}
        self.diag_source = {}
        self.diag_source_max_age = {}
        self.diag_transport = {}
        self.diag_pair = {}
        self.diag_last_report = 0.0
        self.diag_events = deque(maxlen=256)
        self.diag_dropped = 0
        self.diag_event_sequence = 0
        self.tag_id = parameter("selected_tag_id", -1)
        self.tag_size = parameter("tag_size_m", 0.0)
        self.optical_frame = parameter(
            "optical_frame", "camera_down_optical_frame"
        )
        self.translation = parameter("camera_translation_flu", [0.0, 0.0, 0.0])
        self.quaternion = parameter(
            "camera_quaternion_xyzw", [0.0, 0.0, 0.0, 0.0]
        )
        self.dynamic_camera = parameter("dynamic_camera", False)
        self.plane_z = parameter("landing_plane_z_m", float("nan"))
        self.plane_tolerance = parameter("plane_height_tolerance_m", 0.15)
        self.tilt = parameter("plane_tilt_limit_rad", 0.175)
        self.error_limit = parameter("reprojection_limit_px", 3.0)
        self.source_age = parameter("source_age_s", 0.3)
        self.state_age = parameter("state_age_s", 1.0)
        self.pair_gap = parameter("state_pair_gap_s", 0.1)
        numeric = (
            self.plane_tolerance,
            self.tilt,
            self.error_limit,
            self.source_age,
            self.state_age,
            self.pair_gap,
        )
        if (
            not self.optical_frame
            or not finite(numeric)
            or any(value <= 0 for value in numeric)
            or self.tilt >= math.pi / 4
            or self.state_age > 1.0
            or self.source_age > 1.0
            or self.pair_gap > self.source_age
        ):
            raise ValueError("invalid landing observation limits")
        if self.enabled:
            if (
                self.tag_id < 0
                or not math.isfinite(self.tag_size)
                or self.tag_size <= 0
                or len(self.translation) != 3
                or not finite(self.translation)
                or not math.isfinite(self.plane_z)
            ):
                raise ValueError(
                    "explicit tag, extrinsic and landing plane required"
                )
            if not self.dynamic_camera:
                rotation(self.quaternion)
        self.mount_history = MountHistory(
            self.optical_frame, self.source_age, self.pair_gap
        ) if self.dynamic_camera else None
        self.mount_sub = None
        self.states = deque(maxlen=64)
        self.watermark = 0
        self.published_watermark = 0
        self.clock_last = 0
        self.clock_progress = time.monotonic()
        self.last_good = None
        self.last_receipt = 0.0
        self.pending = None
        self.pending_sources = deque(maxlen=32)
        self.generation = 0
        self.reset_count = 0
        self.last_processing = time.monotonic()
        self.publisher = self.create_publisher(
            LandingGuidanceTarget, "/landing/guidance_target", 1
        )
        self.create_receivers()
        self.timer = self.create_timer(
            0.05, self.watchdog, clock=rclpy.clock.Clock()
        )
        self.capability = self.create_service(
            GetCapabilities,
            "/platform/precision_observation/capabilities",
            self.capabilities,
        )
        if self.diagnostics:
            self.diag_timer = self.create_timer(
                # Leave jitter margin for the strict >=1s flush guard: a timer
                # firing microscopically early must not turn every other tick
                # into a skipped flush and overwrite two seconds of evidence.
                1.05, self.diagnostic_snapshot, clock=rclpy.clock.Clock()
            )

    def create_receivers(self):
        """Bind volatile readers; observations favor latest data over repair."""
        generation = self.generation
        self.state_sub = self.create_subscription(
            UAVState, "/uav_state",
            lambda message: self.receive_state(message, generation),
            QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE),
        )
        self.target_sub = self.create_subscription(
            LandingTarget, "/landing_target",
            lambda message: self.receive_target(message, generation),
            QoSProfile(
                depth=1, durability=DurabilityPolicy.VOLATILE,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
        )
        if self.dynamic_camera:
            self.mount_sub = self.create_subscription(
                TransformStamped, "/landing/camera_mount",
                lambda message: self.receive_mount(message, generation),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE),
            )

    def receive_mount(self, message, generation):
        """Observe measured extrinsics; never use a command or latest-TF guess."""
        if generation != self.generation or not self.processing_valid():
            return
        if not self.clock_valid():
            return
        received = time.monotonic()
        p, q = message.transform.translation, message.transform.rotation
        try:
            endpoints = self.get_publishers_info_by_topic('/landing/camera_mount')
            if len(endpoints) > 1:
                self.mount_history.invalidate('dynamic_mount_source_missing_or_ambiguous')
            if not endpoints:
                return  # Wait for graph discovery; no measurement is admitted.
            gid = bytes(endpoints[0].endpoint_gid)
            if not 16 <= len(gid) <= 32 or not any(gid):
                self.mount_history.invalidate('invalid_dynamic_mount_publisher_identity')
            accepted = self.mount_history.receive(
                Mount(stamp(message.header), received, (p.x, p.y, p.z),
                      (q.x, q.y, q.z, q.w)),
                message.header.frame_id, message.child_frame_id,
                gid.hex(), self.get_clock().now().nanoseconds,
                received,
            )
            if accepted:
                self.try_pending()
        except ValueError as error:
            self.invalid(str(error))

    def receive_state(self, message, generation):
        """Ignore a callback already scheduled from a retired reader."""
        if generation == self.generation:
            self.state(message)

    def receive_target(self, message, generation):
        """Retired callbacks cannot clear or refresh new-generation evidence."""
        if generation == self.generation:
            self.target(message)

    def reset_receivers(self):
        """Discard local unread history, pending geometry and cached state."""
        self.generation += 1
        self.reset_count += 1
        self.states.clear()
        self.pending = None
        self.last_good = None
        self.last_receipt = 0.0
        # Preserve the already processed source watermark, not an assumed ROS
        # now barrier: /clock may itself be queued or the source slightly future.
        self.invalid("receiver_processing_gap")
        self.destroy_subscription(self.state_sub)
        self.destroy_subscription(self.target_sub)
        if self.mount_sub is not None:
            self.destroy_subscription(self.mount_sub)
            self.mount_history.samples.clear()
        self.create_receivers()
        self.get_logger().warning(
            "receiver_processing_gap: volatile readers rebuilt; generation="
            + str(self.generation)
        )

    def processing_valid(self, advance=True):
        """Use only local steady progress to reject a processing blackout."""
        now = time.monotonic()
        gap = now - self.last_processing
        if advance:
            self.last_processing = now
        if gap < 0 or gap > self.source_age:
            self.last_processing = now
            self.reset_receivers()
            return False
        return True

    def diagnostic_source(self, name, message):
        """Record source age at callback entry, not a new freshness decision."""
        if self.diagnostics:
            source = stamp(message.header)
            age = (self.get_clock().now().nanoseconds - source) * 1e-9
            self.diag_source[name] = {
                "stamp_ns": source,
                "entry_ros_age_s": age,
            }
            self.diag_source_max_age[name] = max(
                self.diag_source_max_age.get(name, age), age
            )

    def diagnostic_transport(self, topic, info):
        """Record system-clock RMW metadata; never use it for flight admission."""
        if topic not in ("/uav_state", "/landing_target", "/clock"):
            return
        taken = time.time_ns()
        source = info.get("source_timestamp", 0)
        received = info.get("received_timestamp", 0)
        previous = self.diag_transport.get(topic, {})
        transport = (received - source) * 1e-6 if 0 < source <= received else None
        queued = (taken - received) * 1e-6 if 0 < received <= taken else None
        self.diag_transport[topic] = {
            "source_wall_ns": source,
            "received_wall_ns": received,
            "take_wall_ns": taken,
            "transport_ms": transport,
            "receive_to_take_ms": queued,
            "max_transport_ms": max(
                previous.get("max_transport_ms", 0), transport or 0
            ),
            "max_receive_to_take_ms": max(
                previous.get("max_receive_to_take_ms", 0), queued or 0
            ),
        }
        if getattr(self, "diagnostics", False):
            self.diagnostic_event(
                "native_take", source_ns=info.get("header_source_ns", 0),
                topic=topic, **self.diag_transport[topic]
            )

    def diagnostic_event(self, stage, source_ns=0, **fields):
        """Bounded per-frame correlation only; these times never authorize flight."""
        if not self.diagnostics:
            return
        self.diag_event_sequence += 1
        if len(self.diag_events) == self.diag_events.maxlen:
            self.diag_dropped += 1
        self.diag_events.append({
            "sequence": self.diag_event_sequence, "stage": stage,
            "source_ns": source_ns, "steady_s": time.monotonic(),
            "clock_ns": self.get_clock().now().nanoseconds,
            "generation": self.generation, **fields,
        })

    def diagnostic_snapshot(self, force=False):
        """Emit bounded aggregate diagnostics at most once per steady second."""
        steady = time.monotonic()
        if not force and steady - self.diag_last_report < 1.0:
            return
        self.diag_last_report = steady
        snapshot = {
            "clock_ns": self.get_clock().now().nanoseconds,
            "counts": dict(self.diag_counts),
            "max_wall_ms": self.diag_max_ms,
            "latest_source": self.diag_source,
            "max_entry_ros_age_s": self.diag_source_max_age,
            "pending_ns": stamp(self.pending[0].header)
            if self.pending
            else None,
            "pending_receipt_age_s": time.monotonic() - self.pending[1]
            if self.pending
            else None,
            "last_good_ns": stamp(self.last_good.header)
            if self.last_good
            else None,
            "newest_state_ns": stamp(self.states[-1][0].header)
            if self.states
            else None,
            "state_count": len(self.states),
            "watermark_ns": self.watermark,
            "published_watermark_ns": self.published_watermark,
            "pending_count": len(self.pending_sources),
            "rmw_timing": self.diag_transport,
            "pairing": self.diag_pair,
            "generation": self.generation,
            "reset_count": self.reset_count,
            "events": list(self.diag_events),
            "dropped_events": self.diag_dropped,
        }
        self.diag_events.clear()
        self.get_logger().info(
            "landing_diag " + json.dumps(snapshot, sort_keys=True)
        )

    def publish(self, output):
        """Measure only publication when explicitly enabled, not source ages."""
        if not self.diagnostics:
            self.publisher.publish(output)
            return
        started = time.perf_counter()
        self.diagnostic_event(
            "publish_start", stamp(output.header), valid=output.valid,
            reason=output.reason,
        )
        succeeded = False
        try:
            self.publisher.publish(output)
            succeeded = True
        finally:
            self.diagnostic_event(
                "publish_end", stamp(output.header), valid=output.valid,
                reason=output.reason, succeeded=succeeded,
            )
            self.diag_counts[
                "publish_valid" if output.valid else "publish_invalid"
            ] += 1
            self.diag_counts["reason:" + output.reason] += 1
            self.diag_max_ms["publish"] = max(
                self.diag_max_ms.get("publish", 0.0),
                (time.perf_counter() - started) * 1000,
            )

    def clock_valid(self):
        """Invalidate retained evidence on rewind or lack of clock progress."""
        now = self.get_clock().now().nanoseconds
        steady = time.monotonic()
        backwards = self.clock_last > 0 and now < self.clock_last
        if backwards:
            self.states.clear()
            self.watermark = 0
            self.published_watermark = 0
            self.last_good = None
            self.pending = None
            self.pending_sources.clear()
            self.last_receipt = 0.0
        if now != self.clock_last:
            self.clock_progress = steady
        self.clock_last = now
        valid = (
            now > 0
            and not backwards
            and steady - self.clock_progress <= self.source_age
        )
        if not valid:
            self.diagnostic_event(
                "clock_invalid", backwards=backwards,
                progress_gap_s=steady - self.clock_progress,
            )
            self.states.clear()
            self.pending_sources.clear()
            self.pending = None
            self.last_good = None
            self.last_receipt = 0.0
            if self.mount_history is not None:
                self.mount_history.samples.clear()
        return valid

    def fresh(self, header, receipt, age):
        """Check separate source-time and steady-receipt domains."""
        source = stamp(header)
        source_age = (self.get_clock().now().nanoseconds - source) * 1e-9
        arrival_age = time.monotonic() - receipt
        return (
            source > 0
            and -min(0.2, age) <= source_age <= age
            and 0 <= arrival_age <= age
        )

    @timed_callback
    def state(self, message):
        """Retain a small source-time history; defaults cannot authorize geometry."""
        if not self.processing_valid():
            return
        self.diagnostic_source("state", message)
        if not self.clock_valid():
            return
        received = time.monotonic()
        self.diagnostic_event(
            "state_entry", stamp(message.header), receipt_steady_s=received
        )
        values = (
            message.position_x,
            message.position_y,
            message.position_z,
            message.roll,
            message.pitch,
            message.yaw,
        )
        if (
            message.header.frame_id != "odom"
            or not message.connected
            or not message.pose_valid
            or not message.attitude_valid
            or not finite(values)
            or not self.fresh(message.header, received, self.state_age)
        ):
            self.states.clear()
            self.invalid("vehicle_state_invalid")
            return
        self.states.append((message, received))
        self.try_pending()

    def invalid(self, reason, source=None, preserve_pending=False):
        """Publish invalidity without refreshing an old source observation."""
        output = LandingGuidanceTarget()
        self.diagnostic_event(
            "invalid", stamp(source.header) if source is not None else 0,
            reason=reason, preserve_pending=preserve_pending,
            last_good_source_ns=stamp(self.last_good.header) if self.last_good else 0,
            last_good_receipt_steady_s=self.last_receipt,
            pending_count=len(self.pending_sources),
            ingress_watermark_ns=self.watermark,
            published_watermark_ns=self.published_watermark,
        )
        output.header.frame_id = "odom"
        output.tag_id = self.tag_id
        if source is not None:
            output.header.stamp = source.header.stamp
            output.source_header = source.header
        output.reason = reason
        self.last_good = None
        if not preserve_pending:
            self.pending = None
            self.pending_sources.clear()
        self.publish(output)

    @timed_callback
    def target(self, message):
        """Convert only the explicitly selected, increasing-stamp observation."""
        receipt = time.monotonic()
        self.diagnostic_event(
            "target_entry", stamp(message.header), receipt_steady_s=receipt
        )
        if not self.processing_valid():
            if (
                message.valid and message.tag_id == self.tag_id
                and message.header.frame_id == self.optical_frame
                and self.fresh(message.header, receipt, self.source_age)
            ):
                self.watermark = max(self.watermark, stamp(message.header))
            return
        self.diagnostic_source("target", message)
        if not self.enabled:
            self.invalid("not_configured", message)
            return
        if (
            not self.clock_valid()
            or not message.valid
            or message.tag_id != self.tag_id
            or message.header.frame_id != self.optical_frame
            or not self.fresh(message.header, receipt, self.source_age)
            or stamp(message.header) <= self.watermark
        ):
            self.invalid("source_invalid_or_replayed", message)
            return
        self.watermark = stamp(message.header)
        try:
            self.observation_quality(message)
        except ValueError:
            self.invalid("geometry_or_plane_invalid", message)
            return
        self.pending_sources.append((message, receipt))
        self.pending = self.pending_sources[-1]
        self.try_pending()

    def observation_quality(self, message):
        """Reject bad new evidence before retaining or falling back to history."""
        p, q = message.pose.position, message.pose.orientation
        if (
            not finite((p.x, p.y, p.z, message.reprojection_error_px,
                        message.tag_size_m, message.optical_depth_m))
            or p.z <= 0
            or abs(message.tag_size_m - self.tag_size) > 1e-6
            or not 0 <= message.reprojection_error_px <= self.error_limit
            or abs(message.optical_depth_m - p.z) > 1e-6
        ):
            raise ValueError("observation quality")
        rotation((q.x, q.y, q.z, q.w))

    def try_pending(self):
        """Choose newest safe history pair; never renew source stamp or receipt."""
        if not self.pending_sources:
            return
        if not self.clock_valid():
            self.invalid("expired_or_clock_invalid")
            return
        latest = self.pending_sources[-1][0]
        self.pending_sources = deque(
            (item for item in self.pending_sources
             if stamp(item[0].header) > self.published_watermark
             and self.fresh(item[0].header, item[1], self.source_age)),
            maxlen=32,
        )
        self.pending = self.pending_sources[-1] if self.pending_sources else None
        if self.pending is None:
            self.invalid("expired_or_clock_invalid", latest)
            return
        candidates = [
            item
            for item in self.states
            if self.fresh(item[0].header, item[1], self.state_age)
        ]
        if not candidates:
            self.diagnostic_event(
                "pair_pending", stamp(self.pending[0].header),
                first_receipt_steady_s=self.pending[1], state_count=0,
            )
            return
        selected = None
        for message, receipt in reversed(self.pending_sources):
            state, _ = min(
                candidates,
                key=lambda item: abs(
                    stamp(item[0].header) - stamp(message.header)
                ),
            )
            if abs(stamp(state.header) - stamp(message.header)) * 1e-9 <= self.pair_gap:
                selected = (message, receipt, state)
                break
        if selected is None:
            self.diagnostic_event(
                "pair_pending", stamp(self.pending[0].header),
                first_receipt_steady_s=self.pending[1], state_count=len(candidates),
                nearest_gap_s=min(
                    abs(stamp(item[0].header) - stamp(self.pending[0].header)) * 1e-9
                    for item in candidates
                ),
            )
            return
        message, receipt, state = selected
        if self.diagnostics:
            self.diagnostic_event(
                "pair_selected", stamp(message.header), first_receipt_steady_s=receipt,
                state_source_ns=stamp(state.header),
                state_receipt_steady_s=next(t for s, t in candidates if s is state),
                pair_gap_s=abs(stamp(state.header) - stamp(message.header)) * 1e-9,
                wait_s=time.monotonic() - receipt,
                vehicle_position=[state.position_x, state.position_y, state.position_z],
                vehicle_rpy=[state.roll, state.pitch, state.yaw],
                optical_position=[message.pose.position.x, message.pose.position.y,
                                  message.pose.position.z],
                optical_quaternion=[message.pose.orientation.x, message.pose.orientation.y,
                                    message.pose.orientation.z, message.pose.orientation.w],
            )
            waited = time.monotonic() - receipt
            self.diag_pair = {
                "target_ns": stamp(message.header),
                "state_ns": stamp(state.header),
                "wait_ms": waited * 1000,
                "max_wait_ms": max(
                    self.diag_pair.get("max_wait_ms", 0), waited * 1000
                ),
            }
        p, q = message.pose.position, message.pose.orientation
        translation, quaternion = self.translation, self.quaternion
        if self.mount_history is not None:
            try:
                if self.count_publishers('/landing/camera_mount') != 1:
                    raise ValueError('dynamic_mount_source_missing_or_ambiguous')
                translation, quaternion = self.mount_history.at(
                    stamp(message.header), self.get_clock().now().nanoseconds,
                    time.monotonic(),
                )
            except ValueError as error:
                self.invalid(str(error), preserve_pending=not self.mount_history.failure)
                return
        center = normal = None
        try:
            self.observation_quality(message)
            center, normal = transform(
                (p.x, p.y, p.z),
                (q.x, q.y, q.z, q.w),
                translation,
                quaternion,
                (state.position_x, state.position_y, state.position_z),
                (state.roll, state.pitch, state.yaw),
            )
            if abs(center[2] - self.plane_z) > self.plane_tolerance or abs(
                normal[2]
            ) < math.cos(self.tilt):
                raise ValueError("landing plane inconsistent")
        except ValueError as error:
            if self.diagnostics:
                self.diagnostic_event(
                    "geometry_rejected", stamp(message.header), error=str(error),
                    center=center, normal=normal,
                    plane_error_m=abs(center[2] - self.plane_z) if center else None,
                    normal_alignment=abs(normal[2]) if normal else None,
                    height_exceeded=abs(center[2] - self.plane_z) > self.plane_tolerance
                    if center else None,
                    tilt_exceeded=abs(normal[2]) < math.cos(self.tilt) if normal else None,
                    plane_z_m=self.plane_z, plane_tolerance_m=self.plane_tolerance,
                    tilt_limit_rad=self.tilt,
                )
            self.invalid("geometry_or_plane_invalid", message)
            return
        output = LandingGuidanceTarget()
        output.header.stamp = message.header.stamp
        output.header.frame_id = "odom"
        output.source_header = message.header
        output.tag_id = message.tag_id
        output.tag_size_m = message.tag_size_m
        output.position.x, output.position.y, output.position.z = center
        (
            output.plane_normal.x,
            output.plane_normal.y,
            output.plane_normal.z,
        ) = normal
        output.valid = True
        output.reason = ("dynamic_camera_original_time_paired_observation" if self.dynamic_camera
                         else "fixed_camera_source_paired_observation")
        self.published_watermark = stamp(message.header)
        self.pending_sources = deque(
            (item for item in self.pending_sources
             if stamp(item[0].header) > self.published_watermark), maxlen=32
        )
        self.pending = self.pending_sources[-1] if self.pending_sources else None
        self.last_good = output
        self.last_receipt = receipt
        self.publish(output)

    @timed_callback
    def watchdog(self):
        """Expiry publishes invalidity, never a freshly stamped old target."""
        if not self.processing_valid():
            return
        self.try_pending()
        clock_valid = self.clock_valid()
        if self.mount_history is not None and self.last_good is not None:
            try:
                if self.count_publishers('/landing/camera_mount') != 1:
                    raise ValueError('dynamic_mount_source_missing_or_ambiguous')
                self.mount_history.at(stamp(self.last_good.header),
                                      self.get_clock().now().nanoseconds, time.monotonic())
            except ValueError as error:
                self.invalid(str(error))
        if (
            not clock_valid
            or self.last_good is None
            or not self.fresh(
                self.last_good.header, self.last_receipt, self.source_age
            )
        ):
            self.invalid(
                "not_configured"
                if not self.enabled
                else (
                    "vehicle_source_pair_pending"
                    if clock_valid and self.pending is not None
                    else "expired_or_clock_invalid"
                ),
                preserve_pending=clock_valid and self.pending is not None,
            )

    def capabilities(self, _request, response):
        """Expose observation readiness separately from the landing controller."""
        response.header.stamp = self.get_clock().now().to_msg()
        response.names = ["dynamic_camera_landing_observation" if self.dynamic_camera
                          else "fixed_camera_landing_observation"]
        response.implemented = [True]
        response.ready = [
            self.enabled
            and self.processing_valid(advance=False)
            and self.clock_valid()
            and self.last_good is not None
            and self.fresh(
                self.last_good.header, self.last_receipt, self.source_age
            )
        ]
        response.details = [
            "Observation only; requires an explicit plane and original-time measured mounts."
            if self.dynamic_camera else
            "Observation only; requires explicit fixed extrinsic and static plane."
        ]
        return response


def main(args=None):
    """Run the installed observation node."""
    run_observer(LandingTransform, args=args)


def run_observer(node_factory, args=None):
    """Run the observation node with bounded, ordinary executor cleanup."""
    # Native take can still be converting a message when rclpy's default signal
    # handler invalidates its context. Defer shutdown until our executor exits;
    # never suppress conversion/runtime errors from an active context.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    executor = None
    stopping = False
    previous = {}

    def request_stop(_signal, _frame):
        nonlocal stopping
        stopping = True

    try:
        for stop_signal in (signal.SIGINT, signal.SIGTERM):
            previous[stop_signal] = signal.signal(stop_signal, request_stop)
        node = node_factory()
        executor = DiagnosticExecutor(node) if node.diagnostics else SingleThreadedExecutor()
        executor.add_node(node)
        while not stopping and rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except rclpy_implementation.RCLError:
        if rclpy.ok():
            raise
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            if node.diagnostics:
                node.diagnostic_snapshot(force=True)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for stop_signal, handler in previous.items():
            signal.signal(stop_signal, handler)

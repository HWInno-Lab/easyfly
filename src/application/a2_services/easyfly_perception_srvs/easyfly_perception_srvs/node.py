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

"""Query snapshots with explicit readiness, freshness and geometry validity."""
from easyfly_perception_contracts import WorldInput, perception_version
from easyfly_msgs.srv import GetPerceptionV2

import copy
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from easyfly_msgs.msg import (
    Target3DArray,
    ObstacleInfoArray,
    UAVState,
    LandingTarget,
)
from easyfly_msgs.srv import GetPerception, GetCapabilities


class PerceptionServices(Node):
    def __init__(self):
        super().__init__("easyfly_perception_srvs")
        readonly = ParameterDescriptor(read_only=True)
        self.declare_parameter("targets_topic", "/target_positions", readonly)
        self.declare_parameter("obstacles_topic", "/obstacles", readonly)
        self.declare_parameter("max_age", 1.0)
        result = self.validate_parameters([self.get_parameter("max_age")])
        if not result.successful:
            raise ValueError(result.reason)
        self.snapshots = {}
        self.state = None
        self.state_time = 0.0
        self.landing = None
        self.landing_clock = 0
        self.landing_progress = time.monotonic()
        self.precision = None
        self.precision_time = 0.0
        self.precision_future = None
        self.precision_deadline = 0.0
        self.precision_client = self.create_client(
            GetCapabilities, "/platform/precision_landing/capabilities"
        )
        self.topics = {
            k: self.get_parameter(k + "_topic").value
            for k in ("targets", "obstacles")
        }
        self.native_targets = None
        self.v2 = perception_version(self) == 2
        if self.v2:
            self.topics['targets'] = '/perception/v2/world_targets'
            self.native_input = WorldInput(self, self.receive_native, self.invalidate_native)
        else:
            self.create_subscription(
                Target3DArray,
                self.topics["targets"],
                lambda m: self.receive("targets", m),
                10,
            )
        self.create_subscription(
            ObstacleInfoArray,
            self.topics["obstacles"],
            lambda m: self.receive("obstacles", m),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            UAVState, "/uav_state", self.receive_state, 10
        )
        self.create_subscription(
            LandingTarget, "/landing_target", self.receive_landing, 10
        )
        self.create_timer(
            0.05,
            self.landing_ready,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self.create_timer(
            0.1,
            self.poll_precision,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self.create_service(GetPerceptionV2, "/platform/perception/v2/query", self.query_v2)
        self.create_service(
            GetPerception, "/platform/perception/query", self.query
        )
        self.create_service(
            GetCapabilities,
            "/platform/perception/capabilities",
            self.capabilities,
        )
        self.add_on_set_parameters_callback(self.validate_parameters)

    def receive_native(self, native, numeric):
        self.native_targets = native
        self.receive('targets', numeric)

    def invalidate_native(self, reason):
        self.native_targets = None
        self.snapshots.pop('targets', None)

    def query_v2(self, request, response):
        if request.kind == 'targets' and (not self.v2 or self.native_targets is None):
            response.status = response.NOT_READY
            response.message = 'native V2 source required; legacy recordings are read-only V1'
            response.age_sec = -1.
            return response
        legacy = self.query(request, GetPerception.Response())
        for key in ['status', 'message', 'source_topic', 'observed_at', 'age_sec', 'obstacles']:
            setattr(response, key, getattr(legacy, key))
        if request.kind == 'targets':
            response.source_topic = '/perception/v2/world_targets'
            if legacy.status == legacy.VALID:
                response.targets = copy.deepcopy(self.native_targets)
                aliases = {t.track_id for t in legacy.targets.targets}
                response.targets.targets = [t for t in response.targets.targets if t.key.legacy_raw_id in aliases]
        return response

    @staticmethod
    def validate_parameters(parameters):
        for p in parameters:
            if (
                p.name != "max_age"
                or type(p.value) is not float
                or not math.isfinite(p.value)
                or not 0.05 <= p.value <= 10
            ):
                return SetParametersResult(
                    successful=False,
                    reason="only max_age(double, 0.05..10s) is writable",
                )
        return SetParametersResult(successful=True)

    def receive(self, kind, message):
        self.snapshots[kind] = (message, time.monotonic())

    def receive_state(self, message):
        self.state, self.state_time = message, time.monotonic()

    def receive_landing(self, message):
        self.landing = (message, time.monotonic())

    def landing_ready(self):
        now, wall = self.get_clock().now().nanoseconds, time.monotonic()
        rollback = now < self.landing_clock
        if rollback:
            self.landing = None
        if rollback or now > self.landing_clock:
            self.landing_progress = wall
        self.landing_clock = now
        limit = self.get_parameter("max_age").value
        if (
            rollback
            or now <= 0
            or wall - self.landing_progress > limit
            or self.landing is None
        ):
            return False
        message, received = self.landing
        stamp = message.header.stamp
        source = stamp.sec * 1000000000 + stamp.nanosec
        if (
            stamp.sec < 0
            or stamp.nanosec >= 1000000000
            or source <= 0
            or not message.header.frame_id.endswith("_optical_frame")
        ):
            return False
        p, q = message.pose.position, message.pose.orientation
        values = (
            p.x,
            p.y,
            p.z,
            q.x,
            q.y,
            q.z,
            q.w,
            message.yaw_rad,
            message.optical_depth_m,
            message.reprojection_error_px,
        )
        return (
            message.valid
            and message.tag_id >= 0
            and all(math.isfinite(v) for v in values)
            and abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) <= 1e-4
            and p.z > 0
            and abs(message.optical_depth_m - p.z) <= 1e-6
            and abs(message.yaw_rad) <= math.pi
            and message.reprojection_error_px >= 0
            and -0.2 <= (now - source) / 1e9 <= limit
            and wall - received <= limit
        )

    def header_health(self, header, received):
        stamp = header.stamp
        if (
            header.frame_id != "odom"
            or stamp.sec < 0
            or stamp.nanosec >= 1000000000
            or (stamp.sec == 0 and stamp.nanosec == 0)
        ):
            return GetPerception.Response.INVALID, -1.0
        age = (
            self.get_clock().now().nanoseconds
            - stamp.sec * 1000000000
            - stamp.nanosec
        ) / 1e9
        if age < -0.2:
            return GetPerception.Response.INVALID, age
        if (
            max(age, time.monotonic() - received)
            > self.get_parameter("max_age").value
        ):
            return GetPerception.Response.STALE, age
        return GetPerception.Response.VALID, age

    def health(self, kind):
        if kind not in self.snapshots:
            return GetPerception.Response.NOT_READY, -1.0
        msg, received = self.snapshots[kind]
        status, age = self.header_health(msg.header, received)
        if status != GetPerception.Response.VALID:
            return status, age
        items = msg.targets if kind == "targets" else msg.obstacles
        if len(items) > 10000:
            return GetPerception.Response.INVALID, age
        for item in items:
            h, _ = self.header_health(item.header, received)
            if h != GetPerception.Response.VALID:
                return h, age
            if kind == "targets":
                valid = item.position_valid and all(
                    math.isfinite(v)
                    for v in (
                        item.position_x,
                        item.position_y,
                        item.position_z,
                        item.confidence,
                        item.distance,
                    )
                )
                valid = (
                    valid and 0 <= item.confidence <= 1 and item.distance >= 0
                )
            else:
                valid = all(
                    math.isfinite(v)
                    for v in (
                        item.center_x,
                        item.center_y,
                        item.center_z,
                        item.size_x,
                        item.size_y,
                        item.size_z,
                        item.velocity_x,
                        item.velocity_y,
                        item.velocity_z,
                    )
                )
                valid = (
                    valid and min(item.size_x, item.size_y, item.size_z) >= 0
                )
            if not valid:
                return GetPerception.Response.INVALID, age
        return status, age

    def query(self, request, response):
        response.age_sec = -1.0
        if request.kind not in self.topics:
            response.status, response.message = (
                response.UNSUPPORTED,
                "supported kinds: targets, obstacles",
            )
            return response
        response.source_topic = self.topics[request.kind]
        if (
            not math.isfinite(request.range_limit)
            or request.range_limit > 10000
        ):
            response.status, response.message = (
                response.INVALID,
                "invalid query range",
            )
            return response
        response.status, response.age_sec = self.health(request.kind)
        response.message = {
            0: "not_ready",
            1: "valid",
            2: "stale",
            3: "invalid",
        }[response.status]
        if request.kind in self.snapshots:
            response.observed_at = self.snapshots[request.kind][0].header.stamp
        if response.status != response.VALID:
            return response
        msg = copy.deepcopy(self.snapshots[request.kind][0])
        if request.range_limit > 0:
            if self.state is None:
                response.status, response.message = (
                    response.NOT_READY,
                    "range query requires UAVState",
                )
                return response
            s = self.state
            state_status, _ = self.header_health(s.header, self.state_time)
            if (
                state_status != response.VALID
                or not s.connected
                or not s.pose_valid
                or not all(
                    math.isfinite(v)
                    for v in (s.position_x, s.position_y, s.position_z)
                )
            ):
                response.status = (
                    state_status
                    if state_status != response.VALID
                    else response.INVALID
                )
                response.message = "invalid or stale query origin"
                return response

            def within(item):
                xyz = (
                    (item.position_x, item.position_y, item.position_z)
                    if request.kind == "targets"
                    else (item.center_x, item.center_y, item.center_z)
                )
                return (
                    math.dist(xyz, (s.position_x, s.position_y, s.position_z))
                    <= request.range_limit
                )

            if request.kind == "targets":
                msg.targets = [item for item in msg.targets if within(item)]
            else:
                msg.obstacles = [
                    item for item in msg.obstacles if within(item)
                ]
        if request.kind == "targets":
            response.targets = msg
        else:
            response.obstacles = msg
        return response

    def poll_precision(self):
        """Query controller readiness without blocking the snapshot service."""
        wall = time.monotonic()
        if self.precision_future is not None:
            if self.precision_future.done():
                try:
                    self.precision = self.precision_future.result()
                    self.precision_time = wall
                except Exception:
                    self.precision = None
                self.precision_future = None
            elif wall > self.precision_deadline:
                self.precision_client.remove_pending_request(
                    self.precision_future
                )
                self.precision_future = None
                self.precision = None
        elif self.precision_client.service_is_ready():
            self.precision_future = self.precision_client.call_async(
                GetCapabilities.Request()
            )
            self.precision_deadline = wall + 0.5

    def precision_ready(self):
        """Missing, malformed or expired controller responses are not ready."""
        response = self.precision
        if response is None or time.monotonic() - self.precision_time > 0.5:
            return False
        if len(response.names) != len(response.ready) or len(
            response.names
        ) != len(response.implemented):
            return False
        return any(
            name == "precision_landing" and implemented and ready
            for name, implemented, ready in zip(
                response.names, response.implemented, response.ready
            )
        )

    def capabilities(self, _request, response):
        response.header.stamp = self.get_clock().now().to_msg()
        response.header.frame_id = "odom"
        response.names = [
            "targets",
            "obstacles",
            "landing_detection",
            "precision_landing",
        ]
        response.implemented = [True, True, True, True]
        response.ready = [
            self.health(k)[0] == GetPerception.Response.VALID
            for k in ("targets", "obstacles")
        ] + [self.landing_ready(), self.precision_ready()]
        response.details = [
            "snapshot query; source may be absent",
            "geometry snapshot; no motion commands",
            "D4 separate /landing_target; explicit selected tag; perception only",
            "D5 /precision_land; live controller readiness, cache <=0.5s; not auto-enabled",
        ]
        return response


def main():
    rclpy.init()
    node = PerceptionServices()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except rclpy_implementation.RCLError:
        # Humble may race shutdown while constructing the executor wait set.
        # Only an already-invalid context is normal shutdown, not a live error.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

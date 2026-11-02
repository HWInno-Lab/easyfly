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

from easyfly_perception_contracts import WorldInput, perception_version
from easyfly_msgs.msg import ClassifiedObstacleArrayV2

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from easyfly_msgs.msg import (
    ObstacleInfoArray,
    Target3DArray,
    ClassifiedObstacleArray,
)
from easyfly_msgs.srv import GetCapabilities
from .core import classify


class ObstacleClassifier(Node):
    def __init__(self):
        super().__init__('obstacle_classifier')
        frozen = ParameterDescriptor(read_only=True)
        for name, default in (
            ('max_age', 0.5),
            ('association_ttl', 0.5),
            ('association_margin', 0.3),
            ('min_confidence', 0.7),
            ('time_tolerance', 0.15),
        ):
            self.declare_parameter(name, default, frozen)
        self.options = [
            self.get_parameter(name).value
            for name in (
                'max_age',
                'association_ttl',
                'association_margin',
                'min_confidence',
                'time_tolerance',
            )
        ]
        age, ttl, margin, confidence, tolerance = self.options
        if (
            not all(math.isfinite(v) for v in self.options)
            or not 0.05 <= age <= 10
            or not 0.05 <= ttl <= 10
            or not 0 <= margin <= 2
            or not 0 <= confidence <= 1
            or not 0 <= tolerance <= min(0.5, age, ttl)
        ):
            raise ValueError('invalid classification budgets')
        self.geometry = self.targets = None
        self.last_clock = 0
        self.progress = time.monotonic()
        self.publisher = self.create_publisher(
            ClassifiedObstacleArray, '/obstacles_classified', 10
        )
        self.create_subscription(
            ObstacleInfoArray,
            '/obstacles',
            self.receive_geometry,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.native_targets = None
        self.v2 = perception_version(self) == 2
        self.native_pub = self.create_publisher(ClassifiedObstacleArrayV2, '/perception/v2/obstacles_classified', 10)
        if self.v2:
            self.native_input = WorldInput(self, self.receive_native, self.invalidate_native)
        else:
            self.create_subscription(
                Target3DArray, '/target_positions', self.receive_targets, 10
            )

        self.create_timer(
            0.05, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.create_service(
            GetCapabilities,
            '/platform/classification/capabilities',
            self.capabilities,
        )

    def receive_native(self, native, numeric):
        self.native_targets = native
        self.receive_targets(numeric)

    def invalidate_native(self, reason):
        self.native_targets = self.targets = None

    def receive_geometry(self, message):
        self.geometry = (message, time.monotonic())
        self.publish()

    def receive_targets(self, message):
        self.targets = (message, time.monotonic())
        self.publish()

    def output(self):
        now, wall = self.get_clock().now().nanoseconds, time.monotonic()
        rollback = now < self.last_clock
        if rollback:
            self.targets = self.geometry = None
            self.progress = wall
        elif now > self.last_clock:
            self.progress = wall
        self.last_clock = now
        if rollback or now <= 0 or wall - self.progress > self.options[0]:
            return ClassifiedObstacleArray(
                reason='clock not ready/paused/reset'
            )
        if not self.geometry or wall - self.geometry[1] > self.options[0]:
            return ClassifiedObstacleArray(
                reason='geometry not received/receipt stale'
            )
        targets = (
            self.targets[0]
            if self.targets and wall - self.targets[1] <= min(self.options[:2])
            else None
        )
        return classify(self.geometry[0], targets, now, *self.options)

    def publish(self):
        output = self.output()
        self.publisher.publish(output)
        if self.v2:
            native = ClassifiedObstacleArrayV2(header=output.header, geometry=output)
            if self.native_targets is not None and self.targets is not None and time.monotonic() - self.targets[1] <= min(self.options[:2]):
                native.semantic_source = self.native_targets
            self.native_pub.publish(native)

    def capabilities(self, request, response):
        output = self.output()
        response.header.stamp = self.get_clock().now().to_msg()
        response.names, response.implemented = ['obstacle_classification'], [
            True
        ]
        response.ready = [
            output.geometry_valid
            and any(item.association_valid for item in output.obstacles)
        ]
        response.details = [
            output.reason
            + '; independent copies, original occupancy unchanged'
        ]
        return response


def main():
    rclpy.init()
    node = None
    try:
        node = ObstacleClassifier()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

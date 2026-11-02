# Copyright 2026 EasyFly Project
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
"""Simulation gimbal node; never publishes aircraft control commands."""

from easyfly_perception_contracts import WorldInput, perception_version, identity

import math
import time

from easyfly_msgs.msg import GimbalState, Target3DArray, UAVState
from easyfly_msgs.srv import SetGimbalAngle, SetGimbalTracking
import rclpy
from rclpy.node import Node
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool

from easyfly_gimbal_control.model import SimulatedGimbal, target_angles


class GimbalNode(Node):
    """Gate tracking on fresh explicitly selected observations."""

    def __init__(self):
        """Create the simulation backend and ROS endpoints."""
        super().__init__("gimbal_control")

        def parameter(name, default):
            return self.declare_parameter(name, default).value

        limits = [
            (
                parameter(axis + "_min_deg", lo),
                parameter(axis + "_max_deg", hi),
            )
            for axis, lo, hi in [
                ("pitch", -90.0, 90.0),
                ("yaw", -180.0, 180.0),
                ("roll", -45.0, 45.0),
            ]
        ]
        self.device = SimulatedGimbal(limits, parameter("rate_deg_s", 45.0))
        self.observation_timeout = parameter("observation_timeout_s", 0.5)
        self.connection_timeout = parameter("connection_timeout_s", 0.5)
        rate = parameter("update_rate_hz", 50.0)
        for value in [self.observation_timeout, self.connection_timeout, rate]:
            if not math.isfinite(value) or value <= 0:
                raise ValueError("timeouts and update rate must be positive")
        self.tracking_id = -1
        self.uav = None
        self.targets = None
        self.uav_received = self.targets_received = 0.0
        self.heartbeat_enabled = True
        self.last_heartbeat = self.last_tick = time.monotonic()
        self.connected = True
        self.last_ros = None
        self.last_ros_progress = time.monotonic()
        self.publisher = self.create_publisher(
            GimbalState, "/gimbal/state", 10
        )
        self.create_subscription(
            UAVState, "/uav_state", self.receive_uav, qos_profile_sensor_data
        )
        self.native_targets = None
        self.tracking_key = None
        self.v2 = perception_version(self) == 2
        if self.v2:
            self.native_input = WorldInput(self, self.receive_native_targets, self.invalidate_native)
        else:
            self.create_subscription(
                Target3DArray,
                "/target_positions",
                self.receive_targets,
                qos_profile_sensor_data,
            )
        self.create_service(
            SetGimbalAngle, "/gimbal/set_angle", self.set_angle
        )
        self.create_service(
            SetGimbalTracking, "/gimbal/set_tracking", self.set_tracking
        )
        self.create_service(
            SetBool, "/gimbal/simulate_connection", self.simulate_connection
        )
        self.create_timer(
            1.0 / rate,
            self.tick,
            clock=rclpy.clock.Clock(
                clock_type=rclpy.clock.ClockType.STEADY_TIME
            ),
        )

    def receive_native_targets(self, native, numeric):
        self.native_targets = native
        if self.tracking_key is not None and not any(
                identity(t.key) == self.tracking_key and t.key.legacy_raw_id == self.tracking_id
                for t in native.targets):
            self.stop_tracking()
        self.receive_targets(numeric)

    def invalidate_native(self, reason):
        self.native_targets = self.targets = None
        if self.tracking_id >= 0:
            self.stop_tracking()

    def receive_uav(self, message):
        """Record both source and monotonic receipt evidence."""
        self.uav = message
        self.uav_received = time.monotonic()

    def receive_targets(self, message):
        """Replace the full observation batch without guessing identities."""
        self.targets = message
        self.targets_received = time.monotonic()

    def fresh(self, header, receipt):
        """Require odom, valid ROS source time and steady receipt age."""
        stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        return (
            header.frame_id == "odom"
            and stamp > 0
            and now > 0
            and 0 <= now - stamp <= self.observation_timeout
            and 0 <= time.monotonic() - receipt <= self.observation_timeout
        )

    def selected_angles(self):
        """Return only the selected identity using trusted pose/attitude."""
        uav, batch = self.uav, self.targets
        if (
            uav is None
            or batch is None
            or not uav.connected
            or not uav.pose_valid
            or not uav.attitude_valid
            or not self.fresh(uav.header, self.uav_received)
            or not self.fresh(batch.header, self.targets_received)
        ):
            raise ValueError("missing or stale target/UAV evidence")
        matches = [t for t in batch.targets if t.track_id == self.tracking_id]
        if len(matches) != 1:
            raise ValueError("selected identity missing or ambiguous")
        target = matches[0]
        if (
            not target.position_valid
            or not self.fresh(target.header, self.targets_received)
            or not math.isfinite(target.confidence)
            or not 0 <= target.confidence <= 1
        ):
            raise ValueError("invalid target observation")
        return target_angles(
            (
                target.position_x - uav.position_x,
                target.position_y - uav.position_y,
                target.position_z - uav.position_z,
            ),
            (uav.roll, uav.pitch, uav.yaw),
        )

    def stop_tracking(self):
        """Discard tracking identity and all pending motion."""
        self.tracking_id = -1
        self.tracking_key = None
        self.device.stop()

    def set_angle(self, request, response):
        """Manual accepted commands explicitly supersede tracking."""
        if not self.connected or self.get_clock().now().nanoseconds <= 0:
            response.message = "disconnected or ROS clock unavailable"
        elif self.device.command(
            (request.pitch_angle, request.yaw_angle, request.roll_angle)
        ):
            self.tracking_id = -1
            self.tracking_key = None
            response.success = True
            response.message = "accepted; simulated device has not arrived"
        else:
            response.message = "nonfinite or out-of-range angle rejected"
        return response

    def set_tracking(self, request, response):
        """Enable only an explicit currently valid nonnegative identity."""
        if not request.enabled:
            self.stop_tracking()
            response.success = True
            response.message = "tracking disabled; motion stopped"
            return response
        if not self.connected or request.target_id < 0:
            response.message = "disconnected or invalid explicit target ID"
            return response
        old_id = self.tracking_id
        self.tracking_id = request.target_id
        try:
            angles = self.selected_angles()
            if not self.device.command(angles):
                raise ValueError("target outside gimbal limits")
        except ValueError as error:
            self.tracking_id = old_id
            response.message = str(error)
            return response
        if self.v2:
            self.tracking_key = next(identity(t.key) for t in self.native_targets.targets
                                     if t.key.legacy_raw_id == self.tracking_id)
        response.success = True
        response.message = "tracking accepted (simulation only)"
        return response

    def simulate_connection(self, request, response):
        """Simulation-only heartbeat fault injection; recovery stays stopped."""
        self.heartbeat_enabled = request.data
        self.stop_tracking()
        if request.data:
            self.last_heartbeat = time.monotonic()
            self.connected = True
        response.success = True
        response.message = (
            "simulation heartbeat enabled"
            if request.data
            else (
                "simulation heartbeat stopped; connection expires at timeout"
            )
        )
        return response

    def tick(self):
        """Use steady watchdog even when ROS time is frozen or zero."""
        steady = time.monotonic()
        ros = self.get_clock().now().nanoseconds
        if self.heartbeat_enabled:
            self.last_heartbeat = steady
        self.connected = (
            steady - self.last_heartbeat <= self.connection_timeout
        )
        if (
            not self.connected
            or ros <= 0
            or (self.last_ros is not None and ros < self.last_ros)
        ):
            self.stop_tracking()
        elif self.tracking_id >= 0:
            try:
                if not self.device.command(self.selected_angles()):
                    self.stop_tracking()
            except ValueError:
                self.stop_tracking()
        if self.last_ros is None or ros != self.last_ros:
            self.last_ros_progress = steady
        elif steady - self.last_ros_progress > self.observation_timeout:
            self.stop_tracking()
        self.device.update(min(steady - self.last_tick, 0.1))
        self.last_tick, self.last_ros = steady, ros
        state = GimbalState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = "base_link"
        (
            state.pitch_angle,
            state.yaw_angle,
            state.roll_angle,
        ) = self.device.angles
        state.is_connected = self.connected
        state.is_tracking = self.tracking_id >= 0
        state.tracking_target_id = self.tracking_id
        state.gimbal_model = "SIMULATION_ONLY_rate_limited"
        self.publisher.publish(state)


def main():
    """Run with deterministic shutdown and no device compensation commands."""
    rclpy.init()
    node = None
    try:
        node = GimbalNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except rclpy_implementation.RCLError:
        # Humble can invalidate the wait-set context during SIGTERM.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.device.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

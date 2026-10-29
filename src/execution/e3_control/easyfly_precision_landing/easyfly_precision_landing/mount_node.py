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

"""Convert original measured single-axis encoder samples to camera extrinsics."""

import signal
import time

from geometry_msgs.msg import TransformStamped
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

from easyfly_precision_landing.extrinsics import Mount, MountHistory, measured_mount
from easyfly_precision_landing.geometry import finite


class MeasuredMount(Node):
    """No commands or truth input; one pinned original encoder source only."""

    def __init__(self):
        super().__init__('landing_camera_mount')
        fixed = ParameterDescriptor(read_only=True)

        def parameter(name, value):
            return self.declare_parameter(name, value, fixed).value
        self.joint = parameter('joint_name', '')
        self.frame = parameter('optical_frame', '')
        self.pivot = parameter('pivot_translation_flu', [float('nan')]*3)
        self.offset = parameter('optical_offset_flu_at_zero', [float('nan')]*3)
        self.axis = parameter('joint_axis_flu', [float('nan')]*3)
        self.zero = parameter('camera_quaternion_xyzw_zero', [float('nan')]*4)
        if (not parameter('configured', False) or not self.joint or not self.frame
                or not self.get_parameter('use_sim_time').value):
            raise ValueError('explicit calibrated simulation encoder configuration required')
        measured_mount(0, self.pivot, self.offset, self.axis, self.zero)
        self.history = MountHistory(self.frame)
        self.clock_ns = 0
        self.failure = ''
        self.publisher = self.create_publisher(TransformStamped, '/landing/camera_mount',
                                               qos_profile_sensor_data)
        self.subscription = self.create_subscription(JointState, '/landing/gimbal/joint_states',
                                                     self.receive, qos_profile_sensor_data)

    def receive(self, message):
        """Publish at most once per original stamp; downstream checks age/brackets."""
        if self.failure:
            return
        try:
            clock = self.get_clock().now().nanoseconds
            if clock <= 0:
                return
            if clock < self.clock_ns:
                raise ValueError('encoder_clock_rollback')
            self.clock_ns = clock
            endpoints = self.get_publishers_info_by_topic('/landing/gimbal/joint_states')
            if not endpoints:
                return
            if len(endpoints) != 1:
                raise ValueError('ambiguous_encoder_publishers')
            gid = bytes(endpoints[0].endpoint_gid)
            if not 16 <= len(gid) <= 32 or not any(gid):
                raise ValueError('missing_encoder_identity')
            if (message.header.frame_id != 'base_link' or message.name != [self.joint]
                    or len(message.position) != 1 or len(message.velocity) != 1
                    or not finite(message.velocity) or abs(message.velocity[0]) > 1.5):
                raise ValueError('invalid_encoder_fields')
            stamp = message.header.stamp.sec*1000000000 + message.header.stamp.nanosec
            position, quaternion = measured_mount(message.position[0], self.pivot,
                                                  self.offset, self.axis, self.zero)
            now = time.monotonic()
            sample = Mount(stamp, now, position, quaternion)
            if not self.history.receive(sample, 'base_link', self.frame, gid.hex(), clock, now):
                return
            out = TransformStamped()
            out.header = message.header
            out.child_frame_id = self.frame
            (out.transform.translation.x, out.transform.translation.y,
             out.transform.translation.z) = position
            (out.transform.rotation.x, out.transform.rotation.y,
             out.transform.rotation.z, out.transform.rotation.w) = quaternion
            self.publisher.publish(out)
        except (ValueError, TypeError, OverflowError) as error:
            self.failure = str(error)
            self.get_logger().error(self.failure+'; new configured session required')


def main():
    """Join normal ROS shutdown explicitly outside an asynchronous handler."""
    stopping = False

    def stop(*unused):
        nonlocal stopping
        stopping = True
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node = None
    try:
        node = MeasuredMount()
        while not stopping:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

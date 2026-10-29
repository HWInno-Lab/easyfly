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
"""Exercise real DDS source replacement, duplicate publishers and observer loss."""

import time

from mavros_msgs.msg import ExtendedState, State
import rclpy
from rclpy.node import Node

from easyfly_connection_manager.remote_observer import Observer


def test_native_source_epochs_and_observer_heartbeat():
    """Require unique live sources after actual DDS replacement and reader shutdown."""
    rclpy.init()
    node = Node('remote_observer_witness')
    observer = Observer({'simulation_time': False, 'profiles': {
        'sample': {'output_topic': '/unused_output', 'output_type': 'std_msgs/msg/String'}}})
    state = node.create_publisher(State, '/mavros/state', 10)
    landed = node.create_publisher(ExtendedState, '/mavros/extended_state', 10)
    second = None

    def pump(duration, publishers):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            stamp = node.get_clock().now().to_msg()
            message = State()
            message.header.stamp = stamp
            message.connected = True
            for publisher in publishers:
                publisher.publish(message)
            extension = ExtendedState()
            extension.header.stamp = stamp
            extension.landed_state = 1
            landed.publish(extension)
            rclpy.spin_once(node, timeout_sec=.01)
            time.sleep(.025)

    def wait_grounded(expected, publishers, timeout=4):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            pump(.1, publishers)
            if observer.snapshot('sample')['grounded'] is expected:
                return
        raise AssertionError(observer.snapshot('sample'))

    try:
        wait_grounded(True, [state])
        initial = observer.source_generations['state']
        second = node.create_publisher(State, '/mavros/state', 10)
        wait_grounded(False, [state, second])
        node.destroy_publisher(state)
        state = None
        wait_grounded(True, [second])
        assert observer.source_generations['state'] >= initial + 2
        observer.stopping.set()
        observer.thread.join()
        # Source samples remain within their 2.5 s budget; the dead reader alone blocks control.
        time.sleep(.6)
        snapshot = observer.snapshot('sample')
        assert not snapshot['observer_alive'] and not snapshot['grounded']
        assert not snapshot['isolated']
    finally:
        observer.close()
        node.destroy_node()
        rclpy.shutdown()

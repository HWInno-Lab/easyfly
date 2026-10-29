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

"""Bounded native ROS observer and service worker for the Web request thread."""

from concurrent.futures import Future, TimeoutError
import math
import queue
import threading
import time

from easyfly_msgs.msg import FlightStatus, UAVState
from easyfly_msgs.srv import GetMissionDetails, ManageReviewedMission
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict


class Bridge:
    """Keep ROS state, source epochs and service calls on one dedicated executor thread."""

    def __init__(self, simulation_time):
        self.context = Context()
        self.context.init()
        self.node = rclpy.create_node(
            'easyfly_web_bridge', context=self.context,
            parameter_overrides=[Parameter('use_sim_time', value=simulation_time)])
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.queue = queue.Queue(maxsize=8)
        self.pending = None
        self.state = {}
        self.watermarks = {}
        self.generations = {'state': 0, 'flight': 0}
        self.identities = {'state': set(), 'flight': set()}
        self.specs = {'state': (UAVState, '/uav_state'),
                      'flight': (FlightStatus, '/flight_status')}
        self.subscriptions = {}
        for key in self.specs:
            self.subscribe(key)
        self.types = {'details': GetMissionDetails, 'reviewed': ManageReviewedMission}
        self.endpoints = {'details': '/platform/mission/details',
                          'reviewed': '/platform/mission/reviewed'}
        self.clients = {key: self.node.create_client(self.types[key], name)
                        for key, name in self.endpoints.items()}
        self.services_ready = False
        self.observed_at = self.clock_at = 0.
        self.last_clock = 0
        self.clock_fault = False
        self.thread = threading.Thread(target=self.run, name='web-ros-owner')
        self.thread.start()

    def subscribe(self, key):
        """Capture a reader generation so queued messages cannot cross publisher changes."""
        kind, topic = self.specs[key]
        generation = self.generations[key]
        self.subscriptions[key] = self.node.create_subscription(
            kind, topic, lambda msg: self.receive(key, generation, msg), qos_profile_sensor_data)

    def receive(self, key, generation, message):
        """Keep first receipt of distinct source stamps; invalid flags clear previous values."""
        source = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        clock = self.node.get_clock().now().nanoseconds
        with self.lock:
            if generation != self.generations[key] or len(self.identities[key]) != 1:
                return
            previous = self.state.get(key)
            watermark = self.watermarks.get(key, 0)
            source_valid = source > 0 and -.2 <= (clock - source) / 1e9 <= 1.
            valid = source_valid
            if key == 'flight':
                valid = valid and message.valid and math.isfinite(message.receipt_age_s)
                valid = valid and 0 <= message.receipt_age_s <= 1.
            if source_valid:
                self.watermarks[key] = max(watermark, source)
            if not valid or source < watermark:
                self.state.pop(key, None)
            elif source == watermark:
                # Changed aggregate fields at an unchanged source time cannot refresh evidence.
                if key == 'state' and previous and message != previous[2]:
                    self.state.pop(key, None)
            else:
                self.state[key] = (source, time.monotonic(), message)

    def snapshot(self):
        """Return current native validity; expired fields are absent, not zero-valued telemetry."""
        with self.lock:
            now = time.monotonic()
            clock = self.node.get_clock().now().nanoseconds
            alive = self.thread.is_alive() and now - self.observed_at < .5
            clock_ok = not self.clock_fault and clock > 0 and now - self.clock_at <= 1.
            rows = {key: row[2] for key, row in self.state.items() if alive and clock_ok and
                    now - row[1] <= 1. and -.2 <= (clock - row[0]) / 1e9 <= 1.}
            state, flight = rows.get('state'), rows.get('flight')
            value = {'observer_alive': alive, 'clock_reset_requires_restart': self.clock_fault,
                     'mission_service_ready': alive and self.services_ready,
                     'state_valid': state is not None, 'flight_valid': flight is not None,
                     'connected': flight.connected if flight else None,
                     'armed': flight.armed if flight else None,
                     'mode': flight.flight_mode_name if flight else None,
                     'position': None, 'battery': None, 'velocity': None, 'frame': None}
            if state:
                p = [state.position_x, state.position_y, state.position_z]
                v = [state.velocity_x, state.velocity_y, state.velocity_z]
                if state.pose_valid and all(math.isfinite(x) for x in p):
                    value.update(position=p, frame=state.header.frame_id)
                if state.velocity_valid and all(math.isfinite(x) for x in v):
                    value['velocity'] = v
                if (state.battery_valid and math.isfinite(state.battery_remaining) and
                        0 <= state.battery_remaining <= 1):
                    value['battery'] = state.battery_remaining
            return value

    def call(self, key, fields):
        """Wait a bounded interval without retrying or canceling an uncertain remote mutation."""
        future = Future()
        self.queue.put_nowait((key, fields, future, time.monotonic() + 2.))
        try:
            return future.result(timeout=2.2)
        except TimeoutError as exc:
            raise RuntimeError('ROS reply unresolved; query the same request identity') from exc

    def discover(self):
        """Require a single native provider and discard readers when DDS identities change."""
        identities = {key: {bytes(x.endpoint_gid) for x in
                            self.node.get_publishers_info_by_topic(topic)}
                      for key, (_, topic) in self.specs.items()}
        for key, values in identities.items():
            if values != self.identities[key]:
                with self.lock:
                    self.generations[key] += 1
                    self.state.pop(key, None)
                    self.watermarks.pop(key, None)
                    self.identities[key] = values
                self.node.destroy_subscription(self.subscriptions.pop(key))
                self.subscribe(key)
        providers = {name: [] for name in self.endpoints.values()}
        for name, namespace in self.node.get_node_names_and_namespaces():
            services = dict(self.node.get_service_names_and_types_by_node(name, namespace))
            for endpoint in providers:
                if endpoint in services:
                    providers[endpoint].append((name, namespace))
        ready = all(values == [('easyfly_mission_srvs', '/')] for values in providers.values())
        with self.lock:
            self.services_ready = ready
            now = time.monotonic()
            clock = self.node.get_clock().now().nanoseconds
            if clock < self.last_clock:
                self.clock_fault = True
                self.state.clear()
            if clock != self.last_clock:
                self.clock_at = now
            self.last_clock = clock
            self.observed_at = now

    def run(self):
        """Execute at most one native service request and a bounded queue of unsent work."""
        try:
            while not self.stop.is_set():
                self.executor.spin_once(timeout_sec=.02)
                self.discover()
                now = time.monotonic()
                if self.pending:
                    native, result, deadline = self.pending
                    if native.done():
                        try:
                            value = message_to_ordereddict(native.result())
                            result.set_result(value)
                        except Exception as exc:
                            result.set_exception(RuntimeError(str(exc)))
                        self.pending = None
                    elif now > deadline:
                        result.set_exception(RuntimeError('ROS reply unresolved'))
                        native.cancel()
                        self.pending = None
                if self.pending is None:
                    try:
                        key, fields, result, deadline = self.queue.get_nowait()
                    except queue.Empty:
                        continue
                    try:
                        if now > deadline or not self.services_ready or self.clock_fault:
                            raise RuntimeError('native mission service unavailable or ambiguous')
                        request = self.types[key].Request(**fields)
                        if not self.clients[key].service_is_ready():
                            raise RuntimeError('native mission service unavailable')
                        native = self.clients[key].call_async(request)
                        self.pending = (native, result, deadline)
                    except Exception as exc:
                        result.set_exception(RuntimeError(str(exc)))
        finally:
            if self.pending:
                native, result, _ = self.pending
                native.cancel()
                result.set_exception(RuntimeError('Web observer closed; remote result unknown'))
            while not self.queue.empty():
                _, _, result, _ = self.queue.get_nowait()
                result.set_exception(RuntimeError('Web observer closed before dispatch'))

    def close(self):
        """Stop local observers only; browser or gateway exit never cancels a remote task."""
        self.stop.set()
        self.thread.join()
        self.executor.remove_node(self.node)
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.shutdown()

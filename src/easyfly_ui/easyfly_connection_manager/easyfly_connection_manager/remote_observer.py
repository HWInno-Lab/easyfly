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
"""Read native flight interlocks and output activity without initializing GUI ROS contexts."""

import threading
import time

from action_msgs.msg import GoalStatusArray
from mavros_msgs.msg import ExtendedState, State
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from std_srvs.srv import Trigger


class Observer:
    """One daemon-owned context; snapshots expire in both source and wall clocks."""

    def __init__(self, config):
        self.lock = threading.Lock()
        self.context = Context()
        self.context.init()
        self.node = rclpy.create_node(
            'remote_algorithm_observer', context=self.context,
            parameter_overrides=[Parameter('use_sim_time', value=config['simulation_time'])])
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.stopping = threading.Event()
        self.state = {}
        self.actions = {}
        self.outputs = {}
        self.discovered = False
        self.blocked = True
        self.last_discovery = 0.
        self.publisher_ids = {}
        self.source_generations = {}
        self.source_subscriptions = {}
        self.source_specs = {}
        self.fuel_idle = (0., False)
        self.fuel_query = None
        self.fuel_query_at = 0.
        self.fuel_provider_ok = False
        self.fuel_service = self.node.create_client(Trigger, '/exploration/management_idle')
        self.started = time.monotonic()
        self.subscribe_source('state', State, '/mavros/state', qos_profile_sensor_data)
        self.subscribe_source('landed', ExtendedState, '/mavros/extended_state',
                              qos_profile_sensor_data)
        self.action_names = ['execute_mission', 'navigate', 'takeoff', 'land', 'search_area',
                             'track_target', 'explore', 'precision_land']
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for name in self.action_names:
            self.subscribe_source(name, GoalStatusArray, '/' + name + '/_action/status', qos)
        for name, profile in config['profiles'].items():
            self.node.create_subscription(
                get_message(profile['output_type']), profile['output_topic'],
                lambda m, key=name: self.output(key, m), qos_profile_sensor_data)
        self.profiles = config['profiles']
        self.thread = threading.Thread(target=self.run, name='remote-observer')
        self.thread.start()

    def subscribe_source(self, key, message_type, topic, qos):
        """Give every reader its own generation, including deferred callbacks."""
        self.source_specs[key] = (message_type, topic, qos)
        generation = self.source_generations.setdefault(key, 0)
        callback = self.flight if key in ('state', 'landed') else self.action
        self.source_subscriptions[key] = self.node.create_subscription(
            message_type, topic, lambda msg: callback(key, msg, generation), qos)

    def refresh_sources(self, identities):
        """Recreate readers on DDS identity changes so old queued samples cannot cross epochs."""
        for key, values in identities.items():
            if values == self.publisher_ids.get(key, set()):
                continue
            with self.lock:
                self.source_generations[key] += 1
                self.state.pop(key, None)
                if key in self.actions and self.actions[key][0]:
                    self.actions[key] = (True, None)
                else:
                    self.actions.pop(key, None)
            self.node.destroy_subscription(self.source_subscriptions.pop(key))
            self.subscribe_source(key, *self.source_specs[key])

    def flight(self, key, msg, generation):
        """Ignore repeated source stamps and reject invalid clock/source ages."""
        source = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        now = self.node.get_clock().now().nanoseconds
        with self.lock:
            ids = self.publisher_ids.get(key, set())
            if generation != self.source_generations[key] or len(ids) != 1:
                return
            prior = self.state.get(key)
            if not 0 < source or not -200000000 <= now - source <= 2500000000:
                self.state.pop(key, None)
            elif prior and source < prior[0]:
                self.state.clear()
            elif not prior or source > prior[0]:
                good = (msg.connected and not msg.armed if key == 'state'
                        else msg.landed_state == ExtendedState.LANDED_STATE_ON_GROUND)
                self.state[key] = (source, time.monotonic(), good, next(iter(ids)))

    def action(self, key, msg, generation):
        """Latch active/unknown work until a real terminal action update arrives."""
        with self.lock:
            ids = self.publisher_ids.get(key, set())
            if generation != self.source_generations[key] or len(ids) != 1:
                return
            self.actions[key] = (any(item.status in (1, 2, 3) for item in msg.status_list),
                                 next(iter(ids)))

    def output(self, key, msg):
        """Record receipt activity separately from process existence or flight readiness."""
        del msg
        with self.lock:
            self.outputs[key] = time.monotonic()

    def snapshot(self, name):
        """Expose recent interlocks and actual output publisher counts atomically."""
        with self.lock:
            now = time.monotonic()
            source_now = self.node.get_clock().now().nanoseconds
            grounded = len(self.state) == 2 and all(
                row[2] and now - row[1] <= 2.5 and
                -200000000 <= source_now - row[0] <= 2500000000
                for row in self.state.values())
            counts = getattr(self, 'counts', {})
            active = any(item[0] for item in self.actions.values())
            fuel_idle = (self.fuel_idle[1] and now - self.fuel_idle[0] < .5 and
                         self.fuel_provider_ok)
            known_actions = all(not counts.get('/' + key + '/_action/status', 0) or
                                (len(self.publisher_ids.get(key, set())) == 1 and (
                                    self.actions.get(key, (False, None))[1] in
                                    self.publisher_ids.get(key, set()) or
                                    (key == 'explore' and fuel_idle)))
                                for key in self.action_names)
            identities = all(self.state.get(key, (0, 0, False, None))[3] in
                             self.publisher_ids.get(key, set()) for key in ('state', 'landed'))
            observer_alive = now - self.last_discovery < .5
            grounded = bool(grounded and known_actions and identities and observer_alive and
                            not active and
                            counts.get('/mavros/state') == 1 and
                            counts.get('/mavros/extended_state') == 1)
            isolated = bool(self.discovered and observer_alive and not self.blocked and not active)
            return {'grounded': grounded, 'isolated': isolated,
                    'observer_alive': observer_alive,
                    'output_receipt_fresh': now - self.outputs.get(name, -1e30) < 2.5,
                    'output_publishers': counts.get(self.profiles[name]['output_topic'], 0)}

    def run(self):
        """Refresh discovery in the same executor thread and retain conservative admission."""
        topics = ['/mavros/state', '/mavros/extended_state', '/uav_state', '/flight_status']
        topics += ['/' + key + '/_action/status' for key in self.action_names]
        topics += [p['output_topic'] for p in self.profiles.values()]
        while not self.stopping.is_set():
            self.executor.spin_once(timeout_sec=.05)
            now = time.monotonic()
            if self.fuel_query is not None:
                if self.fuel_query.done():
                    try:
                        response = self.fuel_query.result()
                        with self.lock:
                            self.fuel_idle = (now, bool(response and response.success))
                    except Exception:
                        with self.lock:
                            self.fuel_idle = (0., False)
                    self.fuel_query = None
                elif now - self.fuel_query_at > .5:
                    self.fuel_query.cancel()
                    self.fuel_query = None
            if (self.fuel_query is None and now - self.fuel_query_at > .2 and
                    self.fuel_provider_ok and self.fuel_service.service_is_ready()):
                self.fuel_query = self.fuel_service.call_async(Trigger.Request())
                self.fuel_query_at = now
            counts = {topic: self.node.count_publishers(topic) for topic in topics}
            identities = {
                key: {bytes(info.endpoint_gid) for info in
                      self.node.get_publishers_info_by_topic(topic)}
                for key, topic in [('state', '/mavros/state'),
                                   ('landed', '/mavros/extended_state')]}
            for key in self.action_names:
                identities[key] = {bytes(info.endpoint_gid) for info in
                                   self.node.get_publishers_info_by_topic(
                                       '/' + key + '/_action/status')}
            fuel_info = self.node.get_publishers_info_by_topic('/explore/_action/status')
            fuel_provider = False
            if (len(fuel_info) == 1 and fuel_info[0].node_name == 'fuel_exploration' and
                    fuel_info[0].node_namespace == '/'):
                providers = [
                    (name, namespace) for name, namespace in
                    self.node.get_node_names_and_namespaces()
                    if '/exploration/management_idle' in dict(
                        self.node.get_service_names_and_types_by_node(name, namespace))]
                fuel_provider = providers == [('fuel_exploration', '/')]
            self.refresh_sources(identities)
            with self.lock:
                self.counts = counts
                self.publisher_ids = identities
                self.fuel_provider_ok = fuel_provider
                self.last_discovery = time.monotonic()
                self.discovered = time.monotonic() - self.started >= 2.
                self.blocked = any(counts.get(topic, 0) for topic in topics[:12])

    def close(self):
        """Stop the executor before destroying native subscriptions and context."""
        self.stopping.set()
        self.thread.join()
        self.executor.remove_node(self.node)
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.shutdown()

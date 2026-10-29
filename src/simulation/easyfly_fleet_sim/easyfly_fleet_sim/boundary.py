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

"""Explicit one-way sensor and observation boundaries between fleet and vehicle DDS domains."""

import argparse
import copy
import json
import math
import queue
import subprocess
from pathlib import Path
import re
import signal
import threading
import time

from ament_index_python.packages import get_package_prefix
from mavros_msgs.msg import ExtendedState

from easyfly_msgs.msg import FlightStatus, UAVState
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


FRAME = re.compile('[A-Za-z_][A-Za-z0-9_/]{0,127}')


class Boundary:
    """Relay only declared sensor/state types; no control topic or action crosses domains."""

    def __init__(self, manifest, vehicle):
        self.manifest, self.vehicle = manifest, vehicle
        self.name = vehicle['id']
        self.prefix = '/fleet/' + self.name
        self.frame_prefix = self.name + '/'
        self.common_context, self.local_context = Context(), Context()
        self.common_context.init(domain_id=manifest['configuration']['coordinator_domain'])
        self.local_context.init(domain_id=vehicle['domain'])
        options = [Parameter('use_sim_time', value=True)]
        self.common = rclpy.create_node(
            'boundary', namespace=self.prefix, context=self.common_context,
            parameter_overrides=options)
        self.local = rclpy.create_node(
            'fleet_boundary', context=self.local_context, parameter_overrides=options)
        self.executors = [SingleThreadedExecutor(context=self.common_context),
                          SingleThreadedExecutor(context=self.local_context)]
        self.executors[0].add_node(self.common)
        self.executors[1].add_node(self.local)
        self.common.create_subscription(
            Clock, '/clock', self.receive_clock, qos_profile_sensor_data)
        self.last_clock = 0
        self.clock_at = 0.
        self.clock_fault = False
        self.last_sources = {}
        self.metrics = {'clock_observed': 0, 'state_forwarded': 0,
                        'tf_forwarded': 0, 'invalid': 0}
        self.static_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_static = self.common.create_publisher(TFMessage, '/tf_static', self.static_qos)
        self.tf = self.common.create_publisher(TFMessage, '/tf', 10)
        self.local.create_subscription(TFMessage, '/tf_static',
                                       lambda msg: self.transforms(msg, True),
                                       QoSProfile(depth=100,
                                                  durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.local.create_subscription(TFMessage, '/tf',
                                       lambda msg: self.transforms(msg, False), 10)
        self.static_transforms = {}
        self.world_sent = False
        self.alignment = None
        self.alignment_candidates = []
        self.alignment_queue = queue.Queue(maxsize=1)
        self.alignment_thread = None
        self.alignment_attempt_at = 0.
        self.last_vehicle_state = None
        self.landed = None
        self.local.create_subscription(ExtendedState, '/mavros/extended_state',
                                       self.landed_observation, qos_profile_sensor_data)
        for kind, topic in [(UAVState, '/uav_state'), (FlightStatus, '/flight_status'),
                            (Odometry, '/odom')]:
            publisher = self.common.create_publisher(kind, self.prefix + topic,
                                                     qos_profile_sensor_data)
            self.local.create_subscription(
                kind, topic, lambda msg, p=publisher, t=topic: self.observation(t, p, msg),
                qos_profile_sensor_data)
        self.identity_local = self.local.create_publisher(
            String, '/fleet_identity', self.static_qos)
        self.identity_common = self.common.create_publisher(String, self.prefix + '/identity',
                                                            self.static_qos)
        self.status = self.common.create_publisher(String, self.prefix + '/boundary_status', 5)
        self.last_status = 0.
        self.identity = dict(fleet_session=manifest['session'], vehicle_session=vehicle['session'],
                             vehicle_id=self.name, domain=vehicle['domain'],
                             system_id=vehicle['instance'] + 1, namespace=self.prefix,
                             spawn_xy=vehicle['spawn'][:2], frame=self.frame_prefix + 'odom')

    def receive_clock(self, message):
        """Forward original simulation time once and latch a backwards clock as a fleet fault."""
        value = message.clock.sec * 10**9 + message.clock.nanosec
        if value < self.last_clock:
            self.clock_fault = True
        if value != self.last_clock:
            self.last_clock = value
            self.clock_at = time.monotonic()
            self.metrics['clock_observed'] += 1

    def fresh(self, topic, message, kind):
        """Do not extend source observations with relay receipt or heartbeat timestamps."""
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        key = (kind, topic)
        if (self.clock_fault or time.monotonic() - self.clock_at > 1. or
                not 0 < stamp or not -.2 <= (self.last_clock - stamp) / 1e9 <= 1. or
                stamp <= self.last_sources.get(key, 0)):
            self.metrics['invalid'] += 1
            return False
        self.last_sources[key] = stamp
        return True

    def observation(self, topic, publisher, message):
        """Export each vehicle's own source frames under its unique external namespace."""
        # MAVROS flight status may legitimately have an empty frame; it is not pose geometry.
        if (topic != '/flight_status' and not FRAME.fullmatch(message.header.frame_id) or
                not self.fresh(topic, message, 'state')):
            self.metrics['invalid'] += 1
            return
        if topic == '/uav_state':
            self.last_vehicle_state = (time.monotonic(), copy.deepcopy(message))
        value = copy.deepcopy(message)
        if value.header.frame_id:
            value.header.frame_id = self.frame_prefix + value.header.frame_id
        if isinstance(value, Odometry):
            if not FRAME.fullmatch(value.child_frame_id):
                self.metrics['invalid'] += 1
                return
            value.child_frame_id = self.frame_prefix + value.child_frame_id
        publisher.publish(value)
        self.metrics['state_forwarded'] += 1

    def transforms(self, message, static):
        """Prefix both TF ends so common consumers cannot merge two local vehicle trees."""
        if len(message.transforms) > 64:
            self.metrics['invalid'] += 1
            return
        output = []
        for item in message.transforms:
            if (not FRAME.fullmatch(item.header.frame_id) or
                    not FRAME.fullmatch(item.child_frame_id) or
                    (not static and not self.fresh(item.child_frame_id, item, 'tf'))):
                self.metrics['invalid'] += 1
                continue
            value = copy.deepcopy(item)
            value.header.frame_id = self.frame_prefix + value.header.frame_id
            value.child_frame_id = self.frame_prefix + value.child_frame_id
            if static:
                if len(self.static_transforms) >= 64 and value.child_frame_id not in (
                        self.static_transforms):
                    self.metrics['invalid'] += 1
                    continue
                self.static_transforms[value.child_frame_id] = value
            else:
                output.append(value)
        if static:
            self.tf_static.publish(TFMessage(transforms=list(self.static_transforms.values())))
        elif output:
            self.tf.publish(TFMessage(transforms=output))
        self.metrics['tf_forwarded'] += len(output)

    def landed_observation(self, message):
        """Require current native on-ground evidence before estimating world/local alignment."""
        if self.fresh('/mavros/extended_state', message, 'landed'):
            self.landed = (time.monotonic(), message)

    def grounded_state(self):
        """Use both independent landed state and finite stationary native state observations."""
        now = time.monotonic()
        if (not self.last_vehicle_state or not self.landed or
                now - self.last_vehicle_state[0] > .5 or now - self.landed[0] > 1. or
                self.landed[1].landed_state != ExtendedState.LANDED_STATE_ON_GROUND):
            return None
        value = self.last_vehicle_state[1]
        numbers = [value.position_x, value.position_y, value.position_z, value.yaw,
                   value.velocity_x, value.velocity_y, value.velocity_z, value.roll, value.pitch]
        if (value.armed or not value.connected or
                not value.pose_valid or not value.velocity_valid or
                not all(math.isfinite(v) for v in numbers) or
                math.sqrt(sum(v * v for v in numbers[4:7])) > .08 or
                abs(value.roll) > .1 or abs(value.pitch) > .1):
            return None
        return copy.deepcopy(value)

    def query_ground_pose(self, before):
        """Query the real named Gazebo entity on a bounded owned read-only worker."""
        executable = Path(get_package_prefix('easyfly_gazebo_plugins')) / (
            'lib/easyfly_gazebo_plugins/easyfly_model_query')
        value = {'before': before, 'error': ''}
        try:
            result = subprocess.run([str(executable), self.name, '--pose'],
                                    capture_output=True, text=True, timeout=10)
            pose = [float(v) for v in result.stdout.split()]
            if result.returncode or len(pose) != 6 or not all(math.isfinite(v) for v in pose):
                raise ValueError('native model query failed: ' + result.stderr[:500])
            value['pose'] = pose
        except (ValueError, OSError, subprocess.TimeoutExpired) as exc:
            value['error'] = str(exc)
        self.alignment_queue.put(value)

    def align_world(self):
        """Freeze a measured world/local transform only after three stable ground observations."""
        now = time.monotonic()
        if self.alignment:
            return
        if self.alignment_thread and not self.alignment_thread.is_alive():
            self.alignment_thread.join()
            self.alignment_thread = None
            result = self.alignment_queue.get_nowait()
            after, before = self.grounded_state(), result['before']
            if after is not None and not result['error']:
                position = [before.position_x, before.position_y, before.position_z]
                current = [after.position_x, after.position_y, after.position_z]
                if max(abs(a - b) for a, b in zip(position, current)) < .03:
                    truth = result['pose']
                    yaw = math.remainder(truth[5] - before.yaw, 2 * math.pi)
                    c, sine = math.cos(yaw), math.sin(yaw)
                    offset = [truth[0] - c * position[0] + sine * position[1],
                              truth[1] - sine * position[0] - c * position[1],
                              truth[2] - position[2]]
                    candidate = dict(offset=offset, yaw=yaw, truth=truth, local=position,
                                     source_ns=before.header.stamp.sec * 10**9 +
                                     before.header.stamp.nanosec)
                    if self.alignment_candidates:
                        prior = self.alignment_candidates[-1]
                        if (max(abs(a - b) for a, b in zip(offset, prior['offset'])) > .05 or
                                abs(math.remainder(yaw - prior['yaw'], 2 * math.pi)) > .03):
                            self.alignment_candidates.clear()
                    self.alignment_candidates.append(candidate)
                    if len(self.alignment_candidates) == 3:
                        self.alignment = dict(candidate, samples=self.alignment_candidates,
                                              fleet_session=self.manifest['session'],
                                              vehicle_session=self.vehicle['session'])
                        root = Path(self.manifest['output_directory'])
                        (root / (self.name + '-world-alignment.json')).write_text(
                            json.dumps(self.alignment, indent=2) + '\n')
            else:
                self.alignment_candidates.clear()
        before = self.grounded_state()
        if (self.alignment is None and self.alignment_thread is None and before is not None and
                now - self.alignment_attempt_at > .5):
            self.alignment_attempt_at = now
            self.alignment_thread = threading.Thread(target=self.query_ground_pose, args=(before,))
            self.alignment_thread.start()

    def spin(self, stopping):
        """Keep both domains on one owner thread and publish explicit per-vehicle identity."""
        while not stopping.is_set():
            # Service both clock and exported state without blocking on a quiet
            # domain. High-rate native sensor ingress has its own C++ process.
            for executor in [self.executors[0], self.executors[0], self.executors[1]]:
                executor.spin_once(timeout_sec=0)
            now = time.monotonic()
            self.align_world()
            if self.alignment and not self.world_sent:
                transform = TransformStamped()
                transform.header.stamp = self.common.get_clock().now().to_msg()
                transform.header.frame_id = 'world'
                transform.child_frame_id = self.frame_prefix + 'odom'
                transform.transform.translation.x = self.alignment['offset'][0]
                transform.transform.translation.y = self.alignment['offset'][1]
                transform.transform.translation.z = self.alignment['offset'][2]
                transform.transform.rotation.z = math.sin(self.alignment['yaw'] / 2)
                transform.transform.rotation.w = math.cos(self.alignment['yaw'] / 2)
                self.static_transforms[transform.child_frame_id] = transform
                self.tf_static.publish(TFMessage(transforms=list(self.static_transforms.values())))
                self.world_sent = True
            if now - self.last_status >= .2:
                message = String(data=json.dumps(dict(self.identity, source_ns=self.last_clock,
                                                      clock_fault=self.clock_fault,
                                                      world_alignment=self.alignment)))
                self.identity_local.publish(message)
                self.identity_common.publish(message)
                status = dict(self.metrics, clock_recent=now - self.clock_at <= 1.,
                              clock_fault=self.clock_fault)
                self.status.publish(String(data=json.dumps(status)))
                self.last_status = now
            stopping.wait(.0005)

    def close(self):
        """Destroy only these two owned contexts and their readers after the loop stops."""
        if self.alignment_thread:
            self.alignment_thread.join()
        root = Path(self.manifest['output_directory'])
        (root / (self.name + '-boundary-statistics.json')).write_text(
            json.dumps(self.metrics, indent=2) + '\n')
        for executor, node in zip(self.executors, [self.common, self.local]):
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
        self.common_context.shutdown()
        self.local_context.shutdown()


def main():
    """Use the immutable runtime manifest, never an environment-selected arbitrary peer."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', required=True)
    parser.add_argument('--vehicle', required=True)
    arguments = parser.parse_args()
    manifest = json.loads(Path(arguments.manifest).read_text())
    vehicle = next(item for item in manifest['configuration']['vehicles']
                   if item['id'] == arguments.vehicle)
    boundary = Boundary(manifest, vehicle)
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    try:
        boundary.spin(stopping)
    finally:
        boundary.close()

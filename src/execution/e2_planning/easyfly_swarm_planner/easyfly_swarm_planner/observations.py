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

"""Read native per-vehicle state with immutable fleet identity and source-time validity."""

import copy
import hashlib
import itertools
import json
import math
from pathlib import Path
import re
import time

from ament_index_python.packages import get_package_share_directory
from easyfly_msgs.msg import SwarmInfo, UAVState
from easyfly_msgs.srv import GetTrajectoryState
from mavros_msgs.msg import ExtendedState
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

from .configuration import read_json


def source_ns(message):
    """Read an observation's original source timestamp."""
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


class Peer:
    """Own one ROS context and refuse stale, duplicate, replaced or foreign state sources."""

    def __init__(self, manifest, vehicle):
        self.manifest, self.vehicle = manifest, vehicle
        self.name = vehicle['id']
        self.context = Context()
        self.context.init(domain_id=vehicle['domain'])
        self.node = rclpy.create_node(
            'swarm_coordinator_' + self.name, context=self.context,
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.state = self.identity = self.landed = None
        self.state_at = self.identity_at = self.landed_at = self.clock_at = 0.
        self.state_stamp = self.landed_stamp = self.clock_ns = 0
        self.fault = ''
        self.alignment = None
        self.source_ids = None
        self.last_discovery = 0.
        self.node.create_subscription(UAVState, '/uav_state', self.receive_state,
                                      qos_profile_sensor_data)
        self.node.create_subscription(ExtendedState, '/mavros/extended_state', self.receive_landed,
                                      qos_profile_sensor_data)
        self.node.create_subscription(Clock, '/clock', self.receive_clock, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, '/fleet_identity', self.receive_identity, latched)
        self.trajectory_state = self.node.create_client(GetTrajectoryState,
                                                        '/flight_control/trajectory_state')

    def receive_clock(self, message):
        """Latch backwards time and track actual progress independently of message heartbeat."""
        value = message.clock.sec * 10**9 + message.clock.nanosec
        if value < self.clock_ns:
            self.fault = 'clock moved backwards'
        if value > self.clock_ns:
            self.clock_ns, self.clock_at = value, time.monotonic()

    def receive_state(self, message):
        """Accept each source stamp once; invalid source data stays invalid."""
        stamp = source_ns(message)
        if stamp <= self.state_stamp:
            return
        self.state_stamp, self.state_at = stamp, time.monotonic()
        self.state = copy.deepcopy(message)

    def receive_landed(self, message):
        """Require current native landed-state evidence, not low altitude alone."""
        stamp = source_ns(message)
        if stamp > self.landed_stamp:
            self.landed_stamp, self.landed_at = stamp, time.monotonic()
            self.landed = message

    def receive_identity(self, message):
        """Pin measured coordinate alignment and vehicle session throughout the entire task."""
        try:
            if len(message.data) > 16384:
                raise ValueError('identity too large')
            value = json.loads(message.data)
            expected = dict(fleet_session=self.manifest['session'],
                            vehicle_session=self.vehicle['session'], vehicle_id=self.name,
                            domain=self.vehicle['domain'], system_id=self.vehicle['instance'] + 1)
            if any(value.get(key) != field for key, field in expected.items()):
                raise ValueError('fleet identity changed')
            if value['clock_fault']:
                raise ValueError('fleet clock fault')
            alignment = value.get('world_alignment')
            if alignment is not None:
                if (alignment['fleet_session'] != expected['fleet_session'] or
                        alignment['vehicle_session'] != expected['vehicle_session'] or
                        len(alignment['offset']) != 3 or
                        not all(math.isfinite(v) for v in
                                [*alignment['offset'], alignment['yaw']])):
                    raise ValueError('invalid measured world alignment')
                if self.alignment is not None and alignment != self.alignment:
                    raise ValueError('measured world alignment changed')
                self.alignment = alignment
            self.identity, self.identity_at = value, time.monotonic()
        except (ValueError, TypeError, KeyError) as exc:
            self.fault = str(exc)

    def discover(self):
        """Pin discovered publisher identities, since Humble callback MessageInfo lacks the GID."""
        now = time.monotonic()
        if now - self.last_discovery < .2:
            return
        self.last_discovery = now
        current = {}
        for topic in ['/uav_state', '/fleet_identity', '/clock']:
            sources = self.node.get_publishers_info_by_topic(topic)
            if len(sources) > 1:
                self.fault = 'duplicate source: ' + topic
            if len(sources) != 1:
                return
            current[topic] = tuple(sources[0].endpoint_gid)
        if self.source_ids is not None and current != self.source_ids:
            self.fault = 'native publisher identity replaced'
        self.source_ids = current

    def valid(self):
        """Check source and receipt freshness, clock progress, pose validity and exact identity."""
        self.discover()
        now = time.monotonic()
        state = self.state
        if (self.fault or self.source_ids is None or state is None or self.identity is None or
                self.alignment is None or now - self.clock_at > .7 or
                now - self.state_at > .7 or now - self.identity_at > .7 or
                not -.2 <= (self.clock_ns - self.state_stamp) / 1e9 <= .7 or
                not -.2 <= (self.clock_ns - self.identity['source_ns']) / 1e9 <= .7):
            return False
        fields = [state.position_x, state.position_y, state.position_z, state.yaw,
                  state.velocity_x, state.velocity_y, state.velocity_z, state.battery_remaining]
        return (state.connected and state.pose_valid and state.velocity_valid and
                state.attitude_valid and state.battery_valid and
                all(math.isfinite(v) for v in fields) and .15 <= state.battery_remaining <= 1.)

    def grounded(self):
        """Require fresh independent on-ground status and disarm before automatic takeoff."""
        return (self.valid() and not self.state.armed and self.landed is not None and
                time.monotonic() - self.landed_at <= 1.2 and
                -.2 <= (self.clock_ns - self.landed_stamp) / 1e9 <= 1.2 and
                self.landed.landed_state == ExtendedState.LANDED_STATE_ON_GROUND)

    def world_position(self):
        """Transform native local ENU to the session's measured Gazebo world frame."""
        value = self.state
        x, y, z = value.position_x, value.position_y, value.position_z
        c, s = math.cos(self.alignment['yaw']), math.sin(self.alignment['yaw'])
        offset = self.alignment['offset']
        return (c * x - s * y + offset[0], s * x + c * y + offset[1], z + offset[2])

    def local_position(self, position):
        """Invert only this vehicle's frozen world transform for native flight commands."""
        x, y, z = (a - b for a, b in zip(position, self.alignment['offset']))
        c, s = math.cos(self.alignment['yaw']), math.sin(self.alignment['yaw'])
        return (c * x + s * y, -s * x + c * y, z)

    def local_vector(self, vector):
        """Rotate a world velocity or acceleration without applying translation."""
        x, y, z = vector
        c, s = math.cos(self.alignment['yaw']), math.sin(self.alignment['yaw'])
        return (c * x + s * y, -s * x + c * y, z)

    def close(self):
        """Destroy only this coordinator's context; native controllers remain separately owned."""
        self.executor.remove_node(self.node)
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.shutdown()


class FleetObservations:
    """Admit only the live, owned native simulation manifest and publish relative fleet state."""

    def __init__(self, path):
        self.path = Path(path)
        self.manifest = read_json(path)
        value = self.manifest
        if (not value.get('simulation_only') or
                not re.fullmatch('[0-9a-f]{32}', value.get('session', '')) or
                Path(value['output_directory']) != self.path.parent):
            raise ValueError('expected a native simulation session manifest')
        world = Path(get_package_share_directory('easyfly_gazebo_worlds')) / 'empty.world'
        if value['world']['sha256'] != hashlib.sha256(world.read_bytes()).hexdigest():
            raise ValueError('swarm execution requires the declared empty-world input')
        vehicles = value['configuration']['vehicles']
        if (not 2 <= len(vehicles) <= 4 or
                len({v['id'] for v in vehicles}) != len(vehicles) or
                len({v['domain'] for v in vehicles}) != len(vehicles)):
            raise ValueError('invalid manifest vehicle allocation')
        self.status_mtime, self.status_at = None, 0.
        self.peers = {}
        self.context = Context()
        self.context.init(domain_id=value['configuration']['coordinator_domain'])
        self.node = rclpy.create_node('swarm_observer', context=self.context)
        self.publisher = self.node.create_publisher(SwarmInfo, '/swarm/state', 10)
        self.status_publisher = self.node.create_publisher(
            String, '/swarm/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.last_publication = 0.
        try:
            for vehicle in vehicles:
                self.peers[vehicle['id']] = Peer(value, vehicle)
        except BaseException:
            self.close()
            raise

    def live_runtime(self):
        """Require the same running runtime and actual native process start identities."""
        try:
            path = self.path.parent / 'status.json'
            mtime = path.stat().st_mtime_ns
            if mtime != self.status_mtime:
                self.status_mtime, self.status_at = mtime, time.monotonic()
            status = read_json(path)
            if (time.monotonic() - self.status_at > 1. or status['phase'] != 'running' or
                    status['session'] != self.manifest['session']):
                return False
            for process in status['processes']:
                stat_path = Path('/proc') / str(process['pid']) / 'stat'
                fields = stat_path.read_text().rsplit(')', 1)[1].split()
                if fields[0] == 'Z' or fields[19] != process['start_ticks']:
                    return False
            return bool(status['processes'])
        except (OSError, ValueError, KeyError):
            return False

    def pump(self, seconds=.01):
        """Service all owned contexts and emit source-stamped relative state at a bounded rate."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            for peer in self.peers.values():
                peer.executor.spin_once(timeout_sec=.001)
        if time.monotonic() - self.last_publication >= .1 and self.valid():
            self.last_publication = time.monotonic()
            leader = next(iter(self.peers.values()))
            base = leader.world_position()
            for index, peer in enumerate(self.peers.values()):
                position = peer.world_position()
                message = SwarmInfo()
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(
                    min(leader.state_stamp, peer.state_stamp), 10**9)
                message.header.frame_id = 'world'
                message.drone_id = peer.vehicle['instance'] + 1
                message.drone_name = peer.name
                message.position_x, message.position_y, message.position_z = (
                    a - b for a, b in zip(position, base))
                c, s = math.cos(peer.alignment['yaw']), math.sin(peer.alignment['yaw'])
                u = peer.state
                message.velocity_x = c * u.velocity_x - s * u.velocity_y
                message.velocity_y = s * u.velocity_x + c * u.velocity_y
                message.velocity_z = u.velocity_z
                message.is_leader = index == 0
                message.is_connected = True
                message.formation_role = index
                self.publisher.publish(message)

    def valid(self):
        """Invalidate collaborative motion when any member becomes invalid."""
        return self.live_runtime() and all(peer.valid() for peer in self.peers.values())

    def positions(self):
        """Snapshot current world positions only while all native sources remain valid."""
        if not self.valid():
            raise RuntimeError('fleet source identity, clock, state or runtime is invalid')
        return {name: peer.world_position() for name, peer in self.peers.items()}

    def minimum_distance(self):
        """Measure the closest current native vehicle centres, not their commanded targets."""
        return min(math.dist(a, b) for a, b in
                   itertools.combinations(self.positions().values(), 2))

    def close(self):
        """Retire all coordinator readers and contexts without stopping the native fleet."""
        for peer in self.peers.values():
            peer.close()
        self.node.destroy_node()
        self.context.shutdown()

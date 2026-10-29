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

"""Own exact native action UUIDs and execute a certified plan through phase barriers."""

import fcntl
import json
import math
import os
from pathlib import Path
import re
import time
import uuid

from easyfly_msgs.action import FollowTrajectory, Land, Takeoff
from easyfly_msgs.srv import GetTrajectoryState
from quadrotor_msgs.msg import PositionCommand
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from unique_identifier_msgs.msg import UUID

from .configuration import read_json
from .planner import interpolate, phase_duration, Planner, point_segment_distance


def durable(path, value):
    """Persist ownership before sending any native side effect, with atomic replacement."""
    temporary = Path(str(path) + '.pending')
    with temporary.open('w') as stream:
        stream.write(json.dumps(value, indent=2) + '\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


class Operation:
    """A lost response never becomes permission to repeat or adopt a different native action."""

    def __init__(self, owner, name, kind, value):
        self.owner, self.name, self.kind = owner, name, kind
        self.uuid = uuid.uuid4()
        self.handle = self.result = self.future = None
        self.cancel_requested = self.cancel_sent = False
        self.status, self.message, self.success = 'prepared', '', False
        owner.operations.append(self)
        owner.save()
        client = owner.clients[name][kind]
        self.future = client.send_goal_async(value, goal_uuid=UUID(uuid=list(self.uuid.bytes)))
        self.status = 'admission_pending'
        owner.save()

    def poll(self):
        """Retain late admission and result futures, including while cancellation is draining."""
        if self.future is not None and self.handle is None and self.future.done():
            try:
                self.handle = self.future.result()
            except Exception as exc:
                self.status, self.message = 'admission_unknown', str(exc)
                return
            if self.handle is None:
                self.status, self.message = 'admission_unknown', 'missing native goal response'
                return
            if not self.handle.accepted:
                self.status, self.message = 'rejected', 'native action admission rejected'
                return
            self.status = 'active'
            try:
                self.result = self.handle.get_result_async()
            except Exception as exc:
                self.status, self.message = 'result_unknown', str(exc)
        if self.cancel_requested and self.handle and self.handle.accepted and not self.cancel_sent:
            self.cancel_sent = True
            try:
                self.handle.cancel_goal_async()
            except Exception as exc:
                # Preserve the exact goal and keep draining other vehicles too.
                # An uncertain transport send is never permission to retransmit.
                self.message = 'cancel transport unknown: ' + str(exc)
        if self.result is not None and self.result.done() and self.status == 'active':
            try:
                value = self.result.result()
            except Exception as exc:
                self.status, self.message = 'result_unknown', str(exc)
                return
            if value.status not in (4, 5, 6):
                self.status, self.message = 'result_unknown', 'native terminal status unknown'
                return
            self.status = 'terminal'
            self.success = value.status == 4 and value.result.success
            self.message = value.result.message

    def terminal(self):
        """Only an actual rejection or exact native terminal result releases this action."""
        return self.status in ('rejected', 'terminal')

    def record(self):
        """Expose bounded exact ownership without serializing ROS implementation objects."""
        return dict(vehicle=self.name, action=self.kind, goal_id=self.uuid.hex,
                    status=self.status, success=self.success, message=self.message[:1024],
                    cancel_requested=self.cancel_requested, cancel_sent=self.cancel_sent)


class Controller:
    """Hold a fleet ledger and one native trajectory action per moving vehicle."""

    def __init__(self, observations, config, output, stopping):
        self.fleet, self.config = observations, config
        self.output, self.stopping = output, stopping
        self.session = uuid.uuid4().hex
        self.phase = 'preparing'
        self.failure = ''
        self.operations, self.clients, self.publishers = [], {}, {}
        self.minimum_actual = float('inf')
        self.last_save = self.last_command = 0.
        self.phase_index = -1
        self.reference = None
        self.trajectory = {}
        self.controller_states = {}
        self.start = time.monotonic()
        self.lock_path = observations.path.parent / 'swarm-control.lock'
        self.ledger_path = observations.path.parent / 'swarm-control.json'
        self.lock_fd = os.open(self.lock_path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(self.lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            if self.ledger_path.exists() and read_json(self.ledger_path).get('busy'):
                raise RuntimeError('previous swarm ownership unresolved; restart the native fleet')
            for name, peer in self.fleet.peers.items():
                self.clients[name] = {key: ActionClient(peer.node, kind, endpoint)
                                      for key, kind, endpoint in [
                                          ('takeoff', Takeoff, '/takeoff'),
                                          ('land', Land, '/land'),
                                          ('trajectory', FollowTrajectory, '/follow_trajectory')]}
                self.publishers[name] = peer.node.create_publisher(
                    PositionCommand, '/position_cmd', qos_profile_sensor_data)
            self.save()
        except BaseException:
            os.close(self.lock_fd)
            self.lock_fd = None
            raise

    def save(self, released=False):
        """Commit fleet/session bindings and exact actions before publishing status."""
        self.last_save = time.monotonic()
        value = dict(session=self.session, fleet_session=self.fleet.manifest['session'],
                     phase=self.phase, error=self.failure, busy=not released,
                     phase_index=self.phase_index, elapsed_s=time.monotonic() - self.start,
                     source_ns=min(peer.clock_ns for peer in self.fleet.peers.values()),
                     minimum_actual_separation=(self.minimum_actual
                                                if math.isfinite(self.minimum_actual) else None),
                     state_samples={name: dict(position=peer.world_position(),
                                               source_ns=peer.state_stamp,
                                               clock_ns=peer.clock_ns)
                                    for name, peer in self.fleet.peers.items()},
                     operations=[operation.record() for operation in self.operations])
        durable(self.output / 'status.json', value)
        durable(self.ledger_path, value)
        self.fleet.status_publisher.publish(String(data=json.dumps(value)))

    def tick(self, guard=True):
        """Service state/actions and enforce source and physical separation gates."""
        self.fleet.pump(.005)
        for operation in self.operations:
            operation.poll()
        if guard:
            if self.stopping.is_set():
                raise InterruptedError('swarm cancellation requested')
            if not self.fleet.valid():
                raise RuntimeError('fleet source, identity, clock or runtime invalid')
            separation = self.fleet.minimum_distance()
            self.minimum_actual = min(self.minimum_actual, separation)
            if separation < self.config['separation_m']:
                raise RuntimeError('measured separation below declared limit')
            if time.monotonic() - self.start > 600:
                raise TimeoutError('swarm execution wall-clock budget exhausted')
        if self.reference is not None and guard:
            self.stream()
        if time.monotonic() - self.last_save > .2:
            self.save()

    def wait(self, predicate, seconds, reason):
        """Bound all waits while maintaining owned streams and native observation checks."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.tick()
            if predicate():
                return
        raise TimeoutError(reason)

    def submit(self, name, kind, goal):
        """Discover the actual declared per-domain native server before durable admission."""
        self.wait(self.clients[name][kind].server_is_ready, 5, name + ' action unavailable')
        operation = Operation(self, name, kind, goal)
        self.wait(lambda: operation.status != 'admission_pending', 10,
                  name + ' action admission unknown')
        if operation.status not in ('active', 'terminal'):
            raise RuntimeError(name + ' ' + kind + ': ' + operation.status)
        return operation

    def finish(self, operations, seconds):
        """Require native terminal success, never an inferred arrival or cancel response alone."""
        self.wait(lambda: all(op.terminal() for op in operations), seconds,
                  'native action completion timeout')
        if any(not op.success for op in operations):
            raise RuntimeError('native action failed: ' + str([op.record() for op in operations]))

    def query_controllers(self):
        """Read each current trajectory watermark and controller epoch before selecting new IDs."""
        for name, peer in self.fleet.peers.items():
            self.wait(peer.trajectory_state.service_is_ready, 5, 'trajectory state unavailable')
            future = peer.trajectory_state.call_async(GetTrajectoryState.Request())
            self.wait(future.done, 3, 'trajectory state response unavailable')
            state = future.result()
            source = state.header.stamp.sec * 10**9 + state.header.stamp.nanosec
            if (not re.fullmatch('[0-9a-f]{32}', state.controller_session_id) or
                    not -.2 <= (peer.clock_ns - source) / 1e9 <= .7):
                raise RuntimeError(name + ' trajectory state identity or source time invalid')
            if not state.idle or state.highest_trajectory_id == 2**32 - 1:
                raise RuntimeError(name + ' native trajectory slot is busy or exhausted')
            self.controller_states[name] = state
        durable(self.output / 'admission-state.json', {
            name: dict(controller_session_id=value.controller_session_id,
                       previous_trajectory_id=value.highest_trajectory_id,
                       next_trajectory_id=value.highest_trajectory_id + 1)
            for name, value in self.controller_states.items()})

    def stream(self):
        """Publish session-bound quintic commands only from a currently valid joint phase."""
        now = time.monotonic()
        if now - self.last_command < .04:
            return
        self.last_command = now
        for name, reference in self.reference.items():
            operation = self.trajectory.get(name)
            if operation is None or operation.terminal():
                continue
            peer = self.fleet.peers[name]
            position, velocity, acceleration = reference
            command = PositionCommand()
            command.header.stamp.sec, command.header.stamp.nanosec = divmod(peer.clock_ns, 10**9)
            command.header.frame_id = 'odom'
            command.navigation_session_id = self.session + '-' + peer.vehicle['session']
            command.trajectory_id = self.controller_states[name].highest_trajectory_id + 1
            command.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
            command.position.x, command.position.y, command.position.z = (
                peer.local_position(position))
            command.velocity.x, command.velocity.y, command.velocity.z = (
                peer.local_vector(velocity))
            command.acceleration.x, command.acceleration.y, command.acceleration.z = (
                peer.local_vector(acceleration))
            command.yaw = self.yaws[name]
            self.publishers[name].publish(command)

    def execute(self, goals):
        """Take off, execute verified joint phases, then optionally land each vehicle."""
        self.phase = 'takeoff'
        takeoffs = []
        for name, peer in self.fleet.peers.items():
            if not peer.grounded():
                raise RuntimeError('swarm takeoff requires every vehicle disarmed on ground')
            target = self.config['takeoff_world_z'] - peer.alignment['offset'][2]
            takeoffs.append(self.submit(name, 'takeoff', Takeoff.Goal(
                target_altitude=target, climb_speed=.5)))
        self.finish(takeoffs, 60)
        self.phase = 'takeoff_settling'
        self.wait(lambda: all(
            abs(peer.world_position()[2] - self.config['takeoff_world_z']) < .12 and
            math.sqrt(peer.state.velocity_x**2 + peer.state.velocity_y**2 +
                      peer.state.velocity_z**2) < .15 for peer in self.fleet.peers.values()),
            15, 'takeoff has not settled at the requested world altitude')
        self.phase = 'planning_from_measured_hover'
        starts = self.fleet.positions()
        plan = Planner(self.config).plan(starts, goals)
        durable(self.output / 'executed-plan.json', plan)
        self.phase = 'admitting_trajectories'
        self.query_controllers()
        self.yaws = {name: peer.state.yaw for name, peer in self.fleet.peers.items()}
        self.reference = {name: (position, (0., 0., 0.), (0., 0., 0.))
                          for name, position in starts.items()}
        for name, peer in self.fleet.peers.items():
            if math.dist(starts[name], goals[name]) <= .12:
                continue
            target = peer.local_position(goals[name])
            action = FollowTrajectory.Goal(
                target_x=target[0], target_y=target[1], target_z=target[2],
                acceptance_radius=.12, timeout=500.,
                navigation_session_id=self.session + '-' + peer.vehicle['session'])
            self.trajectory[name] = self.submit(name, 'trajectory', action)
        self.phase = 'trajectory'
        paths = plan['paths']
        at = Planner.at
        for phase in range(max(map(len, paths.values())) - 1):
            self.phase_index = phase
            edges = {name: (at(path, phase), at(path, phase + 1)) for name, path in paths.items()}
            duration = phase_duration(list(edges.values()), self.config['speed_mps'],
                                      self.config['acceleration_mps2'])
            start_ns = min(peer.clock_ns for peer in self.fleet.peers.values())
            wall_start, stable = time.monotonic(), None
            while True:
                self.tick()
                elapsed = (min(peer.clock_ns for peer in self.fleet.peers.values()) -
                           start_ns) / 1e9
                positions = self.fleet.positions()
                for name, edge in edges.items():
                    deviation = point_segment_distance(positions[name], *edge)
                    if deviation > self.config['tracking_tube_m']:
                        raise RuntimeError(name + ' left its certified tracking tube')
                    operation = self.trajectory.get(name)
                    if operation and operation.terminal():
                        # A result can arrive before the newest state callback.
                        # Only the final goal segment may finish; its collective
                        # arrival barrier still requires measured endpoint arrival.
                        if not operation.success or math.dist(edge[1], goals[name]) > .15:
                            raise RuntimeError(name + ' native trajectory ended unexpectedly')
                    self.reference[name] = interpolate(*edge, elapsed, duration)
                reached = elapsed >= duration and all(
                    math.dist(positions[name], edge[1]) <= .15 for name, edge in edges.items())
                stable = stable or time.monotonic() if reached else None
                if stable is not None and time.monotonic() - stable >= .3:
                    break
                if time.monotonic() - wall_start > max(15., duration * 4):
                    raise TimeoutError('collective phase arrival barrier timed out')
        self.finish(list(self.trajectory.values()), 10)
        self.reference = None
        if self.config['land_on_complete']:
            self.phase = 'landing'
            self.finish([self.submit(name, 'land', Land.Goal()) for name in self.fleet.peers], 60)
            self.wait(lambda: all(peer.grounded() for peer in self.fleet.peers.values()), 10,
                      'fresh disarm and ground confirmation unavailable')
        self.phase = 'complete'
        self.save()

    def close(self):
        """Stop streams, cancel exact owned actions and durably retain unresolved ownership."""
        self.reference = None
        for operation in self.operations:
            if not operation.terminal():
                operation.cancel_requested = True
        deadline = time.monotonic() + 15
        while any(not operation.terminal() for operation in self.operations):
            self.tick(guard=False)
            if time.monotonic() >= deadline:
                self.phase = 'fault_unresolved_native_ownership'
                break
        released = all(operation.terminal() for operation in self.operations)
        self.save(released=released)
        for clients in self.clients.values():
            for client in clients.values():
                client.destroy()
        os.close(self.lock_fd)
        self.lock_fd = None

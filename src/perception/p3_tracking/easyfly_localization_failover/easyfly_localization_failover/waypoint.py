# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Explicit single native waypoint, with exact-UUID cancellation on EV admission loss."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from easyfly_msgs.action import GoToWaypoint
from easyfly_msgs.msg import UAVState
from easyfly_msgs.srv import GetTrajectoryState
from std_msgs.msg import String
from unique_identifier_msgs.msg import UUID

from .node import PREFIX, lock, stamp


def durable(path, data):
    temporary = path.with_suffix('.pending')
    with temporary.open('w') as stream:
        json.dump(data, stream, indent=2, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    os.replace(temporary, path)
    fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


class Task(Node):
    def __init__(self, args):
        super().__init__('redundant_waypoint')
        if not self.get_parameter('use_sim_time').value:
            raise ValueError('simulation-only task requires use_sim_time')
        self.args = args
        self.output = args.output.resolve()
        self.output.mkdir(parents=True, exist_ok=False)
        self.fd, self.ledger = lock('waypoint')
        self.data = dict(goal_id=uuid.uuid4().hex, controller_session=None, manager_session=None,
                         busy=False, phase='preparing', success=False, cancel_sent=False,
                         target=args.target, reason='', events=[])
        self.health = self.state = None
        self.health_at = self.state_at = float('-inf')
        self.identities = {}
        self.last_graph = 0.
        self.goal_handle = self.admission = self.result = None
        self.phase = 'not_sent'
        self.stopping = False
        self.client = ActionClient(self, GoToWaypoint, '/goto_waypoint')
        self.controller = self.create_client(GetTrajectoryState, '/flight_control/trajectory_state')
        self.create_subscription(String, PREFIX+'/status', self.status, 10)
        self.create_subscription(UAVState, '/uav_state', self.uav, 10)
        self.save(ledger=False)

    def status(self, m):
        if len(m.data) > 65536:
            self.stopping = True
            return
        try:
            self.health = json.loads(m.data)
            self.health_at = time.monotonic()
        except ValueError:
            self.stopping = True

    def uav(self, m):
        self.state, self.state_at = m, time.monotonic()

    def save(self, ledger=True):
        self.data.update(phase=self.phase, busy=self.phase not in ('not_sent', 'rejected', 'terminal'))
        durable(self.output/'result.json', self.data)
        if ledger and self.data['controller_session']:
            durable(self.ledger, self.data)

    def poll(self):
        rclpy.spin_once(self, timeout_sec=.01)
        if self.admission is not None and self.admission.done() and self.phase == 'admission_pending':
            try:
                self.goal_handle = self.admission.result()
                if not self.goal_handle.accepted:
                    self.phase = 'rejected'
                else:
                    self.phase = 'active'
                    self.result = self.goal_handle.get_result_async()
            except Exception as error:
                self.phase = 'ownership_unknown'; self.data['reason'] = str(error)
            self.save()
        if self.result is not None and self.result.done() and self.phase == 'active':
            try:
                answer = self.result.result()
                if answer.status not in (4, 5, 6):
                    raise ValueError('nonterminal native response')
                self.phase = 'terminal'
                self.data.update(native_status=answer.status,
                                 success=answer.status == 4 and answer.result.success,
                                 native_message=answer.result.message)
            except Exception as error:
                self.phase = 'ownership_unknown'; self.data['reason'] = str(error)
            self.save()

    def valid(self):
        now, clock = time.monotonic(), self.get_clock().now().nanoseconds
        h, s = self.health, self.state
        if (not h or not s or not h.get('healthy') or h.get('failure') or not h.get('parameters_verified') or
                now-self.health_at > .4 or now-self.state_at > .4 or
                not -.02e9 <= clock-h.get('output_stamp_ns', 0) <= .4e9 or
                not -.02e9 <= clock-stamp(s) <= .4e9 or not s.connected or not s.armed or
                not s.pose_valid or not s.attitude_valid or s.flight_mode != 3):
            return False
        if self.data['manager_session'] and h['session'] != self.data['manager_session']:
            return False
        if now-self.last_graph >= .1:
            for topic in [PREFIX+'/status', '/uav_state']:
                publishers = self.get_publishers_info_by_topic(topic)
                if len(publishers) != 1:
                    return False
                identity = bytes(publishers[0].endpoint_gid).hex()
                if topic in self.identities and self.identities[topic] != identity:
                    return False
                self.identities[topic] = identity
            self.last_graph = now
        return True

    def execute(self):
        deadline = time.monotonic()+15.
        while time.monotonic() < deadline and not self.stopping:
            self.poll()
            if self.valid() and self.client.server_is_ready() and self.controller.service_is_ready():
                break
        else:
            raise RuntimeError('fresh admitted EV and armed offboard hover required')
        s = self.state
        if (math.dist([s.velocity_x, s.velocity_y, s.velocity_z], [0, 0, 0]) > .08 or
                math.dist([s.position_x, s.position_y, s.position_z], self.args.target) > 10. or
                abs(self.args.target[2]-s.position_z) > .15):
            raise RuntimeError('bounded horizontal waypoint requires settled hover')
        future = self.controller.call_async(GetTrajectoryState.Request())
        deadline = time.monotonic()+3.
        while not future.done() and time.monotonic() < deadline and self.valid() and not self.stopping:
            self.poll()
        if not future.done():
            raise RuntimeError('native controller identity unavailable')
        state = future.result()
        if (not state.idle or not state.controller_session_id or
                not -.02e9 <= self.get_clock().now().nanoseconds-stamp(state) <= .4e9):
            raise RuntimeError('native controller busy or state stale')
        if self.ledger.exists():
            old = json.loads(self.ledger.read_text())
            if old.get('busy') and old.get('controller_session') == state.controller_session_id:
                raise RuntimeError('previous exact waypoint ownership unresolved; new controller session required')
        self.data.update(controller_session=state.controller_session_id,
                         manager_session=self.health['session'], started_ns=self.get_clock().now().nanoseconds)
        self.phase = 'admission_pending'; self.save()
        goal = GoToWaypoint.Goal(target_x=self.args.target[0], target_y=self.args.target[1],
                                target_z=self.args.target[2], target_yaw=float(s.yaw),
                                max_speed=self.args.speed, acceptance_radius=.08)
        self.admission = self.client.send_goal_async(goal, goal_uuid=UUID(uuid=list(bytes.fromhex(self.data['goal_id']))))
        deadline, last_selected = time.monotonic()+self.args.timeout, None
        while self.phase in ('admission_pending', 'active'):
            self.poll()
            if self.stopping or not self.valid():
                raise RuntimeError('EV admission lost or explicit interruption; cancel exact owned UUID')
            if time.monotonic() >= deadline:
                raise RuntimeError('bounded waypoint deadline exceeded')
            if self.health['selected'] != last_selected:
                last_selected = self.health['selected']
                self.data['events'].append(dict(stamp_ns=self.get_clock().now().nanoseconds, selected=last_selected))
                self.save()
        if self.phase != 'terminal' or not self.data['success']:
            raise RuntimeError('native waypoint did not complete successfully')

    def retire(self):
        deadline = time.monotonic()+15.
        while self.phase not in ('not_sent', 'rejected', 'terminal') and time.monotonic() < deadline:
            self.poll()
            if self.goal_handle is not None and self.goal_handle.accepted and not self.data['cancel_sent']:
                self.data['cancel_sent'] = True
                self.data['cancel_ns'] = self.get_clock().now().nanoseconds
                self.save()
                self.goal_handle.cancel_goal_async()
        self.data['finished_ns'] = self.get_clock().now().nanoseconds
        self.save()

    def close(self):
        self.client.destroy(); self.destroy_node(); os.close(self.fd)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', required=True)
    parser.add_argument('--target', type=float, nargs=3, required=True)
    parser.add_argument('--speed', type=float, default=.4)
    parser.add_argument('--timeout', type=float, default=90.)
    parser.add_argument('--output', type=Path, required=True)
    args, ros_args = parser.parse_known_args()
    if (not all(math.isfinite(v) and abs(v) <= 1000 for v in args.target) or
            not .1 <= args.speed <= .6 or not 5 <= args.timeout <= 180):
        parser.error('target, speed or timeout outside reviewed bounds')
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node, code = None, 2
    try:
        node = Task(args)
        def stop(*unused):
            node.stopping = True
        signal.signal(signal.SIGINT, stop); signal.signal(signal.SIGTERM, stop)
        try:
            node.execute(); code = 0
        except Exception as error:
            node.data['reason'] = str(error)
        finally:
            node.retire()
    finally:
        if node is not None:
            node.close()
        rclpy.shutdown()
    return code

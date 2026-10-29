#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Explicit bounded PX4/Gazebo navigation check; never runs as an automatic test."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

from action_msgs.msg import GoalStatusArray
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from easyfly_msgs.action import (
    ExecuteMission, FollowTrajectory, GoToWaypoint, Land, NavigateTo, SearchArea, Takeoff,
)
from easyfly_msgs.msg import MissionStatus, NavigationSession, UAVCommand, UAVState
from easyfly_msgs.srv import GetActionOwnership
from easyfly_precision_landing.simulation_setup import freeze_fastdds, prepare_staging
from easyfly_px4_sitl.hrt_compatibility import select_px4_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from mavros_msgs.msg import AttitudeTarget, PositionTarget
from mavros_msgs.srv import SetMode
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, PointCloud2
from unique_identifier_msgs.msg import UUID

OWNERSHIP_SERVICES = {'/navigate': '/navigation/ownership',
                      '/execute_mission': '/mission/ownership',
                      '/search_area': '/search/ownership'}
CHILD_ACTIONS = {'/navigate': NavigateTo, '/search_area': SearchArea,
                 '/follow_trajectory': FollowTrajectory, '/land': Land,
                 '/takeoff': Takeoff, '/goto_waypoint': GoToWaypoint}
LIFECYCLE_SCENARIOS = tuple(
    f'mission-{step}-{trigger}' for step in ('nav', 'search')
    for trigger in ('cancel', 'timeout', 'land'))


def lifecycle_spec(scenario):
    """Accept only the explicitly implemented simulation lifecycle scenarios."""
    if scenario not in LIFECYCLE_SCENARIOS:
        raise ValueError('unsupported mission lifecycle scenario')
    _, step, trigger = scenario.split('-')
    return step, trigger


def terminal_statuses_observed(statuses, selected):
    """Reject missing, still-executing and unknown child UUIDs."""
    return bool(selected) and all(
        statuses.get(name, {}).get(identity, {}).get('status') in (4, 5, 6)
        for name, identity in selected)


def following_steps_absent(events, land_status, cutoff, total_steps=4):
    """Reject continuation to a canceled/failed mission's final Land sentinel."""
    return (not land_status and not any(
        v['steady'] >= cutoff and
        (v['phase'] == 'land' or v['step'] >= total_steps or v['success'])
        for v in events))


def streams_quiet(events, since, now):
    """Require an observed quiet window, never silence inferred from an ACK."""
    return now - since >= 1.0 and not any(v['steady'] >= since for v in events)


def trajectory_stream_observed(events, now, ros_now):
    """Require recent trajectory intent and actual downstream setpoints before injection."""
    fresh = [v for v in events if 0 <= now - v['steady'] <= 0.3 and v['source_ns'] > 0 and
             -0.2 <= (ros_now - v['source_ns']) * 1e-9 <= 1.0]
    return (any(v['topic'] == '/uav_command' and v.get('command_mode') == UAVCommand.TRAJECTORY
                for v in fresh) and
            any(v['topic'] == '/mavros/setpoint_raw/local' for v in fresh))


def takeover_streams_quiet(events, confirmed, now):
    """Distinguish controller stop from the bridge's existing one-second cached tail."""
    intent = [v for v in events if v['topic'] == '/uav_command']
    output = [v for v in events if v['topic'] != '/uav_command']
    return (streams_quiet(intent, confirmed + 0.5, now) and
            streams_quiet(output, confirmed + 1.5, now))


def terminal_arrival_observation(statuses, identities):
    """Choose the last genuine successful child event, not a later mission feedback pose."""
    candidates = [v['first_terminal'] for identity, v in statuses.items()
                  if identity in identities and v['status'] == 4 and 'first_terminal' in v]
    if not candidates:
        raise RuntimeError('successful trajectory terminal observation missing')
    observation = max(candidates, key=lambda v: v['steady'])
    state = observation['state']
    if (state is None or not state['armed'] or not state['connected'] or state['mode'] != 3 or
            not state['pose_valid'] or not state['velocity_valid'] or state['frame'] != 'odom' or
            state['source_ns'] <= 0 or
            not -0.2 <= (state['ros_now_ns'] - state['source_ns']) * 1e-9 <= 1.0 or
            not 0 <= observation['steady'] - state['receipt'] <= 0.3 or
            not all(math.isfinite(v) for v in state['position'])):
        raise RuntimeError('successful trajectory terminal observation not fresh/valid')
    return observation


def timeout_stop_edge(statuses):
    """Use the timed step's observed cancel/abort edge, not its earlier motion marker."""
    edges = [event['steady'] for value in statuses.values() for event in value['history']
             if event['status'] in (3, 5, 6)]
    if not edges:
        raise RuntimeError('timed step cancel/abort edge not observed')
    return min(edges)


def child_stop_result_valid(result, entry, was_active, trigger, edge):
    """Allow genuine segments completed before timeout while rejecting success after stop."""
    if trigger == 'timeout':
        terminal = entry.get('first_terminal')
        return result['status'] in (5, 6) or (
            result['status'] == 4 and terminal is not None and terminal['steady'] < edge)
    return not was_active or result['status'] in (5, 6)


def state_ready(state, receipt, ros_now, steady_now, armed):
    """Check source and receipt age independently before any new flight action."""
    if (state is None or state.header.stamp.sec < 0 or
            not 0 <= state.header.stamp.nanosec < 1000000000):
        return False
    source = state.header.stamp.sec * 1000000000 + state.header.stamp.nanosec
    return (state.header.frame_id == 'odom' and source > 0 and ros_now > 0 and
            -0.2 <= (ros_now - source) * 1e-9 <= 1.0 and
            0 <= steady_now - receipt <= 0.3 and state.connected and
            state.armed == armed and state.pose_valid and state.velocity_valid and
            state.attitude_valid and state.battery_valid and
            math.isfinite(state.battery_remaining) and 0 <= state.battery_remaining <= 1 and
            all(math.isfinite(v) for v in (
                state.position_x, state.position_y, state.position_z,
                state.velocity_x, state.velocity_y, state.velocity_z,
                state.roll, state.pitch, state.yaw)))


def exact_released(reply, identity):
    """Reject unknown, wrong-UUID and still-owned responses."""
    return (re.fullmatch('[0-9a-f]{32}', identity) is not None and
            reply is not None and reply.goal_id == identity and
            reply.state == GetActionOwnership.Response.RELEASED)


def terminal_matches(value, expected):
    """Require genuine success, cancellation or abort rather than an ACK."""
    return (value is not None and expected in (4, 5, 6) and
            value.status == expected and value.result.success == (expected == 4))


def sessions_closed(notices, identities):
    """Require START and a final STOP for every observed session of each goal."""
    if not identities:
        return False
    for identity in identities:
        matching = [v for v in notices if v['session_id'].startswith(identity + ':')]
        if not matching:
            return False
        for session in {v['session_id'] for v in matching}:
            events = [v['active'] for v in matching if v['session_id'] == session]
            if True not in events or events[-1]:
                return False
    return True


def children(parent):
    """Read only descendants of our launch owner, including GDB inferiors."""
    found = {}
    try:
        values = Path(f'/proc/{parent}/task/{parent}/children').read_text().split()
    except FileNotFoundError:
        return found
    for value in values:
        pid = int(value)
        try:
            exe = str(Path(f'/proc/{pid}/exe').resolve(strict=True))
            stat = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
            found[pid] = {'executable': exe, 'start_ticks': stat[19]}
        except (FileNotFoundError, ProcessLookupError):
            continue
        found.update(children(pid))
    return found


class Probe:
    """Own a single simulation, exact action handles and immutable evidence."""

    def __init__(self, args):
        self.args = args
        self.directory = args.output_dir.resolve(strict=True)
        if not self.directory.is_dir() or any(self.directory.iterdir()):
            raise ValueError('new empty evidence directory required')
        selected = select_px4_directory(args.px4_dir)
        staged = prepare_staging(selected, self.directory / 'px4')
        profile = (Path(get_package_share_directory('easyfly_precision_landing')) /
                   'config/d5_fastdds.xml')
        dds = freeze_fastdds(profile, self.directory / 'fastdds.xml')
        self.environment = {**os.environ, **dds['env'],
                            'ROS_DOMAIN_ID': str(args.domain),
                            'GAZEBO_MASTER_URI': f'http://127.0.0.1:{args.gazebo_port}',
                            'PX4_DIR': staged['staging'],
                            'ROS_LOG_DIR': str(self.directory / 'ros')}
        os.environ.update(self.environment)
        rclpy.init()
        self.node = rclpy.create_node('navigation_sitl_probe', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.latest = {}
        self.receipts = {}
        self.counts = {}
        self.clients = []
        self.active = []
        self.owned = {}
        self.navigation_goals = set()
        self.process = None
        self.report = {'strategy': args.strategy, 'scenario': args.scenario,
                       'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                       'staging': staged, 'dds': dds, 'domain': args.domain,
                       'gazebo_port': args.gazebo_port, 'actions': [],
                       'ownership': [], 'sessions': [], 'forced': False,
                       'error': None, 'passed': False}
        self.report.update(action_status={name: {} for name in CHILD_ACTIONS},
                           child_results=[], mission_events=[], search_feedback=[], streams=[])
        self.states = (self.directory / 'states.jsonl').open('x')
        self.inputs = (self.directory / 'inputs.jsonl').open('x')
        for name, kind in (('/uav_state', UAVState), ('/occupancy_grid', OccupancyGrid),
                           ('/pointcloud_nonground', PointCloud2), ('/odom', Odometry)):
            qos = (QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                   if kind == OccupancyGrid else qos_profile_sensor_data)
            self.node.create_subscription(
                kind, name, lambda msg, topic=name: self.receive(topic, msg), qos)
        self.node.create_subscription(
            NavigationSession, '/planning/navigation_session_state',
            self.session, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for name in CHILD_ACTIONS:
            self.node.create_subscription(
                GoalStatusArray, name + '/_action/status',
                lambda msg, endpoint=name: self.child_status(endpoint, msg),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if args.scenario in LIFECYCLE_SCENARIOS:
            self.node.create_subscription(BatteryState, '/mavros/battery', self.battery,
                                          qos_profile_sensor_data)
            self.node.create_subscription(MissionStatus, '/mission_status',
                                          self.mission_status, 10)
            self.node.create_subscription(SearchArea.Impl.FeedbackMessage,
                                          '/search_area/_action/feedback', self.search_observation,
                                          qos_profile_sensor_data)
            for name, kind in (('/uav_command', UAVCommand),
                               ('/mavros/setpoint_raw/local', PositionTarget),
                               ('/mavros/setpoint_position/local', PoseStamped),
                               ('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist),
                               ('/mavros/setpoint_raw/attitude', AttitudeTarget)):
                self.node.create_subscription(
                    kind, name, lambda msg, endpoint=name: self.stream(endpoint, msg),
                    qos_profile_sensor_data)

    def child_status(self, name, message):
        """Observe UUID lifecycles without treating status as the result payload."""
        for value in message.status_list:
            identity = bytes(value.goal_info.goal_id.uuid).hex()
            if name == '/navigate':
                self.navigation_goals.add(identity)
            entry = self.report['action_status'][name].setdefault(identity, {'history': []})
            if entry.get('status') != value.status:
                entry['history'].append({'steady': time.monotonic(), 'status': value.status})
                if value.status in (4, 5, 6):
                    entry.setdefault('first_terminal',
                                     {'steady': time.monotonic(), 'state': self.state()})
            entry.update(status=value.status, last_steady=time.monotonic())

    def mission_status(self, message):
        """Record actual step transitions, including stop and fault-lock phases."""
        self.report['mission_events'].append(
            {'steady': time.monotonic(), 'phase': message.phase_name,
             'step': message.current_waypoint, 'total': message.total_waypoints,
             'progress': message.progress, 'success': message.mission_success,
             'message': message.message})

    def search_observation(self, message):
        """Keep partial geometric area feedback for the exact search UUID."""
        self.report['search_feedback'].append(
            {'steady': time.monotonic(), 'goal_uuid': bytes(message.goal_id.uuid).hex(),
             'area': message.feedback.area_covered, 'progress': message.feedback.progress})

    def stream(self, name, message):
        """Observe both platform intent and actual downstream setpoint publications."""
        header = getattr(message, 'header', None)
        source = (header.stamp.sec * 1000000000 + header.stamp.nanosec if header else 0)
        self.report['streams'].append(
            {'topic': name, 'steady': time.monotonic(),
             'source_ns': source,
             'command_mode': getattr(message, 'control_mode', None)})

    def battery(self, message):
        """Preserve raw battery health without suppressing invalid samples or protection."""
        self.report.setdefault('battery_samples', []).append(
            {'steady': time.monotonic(), 'source_ns': message.header.stamp.sec * 1000000000 +
             message.header.stamp.nanosec, 'ros_now_ns': self.node.get_clock().now().nanoseconds,
             'voltage': message.voltage if math.isfinite(message.voltage)
             else str(message.voltage),
             'percentage': message.percentage if math.isfinite(message.percentage)
             else str(message.percentage), 'present': message.present,
             'location': message.location, 'serial_number': message.serial_number,
             'status': message.power_supply_status})

    def session(self, message):
        """Record matching lifecycle notices without equating STOP to ownership."""
        self.report['sessions'].append({'steady': time.monotonic(),
                                        'session_id': message.session_id,
                                        'active': message.active})

    def receive(self, topic, message):
        """Save source state and receipt time; preserve original upstream stamps."""
        self.latest[topic] = message
        self.receipts[topic] = time.monotonic()
        self.counts[topic] = self.counts.get(topic, 0) + 1
        if isinstance(message, UAVState):
            self.states.write(json.dumps(self.state()) + '\n')
            confirmed = self.report.get('takeover_confirmed')
            if confirmed and message.armed and message.flight_mode != 7:
                self.report.setdefault('takeover_violations', []).append(self.state())
        else:
            width = height = None
            if isinstance(message, OccupancyGrid):
                width, height = message.info.width, message.info.height
            elif isinstance(message, PointCloud2):
                width, height = message.width, message.height
            self.inputs.write(json.dumps({
                'topic': topic, 'frame': message.header.frame_id,
                'source_ns': message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec,
                'ros_now_ns': self.node.get_clock().now().nanoseconds,
                'receipt': self.receipts[topic], 'width': width, 'height': height}) + '\n')

    def state(self):
        """Return independent source, clock, receipt and vehicle observations."""
        msg = self.latest.get('/uav_state')
        if msg is None:
            return None
        return {'source_ns': msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec,
                'frame': msg.header.frame_id,
                'ros_now_ns': self.node.get_clock().now().nanoseconds,
                'receipt': self.receipts['/uav_state'], 'steady_now': time.monotonic(),
                'armed': msg.armed, 'connected': msg.connected, 'mode': msg.flight_mode,
                'mode_name': msg.flight_mode_name,
                'position': [msg.position_x, msg.position_y, msg.position_z],
                'velocity': [msg.velocity_x, msg.velocity_y, msg.velocity_z],
                'pose_valid': msg.pose_valid, 'velocity_valid': msg.velocity_valid,
                'battery_valid': msg.battery_valid,
                'battery_voltage': msg.battery_voltage if math.isfinite(msg.battery_voltage)
                else str(msg.battery_voltage),
                'battery_remaining': msg.battery_remaining if math.isfinite(msg.battery_remaining)
                else str(msg.battery_remaining)}

    def ready(self, armed):
        """Enforce current observed flight health, not merely cached discovery."""
        msg = self.latest.get('/uav_state')
        return state_ready(msg, self.receipts.get('/uav_state', 0),
                           self.node.get_clock().now().nanoseconds, time.monotonic(), armed)

    def hover(self):
        """Require fresh OFFBOARD and physical low speed before the next motion."""
        msg = self.latest.get('/uav_state')
        return (self.ready(True) and msg.flight_mode == 3 and
                math.hypot(msg.velocity_x, msg.velocity_y, msg.velocity_z) <= 0.3)

    def wait(self, predicate, seconds):
        """Bound every wait and expose premature launch exit as failure."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.03)
            if self.process is not None and self.process.poll() is not None:
                raise RuntimeError('simulation launch exited before requested shutdown')
            if predicate():
                return
        raise TimeoutError(f'evidence deadline {seconds}s; state={self.state()}')

    def stable_hover(self, altitude=None):
        """Observe at least one continuous second without loosening production limits."""
        began = [None]

        def stable():
            at_altitude = (altitude is None or
                           abs(self.latest['/uav_state'].position_z - altitude) <= 0.1)
            if not self.hover() or not at_altitude:
                began[0] = None
                return False
            if began[0] is None:
                began[0] = time.monotonic()
            return time.monotonic() - began[0] >= 1.0

        self.wait(stable, 15)

    def ownership(self, name, identity):
        """Wait for the exact public action UUID to release its execution slot."""
        client = self.node.create_client(GetActionOwnership, name)
        try:
            self.wait(client.service_is_ready, 5)
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                future = client.call_async(GetActionOwnership.Request(goal_id=identity))
                self.wait(future.done, 3)
                reply = future.result()
                self.report['ownership'].append({'service': name, 'requested': identity,
                                                 'returned': reply.goal_id,
                                                 'state': reply.state,
                                                 'steady': time.monotonic()})
                if exact_released(reply, identity):
                    return
            raise TimeoutError('exact action ownership did not release')
        finally:
            self.node.destroy_client(client)

    def send(self, name, kind, goal):
        """Retain the single admission future; never retry an unknown goal."""
        client = ActionClient(self.node, kind, name)
        self.clients.append(client)
        self.wait(client.server_is_ready, 15)
        entry = {'name': name, 'feedback': [], 'admission': 'pending'}
        self.report['actions'].append(entry)

        def feedback(event):
            value = event.feedback
            if hasattr(value, 'distance_remaining'):
                item = {'distance_remaining': value.distance_remaining,
                        'current_segment': value.current_segment,
                        'total_segments': value.total_segments}
            elif hasattr(value, 'current_status'):
                status = value.current_status
                item = {'phase': status.phase_name, 'step': status.current_waypoint,
                        'total': status.total_waypoints, 'progress': status.progress,
                        'state': self.state()}
            elif hasattr(value, 'area_covered'):
                item = {'area_covered': value.area_covered, 'progress': value.progress,
                        'targets_found': value.targets_found, 'state': self.state()}
            else:
                item = {}
            entry['feedback'].append({'steady': time.monotonic(), **item})

        future = client.send_goal_async(goal, feedback_callback=feedback)
        active = {'entry': entry, 'admission': future, 'handle': None, 'result': None}
        self.active.append(active)
        self.wait(future.done, 5)
        handle = future.result()
        entry.update(accepted=handle.accepted, admission='received',
                     goal_uuid=bytes(handle.goal_id.uuid).hex())
        print(json.dumps({k: v for k, v in entry.items() if k != 'feedback'}), flush=True)
        if not handle.accepted:
            raise RuntimeError(name + ' rejected')
        active['handle'] = handle
        active['result'] = handle.get_result_async()
        return active

    def finish(self, active, expected=4, seconds=90):
        """Check the actual result and independent ownership, then release handles."""
        self.wait(active['result'].done, seconds)
        value = active['result'].result()
        entry = active['entry']
        entry.update(status=value.status, success=value.result.success,
                     message=value.result.message, terminal_state=self.state())
        print(json.dumps({k: v for k, v in entry.items() if k != 'feedback'}), flush=True)
        service = OWNERSHIP_SERVICES.get(entry['name'])
        if service:
            self.ownership(service, entry['goal_uuid'])
        if not terminal_matches(value, expected):
            summary = {k: v for k, v in entry.items() if k != 'feedback'}
            raise RuntimeError('unexpected real terminal: ' + str(summary))

    def search(self):
        """Exercise two completed geometric strips through real child navigation."""
        goal = SearchArea.Goal(center_x=1.25, center_y=0.0, area_width=1.5,
                               area_height=2.2, search_altitude=1.5,
                               search_pattern=1, search_speed=0.6)
        action = self.send('/search_area', SearchArea, goal)
        self.finish(action, seconds=240)
        total = goal.area_width * goal.area_height
        feedback = action['entry']['feedback']
        self.wait(lambda: feedback and feedback[-1].get('progress') == 1.0, 5)
        if not feedback or not math.isclose(feedback[-1]['area_covered'], total) or \
                not math.isclose(feedback[-1]['progress'], 1.0):
            raise RuntimeError('completed-strip geometry not observed')
        if len(self.navigation_goals) != 4:
            raise RuntimeError('four search approach/strip navigation UUIDs not observed')
        for identity in sorted(self.navigation_goals):
            self.ownership('/navigation/ownership', identity)
        self.report['search_geometry_area_m2'] = total
        self.report['search_child_navigation_uuids'] = sorted(self.navigation_goals)

    def navigation(self, distance):
        """Create a conservative horizontal goal in the actual odom frame."""
        self.stable_hover()
        state = self.latest['/uav_state']
        goal = NavigateTo.Goal(target_x=state.position_x + distance,
                               target_y=state.position_y, target_z=state.position_z,
                               max_speed=0.6)
        self.report.setdefault('navigation_targets', []).append(
            [goal.target_x, goal.target_y, goal.target_z])
        return self.send('/navigate', NavigateTo, goal)

    def observe_for(self, seconds):
        """Keep observing for the entire window instead of sleeping without callbacks."""
        until = time.monotonic() + seconds
        self.wait(lambda: time.monotonic() >= until, seconds + 2)

    def child_result(self, name, identity):
        """Read an observed child's genuine result using its exact UUID, without canceling it."""
        kind = CHILD_ACTIONS[name]
        client = self.node.create_client(kind.Impl.GetResultService, name + '/_action/get_result')
        try:
            self.wait(client.service_is_ready, 5)
            future = client.call_async(kind.Impl.GetResultService.Request(
                goal_id=UUID(uuid=list(bytes.fromhex(identity)))))
            self.wait(future.done, 10)
            value = future.result()
            if value is None or value.status not in (4, 5, 6) or \
                    value.result.success != (value.status == 4):
                raise RuntimeError(
                    f'child genuine result missing or inconsistent: {name}/{identity}')
            entry = {'name': name, 'goal_uuid': identity, 'status': value.status,
                     'success': value.result.success, 'message': value.result.message,
                     'steady': time.monotonic()}
            self.report['child_results'].append(entry)
            return entry
        finally:
            self.node.destroy_client(client)

    def cancel_exact(self, active):
        """Cancel only the explicitly accepted outer mission and validate the ACK UUID."""
        request = {'goal_uuid': active['entry']['goal_uuid'], 'steady': time.monotonic(),
                   'state': self.state()}
        future = active['handle'].cancel_goal_async()
        self.wait(future.done, 5)
        value = future.result()
        identities = [bytes(v.goal_id.uuid).hex() for v in value.goals_canceling]
        request.update(return_code=value.return_code, ack_uuids=identities)
        self.report['mission_cancel'] = request
        if value.return_code != 0 or request['goal_uuid'] not in identities:
            raise RuntimeError('exact outer mission cancellation rejected')

    def external_land(self):
        """Issue one simulated external mode request without retries or compensation."""
        client = self.node.create_client(SetMode, '/mavros/set_mode')
        try:
            self.wait(client.service_is_ready, 5)
            self.report['takeover_request'] = {'steady': time.monotonic(), 'state': self.state(),
                                               'custom_mode': 'AUTO.LAND'}
            future = client.call_async(SetMode.Request(custom_mode='AUTO.LAND'))
            self.wait(future.done, 5)
            value = future.result()
            self.report['takeover_request']['mode_sent'] = value.mode_sent
            if not value.mode_sent:
                raise RuntimeError('simulation external AUTO.LAND request not sent')
            self.wait(lambda: self.ready(True) and self.latest['/uav_state'].flight_mode == 7, 5)
            self.report['takeover_confirmed'] = {'steady': time.monotonic(), 'state': self.state()}
        finally:
            self.node.destroy_client(client)

    def run_lifecycle(self):
        """Check recursive mission termination without changing production flight policy."""
        step, trigger = lifecycle_spec(self.args.scenario)
        budget = (8 if step == 'nav' else 12) if trigger == 'timeout' else 120
        phase = 'navigate' if step == 'nav' else 'search'
        middle = (f'  - {{type: navigate, x: 4, y: 0, z: 1.5, max_speed: 0.6, '
                  f'timeout: {budget}}}\n' if step == 'nav' else
                  f'  - {{type: search, center_x: 1.25, center_y: 0, area_width: 1.5, '
                  f'area_height: 2.2, altitude: 1.5, pattern: boustrophedon, '
                  f'max_speed: 0.6, timeout: {budget}}}\n')
        prefix = ('schema_version: 1\nframe_id: odom\n'
                  'units: {position: m, angle: rad, time: s}\nfailure_policy: abort\nsteps:\n')
        direct = getattr(self.args, 'lifecycle_preparation', 'isolated') == 'direct'
        self.report['lifecycle_preparation'] = 'direct' if direct else 'isolated'
        takeoff = '  - {type: takeoff, altitude: 1.5, climb_speed: 0.3, timeout: 75}\n'
        if not direct:
            preparation = self.directory / 'preparation.yaml'
            preparation.write_text(prefix + takeoff +
                                   '  - {type: hover, duration: 10, timeout: 20}\n')
            prepared = self.send('/execute_mission', ExecuteMission,
                                 ExecuteMission.Goal(mission_file=str(preparation)))
            self.finish(prepared, seconds=120)
            self.stable_hover(altitude=1.5)
            self.report['preparation'] = {'goal_uuid': prepared['entry']['goal_uuid'],
                                          'state': self.state(), 'steady': time.monotonic(),
                                          'mission_file': str(preparation)}
        fixture = self.directory / 'lifecycle.yaml'
        opening = (takeoff + '  - {type: hover, duration: 2, timeout: 10}\n' if direct else
                   '  - {type: hover, duration: 0.3, timeout: 5}\n')
        fixture.write_text(prefix + opening + middle +
                           '  - {type: land, timeout: 90}\n')
        action = self.send('/execute_mission', ExecuteMission,
                           ExecuteMission.Goal(mission_file=str(fixture)))
        origin = [None]

        def executing_motion():
            if action['result'].done():
                value = action['result'].result()
                action['entry'].update(status=value.status, success=value.result.success,
                                       message=value.result.message, terminal_state=self.state())
                raise RuntimeError('mission terminated before trigger: ' + value.result.message)
            statuses = self.report['action_status']
            nav_ready = len(statuses['/navigate']) >= (
                3 if step == 'search' and trigger == 'cancel' else 1)
            active_curve = any(v['status'] == 2 for v in statuses['/follow_trajectory'].values())
            active_nav = any(v['status'] == 2 for v in statuses['/navigate'].values())
            in_step = any(v['phase'] == phase and v['step'] == (2 if direct else 1)
                          for v in action['entry']['feedback'])
            streaming = trajectory_stream_observed(
                self.report['streams'], time.monotonic(), self.node.get_clock().now().nanoseconds)
            if not (nav_ready and active_curve and active_nav and in_step and streaming and
                    self.ready(True)):
                return False
            position = self.state()['position']
            if origin[0] is None:
                origin[0] = position
            return math.hypot(position[0] - origin[0][0], position[1] - origin[0][1]) >= 0.15

        self.wait(executing_motion, 110)
        statuses = self.report['action_status']
        active_children = [(name, identity) for name in ('/navigate', '/follow_trajectory')
                           for identity, value in statuses[name].items() if value['status'] == 2]
        self.report['lifecycle_trigger'] = {
            'step': phase, 'trigger': trigger, 'budget_s': budget, 'steady': time.monotonic(),
            'motion_origin': origin[0], 'state': self.state(), 'active_children': active_children}
        if step == 'search':
            active_children += [('/search_area', identity)
                                for identity, value in statuses['/search_area'].items()
                                if value['status'] == 2]
            if len(statuses['/search_area']) != 1:
                raise RuntimeError('one owned search step UUID not observed')
        if trigger == 'cancel':
            self.cancel_exact(action)
        elif trigger == 'land':
            self.external_land()
        self.finish(action, expected=5 if trigger == 'cancel' else 6, seconds=40)
        if trigger == 'timeout' and action['entry']['message'] != 'step_timeout':
            raise RuntimeError('failure was not the intended mission step timeout')
        if trigger == 'cancel' and action['entry']['message'] != 'user_cancel':
            raise RuntimeError('mission cancellation was preceded by an unexpected failure')
        observed = [(name, identity)
                    for name in ('/search_area', '/navigate', '/follow_trajectory')
                    for identity in statuses[name]]
        self.wait(lambda: terminal_statuses_observed(statuses, observed), 10)
        timed_parent = '/search_area' if step == 'search' else '/navigate'
        stop_edge = (timeout_stop_edge(statuses[timed_parent])
                     if trigger == 'timeout' else None)
        self.report['timeout_stop_edge_steady'] = stop_edge
        for name, identity in observed:
            result = self.child_result(name, identity)
            if not child_stop_result_valid(
                    result, statuses[name][identity], (name, identity) in active_children,
                    trigger, stop_edge):
                raise RuntimeError('in-flight child incorrectly reported success after stop')
            if name in OWNERSHIP_SERVICES:
                self.ownership(OWNERSHIP_SERVICES[name], identity)
        if trigger == 'timeout' and any(not any(
                v['name'] == name and v['status'] in (5, 6) for v in self.report['child_results'])
                for name in ('/navigate', '/follow_trajectory')):
            raise RuntimeError('unfinished navigation/trajectory cancellation not observed')
        if not sessions_closed(self.report['sessions'], self.navigation_goals):
            raise RuntimeError('recursive mission planner session not stopped')
        negative_ids = {name: list(values) for name, values in statuses.items()}
        self.report['negative_child_uuids'] = negative_ids
        self.observe_for(3 if trigger == 'land' else 2)
        if any(set(values) != set(negative_ids[name]) for name, values in statuses.items()):
            raise RuntimeError('stopped mission started a late additional child action')
        if not following_steps_absent(self.report['mission_events'], statuses['/land'],
                                      self.report['lifecycle_trigger']['steady'],
                                      total_steps=4 if direct else 3):
            raise RuntimeError('terminated mission continued to its Land sentinel')
        self.report['following_steps_absent'] = True
        if step == 'search':
            feedback = self.report['search_feedback']
            if not feedback or any(v['progress'] >= 1 for v in feedback):
                raise RuntimeError('interrupted search falsely completed its geometry')
            area = feedback[-1]['area']
            if trigger == 'cancel' and not math.isclose(area, 1.65):
                raise RuntimeError('completed first strip was not preserved on search cancel')
            self.report['partial_search_area_m2'] = area
        if trigger == 'land':
            confirmed = self.report['takeover_confirmed']['steady']
            if not takeover_streams_quiet(self.report['streams'], confirmed, time.monotonic()):
                raise RuntimeError('platform or downstream setpoints continued after takeover')
            self.wait(lambda: self.ready(False), 60)
            if self.report.get('takeover_violations') or not takeover_streams_quiet(
                    self.report['streams'], confirmed, time.monotonic()):
                raise RuntimeError('platform reclaimed external landing or restarted setpoints')
            self.report['takeover_streams_stopped'] = True
        else:
            self.stable_hover()
            state = self.state()['position']
            target = [state[0] + 0.6, state[1], state[2]]
            recovery = self.directory / 'recovery.yaml'
            recovery.write_text(prefix +
                                '  - {type: hover, duration: 0.3, timeout: 5}\n'
                                f'  - {{type: navigate, x: {target[0]}, y: {target[1]}, '
                                f'z: {target[2]}, max_speed: 0.6, timeout: 60}}\n'
                                '  - {type: land, timeout: 90}\n')
            self.report['recovery_target'] = target
            recovery_action = self.send('/execute_mission', ExecuteMission,
                                        ExecuteMission.Goal(mission_file=str(recovery)))
            self.finish(recovery_action, seconds=180)
            arrived = [v for v in recovery_action['entry']['feedback']
                       if v['phase'] == 'navigate' and v['step'] == 2]
            if not arrived or arrived[-1]['state'] is None:
                raise RuntimeError('recovery mission navigation observation missing')
            self.report['recovery_delayed_feedback_error_m'] = math.dist(
                arrived[-1]['state']['position'], target)
            recovery_ids = set(statuses['/navigate']) - set(negative_ids['/navigate'])
            if len(recovery_ids) != 1:
                raise RuntimeError('one recovery navigation child UUID not observed')
            for identity in recovery_ids:
                self.child_result('/navigate', identity)
                self.ownership('/navigation/ownership', identity)
            curves = set(statuses['/follow_trajectory']) - set(negative_ids['/follow_trajectory'])
            for identity in curves:
                self.child_result('/follow_trajectory', identity)
            observation = terminal_arrival_observation(statuses['/follow_trajectory'], curves)
            self.report['recovery_arrival_observation'] = observation
            error = math.dist(observation['state']['position'], target)
            self.report['recovery_navigation_error_m'] = error
            if error > 0.11:
                raise RuntimeError('recovery mission trajectory terminal arrival not confirmed')
        self.wait(lambda: self.ready(False), 15)
        self.report['final_disarmed_state'] = self.state()
        self.report['sessions_closed'] = sessions_closed(
            self.report['sessions'], self.navigation_goals)
        if not self.report['sessions_closed']:
            raise RuntimeError('final mission navigation sessions not fully stopped')

    def launch_command(self):
        """Build the legacy launch inputs; explicit scene probes may specialize this."""
        command = ['ros2', 'launch', 'easyfly_px4_sitl', 'perception_sitl.launch.py',
                   'headless:=true', 'detection_use_gpu:=false',
                   'detection_input_qos_reliability:=best_effort',
                   'navigation_strategy:=' + self.args.strategy]
        capture = getattr(self.args, 'semantic_capture_shutdown', False)
        if capture:
            command.append('semantic_capture_shutdown:=true')
        self.report['diagnostic_only'] = bool(capture)
        return command

    def start(self):
        """Start and identify one private simulation before any explicit flight scenario."""
        self.log = (self.directory / 'console.log').open('x')
        command = self.launch_command()
        self.report['command'] = command
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT,
                                        env=self.environment, start_new_session=True)
        self.wait(self.startup_ready, 75)
        self.owned = children(self.process.pid)
        server = (Path(get_package_prefix('easyfly_gazebo_plugins')) /
                  'lib/easyfly_gazebo_plugins/easyfly_gzserver').resolve(strict=True)
        if not any(v['executable'] == str(server) for v in self.owned.values()):
            raise RuntimeError('private simulation server not confirmed')
        self.report['server_sha256'] = hashlib.sha256(server.read_bytes()).hexdigest()
        px4 = Path(self.report['staging']['binary']).resolve(strict=True)
        if not any(v['executable'] == str(px4) for v in self.owned.values()):
            raise RuntimeError('selected private PX4 binary not confirmed')
        self.report['px4_sha256'] = hashlib.sha256(px4.read_bytes()).hexdigest()
        self.report['preflight_state'] = self.state()
        self.report['odom_publishers'] = [
            {'node': v.node_name, 'namespace': v.node_namespace, 'type': v.topic_type,
             'depth': v.qos_profile.depth, 'reliability': v.qos_profile.reliability.name}
            for v in self.node.get_publishers_info_by_topic('/odom')]

    def startup_ready(self):
        """Flight checks require the original live map and navigator readiness."""
        return (self.ready(False) and self.counts.get('/occupancy_grid',0)>=3 and
                'ego_planner_node' in self.node.get_node_names())

    def run(self):
        """Execute only the explicitly selected simulation scenario."""
        self.start()
        if self.args.scenario in LIFECYCLE_SCENARIOS:
            self.run_lifecycle()
            return
        if self.args.scenario == 'mission':
            fixture = self.directory / 'mission.yaml'
            fixture.write_text(
                'schema_version: 1\nframe_id: odom\n'
                'units: {position: m, angle: rad, time: s}\nfailure_policy: abort\nsteps:\n'
                '  - {type: takeoff, altitude: 1.5, timeout: 60}\n'
                '  - {type: navigate, x: 2, y: 0, z: 1.5, max_speed: 0.6, timeout: 90}\n'
                '  - {type: land, timeout: 90}\n')
            action = self.send('/execute_mission', ExecuteMission,
                               ExecuteMission.Goal(mission_file=str(fixture)))
            self.finish(action, seconds=240)
            feedback = action['entry']['feedback']
            self.wait(lambda: any(
                v['step'] == 3 and v['total'] == 3 and v['progress'] == 1.0
                for v in feedback), 5)
            if not any(v['step'] == 3 and v['total'] == 3 and v['progress'] == 1.0
                       for v in feedback):
                raise RuntimeError('three completed mission steps not observed')
            arrived = [v for v in feedback if v['phase'] == 'navigate' and v['step'] == 2]
            if not arrived or arrived[-1]['state'] is None:
                raise RuntimeError('mission navigation completion observation missing')
            error = math.dist(arrived[-1]['state']['position'], [2.0, 0.0, 1.5])
            self.report['navigation_terminal_error_m'] = error
            if error > 0.11:
                raise RuntimeError(f'mission navigation arrival not observed: {error}m')
            if len(self.navigation_goals) != 1:
                raise RuntimeError('mission child navigation UUID not observed')
            self.ownership('/navigation/ownership', next(iter(self.navigation_goals)))
        else:
            action = self.send('/takeoff', Takeoff,
                               Takeoff.Goal(target_altitude=1.5, climb_speed=0.3))
            self.finish(action, seconds=75)
            self.stable_hover(altitude=1.5)
            self.report['settled_takeoff_state'] = self.state()
            if self.args.scenario == 'search':
                self.search()
            else:
                self.run_navigation()
            self.stable_hover()
            land = self.send('/land', Land, Land.Goal())
            self.finish(land, seconds=90)
        self.wait(lambda: self.ready(False), 15)
        self.report['final_disarmed_state'] = self.state()
        if self.args.strategy != 'astar':
            self.report['sessions_closed'] = sessions_closed(
                self.report['sessions'], self.navigation_goals)
            if not self.report['sessions_closed']:
                raise RuntimeError('matching START and final STOP not observed')

    def run_navigation(self):
        """Run positive navigation or an explicitly planned negative scenario."""
        nav = self.navigation(2.0 if self.args.scenario == 'happy' else 4.0)
        if self.args.scenario != 'happy':
            start = self.report['navigation_targets'][-1][0] - 4.0
            self.wait(lambda: nav['entry']['feedback'] and self.hover_or_motion(start), 15)
            if nav['result'].done():
                raise RuntimeError('goal ended before requested fault/cancel')
            if self.args.scenario == 'cancel':
                self.report['cancel_request'] = {'goal_uuid': nav['entry']['goal_uuid'],
                                                 'steady': time.monotonic(),
                                                 'state': self.state()}
                cancel = nav['handle'].cancel_goal_async()
                self.wait(cancel.done, 5)
                response = cancel.result()
                self.report['cancel_ack'] = response.return_code
                identities = [bytes(v.goal_id.uuid).hex() for v in response.goals_canceling]
                self.report['cancel_ack_uuids'] = identities
                if response.return_code != 0 or nav['entry']['goal_uuid'] not in identities:
                    raise RuntimeError('exact cancellation rejected')
                self.finish(nav, expected=5, seconds=25)
                recovery = self.navigation(0.6)
                self.finish(recovery)
                error = math.dist(self.report['navigation_targets'][-1],
                                  recovery['entry']['terminal_state']['position'])
                self.report['recovery_navigation_error_m'] = error
                if error > 0.11:
                    raise RuntimeError('recovery navigation actual arrival not confirmed')
            else:
                maps = [pid for pid, data in self.owned.items()
                        if data['executable'].endswith('/semantic_map_node')]
                if len(maps) != 1:
                    raise RuntimeError('unique owned map producer not identified')
                if children(self.process.pid).get(maps[0]) != self.owned[maps[0]]:
                    raise RuntimeError('owned map process identity changed')
                self.report['map_stop'] = {'pid': maps[0], 'steady': time.monotonic(),
                                           'state': self.state()}
                os.kill(maps[0], signal.SIGINT)
                self.finish(nav, expected=6, seconds=25)
        else:
            self.finish(nav)
            target = self.report['navigation_targets'][-1]
            final = self.state()['position']
            error = math.dist(target, final)
            self.report['navigation_terminal_error_m'] = error
            if error > 0.11:
                raise RuntimeError(f'actual navigation did not reach target: {error}m')

    def hover_or_motion(self, start):
        """Prove real movement before cancellation or producer-stop injection."""
        state = self.latest.get('/uav_state')
        return self.ready(True) and state.flight_mode == 3 and state.position_x - start >= 0.15

    def cleanup_actions(self):
        """On failure drain only exact owned handles; never append Land or RTL."""
        for active in self.active:
            try:
                if active['handle'] is None:
                    self.wait(active['admission'].done, 15)
                    handle = active['admission'].result()
                    if not handle.accepted:
                        continue
                    active['handle'] = handle
                    active['result'] = handle.get_result_async()
                    active['entry']['goal_uuid'] = bytes(handle.goal_id.uuid).hex()
                if not active['result'].done():
                    cancel = active['handle'].cancel_goal_async()
                    self.wait(cancel.done, 5)
                    self.wait(active['result'].done, 25)
                entry = active['entry']
                service = OWNERSHIP_SERVICES.get(entry['name'])
                if service:
                    self.ownership(service, entry['goal_uuid'])
            except Exception as error:
                self.report.setdefault('drain_errors', []).append(str(error))

    def close(self):
        """Preserve failed runs and verify owned process shutdown separately."""
        if self.report['error']:
            self.cleanup_actions()
        if self.process is not None:
            if self.process.poll() is None:
                began = time.monotonic()
                self.process.send_signal(signal.SIGINT)
                try:
                    self.process.wait(timeout=25)
                except subprocess.TimeoutExpired:
                    self.report['forced'] = True
                    os.killpg(self.process.pid, signal.SIGTERM)
                    try:
                        self.process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(self.process.pid, signal.SIGKILL)
                        self.process.wait(timeout=3)
                self.report['shutdown_seconds'] = time.monotonic() - began
            self.report['launch_returncode'] = self.process.returncode
        for client in self.clients:
            client.destroy()
        self.node.destroy_node()
        rclpy.try_shutdown()
        self.states.close()
        self.inputs.close()
        if hasattr(self, 'log'):
            self.log.close()
        path = self.directory / 'console.log'
        log = path.read_text(errors='replace') if path.exists() else ''
        started = re.findall(r'process started with pid \[(\d+)\]', log)
        clean = re.findall(r'process has finished cleanly \[pid (\d+)\]', log)
        self.report['started_processes'] = started
        self.report['normal_exits'] = clean
        self.report['abnormal_lines'] = [line for line in log.splitlines() if any(
            word in line for word in ('process has died', 'SIGSEGV', 'SIGABRT',
                                      'SIGKILL', 'trying joining itself', 'terminate called'))]
        self.report['transport_drained'] = 'EasyFly transport drained: live=0 pending=0' in log
        self.report['counts'] = self.counts
        self.report['owned_processes'] = self.owned
        self.report['leftovers'] = []
        for pid, data in self.owned.items():
            try:
                stat = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
                if stat[19] == data['start_ticks']:
                    self.report['leftovers'].append(pid)
            except FileNotFoundError:
                pass
        self.report['passed'] = (self.report['error'] is None and not self.report['forced'] and
                                 self.report.get('launch_returncode') == 0 and
                                 len(clean) == 24 and set(clean) == set(started) and
                                 not self.report['abnormal_lines'] and
                                 self.report['transport_drained'] and
                                 not self.report['leftovers'] and
                                 'final_disarmed_state' in self.report)
        (self.directory / 'result.json').write_text(json.dumps(self.report, indent=2) + '\n')


def main():
    """Require explicit local fixture inputs; default execution is never implicit."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-dir', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--domain', required=True, type=int)
    parser.add_argument('--gazebo-port', required=True, type=int)
    parser.add_argument('--strategy', choices=('astar', 'ego', 'combined'), default='combined')
    parser.add_argument('--semantic-capture-shutdown', action='store_true',
                        help='Opt-in semantic-only stack capture; diagnostic, not normal timing.')
    scenarios = ('happy', 'cancel', 'map-loss', 'mission', 'search') + LIFECYCLE_SCENARIOS
    parser.add_argument('--scenario', choices=scenarios,
                        default='happy')
    parser.add_argument('--lifecycle-preparation', choices=('isolated', 'direct'),
                        default='isolated',
                        help='Direct explicitly checks the original takeoff/hover2s task; '
                        'isolated stable preparation remains the default.')
    args = parser.parse_args()
    if not 0 <= args.domain <= 232 or not 1024 <= args.gazebo_port <= 65535:
        parser.error('invalid explicit domain or local port')
    if args.scenario in LIFECYCLE_SCENARIOS and args.strategy != 'combined':
        parser.error('mission lifecycle scenarios require the complete combined trajectory chain')
    if args.lifecycle_preparation == 'direct' and args.scenario not in LIFECYCLE_SCENARIOS:
        parser.error('direct preparation is only implemented for mission lifecycle scenarios')
    probe = Probe(args)
    try:
        probe.run()
    except (Exception, KeyboardInterrupt) as error:
        probe.report['error'] = type(error).__name__ + ': ' + str(error)
        print(probe.report['error'], flush=True)
    finally:
        probe.close()
    print(json.dumps({'passed': probe.report['passed'],
                      'result': str(probe.directory / 'result.json')}), flush=True)
    raise SystemExit(0 if probe.report['passed'] else 1)


if __name__ == '__main__':
    main()

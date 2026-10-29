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

"""Check strict evidence predicates without spawning any simulation or motion."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

from easyfly_msgs.msg import UAVState
from easyfly_msgs.srv import GetActionOwnership
import pytest

SOURCE = Path(__file__).parents[1] / 'scripts/navigation_sitl.py'
SPEC = importlib.util.spec_from_file_location('navigation_sitl', SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.mark.parametrize('step', ['nav', 'search'])
@pytest.mark.parametrize('trigger', ['cancel', 'timeout', 'land'])
def test_lifecycle_scenarios_are_explicit(step, trigger):
    """Do not silently select a fallback scenario for an unsupported spelling."""
    assert MODULE.lifecycle_spec(f'mission-{step}-{trigger}') == (step, trigger)


@pytest.mark.parametrize('scenario', ['mission', 'mission-nav-rtl', 'mission-track-cancel', ''])
def test_unknown_lifecycle_scenarios_are_rejected(scenario):
    """Unsupported modes cannot implicitly trigger an external control request."""
    with pytest.raises(ValueError):
        MODULE.lifecycle_spec(scenario)


def test_child_status_requires_exact_uuid_and_terminal():
    """Outer completion cannot substitute for a missing or still-running child."""
    selected = [('/navigate', 'a'), ('/follow_trajectory', 'b')]
    statuses = {'/navigate': {'a': {'status': 5}}, '/follow_trajectory': {'b': {'status': 2}}}
    assert not MODULE.terminal_statuses_observed(statuses, selected)
    statuses['/follow_trajectory']['b']['status'] = 6
    assert MODULE.terminal_statuses_observed(statuses, selected)
    assert not MODULE.terminal_statuses_observed(statuses, selected + [('/navigate', 'wrong')])
    assert not MODULE.terminal_statuses_observed(statuses, [])


def test_stopped_mission_cannot_continue_to_land_or_complete():
    """Ignore old completed steps, but reject any later Land or full completion."""
    old = {'steady': 1, 'phase': 'takeoff', 'step': 1, 'success': False}
    idle = {'steady': 3, 'phase': 'idle', 'step': 2, 'success': False}
    assert MODULE.following_steps_absent([old, idle], {}, 2)
    assert not MODULE.following_steps_absent([old, idle], {'land_uuid': {}}, 2)
    for phase, step, success in [('land', 3, False), ('idle', 4, False), ('idle', 2, True)]:
        bad = {'steady': 3, 'phase': phase, 'step': step, 'success': success}
        assert not MODULE.following_steps_absent([old, bad], {}, 2)
    done = {'steady': 3, 'phase': 'idle', 'step': 3, 'success': False}
    assert not MODULE.following_steps_absent([old, done], {}, 2, total_steps=3)
    assert MODULE.following_steps_absent([old, idle], {}, 2, total_steps=3)


def test_takeover_quiet_window_is_actual_observation():
    """Do not accept a short window or a continuing platform/downstream stream."""
    old = {'topic': '/uav_command', 'steady': 1}
    assert MODULE.streams_quiet([old], 2, 3)
    assert not MODULE.streams_quiet([old], 2, 2.9)
    for topic in ('/uav_command', '/mavros/setpoint_raw/local'):
        assert not MODULE.streams_quiet([old, {'topic': topic, 'steady': 2}], 2, 4)


def test_motion_trigger_requires_trajectory_intent_and_actual_stream():
    """Do not inject on an accepted-but-waiting trajectory or stale command history."""
    intent = {'topic': '/uav_command', 'steady': 20, 'source_ns': 10000000000,
              'command_mode': MODULE.UAVCommand.TRAJECTORY}
    output = {'topic': '/mavros/setpoint_raw/local', 'steady': 20, 'source_ns': 10000000000}
    assert MODULE.trajectory_stream_observed([intent, output], 20.1, 10000000000)
    assert not MODULE.trajectory_stream_observed([intent], 20.1, 10000000000)
    assert not MODULE.trajectory_stream_observed([output], 20.1, 10000000000)
    assert not MODULE.trajectory_stream_observed([intent, output], 20.5, 10000000000)
    assert not MODULE.trajectory_stream_observed([intent, output], 20.1, 12000000000)
    intent['command_mode'] = MODULE.UAVCommand.POSITION
    assert not MODULE.trajectory_stream_observed([intent, output], 20.1, 10000000000)


def test_takeover_respects_bridge_tail_but_requires_both_streams_stopped():
    """Keep existing one-second bridge expiry distinct from controller publications."""
    cached = {'topic': '/mavros/setpoint_raw/local', 'steady': 20.9}
    assert MODULE.takeover_streams_quiet([cached], 20, 23)
    assert not MODULE.takeover_streams_quiet([cached], 20, 22)
    for topic, stamp in [('/uav_command', 20.5), ('/mavros/setpoint_position/local', 21.5),
                         ('/mavros/setpoint_raw/attitude', 21.5),
                         ('/mavros/setpoint_velocity/cmd_vel_unstamped', 21.5)]:
        assert not MODULE.takeover_streams_quiet(
            [cached, {'topic': topic, 'steady': stamp}], 20, 23)


def test_arrival_uses_terminal_event_not_delayed_upper_feedback():
    """Retain a precise first terminal observation even after mission feedback drifts."""
    state = {'armed': True, 'connected': True, 'mode': 3, 'frame': 'odom',
             'pose_valid': True, 'velocity_valid': True, 'source_ns': 10000000000,
             'ros_now_ns': 10000000000, 'receipt': 20, 'position': [1.0, 2.0, 1.5]}
    observed = {'steady': 20.1, 'state': state}
    statuses = {'old': {'status': 4, 'first_terminal': {'steady': 19, 'state': state}},
                'current': {'status': 4, 'first_terminal': observed},
                'canceled': {'status': 5, 'first_terminal': {'steady': 21, 'state': state}}}
    assert MODULE.terminal_arrival_observation(
        statuses, {'old', 'current', 'canceled'}) == observed
    with pytest.raises(RuntimeError):
        MODULE.terminal_arrival_observation(statuses, {'wrong', 'canceled'})
    for field, invalid in [('source_ns', 0), ('receipt', 19), ('pose_valid', False),
                           ('frame', 'map'), ('mode', 7), ('position', [float('nan'), 0, 1])]:
        bad = {**state, field: invalid}
        with pytest.raises(RuntimeError):
            MODULE.terminal_arrival_observation(
                {'current': {'status': 4, 'first_terminal': {'steady': 20.1, 'state': bad}}},
                {'current'})


def test_timeout_allows_only_segments_completed_before_stop_edge():
    """Do not confuse a normal approach completed during the step budget with failed cancel."""
    statuses = {'parent': {'history': [
        {'steady': 10, 'status': 2}, {'steady': 22, 'status': 3}, {'steady': 22.2, 'status': 5}]}}
    edge = MODULE.timeout_stop_edge(statuses)
    assert edge == 22
    assert MODULE.child_stop_result_valid(
        {'status': 4}, {'first_terminal': {'steady': 14}}, True, 'timeout', edge)
    assert not MODULE.child_stop_result_valid(
        {'status': 4}, {'first_terminal': {'steady': 22}}, True, 'timeout', edge)
    assert not MODULE.child_stop_result_valid({'status': 4}, {}, True, 'timeout', edge)
    assert MODULE.child_stop_result_valid({'status': 5}, {}, True, 'timeout', edge)
    assert MODULE.child_stop_result_valid({'status': 6}, {}, True, 'timeout', edge)
    assert not MODULE.child_stop_result_valid({'status': 4}, {}, True, 'cancel', None)
    assert MODULE.child_stop_result_valid({'status': 4}, {}, False, 'cancel', None)
    with pytest.raises(RuntimeError):
        MODULE.timeout_stop_edge({'parent': {'history': [{'steady': 10, 'status': 2}]}})


@pytest.mark.parametrize('failure', ['finish', 'stable'])
def test_preparation_failure_never_submits_test_mission(tmp_path, failure):
    """Keep both real preparation success and physical stabilization before test admission."""
    probe = object.__new__(MODULE.Probe)
    probe.args = SimpleNamespace(scenario='mission-nav-cancel')
    probe.directory, probe.report = tmp_path, {}
    calls = []

    def send(name, kind, goal):
        calls.append(goal.mission_file)
        return {'entry': {'goal_uuid': 'a' * 32}}

    def finish(*args, **kwargs):
        if failure == 'finish':
            raise RuntimeError('preparation_failed')

    def stable(**kwargs):
        assert kwargs == {'altitude': 1.5}
        raise TimeoutError('not_stable')

    probe.send, probe.finish, probe.stable_hover = send, finish, stable
    with pytest.raises((RuntimeError, TimeoutError)):
        probe.run_lifecycle()
    assert len(calls) == 1
    assert Path(calls[0]).name == 'preparation.yaml'
    assert not (tmp_path / 'lifecycle.yaml').exists()


def test_stable_preparation_precedes_separate_three_step_mission(tmp_path):
    """Submit the test mission only after successful preparation and observed stabilization."""
    probe = object.__new__(MODULE.Probe)
    probe.args = SimpleNamespace(scenario='mission-search-cancel')
    probe.directory, probe.report = tmp_path, {}
    events = []

    def send(name, kind, goal):
        events.append(Path(goal.mission_file).name)
        if len(events) > 3:
            raise RuntimeError('test_mission_reached')
        return {'entry': {'goal_uuid': 'a' * 32}}

    def finish(*args, **kwargs):
        events.append('success_and_released')

    def stable(**kwargs):
        assert kwargs == {'altitude': 1.5}
        events.append('physical_stabilization')

    probe.send, probe.finish, probe.stable_hover = send, finish, stable
    probe.state = lambda: {'position': [0, 0, 1.5]}
    with pytest.raises(RuntimeError, match='test_mission_reached'):
        probe.run_lifecycle()
    assert events == ['preparation.yaml', 'success_and_released',
                      'physical_stabilization', 'lifecycle.yaml']
    content = (tmp_path / 'lifecycle.yaml').read_text()
    assert 'type: takeoff' not in content
    assert content.count('  - {type:') == 3
    assert 'type: search' in content and 'type: land' in content


def test_explicit_direct_protocol_preserves_original_four_step_task(tmp_path):
    """Do not insert stable preparation into the original failure regression."""
    probe = object.__new__(MODULE.Probe)
    probe.args = SimpleNamespace(scenario='mission-search-timeout',
                                 lifecycle_preparation='direct')
    probe.directory, probe.report = tmp_path, {}

    def send(name, kind, goal):
        assert Path(goal.mission_file).name == 'lifecycle.yaml'
        raise RuntimeError('direct_mission_reached')

    probe.send = send
    with pytest.raises(RuntimeError, match='direct_mission_reached'):
        probe.run_lifecycle()
    assert probe.report['lifecycle_preparation'] == 'direct'
    assert not (tmp_path / 'preparation.yaml').exists()
    content = (tmp_path / 'lifecycle.yaml').read_text()
    assert content.count('  - {type:') == 4
    assert 'climb_speed: 0.3' in content
    assert 'duration: 2, timeout: 10' in content
    assert 'max_speed: 0.6, timeout: 12' in content
    assert 'failure_policy: abort' in content


def state():
    """Create a valid source-stamped sample independently of ROS receipt time."""
    msg = UAVState()
    msg.header.frame_id = 'odom'
    msg.header.stamp.sec = 10
    msg.connected = True
    msg.pose_valid = True
    msg.velocity_valid = True
    msg.attitude_valid = True
    msg.battery_valid = True
    msg.battery_remaining = 1.0
    return msg


def test_fresh_state_requires_source_and_receipt():
    """Do not let a new receipt revive stale or future source measurements."""
    msg = state()
    assert MODULE.state_ready(msg, 20.0, 10000000000, 20.1, False)
    assert not MODULE.state_ready(msg, 20.0, 12000000000, 20.1, False)
    assert not MODULE.state_ready(msg, 20.0, 9000000000, 20.1, False)
    assert not MODULE.state_ready(msg, 20.0, 10000000000, 20.5, False)
    assert not MODULE.state_ready(msg, 21.0, 10000000000, 20.5, False)


@pytest.mark.parametrize('field', [
    'connected', 'pose_valid', 'velocity_valid', 'attitude_valid', 'battery_valid'])
def test_independent_health_flags_are_required(field):
    """Reject a numerically plausible sample whose source health is invalid."""
    msg = state()
    setattr(msg, field, False)
    assert not MODULE.state_ready(msg, 20.0, 10000000000, 20.1, False)


@pytest.mark.parametrize('remaining', [float('nan'), float('inf'), -0.01, 1.01])
def test_invalid_battery_cannot_pass_preflight(remaining):
    """Wait for the same valid battery range required by mission admission."""
    msg = state()
    msg.battery_remaining = remaining
    assert not MODULE.state_ready(msg, 20.0, 10000000000, 20.1, False)


@pytest.mark.parametrize('field', [
    'position_x', 'position_y', 'position_z', 'velocity_x',
    'velocity_y', 'velocity_z', 'roll', 'pitch', 'yaw'])
def test_nonfinite_state_is_not_flight_ready(field):
    """Reject NaN even when the upstream valid flags have been set."""
    msg = state()
    setattr(msg, field, float('nan'))
    assert not MODULE.state_ready(msg, 20.0, 10000000000, 20.1, False)


def test_armed_frame_and_zero_clock_are_checked():
    """Keep ground preflight distinct from an owned OFFBOARD flight state."""
    msg = state()
    msg.armed = True
    assert not MODULE.state_ready(msg, 20.0, 10000000000, 20.1, False)
    msg.armed = False
    msg.header.frame_id = 'map'
    assert not MODULE.state_ready(msg, 20.0, 10000000000, 20.1, False)
    msg.header.frame_id = 'odom'
    assert not MODULE.state_ready(msg, 20.0, 0, 20.1, False)


def test_release_matches_exact_identity():
    """Never mistake another goal's release or UNKNOWN for our own release."""
    identity = 'a' * 32
    released = GetActionOwnership.Response.RELEASED
    assert MODULE.exact_released(GetActionOwnership.Response(
        goal_id=identity, state=released), identity)
    assert not MODULE.exact_released(GetActionOwnership.Response(
        goal_id='b' * 32, state=released), identity)
    assert not MODULE.exact_released(GetActionOwnership.Response(
        goal_id=identity, state=GetActionOwnership.Response.UNKNOWN), identity)
    assert not MODULE.exact_released(GetActionOwnership.Response(
        goal_id=identity, state=GetActionOwnership.Response.OWNED), identity)
    assert not MODULE.exact_released(None, identity)
    assert not MODULE.exact_released(GetActionOwnership.Response(
        goal_id='', state=released), '')


def test_terminal_cannot_be_replaced_by_ack_or_false_success():
    """Check exact action status and result success independently."""
    def result(status, success):
        return SimpleNamespace(status=status, result=SimpleNamespace(success=success))
    assert MODULE.terminal_matches(result(4, True), 4)
    assert not MODULE.terminal_matches(result(4, False), 4)
    assert not MODULE.terminal_matches(result(2, True), 4)
    assert MODULE.terminal_matches(result(5, False), 5)
    assert not MODULE.terminal_matches(result(5, True), 5)
    assert MODULE.terminal_matches(result(6, False), 6)
    assert not MODULE.terminal_matches(result(2, False), 2)
    assert not MODULE.terminal_matches(None, 4)


def test_stamp_representation_is_validated():
    """Reject malformed source clocks even when the computed age looks fresh."""
    msg = state()
    msg.header.stamp.sec = -1
    msg.header.stamp.nanosec = 2000000000
    assert not MODULE.state_ready(msg, 20.0, 1000000000, 20.1, False)
    msg.header.stamp.sec = 9
    msg.header.stamp.nanosec = 1000000000
    assert not MODULE.state_ready(msg, 20.0, 10000000000, 20.1, False)


@pytest.mark.parametrize('events,identities,expected', [
    ([], ['a' * 32], False),
    ([], [], False),
    ([(1, True)], ['a' * 32], False),
    ([(1, False)], ['a' * 32], False),
    ([(1, True), (1, False)], ['a' * 32], True),
    ([(1, True), (1, False)], ['b' * 32], False),
    ([(1, True), (1, False), (2, True)], ['a' * 32], False),
    ([(1, True), (1, False), (2, True), (2, False)], ['a' * 32], True),
])
def test_every_session_requires_matching_start_and_final_stop(events, identities, expected):
    """Do not sign a started later segment using an earlier segment's STOP."""
    notices = [{'session_id': 'a' * 32 + ':' + str(segment), 'active': active}
               for segment, active in events]
    assert MODULE.sessions_closed(notices, identities) == expected

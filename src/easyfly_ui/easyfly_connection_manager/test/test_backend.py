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
"""Lifecycle admission, endpoints and ownership checks without starting a flight backend."""
import stat

import pytest

from easyfly_connection_manager.backend import (
    can_stop, check, command, private, urls, preview_geometry)


def test_scene_preview_uses_only_bounded_static_geometry():
    data = b'''<sdf><world><model><static>true</static><pose>1 2 0 0 0 0</pose>
    <link><collision><geometry><box><size>3 4 1</size></box></geometry></collision></link>
    </model><model><static>false</static><link><collision><geometry><box>
    <size>9 9 9</size></box></geometry></collision></link></model></world></sdf>'''
    assert preview_geometry(data) == [dict(x=1, y=2, width=3, height=4, yaw=0, ground=True)]
    assert preview_geometry(b'x' * (2 * 1024 * 1024 + 1)) == []


def profile():
    return dict(mode='sitl', simulation_time=True, selected_profile='navigation_dev',
                simulation=dict(scene='primitives_sparse_seed42_v1', px4_directory='',
                                gazebo_port=11495, gazebo_gui=False),
                fcu=dict(transport='udp', bind_host='127.0.0.1', bind_port=14540,
                         remote_host='127.0.0.1', remote_port=14580,
                         serial_device='/dev/ttyACM0', baud=921600, target_system=1),
                gcs=dict(enabled=False, host='127.0.0.1', port=14560, bind_port=14558))


def test_transport_urls_are_arguments_and_not_a_shell():
    p = profile()
    assert urls(p) == ('udp://127.0.0.1:14540@127.0.0.1:14580', '')
    assert not any(item.startswith('gcs_url:=') for item in command(p))
    p['gcs']['enabled'] = True
    assert urls(p)[1] == 'udp://127.0.0.1:14558@127.0.0.1:14560'
    args = command(p)
    assert args[:4] == ['ros2', 'launch', 'easyfly_sim_launcher', 'profile.launch.py']
    assert 'gcs_url:=udp://127.0.0.1:14558@127.0.0.1:14560' in args
    assert 'headless:=true' in args
    assert 'rviz:=false' in args
    p['mode'] = 'communication'
    p['fcu']['transport'] = 'serial'
    assert urls(p)[0] == '/dev/ttyACM0:921600'
    assert command(p)[2:4] == ['easyfly_mavlink_bridge', 'mavlink_bridge.launch.py']
    p['fcu']['transport'] = 'tcp'
    assert urls(p)[0] == 'tcp://127.0.0.1:14580'


@pytest.mark.parametrize('change', [
    dict(connected=False), dict(armed=True), dict(landed=2), dict(state_at=90),
    dict(landed_at=90), dict(state_sources=2), dict(landed_sources=0), dict(active_actions=True)])
def test_unknown_stale_armed_airborne_or_multiple_sources_cannot_stop(change):
    state = dict(connected=True, armed=False, landed=1, state_at=100, landed_at=100,
                 state_sources=1, landed_sources=1)
    assert can_stop(state, 101)
    state.update(change)
    assert not can_stop(state, 101)
    assert not can_stop({}, 101)


def test_private_directory_refuses_symlinks_and_public_permissions(tmp_path):
    good = private(tmp_path / 'private')
    assert stat.S_IMODE(good.stat().st_mode) == 0o700
    link = tmp_path / 'link'
    link.symlink_to(good, target_is_directory=True)
    with pytest.raises(ValueError):
        private(link)
    good.chmod(0o755)
    with pytest.raises(ValueError):
        private(good)


def test_bad_serial_is_rejected_before_any_launch():
    p = profile()
    p['mode'] = 'communication'
    p['fcu']['transport'] = 'serial'
    p['fcu']['serial_device'] = '/dev/easyfly-device-does-not-exist'
    with pytest.raises(ValueError, match='串口设备不存在'):
        check(p)


def test_native_gazebo_window_uses_normal_managed_launch():
    p = profile()
    p['simulation']['gazebo_gui'] = True
    assert 'headless:=false' in command(p)


def test_draft_scene_cannot_launch_even_if_installed():
    p = profile()
    p['simulation']['scene'] = 'empty'
    with pytest.raises((ValueError, RuntimeError), match='refused|compatibility|compatible|场景'):
        check(p)


def test_real_ros_diagnostic_byte_level_is_json_serializable():
    import json
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from easyfly_connection_manager.backend import Backend
    p = profile()
    p['gcs']['enabled'] = True
    worker = Backend.__new__(Backend)
    worker.result = {'gcs_url': urls(p)[1]}
    message = DiagnosticArray()
    status = DiagnosticStatus()
    status.name = 'mavros_router: endpoint 1: ' + urls(p)[1]
    status.level = bytes([0])
    status.values = [KeyValue(key='Rx total bytes', value='128')]
    message.status = [status]
    worker.on_diagnostics(message)
    result = json.loads(json.dumps(worker.gcs))
    assert result['level'] == 0
    assert result['values']['Rx total bytes'] == '128'


def test_unexpected_observer_error_keeps_launch_owner_and_clears_old_evidence():
    from types import SimpleNamespace
    from unittest.mock import Mock
    from easyfly_connection_manager.backend import run_owned
    process = Mock()
    process.poll.return_value = None
    worker = SimpleNamespace(process=process, state={'armed': False},
                             run=Mock(side_effect=[RuntimeError('observer broke'), 0]))
    assert run_owned(worker) == 0
    assert worker.state == {}
    assert 'observer broke' in worker.error
    assert worker.run.call_count == 2
    process.send_signal.assert_not_called()
    process.kill.assert_not_called()


def test_source_age_and_uninitialized_clock_do_not_permit_startup_abort_after_contact():
    from types import SimpleNamespace
    from unittest.mock import Mock
    from mavros_msgs.msg import State, ExtendedState
    from easyfly_connection_manager.backend import Backend
    worker = Backend.__new__(Backend)
    worker.state = {}
    worker.ever_connected = False
    worker.node = Mock()
    worker.node.get_clock.return_value.now.return_value = SimpleNamespace(nanoseconds=0)
    message = State()
    message.connected = True
    message.armed = True
    worker.on_state(message)
    assert worker.ever_connected
    assert worker.state == {}
    worker.node.get_clock.return_value.now.return_value = SimpleNamespace(nanoseconds=100 * 10**9)
    message.header.stamp.sec = 90
    worker.on_state(message)
    assert worker.state == {}
    message.armed = False
    message.header.stamp.sec = 100
    worker.on_state(message)
    assert worker.state['armed'] is False
    landing = ExtendedState()
    landing.header.stamp.sec = 90
    landing.landed_state = 1
    worker.on_landed(landing)
    assert 'landed_at' not in worker.state


def test_two_managers_cannot_reserve_another_local_backend(tmp_path, monkeypatch):
    import os
    from easyfly_connection_manager import backend
    from easyfly_connection_manager.backend import reserve
    # Exercise the real flock without contending with an operator's live backend.
    actual_private = backend.private
    monkeypatch.setattr(backend, 'private', lambda _: actual_private(tmp_path / 'reservation'))
    p = profile()
    p['mode'] = 'communication'
    p['fcu']['transport'] = 'tcp'
    descriptor = reserve(p)
    try:
        with pytest.raises(RuntimeError, match='已有本机通信'):
            reserve(p)
    finally:
        os.close(descriptor)


def test_stalled_clock_and_stop_timeout_keep_owner_and_reject_stale_stop(tmp_path):
    import time
    from types import SimpleNamespace
    from unittest.mock import Mock
    from easyfly_connection_manager.backend import Backend
    now = time.monotonic()
    worker = Backend.__new__(Backend)
    worker.profile, worker.directory = profile(), tmp_path
    worker.node = None
    worker.process = SimpleNamespace(poll=lambda: None, returncode=None, pid=123,
                                     send_signal=Mock())
    worker.state = dict(connected=True, armed=False, landed=1, state_sources=1,
                        landed_sources=1, state_at=now - 5, landed_at=now - 5)
    worker.action_states = {}
    worker.ever_connected = True
    worker.started, worker.stopping_at = now - 100, None
    worker.phase, worker.message, worker.gcs = 'ready', '', {}
    worker.result = dict(fcu_url='', gcs_url='')
    worker.clock_ns, worker.clock_progress_at = 10**9, now - 5
    value = worker.status()
    assert value['diagnosis'] == 'clock_stalled'
    assert not value['safe_to_stop'] and not value['can_abort_startup']
    assert value['process_running']
    with pytest.raises(RuntimeError, match='拒绝停止'):
        worker.stop()
    worker.process.send_signal.assert_not_called()
    worker.stopping_at = now - 36
    value = worker.status()
    assert value['diagnosis'] == 'stop_unknown' and value['process_running']
    worker.stop()
    worker.process.send_signal.assert_not_called()

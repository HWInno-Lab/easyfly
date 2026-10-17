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

"""Actual read-only DDS/desktop sessions; no simulator, hardware or flight goals."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import uuid

from ament_index_python.packages import get_package_prefix
from easyfly_connection_manager.ownership import identity


def wait_for(predicate, timeout=12):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        time.sleep(0.1)
    raise AssertionError('condition timed out')


def test_real_sessions(tmp_path):
    prefix = Path(get_package_prefix('easyfly_connection_manager'))
    runtime = tmp_path / 'runtime'
    runtime.mkdir(mode=0o700)
    root = Path(os.environ.get('EASYFLY_CM_EVIDENCE_ROOT', str(tmp_path)))
    root.mkdir(parents=True, exist_ok=True)
    config = tmp_path / 'config'
    config.mkdir(mode=0o700)
    env = dict(os.environ, QT_QPA_PLATFORM='offscreen', ROS_LOCALHOST_ONLY='1',
               XDG_CONFIG_HOME=str(config),
               RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
                'ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'CYCLONEDDS_URI'):
        env.pop(key, None)
    processes = []
    snapshots = []
    supervisor = None
    workspaces = []

    def launch(program, arguments, values, label):
        log = (root / (label + '.log')).open('w')
        process = subprocess.Popen([str(program), *arguments], env=values,
                                   stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        log.close()
        processes.append(process)
        return process

    def rpc(op, **arguments):
        request = dict(v=1, id=str(uuid.uuid4()), op=op, **arguments)
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(2)
            client.connect(str(runtime / 'control.sock'))
            client.sendall(json.dumps(request).encode() + b'\n')
            line = client.makefile('rb').readline(65537)
        assert len(line) <= 65536, 'unbounded response'
        response = json.loads(line)
        assert response['ok'], response
        return response['status']

    def snapshot(label, predicate):
        def sample():
            try:
                status = rpc('status')
            except (ConnectionRefusedError, FileNotFoundError):
                return None
            if predicate(status):
                record = json.loads((runtime / 'session.json').read_text())
                workspace = record.get('workspace_identity', {})
                if workspace and workspace not in workspaces:
                    workspaces.append(workspace)
                snapshots.append(dict(label=label, status=status))
                (root / 'live-snapshots.json').write_text(json.dumps(snapshots, indent=2))
                return status
            return None
        return wait_for(sample)

    def profile(domain):
        return dict(id=str(uuid.uuid4()), name='ROS domain ' + str(domain), mode='observe',
                    domain=domain, scope='local', rmw='rmw_fastrtps_cpp', discovery='default',
                    simulation_time=True, selected_profile='navigation_dev',
                    front_topic='', down_topic='')

    def close_session():
        status = rpc('status')
        if status['process_running']:
            rpc('close', session=status['session'])
        return snapshot('workspace-stopped', lambda value: not value['process_running'])

    try:
        executable = prefix / 'lib/easyfly_connection_manager/easyfly_connection_supervisor'
        supervisor = launch(executable, ['--runtime-directory', str(runtime)], env, 'supervisor')
        wait_for(lambda: (runtime / 'control.sock').exists())
        assert not rpc('status')['process_running']
        # Python fixtures are invoked through the current interpreter, never as executable scripts.
        fixture = Path(__file__).with_name('state_fixture.py')

        def python_source(domain, x, label, no_clock=False):
            args = [str(fixture), '--x', str(x), '--control',
                    str(root / (label + '-control.json')), '--report',
                    str(root / (label + '-counts.json'))]
            if no_clock:
                args.append('--no-clock')
            return launch(sys.executable, args, dict(env, ROS_DOMAIN_ID=str(domain)), label)

        first = python_source(97, 1.25, 'source97')
        rpc('start', profile=profile(97))
        good = snapshot('domain97-fresh', lambda value:
                        value['workspace_fresh'] and value['workspace'].get('state_fresh'))
        assert good['workspace']['domain'] == '97'
        assert good['workspace']['position_x'] == 1.25
        record = json.loads((runtime / 'session.json').read_text())
        original_session = good['session']
        original_workspace = record['workspace_identity']
        supervisor.kill()  # Only this fixture's supervisor; no vehicle is present.
        assert supervisor.wait(timeout=5) != 0
        supervisor = launch(executable, ['--runtime-directory', str(runtime)],
                            env, 'supervisor-recovered')
        recovered = snapshot('authenticated-workspace-recovered', lambda value:
                             value['workspace_fresh'] and value['workspace'].get('state_fresh'))
        assert recovered['session'] == original_session
        assert (json.loads((runtime / 'session.json').read_text())['workspace_identity'] ==
                original_workspace)
        close_session()
        rpc('start', profile=profile(98))
        wrong = snapshot('domain98-no-source', lambda value:
                         value['workspace_fresh'] and
                         value['workspace'].get('state_publishers') == 0)
        assert not wrong['workspace']['state_received']
        assert wrong['workspace']['position_x'] is None
        second = python_source(98, 9.5, 'source98')
        good = snapshot('domain98-fresh', lambda value:
                        value['workspace_fresh'] and value['workspace'].get('state_fresh'))
        assert good['workspace']['position_x'] == 9.5
        other = python_source(98, 88.0, 'duplicate98', no_clock=True)
        conflict = snapshot('duplicate-source-invalid', lambda value:
                            value['workspace_fresh'] and
                            value['workspace'].get('state_publishers') == 2)
        assert not conflict['workspace']['state_fresh']
        assert conflict['workspace']['position_x'] is None
        other.send_signal(signal.SIGINT)
        assert other.wait(timeout=8) == 0
        snapshot('duplicate-removed-recovered', lambda value:
                 value['workspace_fresh'] and value['workspace'].get('state_fresh') and
                 value['workspace'].get('state_publishers') == 1)
        control = root / 'source98-control.json'
        pending = control.with_suffix('.pending')
        pending.write_text(json.dumps(dict(mode='pause', revision=1)))
        pending.replace(control)
        paused = snapshot('paused-clock-invalid', lambda value:
                          value['workspace_fresh'] and
                          not value['workspace'].get('state_fresh') and
                          value['workspace'].get('clock_quality') == '模拟时钟暂停')
        assert not paused['workspace']['position_valid']
        epoch = paused['workspace']['epoch']
        pending.write_text(json.dumps(dict(mode='advance', revision=2, rewind=True)))
        pending.replace(control)
        snapshot('rewind-new-epoch-recovered', lambda value:
                 value['workspace_fresh'] and value['workspace'].get('state_fresh') and
                 value['workspace'].get('epoch') != epoch)
        close_session()
        for process in (first, second):
            process.send_signal(signal.SIGINT)
            assert process.wait(timeout=8) == 0
        for label in ('source97', 'source98', 'duplicate98'):
            counts = json.loads((root / (label + '-counts.json')).read_text())
            assert counts['published'] > 0
            assert all(counts[key] == 0 for key in
                       ('goals', 'cancels', 'commands', 'mission_writes'))
        rpc('exit_service')
        assert supervisor.wait(timeout=5) == 0
        (root / 'live-result.json').write_text(json.dumps(dict(
            passed=True, scenarios=[item['label'] for item in snapshots],
            sources='typed ROS fixtures', flight_goals=0, commands=0,
            sitl_started=False, qgc_started=False), indent=2))
    finally:
        if supervisor is not None and supervisor.poll() is None:
            try:
                close_session()
                rpc('exit_service')
                supervisor.wait(timeout=5)
            except (OSError, AssertionError, subprocess.TimeoutExpired):
                pass
        # A killed supervisor is already reaped, but its read-only desktop can
        # survive. Close only recorded fixture identities, including that case.
        for workspace in workspaces:
            pid = workspace['pid']
            try:
                environment = (Path('/proc') / str(pid) / 'environ').read_bytes().split(b'\0')
                proof = ('EASYFLY_CONNECTION_SOCKET=' + str(runtime / 'control.sock')).encode()
                if identity(pid) == workspace and proof in environment:
                    os.kill(pid, signal.SIGINT)
            except (OSError, ValueError):
                pass
        # These are private test process groups created above, not production discovery/cleanup.
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)

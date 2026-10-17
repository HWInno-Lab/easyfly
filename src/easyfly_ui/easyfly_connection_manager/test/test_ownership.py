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
"""Exercise the real private socket, credentials, bounded input and terminal receipt."""
import json
import os
import socket
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from easyfly_connection_manager.ownership import OwnerChannel, identity


def test_authenticated_queries_and_stop_rejection_keep_owner(tmp_path):
    tmp_path.chmod(0o700)
    worker = SimpleNamespace(directory=tmp_path,
                             stop=Mock(side_effect=RuntimeError('state unknown')),
                             status=lambda: dict(process_running=True, safe_to_stop=False))
    token = 'a' * 64
    channel = OwnerChannel(tmp_path, token, 'session-a', worker)

    def request(**fields):
        peer = socket.socket(socket.AF_UNIX)
        peer.settimeout(0.2)
        peer.connect(str(tmp_path / 'backend.sock'))
        peer.sendall(json.dumps(fields).encode() + b'\n')
        for _ in range(3):
            channel.poll()
        result = peer.recv(32769)
        peer.close()
        return json.loads(result) if result else None

    try:
        assert not request(token='b' * 64, session='session-a', op='stop')
        assert not request(token=token, session='session-other', op='stop')
        assert not request(token=token, session='session-a', op='kill')
        worker.stop.assert_not_called()
        status = request(token=token, session='session-a', op='status')
        assert status['owner'] == identity(os.getpid())
        assert status['status']['process_running']
        assert request(token=token, session='session-a', op='stop')['error'] == 'state unknown'
        worker.stop.assert_called_once()
        terminal = dict(process_running=False, exit_code=0, phase='stopped')
        channel.finish(terminal)
        receipt = json.loads((tmp_path / 'terminal.json').read_text())
        assert receipt['status'] == terminal and receipt['token'] == token
        assert (tmp_path / 'terminal.json').stat().st_mode & 0o077 == 0
    finally:
        channel.close()
    assert not (tmp_path / 'backend.sock').exists()


def test_slow_and_oversized_peers_are_bounded(tmp_path):
    worker = SimpleNamespace(directory=tmp_path, stop=Mock(), status=Mock())
    channel = OwnerChannel(tmp_path, 'c' * 64, 's', worker)
    peer = socket.socket(socket.AF_UNIX)
    peer.settimeout(0.5)
    try:
        peer.connect(str(tmp_path / 'backend.sock'))
        channel.poll()
        peer.sendall(b'x' * 16385)
        channel.poll()
        assert peer.recv(1) == b''
        assert not channel.clients
        worker.stop.assert_not_called()
    finally:
        peer.close()
        channel.close()


def test_invalid_credential_never_creates_socket(tmp_path):
    with pytest.raises(ValueError):
        OwnerChannel(tmp_path, '', 's', None)
    assert not (tmp_path / 'backend.sock').exists()


def test_owned_preflight_failure_writes_terminal_receipt_without_backend(tmp_path):
    import subprocess
    import sys
    from test_backend import profile
    run = tmp_path / 'run'
    run.mkdir(mode=0o700)
    control = tmp_path / 'control'
    control.mkdir(mode=0o700)
    value = profile()
    value['mode'] = 'communication'
    value['fcu']['transport'] = 'serial'
    value['fcu']['serial_device'] = '/dev/easyfly-absent-recovery-test'
    result = subprocess.run(
        [sys.executable, '-m', 'easyfly_connection_manager.backend', '--directory', str(run),
         '--control-directory', str(control)], input=json.dumps(value) + '\n', text=True,
        env=dict(os.environ, EASYFLY_BACKEND_OWNER_TOKEN='d' * 64,
                 EASYFLY_CONNECTION_SESSION='preflight-fixture'), capture_output=True, timeout=15)
    assert result.returncode == 2
    receipt = json.loads((run / 'terminal.json').read_text())
    assert receipt['token'] == 'd' * 64 and receipt['session'] == 'preflight-fixture'
    assert receipt['status']['exit_code'] == 2 and not receipt['status']['process_running']
    assert not (run / 'backend.log').exists()

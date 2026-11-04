# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_semantic_map/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project

"""Negative-path debugger tests using a dedicated fixture, not production crash injection."""

import importlib.util
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from unittest.mock import Mock

import pytest


@pytest.fixture(scope='module')
def helper():
    """Load the source helper without starting ROS or any existing process."""
    source = Path(__file__).parents[1] / 'scripts/semantic_shutdown_capture.py'
    spec = importlib.util.spec_from_file_location('semantic_capture_negative', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def executable(tmp_path_factory):
    """Compile an isolated short-lived executable with source-level symbols."""
    output = tmp_path_factory.mktemp('capture-fixture') / 'semantic_map_node'
    subprocess.run(['c++', '-g', '-O0', str(Path(__file__).with_name('capture_fixture.cpp')),
                    '-o', str(output)], check=True, timeout=20)
    return output.resolve(strict=True)


@pytest.mark.parametrize('mode,expected', [('0', 0), ('7', 7), ('terminate', 255)])
def test_capture_preserves_failure(tmp_path, helper, executable, mode, expected):
    """Verify real GDB status and fatal stack without claiming original SIGSEGV reproduction."""
    result = subprocess.run([sys.executable, helper.__file__, str(executable), mode],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, timeout=15)
    (tmp_path / 'capture.log').write_text(result.stdout)
    assert result.returncode == expected, result.stdout
    assert 'forced cleanup' not in result.stdout
    if mode == 'terminate':
        assert 'received signal SIGABRT' in result.stdout
        assert 'fixture_terminate' in result.stdout
        assert 'capture_fixture.cpp:' in result.stdout
    elif mode == '0':
        assert 'exited normally' in result.stdout
    # GDB must have reaped even a stopped inferior after printing its stack.
    pids = re.findall(r'\(process (\d+)\)', result.stdout)
    if mode == 'terminate':
        # Fatal stops need not include GDB's normal-exit PID line.
        pids += re.findall(r'semantic capture inferior: pid=(\d+)', result.stdout)
        pids += re.findall(r'\(LWP (\d+)\)', result.stdout)
    assert pids, result.stdout
    for pid in pids:
        assert helper.identity(int(pid), executable) is None


def test_capture_shutdown_timeout(tmp_path, helper, executable):
    """Keep a dedicated fixture alive to verify bounded cleanup is reported as failure."""
    log_path = tmp_path / 'capture.log'
    with log_path.open('w') as output:
        process = subprocess.Popen(
            [sys.executable, helper.__file__, str(executable), 'ignore-shutdown'],
            stdout=output, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 5
            while not all(marker in log_path.read_text() for marker in
                          ('semantic capture inferior:', 'fixture shutdown handlers ready')):
                assert process.poll() is None, log_path.read_text()
                assert time.monotonic() < deadline, log_path.read_text()
                time.sleep(.02)
            process.send_signal(signal.SIGINT)
            assert process.wait(timeout=25) != 0
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=25)
    log = log_path.read_text()
    assert 'semantic capture forwarded:' in log
    assert 'semantic capture forced cleanup: FAIL' in log
    assert 'did not finish after shutdown' in log
    pids = re.findall(r'semantic capture inferior: pid=(\d+)', log)
    assert pids, log
    for pid in pids:
        assert helper.identity(int(pid), executable) is None


@pytest.mark.parametrize('failure', [ProcessLookupError, RuntimeError])
def test_cleanup_reaps_after_inferior_error(monkeypatch, helper, failure):
    """Guard the cleanup error branch using mocks, never kill an unrelated process."""
    debugger = Mock(pid=123)
    debugger.poll.return_value = None
    owned = (456, '789')
    monkeypatch.setattr(helper, 'identity', Mock(return_value=owned))
    sender = Mock(side_effect=failure('cleanup test'))
    monkeypatch.setattr(helper, 'signal_owned', sender)
    kill_group = Mock()
    monkeypatch.setattr(helper.os, 'killpg', kill_group)
    if failure is RuntimeError:
        with pytest.raises(RuntimeError, match='cleanup test'):
            helper.cleanup_debugger(debugger, owned, Path('/fixture'))
    else:
        helper.cleanup_debugger(debugger, owned, Path('/fixture'))
    sender.assert_called_once_with(owned, Path('/fixture'), signal.SIGKILL)
    kill_group.assert_called_once_with(123, signal.SIGKILL)
    debugger.wait.assert_called_once_with(timeout=3)

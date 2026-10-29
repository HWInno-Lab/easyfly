# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Real native process shutdown, including an interrupted critical section."""
import os
import signal
import subprocess
import time

import pytest


@pytest.mark.parametrize('sim_time', [False, True])
@pytest.mark.parametrize('mode', ['critical', 'SIGINT', 'SIGTERM'])
def test_native_signal_shutdown(tmp_path, sim_time, mode):
    events = tmp_path / 'events.log'
    environment = dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='231',
                       LD_PRELOAD=os.environ['FASTLIO_TEST_LIBRARY'],
                       EASYFLY_SIGNAL_PROBE_LOG=str(events))
    environment.pop('EASYFLY_SIGNAL_PROBE_CRITICAL', None)
    if mode == 'critical':
        environment['EASYFLY_SIGNAL_PROBE_CRITICAL'] = '1'
    command = [os.environ['FASTLIO_TEST_EXECUTABLE'], '--ros-args', '--params-file',
               os.environ['FASTLIO_TEST_CONFIG'], '-p',
               'use_sim_time:=' + str(sim_time).lower()]
    with (tmp_path / 'console.log').open('w+') as console:
        process = subprocess.Popen(command, env=environment, stdout=console,
                                   stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                if events.exists() and 'ready ' in events.read_text():
                    break
                assert process.poll() is None, 'native process exited before probe readiness'
                time.sleep(.01)
            else:
                pytest.fail('native process never became ready')
            if mode != 'critical':
                process.send_signal(getattr(signal, mode))
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pytest.fail('native shutdown deadlocked: ' + events.read_text())
            console.seek(0)
            assert process.returncode == 0, console.read()
            lines = events.read_text().splitlines()
            assert any(line.startswith('callback_acquired ') for line in lines)
            if mode == 'critical':
                assert any(line.startswith('signal_returned ') for line in lines)
                tids = {line.split()[0]: line.split('tid=')[1] for line in lines}
                assert tids['critical_signal'] != tids['callback_enter']
        finally:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=3)

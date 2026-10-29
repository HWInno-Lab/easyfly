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
"""Real launch shutdown cannot race a background component loader."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.parametrize('phase,code,calls', [
    ('waiting', 0, []), ('response', 0, ['first']),
    ('loaded', 0, ['first', 'second']), ('rejected', 1, ['first']),
    ('scoped', 0, ['first', 'second'])])
def test_component_loading_shutdown_and_errors(tmp_path, phase, code, calls):
    report = tmp_path / 'result.json'
    run = subprocess.run(
        [sys.executable, str(Path(__file__).with_name('composition_fixture.py')),
         phase, str(report)],
        env=dict(os.environ, ROS_DOMAIN_ID='197', ROS_LOCALHOST_ONLY='1',
                 ROS_LOG_DIR=str(tmp_path / 'ros')),
        capture_output=True, text=True, timeout=15)
    log = run.stdout + run.stderr
    (tmp_path / 'console.log').write_text(log)
    assert run.returncode == code, log
    result = json.loads(report.read_text())
    assert result['calls'] == calls, log
    assert not result['thread_alive']
    assert result['adapter_guard_released']
    assert result['adapter_context_closed']
    assert 'InvalidHandle' not in log and 'destruction was requested' not in log
    if phase == 'rejected':
        assert 'fixture rejection' in log
    if phase == 'scoped':
        assert result['requests'] == [dict(name='scoped_component', namespace='/scoped',
                                           remaps=['/input:=/scoped/output'], gains=[2.5])] * 2

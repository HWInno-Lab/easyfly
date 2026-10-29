# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Replay the original map-change snapshot through the bounded native worker."""
import gzip
import hashlib
import os
from pathlib import Path
import subprocess
import pytest


@pytest.mark.parametrize('fixture', ['map-change', 'map-change-return', 'map-change-braking'])
def test_original_map_change_snapshot(tmp_path, fixture):
    data = gzip.decompress((Path(__file__).parent/f'fixtures/{fixture}.request.gz').read_bytes())
    if fixture == 'map-change':
        assert hashlib.sha256(data).hexdigest() == \
            '2d4693f594aa1283b506abd7ba1a9b39f8d857442b2bd6a833ff7eab8db9bbac'
    plans = []
    for repeat in range(3):
        request = tmp_path/f'job-{repeat}.request'
        plan = tmp_path/f'job-{repeat}.plan'
        request.write_bytes(data)
        result = subprocess.run([os.environ['SUPER_WORKER'], str(request), str(plan)],
                                capture_output=True, text=True, timeout=10)
        # optimize() performs continuous polynomial/occupied-cell certification
        # and dynamic-limit checks before returning; native success alone fails.
        assert result.returncode == 0, result.stdout+result.stderr
        assert 'SUPER accepted:' in result.stdout
        plans.append(plan.read_bytes())
    assert plans[0] == plans[1] == plans[2]


def test_blocked_snapshot_still_rejected(tmp_path):
    data = gzip.decompress((Path(__file__).parent/'fixtures/map-change.request.gz').read_bytes())
    lines = data.decode().splitlines()
    # Preserve dimensions, measured state and limits, but occupy every cell.
    lines[9:] = [' '.join(['100']*(60*40*18))]
    request, plan = tmp_path/'blocked.request', tmp_path/'blocked.plan'
    request.write_text('\n'.join(lines)+'\n')
    result = subprocess.run([os.environ['SUPER_WORKER'], str(request), str(plan)],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode != 0
    assert not plan.exists()


@pytest.mark.parametrize('fixture,expected_sha', [
    ('high-speed-over', '87d6d83a44d8e8bfe6679ec8aab9788e1990cea91750438dd7ca299f978abe13'),
    ('high-speed-channel', '6601be65bbd621d355f0a6eb9b1856ef4bd00e832b1b9453de048b4f2cc53c85')])
def test_original_higher_speed_request_reserves_tracking_dynamics(tmp_path, fixture, expected_sha):
    data = gzip.decompress((Path(__file__).parent/f'fixtures/{fixture}.request.gz').read_bytes())
    assert hashlib.sha256(data).hexdigest() == expected_sha
    request, plan = tmp_path/'job.request', tmp_path/'job.plan'
    request.write_bytes(data)
    result = subprocess.run([os.environ['SUPER_WORKER'], str(request), str(plan)],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout+result.stderr
    (tmp_path/'worker.log').write_text(result.stdout+result.stderr)
    header = plan.read_text().splitlines()[2].split()
    assert float(header[-2]) <= .15, 'native acceleration exceeds control tracking budget'
    assert float(header[-1]) <= .3, 'native jerk exceeds control tracking budget'


def test_native_backup_executes_original_coefficients_before_unknown(tmp_path):
    import json
    cells = []
    for z in range(18):
        for y in range(40):
            for x in range(60):
                cx, cy, cz = -3+(x+.5)*.2, -4+(y+.5)*.2, -.2+(z+.5)*.2
                cells.append(100 if cz < .6 or cz > 2.4 or abs(cy) > 3. or
                             cx < -2. or cx > 8. else (-1 if cx > 3. else 0))
    request, plan = tmp_path/'job.request', tmp_path/'job.plan'
    request.write_text('EASYFLY_SUPER_REQUEST_2\n60 40 18\n-3 -4 -0.2\n0.2\n'
                       '0 0 1.5\n0 0 0\n7 0 1.5\n0\n0.6 2 8\n1\n' +
                       ' '.join(map(str, cells))+'\n')
    result = subprocess.run([os.environ['SUPER_WORKER'], str(request), str(plan)],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout+result.stderr
    lines = plan.read_text().splitlines()
    assert lines[0] == 'EASYFLY_SEPTIC_2'
    duration, _, acceleration, jerk = map(float, lines[2].split())
    backup, partial = map(float, lines[3].split())
    assert 0 < backup < duration and partial == 1 and acceleration <= .15 and jerk <= .3
    raw = json.loads((tmp_path/'job.request.native/native-trajectories.json').read_text())
    assert int(lines[1]) == len(raw['committed'])
    for index, piece in enumerate(raw['committed']):
        assert float(lines[4+2*index]) == piece['duration']
        assert list(map(float, lines[5+2*index].split())) == piece['coefficients']
    last = raw['committed'][-1]
    terminal_x = sum(v * last['duration']**(7-i) for i, v in
                     enumerate(last['coefficients'][:8]))
    assert .25 < terminal_x < 3.0, 'native brake must end in observed free cells'
    # Exactly the same unknown goal remains forbidden in the default profile.
    legacy = tmp_path/'legacy.request'
    text = request.read_text().splitlines()
    text[0] = 'EASYFLY_SUPER_REQUEST_1'
    del text[9]
    legacy.write_text('\n'.join(text)+'\n')
    rejected = subprocess.run([os.environ['SUPER_WORKER'], str(legacy), str(tmp_path/'bad.plan')],
                              capture_output=True, text=True, timeout=10)
    assert rejected.returncode != 0 and not (tmp_path/'bad.plan').exists()


def test_original_short_stop_failure_retries_native_solver_within_budget(tmp_path):
    data = gzip.decompress((Path(__file__).parent/
                            'fixtures/committed-short-stop.request.gz').read_bytes())
    assert hashlib.sha256(data).hexdigest() == \
        'cb206e9c861df7473ccdcd44eb17adcd3e82514e7dc7e8d52b0bd98f8a5b4f44'
    request, plan = tmp_path/'job.request', tmp_path/'job.plan'
    request.write_bytes(data)
    result = subprocess.run([os.environ['SUPER_WORKER'], str(request), str(plan)],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout+result.stderr
    values = list(map(float, plan.read_text().splitlines()[2].split()))
    assert values[-2] <= .15 and values[-1] <= .3
    assert 'SUPER native retry:' in result.stdout

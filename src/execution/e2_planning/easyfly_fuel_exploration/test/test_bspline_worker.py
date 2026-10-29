# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Installed native FUEL solver against recorded inputs and independent de Boor evaluation."""
import gzip
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import numpy as np
import pytest
from scipy.interpolate import BSpline
import yaml


def run(tmp_path, case='initial', minimum=None, mutate=None):
    data = gzip.decompress((Path(__file__).parent/'data'/(case+'.request.gz')).read_bytes()).decode()
    if mutate:
        rows = data.splitlines();mutate(rows);data = '\n'.join(rows)+'\n'
    request, output = tmp_path/'input.request', tmp_path/'output.polynomial'
    request.write_text(data)
    worker = Path(get_package_prefix('easyfly_fuel_exploration'))/'lib/easyfly_fuel_exploration/fuel_bspline_worker'
    command = [str(worker), str(request), str(output)]
    if minimum is not None:
        command.append(str(minimum))
    began = time.monotonic()
    result = subprocess.run(command, capture_output=True, text=True, timeout=4)
    assert time.monotonic()-began < 3.
    (tmp_path/'worker.log').write_text(result.stdout+result.stderr)
    return result, output, data.splitlines()


@pytest.mark.parametrize('case', ['initial', 'dynamic-detour', 'reverse', 'long-return', 'long-return-timing'])
def test_recorded_native_trajectory_matches_independent_bspline(tmp_path, case):
    result, output, request = run(tmp_path, case)
    assert result.returncode == 0, result.stderr
    evidence = yaml.safe_load(Path(str(output)+'.native.yaml').read_text())
    spline = BSpline(evidence['knots'], evidence['controls'], 3, extrapolate=False)
    rows = output.read_text().splitlines()
    pieces = int(rows[1]);span = evidence['dt'];start = evidence['knots'][3]
    observed = []
    for i in range(pieces):
        duration = float(rows[3+2*i]);assert abs(duration-span)<1e-8
        coefficients = np.fromstring(rows[4+2*i], sep=' ').reshape(3, 6)
        times = np.linspace(0, duration, 129)
        for derivative in range(3):
            polynomial = np.array([np.polyval(np.polyder(row, derivative), times) for row in coefficients]).T
            absolute = start+i*span+times
            assert absolute[-1] <= evidence['knots'][-4]+1e-10
            reference = spline(np.minimum(absolute, evidence['knots'][-4]), nu=derivative)
            assert np.max(np.linalg.norm(polynomial-reference, axis=1))<1e-7
        observed.extend(spline(np.minimum(start+i*span+times, evidence['knots'][-4])).tolist())
    assert np.linalg.norm(spline(start)-np.fromstring(request[4], sep=' '))<1e-7
    assert np.linalg.norm(spline(start, nu=1)-np.fromstring(request[5], sep=' '))<1e-7
    assert np.linalg.norm(spline(evidence['knots'][-4])-np.fromstring(request[6], sep=' '))<1e-7
    assert np.linalg.norm(spline(evidence['knots'][-4], nu=1))<1e-7
    metrics = np.fromstring(rows[2], sep=' ')
    if case.startswith('long-return'):assert metrics[1]>=.8
    assert metrics[1] <= float(request[7].split()[0]) and metrics[2] <= .15 and metrics[3] <= .3
    size = np.fromstring(request[1], sep=' ', dtype=int)
    origin = np.fromstring(request[2], sep=' ');resolution=float(request[3])
    cells = np.fromstring(request[8], sep=' ', dtype=int).reshape(tuple(size[::-1]))
    indices = np.floor((np.array(observed)-origin)/resolution).astype(int)
    assert ((indices>=0)&(indices<size)).all()
    assert np.all(cells[indices[:, 2], indices[:, 1], indices[:, 0]]==0)


def test_required_yaw_duration_preserves_measured_initial_velocity(tmp_path):
    result, output, request = run(tmp_path, 'reverse', minimum=50.)
    assert result.returncode == 0, result.stderr
    evidence = yaml.safe_load(Path(str(output)+'.native.yaml').read_text())
    spline = BSpline(evidence['knots'], evidence['controls'], 3, extrapolate=False)
    start, end = evidence['knots'][3], evidence['knots'][-4]
    assert end-start>=50.-1e-7
    assert np.linalg.norm(spline(start, nu=1)-np.fromstring(request[5], sep=' '))<1e-7


@pytest.mark.parametrize('fault', ['moving_start', 'unknown_goal', 'bad_minimum'])
def test_invalid_native_requests_never_produce_executable_output(tmp_path, fault):
    def mutate(rows):
        if fault=='moving_start':
            rows[5]='.31 0 0'
        elif fault=='unknown_goal':
            size=np.fromstring(rows[1],sep=' ',dtype=int);origin=np.fromstring(rows[2],sep=' ')
            goal=np.fromstring(rows[6],sep=' ');index=np.floor((goal-origin)/float(rows[3])).astype(int)
            cells=rows[8].split();cells[index[0]+size[0]*(index[1]+size[1]*index[2])]='-1';rows[8]=' '.join(cells)
    result, output, _ = run(tmp_path, minimum='nan' if fault=='bad_minimum' else None, mutate=mutate)
    assert result.returncode!=0 and not output.exists()

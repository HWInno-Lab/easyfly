#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Run sequential isolated cold starts, bounded contention and a guard pause.

This diagnoses static simulation input admission; it never enables navigation.
It does not inject packet loss or modify host networking/scheduler settings.
"""

import argparse
import json
from pathlib import Path
import subprocess
import time


HERE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--native-executable', type=Path, required=True)
    parser.add_argument('--gazebo-camera-executable', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    native = args.native_executable.resolve(strict=True)
    camera = args.gazebo_camera_executable.resolve(strict=True)
    profile = HERE / 'fastdds_imu_profile/guard_input_recovery.xml'
    cases = [('ordinary-1', False, False), ('candidate-1', True, False),
             ('pressure-candidate-1', True, True), ('pressure-ordinary-1', False, True),
             ('pressure-ordinary-2', False, True), ('pressure-candidate-2', True, True),
             ('pause-candidate-200ms', True, False)]
    report = dict(scope=__doc__, error=None, cases=[])
    for index, (name, candidate, pressure) in enumerate(cases):
        directory = out / name
        command = ['python3', str(HERE / 'run_imu_observer_comparison.py'),
                   '--native-executable', str(native), '--gazebo-camera-executable', str(camera),
                   '--output', str(directory), '--seconds', '65' if index == 6 else '120',
                   '--domain-id', str(220+index), '--master-port', str(11741+index),
                   '--native-camera-load', '--no-native-notifications', '--audit-guard']
        if candidate:
            command += ['--profile', str(profile)]
        if pressure:
            command += ['--cpu-limit', '4', '--cpu-workers', '2']
        if index == 6:
            command += ['--guard-pause-after', '35', '--guard-pause-ms', '200']
        print('START ' + name, flush=True)
        began = time.monotonic()
        with (out / (name + '.log')).open('w') as log:
            process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
        result = dict(name=name, command=command, returncode=process.returncode,
                      elapsed_s=time.monotonic()-began)
        report['cases'].append(result)
        (out / 'matrix.json').write_text(json.dumps(report, indent=2) + '\n')
        print('END ' + name + ' exit=' + str(process.returncode), flush=True)
        if process.returncode:
            report['error'] = 'case infrastructure failed: ' + name
            break
    (out / 'matrix.json').write_text(json.dumps(report, indent=2) + '\n')
    return 1 if report['error'] else 0


if __name__ == '__main__':
    raise SystemExit(main())

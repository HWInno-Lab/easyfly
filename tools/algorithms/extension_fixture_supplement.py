#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Build/test EXT-00 UI consumers and static PX4 launch/model fixtures.

Uses only the new baseline prefix plus system ROS. Does not build/start PX4,
Gazebo, a connection manager, or a real flight backend. Run build after the main
baseline build and before its tests; run test after the main serial tests.
"""
import argparse
import json
from pathlib import Path
import subprocess

from extension_baseline import active_domains, run, write, sha

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', required=True, type=Path)
    parser.add_argument('--phase', required=True, choices=['build', 'test'])
    args = parser.parse_args()
    baseline = args.baseline.resolve()
    summary = json.loads((baseline / 'summary.json').read_text())
    if not summary.get('build_passed'):
        parser.error('completed isolated baseline build required')
    out = baseline / 'supplement'
    out.mkdir(exist_ok=True)
    record = {'scope': 'UI control software consumer and static PX4 assets; no SITL',
              'commands': [], 'runner_sha256': sha(Path(__file__))}
    state = out / 'summary.json'
    if state.exists():
        record = json.loads(state.read_text())
        if record['runner_sha256'] != sha(Path(__file__)):
            parser.error('runner changed; preserve prior evidence')
    if args.phase == 'build':
        # Install into the same NEW baseline prefix so its unmodified test
        # runner can discover static model assets. Build trees stay separate.
        command = ['bash', '--noprofile', '--norc', '-c',
                   'source "$1/local_setup.bash" && shift && exec "$@"',
                   'bash', str(baseline / 'install'), 'colcon', '--log-base',
                   str(out / 'colcon-log'), 'build', '--base-paths',
                   str(ROOT / 'src/easyfly_ui/easyfly_ui_control'),
                   str(ROOT / 'src/simulation/easyfly_px4_sitl'), '--build-base',
                   str(out / 'build'), '--install-base', str(baseline / 'install'),
                   '--packages-select', 'easyfly_ui_control', 'easyfly_px4_sitl',
                   '--executor', 'sequential', '--symlink-install', '--cmake-args',
                   '-DBUILD_TESTING=ON', '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON']
        code = run(out, record, 'build', command)
        record['build_passed'] = code == 0
    else:
        if not record.get('build_passed'):
            parser.error('supplement build required')
        if {'218', '132', '128', '123', '113'} & active_domains().keys():
            parser.error('supplement test DDS domain is in use')
        junit = out / 'easyfly_ui_control.xml'
        command = ['bash', '--noprofile', '--norc', '-c',
                   'source "$1/local_setup.bash" && shift && exec "$@"',
                   'bash', str(baseline / 'install'), 'ctest', '--test-dir',
                   str(out / 'build/easyfly_ui_control'), '-R', '^test_',
                   '--output-on-failure', '-j1', '--timeout', '180', '--output-junit', str(junit)]
        code = run(out, record, 'test-ui-control', command)
        record['ui_control_tests_passed'] = code == 0 and junit.exists()
    write(state, record)
    return code


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""EXT-00 source inventory and isolated software baseline; never launches SITL.

Run capture, build and test against a NEW output directory, in that order.
The explicit package set is a build/software-fixture set, not a full runtime
deployment closure. System ROS is the only underlay. Existing installs are not
sourced. Tests run serially; active DDS domains are rejected before execution.
"""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import subprocess
import time
import xml.etree.ElementTree as ET

from verify_api_install import clean_environment

ROOT = Path(__file__).resolve().parents[2]
PACKAGES = """easyfly_msgs easyfly_launch easyfly_mission_format easyfly_core
easyfly_pybind easyfly_params easyfly_perception_srvs easyfly_mission_srvs
easyfly_tracking_api easyfly_sort easyfly_track_id_fusion easyfly_multi_object_tracking
easyfly_grid_planner_api easyfly_astar easyfly_coverage_planning easyfly_global_planner
easyfly_mission_fsm easyfly_projection easyfly_pixel_to_3d easyfly_motion_prediction
easyfly_target_prediction easyfly_detection_api easyfly_single_tracking_api
easyfly_yolo_onnx easyfly_tag_pose easyfly_csrt easyfly_object_detection
easyfly_single_object_tracking easyfly_ui_bridge easyfly_precision_landing
easyfly_flight_control easyfly_mavlink_bridge easyfly_local_planner quadrotor_msgs traj_utils traj_utils_so
plan_env path_searching bspline_opt ego_planner""".split()
TEST_PACKAGES = [p for p in PACKAGES if p not in {
    'easyfly_msgs', 'easyfly_single_tracking_api', 'easyfly_local_planner',
    'quadrotor_msgs', 'traj_utils', 'traj_utils_so', 'plan_env', 'path_searching',
    'bspline_opt', 'easyfly_mavlink_bridge'}]
TYPES = ['DetectionInfo', 'DetectionInfoArray', 'TrackInfo', 'TrackInfoArray',
         'Target3D', 'Target3DArray', 'TargetPrediction', 'TargetPredictionArray',
         'LocalizationStatus', 'NavigationSession', 'PositionCommand',
         'PolynomialTrajectory', 'SepticTrajectory']


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def active_domains():
    result = {}
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            entries = proc.joinpath('environ').read_bytes().split(b'\0')
            domain = next((v.split(b'=', 1)[1].decode() for v in entries
                           if v.startswith(b'ROS_DOMAIN_ID=')), None)
            if domain is not None:
                result.setdefault(domain, []).append({
                    'pid': int(proc.name), 'name': proc.joinpath('comm').read_text().strip()})
        except (OSError, UnicodeError):
            pass
    return result


def capture(out):
    out.mkdir(parents=True, exist_ok=False)
    listing = subprocess.check_output(['colcon', 'list', '--base-paths', 'src'],
                                     cwd=ROOT, text=True)
    (out / 'packages.tsv').write_text(listing)
    packages = {}
    owners = []
    for row in listing.splitlines():
        name, path, kind = row.split('\t')
        base = ROOT / path
        deps = []
        if (base / 'package.xml').is_file():
            xml = ET.parse(base / 'package.xml').getroot()
            deps = [{'kind': node.tag, 'name': node.text}
                    for node in xml if node.tag.endswith('depend')]
        packages[name] = {'path': path, 'kind': kind, 'dependencies': deps}
        owners.append((base, name))
    owners.sort(key=lambda item: len(item[0].parts), reverse=True)
    write(out / 'package-dependencies.json', packages)
    references = {name: [] for name in TYPES}
    calls, configs = [], {}
    tokens = {name: re.compile(r'\b' + re.escape(name) + r'\b') for name in TYPES}
    for prefix in ('src', 'tools', 'scripts'):
        for path in sorted((ROOT / prefix).rglob('*')):
            if not path.is_file() or any(v in path.parts for v in (
                    '.git', '__pycache__', '.pytest_cache')):
                continue
            if path.suffix not in {'.cpp', '.hpp', '.h', '.cc', '.py', '.msg',
                                   '.srv', '.action', '.yaml', '.json', '.cmake'} and (
                    path.name != 'CMakeLists.txt'):
                continue
            try:
                source = path.read_text()
            except UnicodeError:
                continue
            relative = str(path.relative_to(ROOT))
            owner = next((name for base, name in owners if path.is_relative_to(base)), None)
            is_test = any(p in {'test', 'tests', 'probes'} for p in path.parts)
            if path.suffix in {'.yaml', '.json'} and prefix == 'src' and (
                    'third_party' not in path.parts):
                configs[relative] = sha(path)
            for number, line in enumerate(source.splitlines(), 1):
                record = {'file': relative, 'line': number, 'package': owner,
                          'test_or_probe': is_test, 'text': line.strip()}
                for name, pattern in tokens.items():
                    if pattern.search(line):
                        references[name].append(record)
                if re.search(r'\b(make_tracker|make_planner|createBackend)\s*\(', line):
                    calls.append(record)
    write(out / 'message-references.json', references)
    write(out / 'factory-calls.json', calls)
    write(out / 'configuration-sha256.json', configs)
    tracked = subprocess.check_output(['git', 'ls-files', '-z'], cwd=ROOT).decode().split('\0')
    fingerprints = {name: sha(ROOT / name) for name in tracked
                    if name and (ROOT / name).is_file()}
    fingerprints[str(Path(__file__).relative_to(ROOT))] = sha(Path(__file__))
    write(out / 'source-sha256.json', fingerprints)
    (out / 'runner-at-capture.py').write_bytes(Path(__file__).read_bytes())
    for name in ('catalog.json', 'sourcing-and-patches.md'):
        source = ROOT / 'src/third_party' / name
        (out / ('third-party-' + name)).write_bytes(source.read_bytes())
    for name in ('未完成.md', '扩展性.md'):
        (out / name).write_bytes((ROOT / name).read_bytes())
    sdk = ROOT / 'src/third_party/runtimes/onnxruntime'
    assets = {}
    for base in (ROOT / 'models', sdk):
        if base.exists():
            for p in sorted(base.rglob('*')):
                if p.is_file():
                    assets[str(p.relative_to(ROOT))] = {
                        'sha256': sha(p), 'size': p.stat().st_size,
                        'symlink_target': os.readlink(p) if p.is_symlink() else None}
    write(out / 'model-runtime-sha256.json', assets)
    (out / 'working-tree.patch').write_bytes(subprocess.check_output(
        ['git', 'diff', 'HEAD', '--binary'], cwd=ROOT))
    env = {'captured_utc': now(), 'platform': platform.platform(),
           'cpu_count': os.cpu_count(), 'active_dds_domains': active_domains(),
           'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'],
                                               cwd=ROOT, text=True).strip(),
           'git_status': subprocess.check_output(['git', 'status', '--porcelain=v1'],
                                                 cwd=ROOT, text=True)}
    for name, cmd in {'compiler': ['g++', '--version'], 'cmake': ['cmake', '--version'],
                      'opencv': ['pkg-config', '--modversion', 'opencv4'],
                      'system_packages': ['dpkg-query', '-W', '-f=${Package}\t${Version}\n',
                                          'ros-humble-*', 'libopencv*', 'libeigen3-dev',
                                          'libpcl*', 'libceres*', 'libyaml-cpp*']}.items():
        result = subprocess.run(cmd, capture_output=True, text=True, check=False)
        env[name] = {'command': cmd, 'returncode': result.returncode,
                     'stdout': result.stdout, 'stderr': result.stderr}
    write(out / 'environment.json', env)
    module_status = subprocess.run(['git', 'submodule', 'status', '--recursive'],
                                   cwd=ROOT, capture_output=True, text=True, check=False)
    write(out / 'submodule-status.json', {'returncode': module_status.returncode,
          'stdout': module_status.stdout, 'stderr': module_status.stderr})
    write(out / 'summary.json', {'schema_version': 1, 'started_utc': now(),
          'scope': 'Explicit software-fixture packages; no SITL/hardware/full deployment',
          'packages': PACKAGES, 'test_packages': TEST_PACKAGES,
          'source_manifest': 'source-sha256.json', 'commands': [], 'tests': []})
    print(f'Captured {len(packages)} packages; inventory: {out}', flush=True)


def run(out, record, label, command, overlay=False, ros=True, timeout=2400):
    env = clean_environment()
    env.update(ROS_DOMAIN_ID='218', ROS_LOCALHOST_ONLY='1', PYTHONNOUSERSITE='1',
               QT_QPA_PLATFORM='offscreen', ROS_LOG_DIR=str(out / 'ros-logs'))
    script = 'source /opt/ros/humble/setup.bash && '
    if overlay:
        script += 'source "$1/local_setup.bash" && shift && '
    actual = ['bash', '--noprofile', '--norc', '-c', script + 'exec "$@"', 'bash']
    if overlay:
        actual += [str(out / 'install')]
    actual += [str(v) for v in command]
    if not ros:
        actual = [str(v) for v in command]
    log = out / (label + '.log')
    if log.exists():
        raise ValueError(f'preserve previous evidence: {log}')
    print(label, flush=True)
    started = time.monotonic()
    with log.open('w') as stream:
        try:
            result = subprocess.run(actual, cwd=ROOT, env=env,
                                    stdout=stream, stderr=subprocess.STDOUT, timeout=timeout)
            code = result.returncode
        except subprocess.TimeoutExpired:
            code = 124
    record['commands'].append({'name': label, 'command': actual, 'returncode': code,
                              'seconds': round(time.monotonic() - started, 3),
                              'log': log.name, 'log_sha256': sha(log)})
    write(out / 'summary.json', record)
    return code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--phase', choices=['capture', 'build', 'test'], required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    if args.phase == 'capture':
        capture(out)
        return 0
    record = json.loads((out / 'summary.json').read_text())
    original = json.loads((out / 'source-sha256.json').read_text())
    changed = [p for p, value in original.items()
               if not (ROOT / p).is_file() or sha(ROOT / p) != value]
    if changed:
        raise ValueError(f'Baseline sources changed; use a new capture: {changed}')
    if args.phase == 'build':
        packages = json.loads((out / 'package-dependencies.json').read_text())
        # Discovery itself must be restricted: scanning all src makes colcon
        # require environment hooks from excluded transitive exec dependencies,
        # even though these software fixtures do not start those components.
        bases = [ROOT / packages[name]['path'] for name in PACKAGES]
        code = run(out, record, 'build', ['colcon', '--log-base', out / 'colcon-log',
            'build', '--base-paths', *bases, '--build-base', out / 'build',
            '--install-base', out / 'install', '--packages-select', *PACKAGES,
            '--executor', 'sequential', '--symlink-install', '--cmake-args',
            '-DBUILD_TESTING=ON', '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'])
        record['build_passed'] = code == 0
        write(out / 'summary.json', record)
        return code
    if not record.get('build_passed'):
        raise ValueError('a completed build is required')
    active = active_domains()
    if '218' in active:
        raise ValueError('baseline DDS domain 218 is already in use')
    failures = 0
    for package in TEST_PACKAGES:
        pattern = r'(^test_|detection_contract$)'
        if package == 'easyfly_precision_landing':
            pattern = r'^test_(policy|geometry|configuration|observation_ros|stream_pair_ros|frame_timing_ros)$'
        code = run(out, record, 'discover-' + package, ['ctest', '--test-dir',
            out / 'build' / package, '--show-only=json-v1', '-R', pattern], overlay=True)
        if code:
            failures += 1
            continue
        discovery = json.loads((out / ('discover-' + package + '.log')).read_text())
        tests = discovery.get('tests', [])
        if not tests:
            record['tests'].append({'package': package, 'status': 'not_run_no_tests'})
            failures += 1
            continue
        conflicts = []
        for test in tests:
            for prop in test.get('properties', []):
                if prop['name'] == 'ENVIRONMENT':
                    for entry in prop['value']:
                        if entry.startswith('ROS_DOMAIN_ID=') and entry.split('=', 1)[1] in active:
                            conflicts.append(test['name'])
        if conflicts:
            record['tests'].append({'package': package, 'status': 'not_run_domain_conflict',
                                    'tests': conflicts})
            failures += 1
            continue
        junit = out / (package + '.xml')
        code = run(out, record, 'test-' + package, ['ctest', '--test-dir',
            out / 'build' / package, '--output-on-failure', '-j1', '--timeout', '180',
            '-R', pattern, '--output-junit', junit], overlay=True)
        cases = []
        if junit.exists():
            for case in ET.parse(junit).getroot().iter('testcase'):
                status = 'failed' if case.find('failure') is not None else (
                    'skipped' if case.find('skipped') is not None else 'passed')
                cases.append({'name': case.get('name'), 'status': status})
        record['tests'].append({'package': package, 'returncode': code,
                                'selected': len(tests), 'cases': cases,
                                'complete': len(cases) == len(tests)})
        failures += int(code != 0 or len(cases) != len(tests))
        write(out / 'summary.json', record)
    record['software_checks_passed'] = failures == 0
    record['finished_utc'] = now()
    write(out / 'summary.json', record)
    print('Software baseline completed; failed/incomplete groups:', failures, flush=True)
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())

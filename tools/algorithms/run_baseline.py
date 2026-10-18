#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""Run the registered ALG-0 software regressions serially in test DDS domains.

The allowlist is migration-inventory.json, not arbitrary CTest or SITL entries.
Source the intended ROS underlay and baseline overlay before invoking this file.
"""

import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
PATTERN = r'(^test_|.*contract$)'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-base', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path,
                        help='New directory for commands, logs, JUnit and summary')
    parser.add_argument('--domain', type=int, default=212)
    parser.add_argument('--packages', nargs='+')
    args = parser.parse_args()
    inventory = json.loads((ROOT / 'src/algorithms/migration-inventory.json').read_text())
    packages = args.packages or inventory['baseline_packages']
    if len(packages) != len(set(packages)):
        parser.error('duplicate package')
    if not set(packages).issubset(inventory['baseline_packages']):
        parser.error('package is outside the ALG-0 software-test allowlist')
    if args.domain not in range(1, 233):
        parser.error('use a nonzero DDS test domain in 1..232')
    if args.output.exists():
        parser.error(f'output already exists: {args.output}')

    build = args.build_base.resolve()
    output = args.output.resolve()
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain))
    discovery = {}
    # Check every selected package before starting any test.
    for package in packages:
        command = ['ctest', '--test-dir', str(build / package),
                   '--show-only=json-v1', '-R', PATTERN]
        proc = subprocess.run(command, env=env, text=True, capture_output=True, check=True)
        data = json.loads(proc.stdout)
        if not data['tests']:
            parser.error(f'no selected software tests for {package}')
        for test in data['tests']:
            for prop in test.get('properties', []):
                if prop['name'] == 'ENVIRONMENT':
                    for value in prop['value']:
                        if value.startswith('ROS_DOMAIN_ID=') and value.split('=', 1)[1] == '0':
                            parser.error(f'{package}/{test["name"]} overrides the test domain to 0')
        discovery[package] = data

    output.mkdir(parents=True)
    (output / 'discovery.json').write_text(json.dumps(discovery, indent=2) + '\n')
    summary = {
        'schema_version': 1,
        'source_commit': subprocess.check_output(
            ['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip(),
        'started_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'build_base': str(build), 'default_test_domain': args.domain,
        'selection_pattern': PATTERN,
        'environment': {key: env.get(key) for key in (
            'ROS_DISTRO', 'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY',
            'AMENT_PREFIX_PATH', 'LD_LIBRARY_PATH', 'PYTHONPATH')},
        'packages': [],
        'scope': 'Existing registered software tests; no Gazebo/PX4/SITL launch.',
    }
    for package in packages:
        junit = output / f'{package}.xml'
        command = ['ctest', '--test-dir', str(build / package), '--output-on-failure',
                   '-j1', '-R', PATTERN, '--output-junit', str(junit)]
        started = time.monotonic()
        print(f'Running {package}: {len(discovery[package]["tests"])} CTest entries', flush=True)
        with (output / f'{package}.log').open('w') as log:
            code = subprocess.call(command, env=env, stdout=log, stderr=subprocess.STDOUT)
        tests = []
        if junit.exists():
            for case in ET.parse(junit).getroot().iter('testcase'):
                failure = case.find('failure')
                skipped = case.find('skipped')
                tests.append({
                    'name': case.get('name'), 'time': case.get('time'),
                    'status': 'failed' if failure is not None else
                              'skipped' if skipped is not None else 'passed',
                    'failure': failure.get('message') if failure is not None else None,
                })
        record = {
            'package': package, 'command': command, 'returncode': code,
            'elapsed_seconds': round(time.monotonic() - started, 3),
            'selected_count': len(discovery[package]['tests']), 'tests': tests,
            'log_sha256': hashlib.sha256((output / f'{package}.log').read_bytes()).hexdigest(),
        }
        summary['packages'].append(record)
        (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
        print(f'{package}: exit={code}', flush=True)

    summary['finished_utc'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
    summary['ctest_entries'] = sum(len(p['tests']) for p in summary['packages'])
    summary['counts'] = {status: sum(t['status'] == status for p in summary['packages']
                                     for t in p['tests'])
                         for status in ('passed', 'failed', 'skipped')}
    complete = all(len(p['tests']) == p['selected_count'] for p in summary['packages'])
    summary['junit_complete'] = complete
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary['counts']), flush=True)
    return 0 if complete and all(p['returncode'] == 0 for p in summary['packages']) else 1


if __name__ == '__main__':
    raise SystemExit(main())

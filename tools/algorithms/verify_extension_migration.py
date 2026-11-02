#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Replay EXT-02/03 software evidence with explicit prefixes and isolated ROS domains."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET

from extension_baseline import active_domains
from replay_ctest_isolated import quote

ROOT = Path(__file__).resolve().parents[2]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    parser.add_argument('--underlay', type=Path, action='append', default=[])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--build-base', type=Path)
    parser.add_argument('--native-only', action='store_true')
    parser.add_argument('--native-test', type=Path, action='append', help='explicit pytest fixture file(s); defaults to EXT-02 native checks')
    parser.add_argument('--python-executable', type=Path, default=Path('/usr/bin/python3'))
    parser.add_argument('--test-regex', default='^test_', help='select CTest entries within selected package groups')
    parser.add_argument('--packages', nargs='+', help='rerun only these package groups')
    parser.add_argument('--exclude-test', action='append', default=[],
                        help='explicit package:test entry, recorded as excluded (not passed)')
    parser.add_argument('--skip-native', action='store_true')
    parser.add_argument('--validation-runtime', type=Path, help='isolated jsonschema dependency directory')
    parser.add_argument('--python-runtime', type=Path, help='append an explicit compatible ONNX Runtime site-packages after system NumPy/OpenCV')
    parser.add_argument('--runtime-library-dir', action='append', type=Path, default=[],
                        help='explicit private runtime library directory, preserved across isolated setup')
    parser.add_argument('--domain', type=int, default=167)
    args = parser.parse_args()
    if not 1 <= args.domain <= 230: parser.error('domain must be 1..230')
    args.output.mkdir(parents=True, exist_ok=False)
    prefixes = [p.resolve() for p in args.underlay] + [args.prefix.resolve()]
    for prefix in prefixes:
        if not (prefix / 'local_setup.bash').is_file(): parser.error('missing explicit setup: ' + str(prefix))
    # setup file paths are positional arguments, never shell-interpolated.
    shell = 'source /opt/ros/humble/setup.bash\n'
    for _ in prefixes: shell += 'source "$1/local_setup.bash"\nshift\n'
    shell += 'exec "$@"'
    clean = {k: v for k, v in os.environ.items() if not k.startswith(
        ('AMENT', 'COLCON', 'CMAKE_PREFIX', 'PYTHONPATH', 'LD_LIBRARY'))}
    clean['PATH'] = str(args.python_executable.absolute().parent) + ':' + clean['PATH']
    clean.update(PYTHONNOUSERSITE='1', ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1',
                 ROS_LOG_DIR=str(args.output.resolve() / 'ros-logs'), QT_QPA_PLATFORM='offscreen')
    if args.runtime_library_dir:
        for directory in args.runtime_library_dir:
            if not directory.is_dir(): parser.error('missing private runtime directory: '+str(directory))
        clean['LD_LIBRARY_PATH'] = ':'.join(str(p.resolve()) for p in args.runtime_library_dir)
    if args.python_runtime or args.validation_runtime:
        site = args.output.resolve() / 'runtime-site'; site.mkdir()
        lines = ['import sys']
        if args.validation_runtime:
            runtime = args.validation_runtime.resolve()
            if not (runtime / 'jsonschema/__init__.py').is_file(): parser.error('validation runtime has no jsonschema')
            lines.append('sys.path.insert(0, ' + repr(str(runtime)) + ')')
        if args.python_runtime:
            runtime = args.python_runtime.resolve()
            if not (runtime / 'onnxruntime/__init__.py').is_file(): parser.error('runtime has no ONNX Runtime module')
            lines.append('sys.path.append(' + repr(str(runtime)) + ')')
        (site / 'sitecustomize.py').write_text('\n'.join(lines)+'\n')
        clean['PYTHONPATH'] = str(site)
    occupied = active_domains()
    if str(args.domain) in occupied:
        parser.error('requested fixture domain is already occupied')
    commands = []
    def run(name, command, timeout=180):
        cmd = ['bash', '--noprofile', '--norc', '-c', shell, 'bash', *map(str, prefixes), *map(str, command)]
        started = time.monotonic()
        with (args.output / (name + '.log')).open('w') as log:
            try: rc = subprocess.run(cmd, cwd=ROOT, env=clean, stdout=log, stderr=subprocess.STDOUT, timeout=timeout).returncode
            except subprocess.TimeoutExpired: rc = 124
        commands.append(dict(name=name, command=cmd, returncode=rc, seconds=time.monotonic()-started))
        (args.output / 'commands.json').write_text(json.dumps(commands, indent=2))
        print(name, rc, flush=True)
    if not args.skip_native: run('native', [str(args.python_executable.absolute()), '-m', 'pytest', *(args.native_test or [ROOT/'tools/algorithms/test_ext02_native.py']), '-q',
        '--junitxml=' + str(args.output.resolve()/'native.xml'),
        '--basetemp=' + str(args.output.resolve()/'native-fixtures')], 180)
    if args.build_base and not args.native_only:
        for directory in sorted(args.build_base.iterdir()):
            if args.packages and directory.name not in args.packages: continue
            cmake = directory / 'CTestTestfile.cmake'
            if not cmake.is_file(): continue
            discovered = json.loads(subprocess.check_output(
                ['ctest', '--test-dir', str(directory), '--show-only=json-v1'], text=True))
            selected = [t for t in discovered['tests'] if re.search(args.test_regex, t['name'])
                        and directory.name + ':' + t['name'] not in args.exclude_test]
            if not selected: continue
            (args.output / ('discover-' + directory.name + '.json')).write_text(json.dumps(discovered, indent=2))
            mirror = args.output.resolve() / 'ctest' / directory.name
            mirror.mkdir(parents=True)
            lines, remaps = [], []
            for test in selected:
                command = [str(args.python_executable.absolute()) if x in ('/usr/bin/python3', '/usr/bin/python3.10') else x for x in test['command']]
                lines.append('add_test(' + quote(test['name']) + ' ' + ' '.join(quote(x) for x in command) + ')')
                for prop in test.get('properties', []):
                    name, value = prop['name'], prop['value']
                    if name == 'ENVIRONMENT':
                        updated = []
                        for entry in value:
                            if entry.startswith('ROS_DOMAIN_ID=') and entry.split('=', 1)[1] in occupied:
                                remaps.append(dict(test=test['name'], old=entry, new='ROS_DOMAIN_ID='+str(args.domain)))
                                entry = 'ROS_DOMAIN_ID='+str(args.domain)
                            updated.append(entry)
                        value = updated
                    if isinstance(value, list): value = ';'.join(map(str, value))
                    if isinstance(value, bool): value = 'TRUE' if value else 'FALSE'
                    lines.append('set_tests_properties(' + quote(test['name']) + ' PROPERTIES ' + name + ' ' + quote(value) + ')')
            (mirror / 'CTestTestfile.cmake').write_text('\n'.join(lines)+'\n')
            (mirror / 'remaps.json').write_text(json.dumps(remaps, indent=2))
            run(directory.name, ['ctest', '--test-dir', mirror, '--output-on-failure', '-j1',
                '--output-junit', mirror/'ctest.xml'], 900)
            native_results = directory / 'test_results'
            if native_results.is_dir(): shutil.copytree(native_results, mirror/'native-results')
    results = []
    for path in args.output.rglob('*.xml'):
        try:
            root = ET.parse(path).getroot()
            cases = root.findall('.//testcase')
            if root.tag == 'testcase': cases = [root]
            results.append(dict(path=str(path.relative_to(args.output)), cases=len(cases),
                failed=sum(c.find('failure') is not None or c.find('error') is not None for c in cases),
                skipped=sum(c.find('skipped') is not None for c in cases)))
        except ET.ParseError: pass
    underlying = []
    if args.build_base:
        for path in args.build_base.rglob('test_results/*/*.xml'):
            try:
                cases = list(ET.parse(path).getroot().iter('testcase'))
                underlying.append(dict(path=str(path), cases=len(cases),
                    failed=sum(c.find('failure') is not None or c.find('error') is not None for c in cases),
                    skipped=sum(c.find('skipped') is not None for c in cases)))
            except ET.ParseError: pass
    status = dict(underlying_xml=underlying, passed=all(c['returncode']==0 for c in commands), commands=commands, native_xml=results)
    (args.output/'summary.json').write_text(json.dumps(status, indent=2))
    raise SystemExit(0 if status['passed'] else 1)

if __name__ == '__main__': main()

#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""EXT-07 public software acceptance: explicit build, regression and evidence layers.

No workspace install is sourced. Full deployment/flight and known baseline failures
are reported separately. Existing output and original failing logs are preserved.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

from extension_baseline import active_domains
from verify_api_install import clean_environment

ROOT = Path(__file__).resolve().parents[2]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, default=Path(__file__).with_name('extension_acceptance.json'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--phase', choices=['all', 'build', 'resume-build', 'test', 'report'], default='all')
    args = parser.parse_args()
    config = json.loads(args.manifest.read_text())
    out = args.output.resolve()
    if args.phase in ['all', 'build']:
        out.mkdir(parents=True, exist_ok=False)
    elif not (out/'build-summary.json').is_file():
        parser.error('build phase required in this output directory')
    def path(key):
        return ROOT/config[key]
    env = clean_environment()
    env.update(PATH='/usr/bin:/bin', PYTHONNOUSERSITE='1', MAKEFLAGS='-j2',
               CMAKE_BUILD_PARALLEL_LEVEL='2', ROS_DOMAIN_ID=str(config['domain']),
               ROS_LOCALHOST_ONLY='1', QT_QPA_PLATFORM='offscreen',
               EASYFLY_TEST_ROS_DOMAIN_ID=str(config['domain']),
               EASYFLY_PX4_HRT_COMPAT_DIR=str(path('px4_compatibility_root')))
    if config.get('dependency_prefixes'):
        env['CMAKE_PREFIX_PATH'] = ':'.join(str(ROOT/p) for p in config['dependency_prefixes'])
    if config.get('runtime_library_dirs'):
        env['LD_LIBRARY_PATH'] = ':'.join(str(ROOT/p) for p in config['runtime_library_dirs'])
    commands = []
    def run(label, command, prefixes=(), timeout=None):
        cmd = list(map(str, command))
        if prefixes:
            script = 'source /opt/ros/humble/setup.bash\n'
            script += 'source "$1/local_setup.bash"\nshift\n' * len(prefixes)
            cmd = ['bash', '--noprofile', '--norc', '-c', script+'exec "$@"',
                   'bash', *map(str, prefixes), *cmd]
        print(label, flush=True)
        logfile = out/(label+'.log')
        if logfile.exists():
            raise ValueError('refusing to overwrite: '+str(logfile))
        start = time.monotonic()
        with logfile.open('w') as log:
            try:
                code = subprocess.run(cmd, cwd=ROOT, env=env, stdout=log,
                                      stderr=subprocess.STDOUT, timeout=timeout).returncode
            except subprocess.TimeoutExpired:
                code = 124
        commands.append({'name': label, 'command': cmd, 'returncode': code,
                         'seconds': time.monotonic()-start, 'log_sha256': sha(logfile)})
        (out/(args.phase+'-commands.json')).write_text(json.dumps(commands, indent=2)+'\n')
        return code

    underlays = [ROOT/p for p in config['fixture_underlays']]
    for prefix in underlays:
        if not (prefix/'local_setup.bash').is_file():
            parser.error('missing explicit fixture underlay: '+str(prefix))
    listing = subprocess.check_output(['/usr/bin/colcon', 'list', '--base-paths', str(ROOT/'src')], env=env, text=True)
    discovered = {name: Path(directory) for name, directory, kind in
                  (line.split('\t') for line in listing.splitlines())}
    packages = {n: discovered[n] for n in config['packages']}
    packages['easyfly_ext_validation_env'] = ROOT/'tools/algorithms/fixtures/easyfly_ext_validation_env'
    files = [p for d in packages.values() for p in d.rglob('*') if p.is_file()
             and not any(x in p.parts for x in ['__pycache__', '.pytest_cache', 'build', 'Log', 'PCD'])
             and (p.suffix in {'.cpp', '.hpp', '.h', '.py', '.xml', '.yaml', '.json', '.cmake', '.msg', '.srv', '.action'}
                  or p.name == 'CMakeLists.txt')]
    fingerprints = {str(p.relative_to(ROOT)): sha(p) for p in sorted(files)}
    if args.phase in ['all', 'build', 'resume-build']:
        selected = list(packages)
        label = 'build'
        build_underlays = underlays
        if args.phase == 'resume-build':
            previous = json.loads((out/'manifest.json').read_text())
            allowed = {'dependency_prefixes', 'runtime_library_dirs'}
            if {k: v for k, v in previous.items() if k not in allowed} != {
                    k: v for k, v in config.items() if k not in allowed}:
                raise ValueError('only explicit dependency-prefix repairs are allowed on resume')
            if json.loads((out/'source-files.json').read_text()) != fingerprints:
                raise ValueError('production/test source changed; use a fresh output')
            selected = [n for n in packages if not (out/'install'/n/'share/ament_index/resource_index/packages'/n).is_file()]
            if not selected:
                raise ValueError('no missing packages to resume')
            label = 'build-resume-'+str(1+len(list(out.glob('build-resume-*.log'))))
            (out/(label+'-previous-manifest.json')).write_bytes((out/'manifest.json').read_bytes())
            (out/(label+'-previous-summary.json')).write_bytes((out/'build-summary.json').read_bytes())
            build_underlays = [*underlays, out/'install']
        (out/'manifest.json').write_text(json.dumps(config, indent=2)+'\n')
        (out/'source-files.json').write_text(json.dumps(fingerprints, indent=2)+'\n')
        (out/'packages.tsv').write_text(listing)
        (out/('domains-before.json' if label == 'build' else label+'-domains-before.json')).write_text(json.dumps(active_domains(), indent=2)+'\n')
        code = run(label, ['colcon', '--log-base', out/('colcon-log-'+label), 'build',
            '--base-paths', *packages.values(), '--build-base', out/'build',
            '--install-base', out/'install', '--executor', 'sequential', '--continue-on-error',
            '--packages-select', *selected,
            '--allow-overriding', *config['packages'], '--cmake-args', '-DBUILD_TESTING=ON',
            '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON', '-DORT_ROOT='+str(path('ort_sdk')),
            '-DEASYFLY_VALIDATION_PYTHON_SITE='+str(path('validation_runtime')),
            *config.get('cmake_args', [])], prefixes=build_underlays)
        missing = [n for n in packages if not (out/'install'/n/'share/ament_index/resource_index/packages'/n).is_file()]
        (out/'build-summary.json').write_text(json.dumps({'passed': code == 0 and not missing, 'packages': list(packages),
            'selected_this_attempt': selected, 'missing_after_build': missing,
            'command': commands[-1], 'fixture_underlays': list(map(str, underlays))}, indent=2)+'\n')
        if code:
            raise SystemExit(code)
    else:
        if json.loads((out/'manifest.json').read_text()) != config:
            raise ValueError('manifest changed since build')
        if json.loads((out/'source-files.json').read_text()) != fingerprints:
            raise ValueError('build inputs changed; use a fresh output directory')
    if args.phase in ['build', 'resume-build']:
        return
    if args.phase in ['all', 'test']:
        tooling = [p for p in (ROOT/'tools/algorithms').rglob('*') if p.is_file()
                   and '__pycache__' not in p.parts and '.pytest_cache' not in p.parts
                   and (p.suffix in ['.py', '.cpp', '.hpp', '.cmake', '.json', '.md'] or p.name == 'CMakeLists.txt')]
        (out/'tooling-inputs.json').write_text(json.dumps({str(p.relative_to(ROOT)): sha(p) for p in tooling}, indent=2)+'\n')
        for p in tooling:
            destination = out/'tooling-at-test-start'/p.relative_to(ROOT)
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(p.read_bytes())
        environment = {'platform': platform.platform(), 'scope': __doc__,
                       'fixture_underlays': list(map(str, underlays)),
                       'validation_python': str(path('validation_python'))}
        for label, cmd in {
            'compiler': ['c++', '--version'], 'cmake': ['cmake', '--version'],
            'opencv': ['pkg-config', '--modversion', 'opencv4'],
            'packages': ['dpkg-query', '-W', '-f=${binary:Package}\t${Version}\n',
                         'ros-humble-*', 'libopencv*', 'libeigen3-dev', 'libpcl*',
                         'libnlopt*', 'libyaml-cpp*', 'libqt5*']}.items():
            result = subprocess.run(cmd, env=env, capture_output=True, text=True)
            environment[label] = {'command': cmd, 'returncode': result.returncode,
                                  'stdout': result.stdout, 'stderr': result.stderr}
        (out/'environment.json').write_text(json.dumps(environment, indent=2)+'\n')
        assets = [p for d in [path('ort_sdk'), ROOT/'models',
                  out/'install/easyfly_vehicle_reid/share/easyfly_vehicle_reid/models',
                  *[ROOT/p for p in config.get('dependency_prefixes', [])]]
                  for p in d.rglob('*') if p.is_file()]
        (out/'asset-inputs.json').write_text(json.dumps({str(p): sha(p) for p in assets}, indent=2)+'\n')
        occupied = active_domains()
        if str(config['domain']) in occupied or config['domain'] in config['protected_domains']:
            raise ValueError('fixture domain is occupied/protected')
        # Some native fixtures set their own domains. Refuse conflicts rather
        # than taking over another service; tests execute serially.
        hardcoded = set()
        for p in files:
            if p.suffix == '.py' and 'test' in p.parts:
                hardcoded.update(re.findall(r'ROS_DOMAIN_ID[\"\x27]\]\s*=\s*[\"\x27](\d+)', p.read_text()))
                hardcoded.update(re.findall(r'ROS_DOMAIN_ID\s*=\s*[\"\x27](\d+)', p.read_text()))
        conflicts = hardcoded & occupied.keys()
        if conflicts:
            raise ValueError('hardcoded fixture domain occupied: '+str(conflicts))
        replay = [path('validation_python'), ROOT/'tools/algorithms/verify_extension_migration.py',
            '--prefix', out/'install', '--output', out/'regression', '--build-base', out/'build',
            '--python-executable', path('validation_python'), '--validation-runtime', path('validation_runtime'),
            '--python-runtime', path('ort_python_runtime'), '--domain', config['domain'],
            '--test-regex', config['test_pattern']]
        for prefix in underlays:
            replay += ['--underlay', prefix]
        for directory in config.get('runtime_library_dirs', []):
            replay += ['--runtime-library-dir', ROOT/directory]
        for name in ['test_ext02_native.py', 'test_ext04_routes.py', 'test_ext05_rollout.py', 'test_ext08_kcf.py']:
            replay += ['--native-test', ROOT/'tools/algorithms'/name]
        for entry in config['known_failures']:
            replay += ['--exclude-test', entry['package']+':'+entry['test']]
        run('regression', replay)
        known = []
        index = 0
        while index < len(replay):
            if replay[index] == '--exclude-test':
                index += 2
                continue
            known.append(replay[index])
            index += 1
        known[known.index('--output')+1] = out/'known-baseline-checks'
        known[known.index('--test-regex')+1] = '^('+'|'.join(e['test'] for e in config['known_failures'])+')$'
        known += ['--skip-native', '--packages', *sorted({e['package'] for e in config['known_failures']})]
        if config['known_failures']:
            run('known-baseline-checks', known)
        run('native-fuel-numerical', [path('validation_python'), ROOT/'tools/algorithms/probes/test_fuel_native.py',
            '--output', out/'native-fuel-numerical'], prefixes=[*underlays, out/'install'], timeout=120)
        run('native-library-resolution', ['ldd', out/'install/easyfly_fuel_exploration/lib/easyfly_fuel_exploration/fuel_worker'],
            prefixes=[*underlays, out/'install'], timeout=30)
        run('installed-factories', ['/usr/bin/python3', ROOT/'tools/algorithms/verify_backend_factories_install.py',
            '--prefix', out/'install', '--model', out/'install/easyfly_object_detection/share/easyfly_object_detection/models/yolo11n.onnx',
            '--output', out/'installed-factories'], timeout=120)
        run('performance', ['/usr/bin/python3', ROOT/'tools/algorithms/measure_extension_backends.py',
            '--prefix', out/'install', '--output', out/'performance'], timeout=180)
        (out/'domains-after.json').write_text(json.dumps(active_domains(), indent=2)+'\n')
    if args.phase in ['all', 'report', 'test']:
        # Result classification and baseline comparison are kept in a separate
        # deterministic reporter; it never runs ROS or changes the test results.
        from extension_acceptance_report import report
        raise SystemExit(report(out, ROOT))


if __name__ == '__main__':
    main()

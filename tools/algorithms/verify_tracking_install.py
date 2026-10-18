#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""Verify ALG-2 real tracking libraries, adapter and ROS feedback in two installs."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from verify_api_install import clean_environment


ROOT = Path(__file__).resolve().parents[2]
PACKAGES = ['easyfly_tracking_api', 'easyfly_sort', 'easyfly_track_id_fusion',
            'easyfly_multi_object_tracking']


def fingerprints():
    roots = [ROOT / 'src/algorithms/tracking' / name for name in PACKAGES[:3]]
    roots += [ROOT / 'src/perception/p3_tracking/easyfly_multi_object_tracking',
              ROOT / 'tools/algorithms/tracking_consumer']
    files = [p for root in roots for p in root.rglob('*') if p.is_file()
             and '__pycache__' not in p.parts and
             (p.suffix in {'.cpp', '.hpp', '.py', '.xml', '.yaml', '.md', '.cmake'} or
              p.name == 'CMakeLists.txt')]
    files += [Path(__file__).resolve(), ROOT / 'tools/algorithms/verify_api_install.py']
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(files)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--symlink-prefix', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out, symlink = args.output.resolve(), args.symlink_prefix.resolve()
    if out.exists():
        parser.error('output already exists; preserve prior evidence')
    if not (symlink / 'local_setup.bash').is_file():
        parser.error('build the isolated symlink prefix first')
    out.mkdir(parents=True)
    env = clean_environment()
    record = {'schema_version': 1, 'source_files': fingerprints(), 'commands': [], 'modes': []}

    def run(name, command, ros=False, overlay=None):
        actual = command
        if ros:
            script = ('source /opt/ros/humble/setup.bash && source "$1/install/setup.bash" && '
                      'source "$2/local_setup.bash" && shift 2 && ')
            args = [str(ROOT), str(symlink)]
            if overlay is not None:
                script += 'source "$1/local_setup.bash" && shift && '
                args.append(str(overlay))
            actual = ['bash', '--noprofile', '--norc', '-c', script + 'exec "$@"',
                      'bash', *args, *command]
        with (out / (name + '.log')).open('w') as log:
            code = subprocess.call(actual, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
        record['commands'].append({'name': name, 'command': actual, 'returncode': code,
                                  'log_sha256': hashlib.sha256((out / (name + '.log')).read_bytes()).hexdigest()})
        (out / 'summary.json').write_text(json.dumps(record, indent=2) + '\n')
        if code:
            raise RuntimeError(f'{name} failed; see {out / (name + ".log")}')

    run('core-closure', ['colcon', 'list', '--base-paths', 'src', '--packages-up-to',
                        *PACKAGES[:3], '--names-only'], ros=True)
    assert set((out / 'core-closure.log').read_text().splitlines()) == set(PACKAGES[:3])
    for mode in ('symlink', 'copy'):
        directory = out / mode
        build = symlink.parent / 'build' if mode == 'symlink' else directory / 'build'
        install = symlink if mode == 'symlink' else directory / 'install'
        print('Verifying', mode, flush=True)
        command = ['colcon', '--log-base', str(directory / 'log'), 'build', '--base-paths',
                   'src', '--build-base', str(build), '--install-base', str(install),
                   '--packages-select', *PACKAGES, '--allow-overriding', *PACKAGES,
                   '--executor', 'sequential', '--cmake-args', '-DBUILD_TESTING=ON']
        if mode == 'symlink':
            command.insert(command.index('--cmake-args'), '--symlink-install')
        run(mode + '-build', command, ros=True)
        selected = PACKAGES[1:]
        for package in selected:
            run(mode + '-' + package, ['ctest', '--test-dir', str(build / package),
                                      '--output-on-failure', '-j1', '--output-junit',
                                      str(directory / (package + '.xml'))], ros=True, overlay=install)
        if mode == 'copy':
            # Runtime tests above use the normal install; plain consumption below
            # independently checks the relocated exported targets and library.
            relocated = directory / 'relocated-install'
            install.rename(relocated)
            install = relocated
            for package in PACKAGES:
                for p in (install / package / 'include').rglob('*.hpp'):
                    assert not p.is_symlink(), p
        source = directory / 'consumer-source'
        shutil.copytree(ROOT / 'tools/algorithms/tracking_consumer', source)
        consumer = directory / 'consumer-build'
        prefixes = ';'.join(str(install / package) for package in PACKAGES)
        run(mode + '-consumer-configure', ['cmake', '-S', str(source), '-B', str(consumer),
                                          f'-DCMAKE_PREFIX_PATH={prefixes}',
                                          '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'])
        run(mode + '-consumer-build', ['cmake', '--build', str(consumer), '--parallel', '2'])
        run(mode + '-consumer-test', ['ctest', '--test-dir', str(consumer), '--output-on-failure',
                                     '--output-junit', str(directory / 'consumer.xml')])
        assert str(ROOT / 'src/') not in (consumer / 'compile_commands.json').read_text()
        run(mode + '-consumer-libraries', ['ldd', str(consumer / 'tracking_consumer')])
        libraries = (out / (mode + '-consumer-libraries.log')).read_text()
        assert '/opt/ros/' not in libraries and 'rclcpp' not in libraries
        assert str(install / 'easyfly_multi_object_tracking/lib') in libraries
        record['modes'].append({'mode': mode, 'consumer_prefix': str(install),
                                'real_ros_feedback_passed': True,
                                'plain_consumer_without_ros_runtime_passed': True})
    assert record['source_files'] == fingerprints(), 'sources changed during validation'
    record['passed'] = True
    (out / 'summary.json').write_text(json.dumps(record, indent=2) + '\n')
    print('ALG-2 install and runtime checks passed:', out, flush=True)


if __name__ == '__main__':
    main()

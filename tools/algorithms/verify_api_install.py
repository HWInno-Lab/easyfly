#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""Build only ALG-1 API packages; verify symlink/copy installs with a plain consumer.

Uses /opt/ros/humble for builds/ament test wrappers, then a consumer with no ROS overlay.
No production nodes, simulation, flight actions, downloads or source moves.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
PACKAGES = {
    'easyfly_tracking_api': ROOT / 'src/algorithms/tracking/easyfly_tracking_api',
    'easyfly_grid_planner_api': ROOT / 'src/algorithms/planning/easyfly_grid_planner_api',
}


def clean_environment():
    env = dict(os.environ)
    for name in list(env):
        if name.startswith(('AMENT_', 'COLCON_', 'ROS_', 'RMW_', 'FASTRTPS_', 'FASTDDS_')) or name in {
            'CMAKE_PREFIX_PATH', 'LD_LIBRARY_PATH', 'PYTHONPATH', 'CXXFLAGS', 'CPPFLAGS',
            'LDFLAGS', 'CPATH', 'CPLUS_INCLUDE_PATH', 'C_INCLUDE_PATH', 'CMAKE_TOOLCHAIN_FILE'}:
            env.pop(name)
    env['CMAKE_BUILD_PARALLEL_LEVEL'] = '2'
    env['MAKEFLAGS'] = '-j2'
    return env


def source_fingerprints():
    paths = [p for package in PACKAGES.values() for p in package.rglob('*') if p.is_file()]
    paths += [p for p in (ROOT / 'tools/algorithms/consumer').rglob('*') if p.is_file()]
    paths.append(Path(__file__).resolve())
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(paths)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    out = args.output.resolve()
    if out.exists():
        parser.error(f'output already exists: {out}')
    out.mkdir(parents=True)
    env = clean_environment()
    record = {'schema_version': 1, 'commands': [], 'modes': [], 'scope': __doc__.strip(),
              'source_files': source_fingerprints()}

    def run(name, command, ros=False):
        actual = command
        if ros:
            actual = ['bash', '--noprofile', '--norc', '-c',
                      'source /opt/ros/humble/setup.bash && exec "$@"', 'bash', *command]
        with (out / f'{name}.log').open('w') as stream:
            code = subprocess.call(actual, cwd=ROOT, env=env, stdout=stream, stderr=subprocess.STDOUT)
        record['commands'].append({'name': name, 'command': actual, 'returncode': code,
                                  'log_sha256': hashlib.sha256((out / f'{name}.log').read_bytes()).hexdigest()})
        (out / 'summary.json').write_text(json.dumps(record, indent=2) + '\n')
        if code:
            raise RuntimeError(f'{name} failed ({code}); see {out / (name + ".log")}')

    for name, package in PACKAGES.items():
        xml = ET.parse(package / 'package.xml').getroot()
        assert xml.findtext('name') == name
        assert not any(child.tag in {'depend', 'build_depend', 'build_export_depend', 'exec_depend'}
                       for child in xml), f'{name}: unexpected runtime/build dependency'
    run('discovery', ['colcon', 'list', '--base-paths', str(ROOT / 'src'),
                      '--packages-up-to', *PACKAGES, '--names-only'], ros=True)
    discovered = set((out / 'discovery.log').read_text().splitlines())
    assert discovered == set(PACKAGES), f'unexpected dependency closure: {discovered}'

    for mode in ('symlink', 'copy'):
        print(f'Building {mode} API packages', flush=True)
        directory = out / mode
        command = ['colcon', '--log-base', str(directory / 'log'), 'build', '--base-paths',
                   *[str(path) for path in PACKAGES.values()], '--build-base', str(directory / 'build'),
                   '--install-base', str(directory / 'install'), '--executor', 'sequential',
                   '--cmake-args', '-DBUILD_TESTING=ON']
        if mode == 'symlink':
            command.insert(command.index('--cmake-args'), '--symlink-install')
        run(f'{mode}-build', command, ros=True)
        for package in PACKAGES:
            run(f'{mode}-{package}-tests', ['ctest', '--test-dir', str(directory / 'build' / package),
                                          '--output-on-failure', '--output-junit',
                                          str(directory / f'{package}.xml')], ros=True)

        prefix = directory / 'install'
        if mode == 'copy':
            # The consumer must use exported targets after installation relocation.
            relocated = directory / 'relocated-install'
            prefix.rename(relocated)
            prefix = relocated
            for package in PACKAGES:
                for header in (prefix / package / 'include').rglob('*.hpp'):
                    assert not header.is_symlink(), f'copy install header is a symlink: {header}'
        source = directory / 'consumer-source'
        shutil.copytree(ROOT / 'tools/algorithms/consumer', source)
        build = directory / 'consumer-build'
        prefixes = ';'.join(str(prefix / package) for package in PACKAGES)
        run(f'{mode}-consumer-configure', ['cmake', '-S', str(source), '-B', str(build),
                                          f'-DCMAKE_PREFIX_PATH={prefixes}',
                                          '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'])
        run(f'{mode}-consumer-build', ['cmake', '--build', str(build), '--parallel', '2'])
        run(f'{mode}-consumer-tests', ['ctest', '--test-dir', str(build), '--output-on-failure',
                                     '--output-junit', str(directory / 'consumer.xml')])
        includes = (build / 'interface-includes.txt').read_text()
        compile_commands = (build / 'compile_commands.json').read_text()
        assert str(ROOT / 'src/algorithms') not in includes + compile_commands
        for package in PACKAGES:
            assert str(prefix / package / 'include') in includes
        run(f'{mode}-consumer-libraries', ['ldd', str(build / 'api_consumer')])
        libraries = (out / f'{mode}-consumer-libraries.log').read_text()
        assert '/opt/ros/' not in libraries and 'rclcpp' not in libraries
        counts = []
        for xml_file in directory.glob('*.xml'):
            for suite in ET.parse(xml_file).getroot().iter('testsuite'):
                assert int(suite.get('failures', '0')) == 0 and int(suite.get('errors', '0')) == 0
                counts.append({'file': xml_file.name, 'tests': suite.get('tests')})
        record['modes'].append({'mode': mode, 'consumer_prefix': str(prefix),
                                'interface_includes': includes, 'junit': counts,
                                'consumer_has_ros_runtime_dependency': False})
        print(f'{mode}: package tests, installed consumer and dependency checks passed', flush=True)
    assert record['source_files'] == source_fingerprints(), 'API sources changed during validation'
    record['passed'] = True
    (out / 'summary.json').write_text(json.dumps(record, indent=2) + '\n')
    print(f'ALG-1 install matrix passed: {out / "summary.json"}', flush=True)


if __name__ == '__main__':
    main()

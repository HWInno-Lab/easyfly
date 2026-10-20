#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Build/test both detectors and consume installed cores without a ROS overlay."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
CORES = ['easyfly_ellipse', 'easyfly_color_line']
NODES = ['easyfly_ellipse_detection', 'easyfly_color_line_detection']
SOURCES = [ROOT / 'src/algorithms/detection' / name for name in CORES] + [
    ROOT / 'src/perception/p2_detection' / name
    for name in NODES + ['easyfly_image_detection_common']
] + [ROOT / 'src/easyfly_common/easyfly_msgs']


def fingerprints():
    result = {}
    for directory in SOURCES + [ROOT / 'tools/algorithms/classical_detection_consumer']:
        for path in sorted(directory.rglob('*')):
            if path.is_file() and path.suffix in {
                    '.hpp', '.cpp', '.py', '.yaml', '.xml', '.msg',
                    '.srv', '.action', '.cmake', '.txt'}:
                result[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    result[str(Path(__file__).relative_to(ROOT))] = hashlib.sha256(
        Path(__file__).read_bytes()).hexdigest()
    return result


def run(command, log, environment, ros_prefix=None):
    invocation = list(map(str, command))
    if ros_prefix is not None:
        setup = 'source /opt/ros/humble/setup.bash; '
        if ros_prefix:
            setup += 'source "$1/setup.bash"; shift; '
        invocation = ['bash', '--noprofile', '--norc', '-c', setup + 'exec "$@"', '_'] + (
            [str(ros_prefix)] if ros_prefix else []) + invocation
    with log.open('w') as stream:
        stream.write(json.dumps(invocation) + '\n')
        stream.flush()
        subprocess.run(invocation, cwd=ROOT, env=environment, stdout=stream,
                       stderr=subprocess.STDOUT, check=True, timeout=600)


def consumer(install, directory, environment):
    directory.mkdir()
    prefixes = [install / name for name in CORES]
    local_env = dict(environment, LD_LIBRARY_PATH=':'.join(str(p / 'lib') for p in prefixes))
    # A private source copy prevents accidental sibling includes in the consumption fixture.
    with tempfile.TemporaryDirectory(prefix='easyfly_classical_consumer_') as temp:
        source = Path(temp) / 'consumer'
        shutil.copytree(ROOT / 'tools/algorithms/classical_detection_consumer', source)
        build = directory / 'build'
        run(['cmake', '-S', source, '-B', build, '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON',
             '-DCMAKE_PREFIX_PATH=' + ';'.join(map(str, prefixes))],
            directory / 'configure.log', local_env)
        run(['cmake', '--build', build, '-j2'], directory / 'build.log', local_env)
        run(['ctest', '--test-dir', build, '--output-on-failure'],
            directory / 'test.log', local_env)
        commands = (build / 'compile_commands.json').read_text()
        if '/src/algorithms/' in commands or '/opt/ros/' in commands:
            raise RuntimeError('consumer unexpectedly compiled against ROS/source headers')
        run(['ldd', build / 'consumer'], directory / 'ldd.log', local_env)
        libraries = (directory / 'ldd.log').read_text()
        if '/opt/ros/' in libraries or 'not found' in libraries:
            raise RuntimeError('consumer has a missing or ROS runtime library')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mode', choices=['both', 'copy', 'symlink'], default='both')
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    environment = {key: os.environ[key] for key in ['HOME', 'USER', 'LOGNAME']
                   if key in os.environ}
    environment.update(PATH='/usr/bin:/bin:/usr/sbin:/sbin', PYTHONNOUSERSITE='1',
                       ROS_LOCALHOST_ONLY='1', LC_ALL='C.UTF-8')
    before = fingerprints()
    (output / 'sources.json').write_text(json.dumps(before, indent=2) + '\n')
    results = {}
    for mode in ['copy', 'symlink'] if args.mode == 'both' else [args.mode]:
        directory = output / mode
        directory.mkdir()
        build, install = directory / 'build', directory / 'install'
        print('Building and testing ' + mode, flush=True)
        command = ['colcon', '--log-base', directory / 'build-log', 'build',
                   '--base-paths', *SOURCES, '--build-base', build, '--install-base', install,
                   '--executor', 'sequential', '--cmake-args', '-DBUILD_TESTING=ON',
                   '-DCMAKE_BUILD_TYPE=RelWithDebInfo']
        if mode == 'symlink':
            command.insert(command.index('--cmake-args'), '--symlink-install')
        run(command, directory / 'build.log', environment, ros_prefix='')
        run(['colcon', '--log-base', directory / 'test-log', 'test', '--base-paths', *SOURCES,
             '--build-base', build, '--install-base', install, '--packages-select',
             *CORES, *NODES, 'easyfly_image_detection_common', '--executor', 'sequential',
             '--return-code-on-test-failure'], directory / 'test.log', environment,
            ros_prefix=install)
        run(['colcon', 'test-result', '--test-result-base', build, '--verbose'],
            directory / 'results.log', environment, ros_prefix='')
        consumer(install, directory / 'consumer', environment)
        if mode == 'copy':
            moved = directory / 'relocated-install'
            install.rename(moved)
            for name in CORES:
                header = moved / name / 'include' / name / 'detector.hpp'
                if header.is_symlink():
                    raise RuntimeError('copy install exported a source symlink')
            consumer(moved, directory / 'relocated-consumer', environment)
        results[mode] = {'build': 'passed', 'tests': 'passed', 'consumer': 'passed',
                         'relocated_consumer': 'passed' if mode == 'copy' else 'not_applicable'}
    if fingerprints() != before:
        raise RuntimeError('sources changed during validation')
    (output / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
    print(json.dumps(results, indent=2), flush=True)


if __name__ == '__main__':
    main()

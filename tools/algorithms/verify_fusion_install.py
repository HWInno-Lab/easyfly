#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""ALG-6: projection/association equivalence, real ROS, two installs and pure consumers."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from verify_api_install import clean_environment

ROOT = Path(__file__).resolve().parents[2]
CORE = ['easyfly_projection', 'easyfly_projection_py', 'easyfly_obstacle_association']
FUNCTIONAL = ['easyfly_pixel_to_3d', 'easyfly_semantic_map', 'easyfly_lidar_camera_fusion']
PATHS = ['src/algorithms/fusion_mapping/' + p for p in CORE]
PATHS += ['src/perception/p4_fusion/' + p for p in FUNCTIONAL]
PATHS += ['tools/algorithms/projection_consumer']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fingerprints():
    files = [p for path in PATHS for p in (ROOT / path).rglob('*') if p.is_file()
             and '__pycache__' not in p.parts and '.pytest_cache' not in p.parts]
    files += [Path(__file__).resolve(), ROOT / 'tools/algorithms/verify_api_install.py']
    files += [ROOT / 'tools/algorithms/probes' / p for p in ('projection_probe.cpp', 'fusion_differential.py')]
    files += [ROOT / 'src/perception/p3_tracking/easyfly_multi_object_tracking/test/test_tracking_feedback_ros.py']
    return {str(p.relative_to(ROOT)): sha(p) for p in sorted(files)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--symlink-prefix', type=Path, required=True)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out, symlink, baseline = args.output.resolve(), args.symlink_prefix.resolve(), args.baseline.resolve()
    if out.exists():
        parser.error('output already exists; preserve prior evidence')
    if not (symlink / 'local_setup.bash').is_file():
        parser.error('build isolated symlink prefix first')
    out.mkdir(parents=True)
    env = clean_environment()
    env.update(ROS_DOMAIN_ID='218', ROS_LOCALHOST_ONLY='1')
    record = {'schema_version': 1, 'source_files': fingerprints(), 'commands': [], 'modes': []}

    def run(name, command, ros=False, overlay=None):
        actual = command
        if ros:
            script = ('source /opt/ros/humble/setup.bash && source "$1/install/setup.bash" && '
                      'source "$1/artifacts/scene-s3-20261016/normal/install/local_setup.bash" && '
                      'source "$1/artifacts/algorithms/alg2-20261018/install/local_setup.bash" && '
                      'source "$2/local_setup.bash" && shift 2 && ')
            paths = [str(ROOT), str(symlink)]
            if overlay is not None:
                script += 'source "$1/local_setup.bash" && shift && '
                paths.append(str(overlay))
            actual = ['bash', '--noprofile', '--norc', '-c', script + 'exec "$@"', 'bash', *paths, *command]
        print(name, flush=True)
        log = out / (name + '.log')
        with log.open('w') as stream:
            code = subprocess.call(actual, cwd=ROOT, env=env, stdout=stream, stderr=subprocess.STDOUT)
        record['commands'].append({'name': name, 'command': actual, 'returncode': code, 'log_sha256': sha(log)})
        (out / 'summary.json').write_text(json.dumps(record, indent=2) + '\n')
        if code:
            raise RuntimeError(f'{name} failed; see {log}')
        return log.read_text()

    snapshot = json.loads((baseline / 'input-snapshot.json').read_text())
    for name, digest in snapshot['files'].items():
        assert sha(baseline / 'legacy' / name) == digest
    record['baseline_snapshot_sha256'] = sha(baseline / 'input-snapshot.json')
    record['baseline_source_commit'] = snapshot['source_commit']
    legacy_dir = baseline / 'legacy/src/perception/p4_fusion'
    old = (legacy_dir / 'easyfly_pixel_to_3d/src/pixel_to_3d_node.cpp').read_text()
    start, end = old.index('    float medianDepth'), old.index('\n    void onTracks')
    method = old[start:end].replace('easyfly_msgs::msg::TrackInfo', 'SampleBox').replace('sensor_msgs::msg::Image', 'SampleImage')
    (out / 'legacy_projection.hpp').write_text('struct LegacySampler { SampleOptions params_;\n' + method + '\n};\n')
    run('legacy-cpp-build', ['g++', '-std=c++17', '-O2', '-DEASYFLY_LEGACY_PROJECTION',
        '-I' + str(out), str(ROOT / 'tools/algorithms/probes/projection_probe.cpp'), '-o', str(out / 'legacy-projection')])
    legacy_trace = run('legacy-cpp', [str(out / 'legacy-projection')])
    closure = run('core-closure', ['colcon', 'list', '--base-paths', 'src', '--packages-up-to', *CORE, '--names-only'], ros=True)
    assert set(closure.splitlines()) == set(CORE)
    for mode in ('symlink', 'copy'):
        directory = out / mode
        directory.mkdir()
        build = symlink.parent / 'build' if mode == 'symlink' else directory / 'build'
        install = symlink if mode == 'symlink' else directory / 'install'
        command = ['colcon', '--log-base', str(directory / 'log'), 'build', '--base-paths', 'src',
                   '--build-base', str(build), '--install-base', str(install), '--packages-select', *CORE, *FUNCTIONAL,
                   '--allow-overriding', *CORE, *FUNCTIONAL, '--executor', 'sequential']
        if mode == 'symlink':
            command += ['--symlink-install']
        command += ['--cmake-args', '-DBUILD_TESTING=ON']
        run(mode + '-build', command, ros=True)
        for package in CORE + FUNCTIONAL:
            pattern = '^test_'
            if package == 'easyfly_semantic_map':
                pattern = '^test_classification(_ros)?$'
            run(mode + '-' + package, ['ctest', '--test-dir', str(build / package), '--output-on-failure',
                '-j1', '-R', pattern, '--output-junit', str(directory / (package + '.xml'))], ros=True, overlay=install)
        run(mode + '-full-message-differential', ['python3', str(ROOT / 'tools/algorithms/probes/fusion_differential.py'),
            '--legacy-dir', str(legacy_dir), '--output', str(directory / 'differential.json')], ros=True, overlay=install)
        differential = json.loads((directory / 'differential.json').read_text())
        for package in ('easyfly_lidar_camera_fusion', 'easyfly_semantic_map'):
            assert differential['prefixes'][package] == str(install / package)
        run(mode + '-tracking-world-feedback', ['python3', '-m', 'pytest', '-q',
            str(ROOT / 'src/perception/p3_tracking/easyfly_multi_object_tracking/test/test_tracking_feedback_ros.py'),
            '--junitxml=' + str(directory / 'tracking-feedback.xml')], ros=True, overlay=install)
        if mode == 'copy':
            relocated = directory / 'relocated-install'
            install.rename(relocated)
            install = relocated
            for package in CORE:
                for path in (install / package).rglob('*'):
                    if path.suffix in {'.hpp', '.py'}:
                        assert not path.is_symlink(), path
        source = directory / 'consumer-source'
        shutil.copytree(ROOT / 'tools/algorithms/projection_consumer', source)
        shutil.copy2(ROOT / 'tools/algorithms/probes/projection_probe.cpp', source / 'projection_probe.cpp')
        consumer = directory / 'consumer-build'
        run(mode + '-consumer-configure', ['cmake', '-S', str(source), '-B', str(consumer),
            '-DCMAKE_PREFIX_PATH=' + str(install / CORE[0]), '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'])
        run(mode + '-consumer-build', ['cmake', '--build', str(consumer), '--parallel', '2'])
        run(mode + '-consumer-test', ['ctest', '--test-dir', str(consumer), '--output-on-failure',
            '--output-junit', str(directory / 'consumer.xml')])
        assert str(ROOT / 'src/') not in (consumer / 'compile_commands.json').read_text()
        trace = run(mode + '-cpp-differential', [str(consumer / 'projection_probe')])
        assert trace == legacy_trace
        run(mode + '-python-consumer', ['/usr/bin/python3', '-I', str(source / 'python_consumer.py'),
            str(install / CORE[1]), str(install / CORE[2])])
        libraries = run(mode + '-libraries', ['ldd', str(consumer / 'projection_consumer')])
        assert '/opt/ros/' not in libraries and 'rclcpp' not in libraries and 'not found' not in libraries
        record['modes'].append({'mode': mode, 'consumer_prefix': str(install), 'differential': differential,
            'cpp_trace_sha256': hashlib.sha256(trace.encode()).hexdigest(), 'cpp_trace_cases': 240,
            'plain_cpp_and_python_without_ros_passed': True, 'real_ros_and_world_feedback_passed': True})
    assert record['source_files'] == fingerprints(), 'sources changed during validation'
    assert record['modes'][0]['differential']['projection_trace_sha256'] == record['modes'][1]['differential']['projection_trace_sha256']
    assert record['modes'][0]['differential']['classification_trace_sha256'] == record['modes'][1]['differential']['classification_trace_sha256']
    record['passed'] = True
    (out / 'summary.json').write_text(json.dumps(record, indent=2) + '\n')
    print('ALG-6 matrix passed:', out, flush=True)


if __name__ == '__main__':
    main()

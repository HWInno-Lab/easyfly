#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""ALG-5: real vision differentials, ROS regressions, two installs and relocation."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from verify_api_install import clean_environment

ROOT = Path(__file__).resolve().parents[2]
CORE = ['easyfly_detection_api', 'easyfly_single_tracking_api', 'easyfly_yolo_onnx',
        'easyfly_tag_pose', 'easyfly_csrt']
FUNCTIONAL = ['easyfly_object_detection', 'easyfly_apriltag_detection',
              'easyfly_single_object_tracking', 'easyfly_landing_detection',
              'easyfly_precision_landing']
PATHS = ['src/algorithms/detection/' + p for p in CORE if p not in CORE[1:2] + CORE[4:]]
PATHS += ['src/algorithms/tracking/' + p for p in (CORE[1], CORE[4])]
PATHS += ['src/perception/p2_detection/' + p for p in FUNCTIONAL[:2] + FUNCTIONAL[3:4]]
PATHS += ['src/perception/p3_tracking/' + FUNCTIONAL[2],
          'src/execution/e3_control/' + FUNCTIONAL[4], 'tools/algorithms/vision_consumer']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fingerprints():
    files = [p for root in PATHS for p in (ROOT / root).rglob('*') if p.is_file()
             and '__pycache__' not in p.parts and '.pytest_cache' not in p.parts]
    files += [Path(__file__).resolve(), ROOT / 'tools/algorithms/verify_api_install.py']
    files += [ROOT / 'tools/algorithms/probes' / p for p in ('detection_probe.cpp', 'vision_probe.cpp')]
    return {str(p.relative_to(ROOT)): sha(p) for p in sorted(files)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--symlink-prefix', type=Path, required=True)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out, symlink, baseline = args.output.resolve(), args.symlink_prefix.resolve(), args.baseline.resolve()
    if out.exists():
        parser.error('output exists; preserve prior evidence')
    out.mkdir(parents=True)
    env = clean_environment()
    env.update(ROS_DOMAIN_ID='217', ROS_LOCALHOST_ONLY='1')
    record = {'schema_version': 1, 'source_files': fingerprints(), 'commands': [], 'modes': []}

    def save():
        (out / 'summary.json').write_text(json.dumps(record, indent=2) + '\n')

    def run(name, command, ros=False, overlay=None):
        actual = command
        if ros:
            script = ('source /opt/ros/humble/setup.bash && source "$1/install/setup.bash" && '
                      'source "$1/artifacts/scene-s3-20261016/normal/install/local_setup.bash" && '
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
        save()
        if code:
            raise RuntimeError(f'{name} failed; see {log}')
        return log.read_text()

    sdk = ROOT / 'src/third_party/runtimes/onnxruntime'
    snapshot = json.loads((baseline / 'input-snapshot.json').read_text())
    record['input_snapshot_sha256'] = sha(baseline / 'input-snapshot.json')
    expected = json.loads((baseline / 'legacy-detection.json').read_text())
    data = out / 'data'
    data.mkdir()
    frame = ROOT / 'src/perception/p2_detection/easyfly_object_detection/test/m2_frame.bin'
    config = ROOT / 'src/perception/p2_detection/easyfly_object_detection/config/detection.yaml'
    shutil.copy2(frame, data / frame.name)
    shutil.copy2(config, data / config.name)
    record['assets'] = {'frame_sha256': sha(frame), 'model_sha256': sha(ROOT / 'models/yolo11n.onnx')}
    # Compile preserved pre-migration headers against the same OpenCV implementation.
    cv = subprocess.check_output(['pkg-config', '--cflags', '--libs', 'opencv4'], text=True).split()
    run('legacy-vision-build', ['g++', '-std=c++17', '-O2', '-DEASYFLY_LEGACY_VISION',
        str(ROOT / 'tools/algorithms/probes/vision_probe.cpp'),
        '-I' + str(baseline / 'legacy/src/perception/p2_detection/easyfly_apriltag_detection/include'),
        '-I' + str(baseline / 'legacy/src/perception/p3_tracking/easyfly_single_object_tracking/include'),
        '-o', str(out / 'legacy-vision'), *cv])
    legacy_trace = run('legacy-vision', [str(out / 'legacy-vision')])
    closure = run('core-closure', ['colcon', 'list', '--base-paths', 'src', '--packages-up-to', *CORE, '--names-only'], ros=True)
    assert set(closure.splitlines()) == set(CORE)
    external = out / 'external-ort-sdk'
    external.symlink_to(sdk, target_is_directory=True)
    for mode in ('symlink', 'copy'):
        directory = out / mode
        directory.mkdir()
        build = symlink.parent / 'build' if mode == 'symlink' else directory / 'build'
        install = symlink if mode == 'symlink' else directory / 'install'
        command = ['colcon', '--log-base', str(directory / 'log'), 'build', '--base-paths', 'src',
                   '--build-base', str(build), '--install-base', str(install), '--packages-select',
                   *CORE, *FUNCTIONAL, '--allow-overriding', *CORE, *FUNCTIONAL,
                   '--executor', 'sequential']
        if mode == 'symlink':
            command += ['--symlink-install']
        command += ['--cmake-args', '-DBUILD_TESTING=ON']
        if mode == 'copy':
            command += ['-DORT_ROOT=' + str(external)]
        run(mode + '-build', command, ros=True)
        for package in CORE + FUNCTIONAL:
            if package == CORE[1]:
                continue  # Interface-only package; exercised by real CSRT tests.
            pattern = '^test_|detection_contract$'
            if package == 'easyfly_precision_landing':
                pattern = '^test_(configuration|geometry|replay_tag_frames|observation_ros|stream_pair_ros|frame_timing_ros)$'
            run(mode + '-' + package, ['ctest', '--test-dir', str(build / package), '--output-on-failure',
                '-j1', '-R', pattern, '--output-junit', str(directory / (package + '.xml'))], ros=True, overlay=install)
        source = directory / 'consumer-source'
        shutil.copytree(ROOT / 'tools/algorithms/vision_consumer', source)
        for probe in ('detection_probe.cpp', 'vision_probe.cpp'):
            shutil.copy2(ROOT / 'tools/algorithms/probes' / probe, source / probe)
        legacy_build = directory / 'legacy-consumer-build'
        run(mode + '-legacy-configure', ['cmake', '-S', str(source), '-B', str(legacy_build),
            '-DEASYFLY_CHECK_LEGACY=ON', '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'], ros=True, overlay=install)
        run(mode + '-legacy-build', ['cmake', '--build', str(legacy_build), '--parallel', '2'], ros=True, overlay=install)
        run(mode + '-legacy-test', ['ctest', '--test-dir', str(legacy_build), '--output-on-failure'], ros=True, overlay=install)
        assert str(ROOT / 'src/') not in (legacy_build / 'compile_commands.json').read_text()
        cache = (build / 'easyfly_yolo_onnx/CMakeCache.txt').read_text()
        assert 'ORT_ROOT:PATH=' + (str(external) if mode == 'copy' else str(ROOT / 'src/algorithms/detection/easyfly_yolo_onnx/../../../third_party/runtimes/onnxruntime')) in cache
        if mode == 'copy':
            relocated = directory / 'relocated-install'
            install.rename(relocated)
            install = relocated
            for package in CORE:
                assert all(not p.is_symlink() for p in (install / package / 'include').rglob('*.hpp'))
        consumer = directory / 'consumer-build'
        run(mode + '-consumer-configure', ['cmake', '-S', str(source), '-B', str(consumer),
            '-DCMAKE_PREFIX_PATH=' + ';'.join(str(install / p) for p in CORE), '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'])
        run(mode + '-consumer-build', ['cmake', '--build', str(consumer), '--parallel', '2'])
        run(mode + '-consumer-test', ['ctest', '--test-dir', str(consumer), '--output-on-failure',
            '--output-junit', str(directory / 'consumer.xml')])
        assert str(ROOT / 'src/') not in (consumer / 'compile_commands.json').read_text()
        trace = run(mode + '-vision-differential', [str(consumer / 'vision_probe')])
        assert trace == legacy_trace, 'tag/CSRT normal and rejection traces changed'
        results = []
        model = install / 'easyfly_object_detection/share/easyfly_object_detection/models/yolo11n.onnx'
        assert sha(model) == record['assets']['model_sha256']
        for native in expected['runs']:
            output = run(mode + '-detection-' + native['requested'], [str(consumer / 'detection_probe'),
                str(model), str(data / frame.name), str(data / config.name), native['requested']])
            result = json.loads(next(line[7:] for line in output.splitlines() if line.startswith('RESULT ')))
            assert result['provider'] == native['result']['provider']
            assert result['caller_timing_preserved']
            assert len(result['detections']) == len(native['result']['detections']) == 5
            for a, b in zip(result['detections'], native['result']['detections']):
                assert (a['class_id'], a['class_name']) == (b['class_id'], b['class_name'])
                assert abs(a['confidence'] - b['confidence']) <= 1e-5
                assert max(abs(x-y) for x, y in zip(a['box'], b['box'])) <= 1e-3
            results.append({'requested': native['requested'], 'result': result})
        libraries = run(mode + '-libraries', ['ldd', str(consumer / 'detection_probe')])
        assert '/opt/ros/' not in libraries and 'rclcpp' not in libraries and 'not found' not in libraries
        if mode == 'copy':
            assert str(sdk) not in libraries
            for line in libraries.splitlines():
                if 'libeasyfly_yolo_onnx' in line or 'libonnxruntime.so' in line:
                    assert str(install / 'easyfly_yolo_onnx/lib') in line
            dynamic = run(mode + '-rpath', ['readelf', '-d', str(install / 'easyfly_yolo_onnx/lib/libeasyfly_yolo_onnx.so')])
            assert '$ORIGIN' in dynamic and str(ROOT / 'src') not in dynamic
        runtime = install / 'easyfly_yolo_onnx'
        for library in (sdk / 'lib').glob('*.so*'):
            assert sha(runtime / 'lib' / library.name) == sha(library)
        assert (runtime / 'share/easyfly_yolo_onnx/onnxruntime/LICENSE').is_file()
        record['modes'].append({'mode': mode, 'consumer_prefix': str(install), 'detections': results,
            'vision_trace_identical': True, 'installed_legacy_headers_passed': True,
            'plain_consumer_without_ros_runtime_passed': True, 'runtime_libraries_identical': True})
        save()
    assert record['source_files'] == fingerprints(), 'sources changed during validation'
    record['passed'] = True
    save()
    print('ALG-5 matrix passed:', out, flush=True)


if __name__ == '__main__':
    main()

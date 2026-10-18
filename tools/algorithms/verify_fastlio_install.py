#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""ALG-8.1 whole-tree relocation, launch/TF, actual installed process and message consumption."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from verify_api_install import clean_environment

ROOT = Path(__file__).resolve().parents[2]
OLD = 'src/third_party/slam/fast_lio'
NEW = 'src/third_party/algorithms/localization/fast_lio'
FUNCTIONAL = 'src/perception/p3_tracking/easyfly_fast_lio_integration'
PACKAGES = ['fast_lio', 'easyfly_fast_lio_integration', 'easyfly_px4_sitl']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fingerprints(snapshot):
    files = [ROOT / NEW / n[len(OLD)+1:] for n in snapshot['files'] if n.startswith(OLD + '/')]
    files += [p for base in [FUNCTIONAL, 'tools/algorithms/fastlio_consumer']
              for p in (ROOT / base).rglob('*') if p.is_file()
              and '__pycache__' not in p.parts and '.pytest_cache' not in p.parts]
    files += [Path(__file__).resolve(), ROOT / 'tools/algorithms/fastlio_runtime_probe.py',
              ROOT / 'tools/algorithms/verify_api_install.py', ROOT / '.gitignore',
              ROOT / 'src/third_party/README.md', ROOT / 'src/third_party/algorithms/localization/README.md',
              ROOT / 'src/third_party/algorithms/localization/fast_lio.provenance.json']
    return {str(p.relative_to(ROOT)): sha(p) for p in sorted(files)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--symlink-prefix', type=Path, required=True)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out, symlink, baseline = args.output.resolve(), args.symlink_prefix.resolve(), args.baseline.resolve()
    if out.exists():
        parser.error('preserve prior evidence; output exists')
    snapshot = json.loads((baseline / 'input-snapshot.json').read_text())
    for name, digest in snapshot['files'].items():
        assert sha(baseline / 'legacy' / name) == digest
        if name.startswith(OLD + '/') and not name.endswith('/CMakeLists.txt'):
            assert sha(ROOT / NEW / name[len(OLD)+1:]) == digest, name
    for n, h in snapshot['demo_assets'].items():
        assert sha(ROOT / NEW / n) == h
    for n, h in snapshot['underlay_binaries'].items():
        assert sha(ROOT / n) == h
    old_cmake = (baseline / 'legacy' / OLD / 'CMakeLists.txt').read_text()
    expected = old_cmake.replace('add_definitions(-DROOT_DIR=\\\"${CMAKE_CURRENT_SOURCE_DIR}/\\\")', '''# Keep runtime outputs separate from source/install trees when explicitly configured.
# Default retains the upstream layout; relocation validation supplies an isolated writable root.
set(FASTLIO_OUTPUT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "Writable FAST-LIO Log/ and PCD/ root")
file(MAKE_DIRECTORY "${FASTLIO_OUTPUT_ROOT}/Log" "${FASTLIO_OUTPUT_ROOT}/PCD")
add_definitions(-DROOT_DIR=\\\"${FASTLIO_OUTPUT_ROOT}/\\\")''').replace(
        'if(BUILD_TESTING)\n', 'install(FILES LICENSE README.md DESTINATION share/${PROJECT_NAME})\n\nif(BUILD_TESTING)\n')
    assert expected == (ROOT / NEW / 'CMakeLists.txt').read_text(), 'unexpected vendor build changes'
    assert (ROOT / OLD).is_symlink() and (ROOT / OLD).resolve() == (ROOT / NEW).resolve()
    assert (ROOT / OLD).parent.joinpath('COLCON_IGNORE').is_file()
    out.mkdir(parents=True)
    env = clean_environment()
    env.update(ROS_DOMAIN_ID='222', ROS_LOCALHOST_ONLY='1', CMAKE_BUILD_PARALLEL_LEVEL='1', MAKEFLAGS='-j1')
    env.pop('EASYFLY_PX4_HRT_COMPAT_DIR', None)
    env.pop('PX4_DIR', None)
    record = {'schema_version': 1, 'source_files': fingerprints(snapshot), 'commands': [], 'modes': [],
              'source_commit': snapshot['source_commit'], 'vendor_non_cmake_files_unchanged': 45,
              'demo_assets_unchanged': len(snapshot['demo_assets']), 'historical_issue': 'ALG0-B02'}

    def save():
        (out / 'summary.json').write_text(json.dumps(record, indent=2) + '\n')

    def run(name, command, ros=False, overlay=None, relocated=None, include_current=True, expected=0):
        actual = command
        if ros:
            script = ('source /opt/ros/humble/setup.bash && source "$1/install/setup.bash" && '
                      'source "$1/artifacts/scene-s3-20261016/normal/install/local_setup.bash" && shift && '
                      'source "$1/easyfly_px4_sitl/share/easyfly_px4_sitl/local_setup.bash" && shift && ')
            paths = [str(ROOT), str(symlink)]
            if include_current:
                script += 'source "$1/local_setup.bash" && shift && '
                paths.append(str(symlink))
            if overlay is not None:
                script += 'source "$1/local_setup.bash" && shift && '
                paths.append(str(overlay))
            if relocated is not None:
                # Verify native exports/loading, independently of generated colcon shell relocation.
                script += ('export AMENT_PREFIX_PATH="$1/fast_lio:$AMENT_PREFIX_PATH" && '
                           'export LD_LIBRARY_PATH="$1/fast_lio/lib:$LD_LIBRARY_PATH" && shift && ')
                paths.append(str(relocated))
            actual = ['bash', '--noprofile', '--norc', '-c', script + 'exec "$@"', 'bash', *paths, *command]
        print(name, flush=True)
        log = out / (name + '.log')
        with log.open('w') as stream:
            code = subprocess.call(actual, cwd=ROOT, env=env, stdout=stream, stderr=subprocess.STDOUT)
        record['commands'].append({'name': name, 'command': actual, 'returncode': code,
                                   'expected_returncode': expected, 'log_sha256': sha(log)})
        save()
        if code != expected:
            raise RuntimeError(f'{name}: {code}, expected {expected}; see {log}')
        return log.read_text()

    discovery = run('discovery', ['colcon', 'list', '--base-paths', 'src'], ros=True)
    entries = [line.split() for line in discovery.splitlines() if line.strip()]
    assert len(entries) == 91 and next(row[1] for row in entries if row[0] == 'fast_lio') == NEW
    assert not run('legacy-root-discovery', ['colcon', 'list', '--base-paths', str((ROOT / OLD).parent)], ros=True).strip()
    ignored = run('demo-ignore', ['git', 'check-ignore', str(ROOT / NEW / 'doc/Fast_LIO_2.pdf'), str(ROOT / OLD)], ros=True)
    assert 'Fast_LIO_2.pdf' in ignored
    # Replay the original fixture with current support modules. The expected fail-closed HRT
    # error is retained as history; the repaired contract below isolates only its unrelated boundary.
    replay = out / 'legacy-launch-replay.py'
    replay.write_text('''import importlib.util
from pathlib import Path
spec = importlib.util.spec_from_file_location('legacy_fastlio_contract', r"''' + str(baseline / 'legacy' / FUNCTIONAL / 'test/test_fastlio_launch_contract.py') + '''")
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
m.PACKAGE_DIR = Path(r"''' + str(baseline / 'legacy' / FUNCTIONAL) + '''")
m.FASTLIO_PATH = m.PACKAGE_DIR / 'launch/fastlio.launch.py'
m.WORKSPACE_SRC = Path(r"''' + str(ROOT / 'src') + '''")
m.PERCEPTION_PATH = m.WORKSPACE_SRC / 'simulation/easyfly_px4_sitl/launch/perception_sitl.launch.py'
m.main()
''')
    failure = run('legacy-launch-hrt-block', ['python3', str(replay)], ros=True, include_current=False, expected=1)
    assert 'SITL refused' in failure and 'HRT' in failure and 'no unpatched fallback' in failure
    run('hrt-native-imu-guards', ['python3', '-m', 'pytest', '-q',
        str(ROOT / 'src/simulation/easyfly_px4_sitl/test/test_hrt_selection.py'),
        str(ROOT / 'src/simulation/easyfly_px4_sitl/test/test_imu_compatibility.py'),
        '--junitxml=' + str(out / 'hrt-native-imu.xml')], ros=True)
    for mode in ('symlink', 'copy'):
        directory = out / mode
        directory.mkdir()
        build = symlink.parent / 'build' if mode == 'symlink' else directory / 'build'
        install = symlink if mode == 'symlink' else directory / 'install'
        runtime = symlink.parent / 'runtime' if mode == 'symlink' else directory / 'runtime'
        command = ['colcon', '--log-base', str(directory / 'log'), 'build', '--base-paths', 'src',
                   '--build-base', str(build), '--install-base', str(install), '--packages-select', *PACKAGES,
                   '--allow-overriding', *PACKAGES, '--executor', 'sequential']
        if mode == 'symlink':
            command += ['--symlink-install']
        command += ['--cmake-args', '-DBUILD_TESTING=ON', '-DFASTLIO_OUTPUT_ROOT=' + str(runtime)]
        run(mode + '-build', command, ros=True)
        for package, pattern in [('fast_lio', '^test_extrinsic_validation$'),
                                 ('easyfly_fast_lio_integration', '^(easyfly_fast_lio_launch_contract|test_map_odom_tf_ros)$')]:
            run(mode + '-' + package + '-tests', ['ctest', '--test-dir', str(build / package), '--output-on-failure',
                '-j1', '-R', pattern, '--output-junit', str(directory / (package + '.xml'))], ros=True, overlay=install)
        for group in ('config', 'launch', 'rviz'):
            for src in (ROOT / NEW / group).rglob('*'):
                if src.is_file() and '__pycache__' not in src.parts:
                    assert sha(src) == sha(install / 'fast_lio/share/fast_lio' / src.relative_to(ROOT / NEW))
        assert sha(install / 'fast_lio/share/fast_lio/LICENSE') == sha(ROOT / NEW / 'LICENSE')
        if mode == 'copy':
            relocated = directory / 'relocated-install'
            install.rename(relocated)
            install = relocated
            assert not (install / 'fast_lio/share/fast_lio/config/mid360.yaml').is_symlink()
        source = directory / 'consumer-source'
        shutil.copytree(ROOT / 'tools/algorithms/fastlio_consumer', source)
        consumer = directory / 'consumer-build'
        run(mode + '-consumer-configure', ['cmake', '-S', str(source), '-B', str(consumer),
            '-Dfast_lio_DIR=' + str(install / 'fast_lio/share/fast_lio/cmake'), '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'],
            ros=True, relocated=install)
        run(mode + '-consumer-build', ['cmake', '--build', str(consumer), '--parallel', '1'], ros=True, relocated=install)
        run(mode + '-consumer-test', ['ctest', '--test-dir', str(consumer), '--output-on-failure',
            '--output-junit', str(directory / 'consumer.xml')], ros=True, relocated=install)
        assert str(ROOT / 'src/') not in (consumer / 'compile_commands.json').read_text()
        libs = run(mode + '-consumer-libraries', ['ldd', str(consumer / 'fastlio_message_consumer')], ros=True, relocated=install)
        assert str(install / 'fast_lio/lib') in libs and 'not found' not in libs
        binary = install / 'fast_lio/lib/fast_lio/fastlio_mapping'
        assert OLD.encode() not in binary.read_bytes(), 'installed binary retains old source root'
        libs = run(mode + '-node-libraries', ['ldd', str(binary)], ros=True, relocated=install)
        assert 'not found' not in libs
        run(mode + '-runtime', ['env', 'ROS_DOMAIN_ID=223', 'python3',
            str(ROOT / 'tools/algorithms/fastlio_runtime_probe.py'), '--prefix', str(install),
            '--output', str(directory / 'runtime-probe')], ros=True, relocated=install)
        record['modes'].append({'mode': mode, 'install': str(install), 'installed_config_and_license_identical': True,
                               'launch_tf_and_extrinsics_passed': True, 'installed_message_roundtrip_passed': True,
                               'runtime_cases': 4, 'isolated_output_root': str(runtime), 'old_source_root_absent_in_binary': True})
    for n, h in snapshot['underlay_binaries'].items():
        assert sha(ROOT / n) == h
    for n, h in snapshot['demo_assets'].items():
        assert sha(ROOT / NEW / n) == h
    assert record['source_files'] == fingerprints(snapshot), 'sources changed during verification'
    record['active_original_fastlio_pid_still_present'] = Path('/proc', str(snapshot['active_fastlio_pid'])).exists()
    record['passed'] = True
    save()
    print('ALG-8.1 matrix passed:', out, flush=True)


if __name__ == '__main__':
    main()

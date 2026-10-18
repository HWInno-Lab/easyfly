#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""ALG-8.3: SDK identity, native builds, actual ROS driver linkage and relocation.

No device/node construction, SDK initialization, firmware, udev or system install.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from verify_api_install import clean_environment

ROOT = Path(__file__).resolve().parents[2]
SDK_PARENT = 'src/third_party/sensor_sdks'
PACKAGES = ['livox_ros_driver2', 'realsense2_camera_msgs', 'realsense2_camera']
NESTED = ROOT / 'src/communication/c1_drivers/realsense_ros'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def canonical(name):
    return name.replace('src/third_party/livox_sdk2', SDK_PARENT + '/livox_sdk2', 1).replace(
        'src/third_party/librealsense', SDK_PARENT + '/librealsense', 1)


def fingerprints(snapshot):
    files = [ROOT / canonical(n) for n in snapshot['files']]
    files += [ROOT / n for n in snapshot['drivers']]
    files += list((ROOT / 'tools/algorithms/sdk_consumer').glob('*'))
    files += [Path(__file__).resolve(), ROOT / 'tools/algorithms/sdk_offline.cmake',
              ROOT / 'tools/algorithms/verify_api_install.py', ROOT / '.gitignore',
              ROOT / 'src/third_party/README.md', ROOT / SDK_PARENT / 'README.md']
    files += list((ROOT / SDK_PARENT).glob('*.provenance.json'))
    return {str(p.relative_to(ROOT)): sha(p) for p in sorted(files) if p.is_file()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--native-build', type=Path, required=True)
    parser.add_argument('--default-prefix', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    baseline, native, default, out = [p.resolve() for p in (
        args.baseline, args.native_build, args.default_prefix, args.output)]
    if out.exists():
        parser.error('preserve prior evidence; output exists')
    snapshot = json.loads((baseline / 'input-snapshot.json').read_text())
    for name, digest in snapshot['files'].items():
        assert sha(baseline / 'legacy' / name) == sha(ROOT / canonical(name)) == digest, name
    for name, digest in snapshot['drivers'].items():
        assert sha(ROOT / name) == digest, name
    for sdk, tree in snapshot['trees'].items():
        assert not (ROOT / 'src/third_party' / sdk).exists()
        for name, digest in tree['extra_source_files'].items():
            assert sha(ROOT / canonical(name)) == digest
        assert sha(ROOT / SDK_PARENT / sdk / 'build/CMakeCache.txt') == tree['old_build_cache_sha256']
    for name, digest in snapshot['protected_installed_files'].items():
        assert sha(Path(name)) == digest
    for name, digest in snapshot['json_cache']['files'].items():
        assert sha(baseline / 'json-source' / name) == digest
    assert subprocess.check_output(['git', 'status', '--porcelain'], cwd=NESTED, text=True) == snapshot['realsense_ros']['status']
    assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=NESTED, text=True).strip() == snapshot['realsense_ros']['head']
    out.mkdir(parents=True)
    env = clean_environment()
    env.update(ROS_DOMAIN_ID='225', ROS_LOCALHOST_ONLY='1')
    record = {'schema_version': 1, 'source_commit': snapshot['source_commit'],
              'source_files': fingerprints(snapshot), 'commands': [], 'modes': [],
              'sdk_source_files_byte_identical': len(snapshot['files']),
              'driver_files_byte_identical': len(snapshot['drivers']),
              'json_source_commit': snapshot['json_cache']['source_commit']}

    def save():
        (out / 'summary.json').write_text(json.dumps(record, indent=2) + '\n')

    def run(name, command, driver_prefix=None, sdk=None, install=False):
        paths = []
        script = 'source /opt/ros/humble/setup.bash && '
        if driver_prefix:
            roots = ':'.join(str(driver_prefix / p) for p in PACKAGES)
            libs = ':'.join(str(driver_prefix / p / 'lib') for p in PACKAGES)
            script += ('export AMENT_PREFIX_PATH="$1:$AMENT_PREFIX_PATH" CMAKE_PREFIX_PATH="$1:$CMAKE_PREFIX_PATH" && '
                       'export LD_LIBRARY_PATH="$2:$LD_LIBRARY_PATH" && shift 2 && ')
            paths += [roots, libs]
        if sdk:
            script += 'export LD_LIBRARY_PATH="$1/lib:$LD_LIBRARY_PATH" && shift && '
            paths += [str(sdk)]
        if install:
            script += 'export PATH="$1:$PATH" && shift && '
            paths += [str(out / 'private-bin')]
        actual = ['bash', '--noprofile', '--norc', '-c', script + 'exec "$@"', 'bash', *paths, *command]
        print(name, flush=True)
        log = out / (name + '.log')
        with log.open('w') as stream:
            code = subprocess.call(actual, cwd=ROOT, env=env, stdout=stream, stderr=subprocess.STDOUT)
        record['commands'].append({'name': name, 'command': actual, 'returncode': code, 'log_sha256': sha(log)})
        save()
        if code:
            raise RuntimeError(f'{name}: {code}; see {log}')
        return log.read_text()

    before = snapshot['colcon_before']
    after = run('discovery', ['colcon', 'list', '--base-paths', 'src'])
    assert canonical(before) == after and len(after.splitlines()) == 91
    ignored = run('sdk-build-ignores', ['git', 'check-ignore', *[
        str(ROOT / SDK_PARENT / s / 'build/CMakeCache.txt') for s in snapshot['trees']]])
    assert len(ignored.splitlines()) == 2
    sdk_install = native / 'sdk-install'
    # Reconfigure the native builds against current inputs; never reuse old in-source caches.
    run('livox-configure', ['cmake', '-S', str(ROOT / SDK_PARENT / 'livox_sdk2'), '-B', str(native / 'livox-build'),
        '-G', 'Ninja', '-DCMAKE_BUILD_TYPE=Release', '-DCMAKE_INSTALL_PREFIX=' + str(sdk_install)])
    run('livox-build', ['cmake', '--build', str(native / 'livox-build'), '--parallel', '2'])
    run('livox-install', ['cmake', '--install', str(native / 'livox-build')])
    flags = ['-DBUILD_EXAMPLES=OFF', '-DBUILD_GRAPHICAL_EXAMPLES=OFF', '-DBUILD_TOOLS=OFF',
             '-DBUILD_GLSL_EXTENSIONS=OFF', '-DBUILD_UNIT_TESTS=OFF', '-DIMPORT_DEPTH_CAM_FW=OFF',
             '-DCHECK_FOR_UPDATES=OFF', '-DBUILD_WITH_DDS=OFF', '-DENABLE_CCACHE=OFF']
    run('realsense-configure', ['cmake', '-S', str(ROOT / SDK_PARENT / 'librealsense'),
        '-B', str(native / 'realsense-build'), '-G', 'Ninja', '-DCMAKE_BUILD_TYPE=Release',
        '-DCMAKE_INSTALL_LIBDIR=lib', '-DCMAKE_INSTALL_PREFIX=' + str(sdk_install),
        '-DCMAKE_PROJECT_realsense2_INCLUDE=' + str(ROOT / 'tools/algorithms/sdk_offline.cmake'),
        '-DALG83_JSON_SOURCE=' + str(baseline / 'json-source'), *flags])
    run('realsense-build', ['cmake', '--build', str(native / 'realsense-build'), '--parallel', '2'])
    # The upstream install script invokes ldconfig even with a private prefix.
    # Suppress only this global cache update in this process, without editing SDK sources.
    private_bin = out / 'private-bin'
    private_bin.mkdir()
    stub = private_bin / 'ldconfig'
    stub.write_text('#!/bin/sh\nprintf "%s\\n" "ALG-8.3: private install; system ldconfig intentionally skipped"\n')
    stub.chmod(0o755)
    run('realsense-install', ['cmake', '--install', str(native / 'realsense-build')], install=True)
    relocated_sdk = native / 'relocated-sdk-install'
    assert not relocated_sdk.exists()
    sdk_install.rename(relocated_sdk)
    for mode in ('default-symlink', 'explicit-copy'):
        directory = out / mode
        directory.mkdir()
        sdk = Path('/usr/local') if mode == 'default-symlink' else relocated_sdk
        build = default.parent / 'build' if mode == 'default-symlink' else directory / 'build'
        install_prefix = default if mode == 'default-symlink' else directory / 'install'
        command = ['colcon', '--log-base', str(directory / 'log'), 'build', '--base-paths', 'src',
                   '--build-base', str(build), '--install-base', str(install_prefix),
                   '--packages-select', *PACKAGES, '--executor', 'sequential']
        if mode == 'default-symlink':
            command += ['--symlink-install']
        extra = [] if mode == 'default-symlink' else [
            '-Drealsense2_DIR:PATH=' + str(sdk / 'lib/cmake/realsense2'),
            '-DLIVOX_LIDAR_SDK_LIBRARY:FILEPATH=' + str(sdk / 'lib/liblivox_lidar_sdk_shared.so'),
            '-DLIVOX_LIDAR_SDK_INCLUDE_DIR:PATH=' + str(sdk / 'include')]
        # Upstream BUILD_TESTING probes cameras and registers hardware/bag tests.
        # Validate installation with actual software consumers instead of dummy template tests.
        command += ['--cmake-args', '-DBUILD_TESTING=OFF', '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON', *extra]
        run(mode + '-drivers-build', command)
        livox_cache = (build / 'livox_ros_driver2/CMakeCache.txt').read_text()
        rs_cache = (build / 'realsense2_camera/CMakeCache.txt').read_text()
        assert 'LIVOX_LIDAR_SDK_LIBRARY:FILEPATH=' + str(sdk / 'lib/liblivox_lidar_sdk_shared.so') in livox_cache
        assert 'LIVOX_LIDAR_SDK_INCLUDE_DIR:PATH=' + str(sdk / 'include') in livox_cache
        assert 'realsense2_DIR:PATH=' + str(sdk / 'lib/cmake/realsense2') in rs_cache
        if mode == 'explicit-copy':
            relocated = directory / 'relocated-install'
            install_prefix.rename(relocated)
            install_prefix = relocated
        source = directory / 'consumer-source'
        shutil.copytree(ROOT / 'tools/algorithms/sdk_consumer', source)
        consumer = directory / 'consumer-build'
        run(mode + '-consumer-configure', ['cmake', '-S', str(source), '-B', str(consumer),
            '-DBUILD_DRIVER_CONSUMER=ON', '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON',
            '-Drealsense2_DIR:PATH=' + str(sdk / 'lib/cmake/realsense2'),
            '-DLIVOX_LIBRARY=' + str(sdk / 'lib/liblivox_lidar_sdk_shared.so'),
            '-DLIVOX_INCLUDE=' + str(sdk / 'include')], driver_prefix=install_prefix, sdk=sdk)
        run(mode + '-consumer-build', ['cmake', '--build', str(consumer), '--parallel', '2'],
            driver_prefix=install_prefix, sdk=sdk)
        trace = run(mode + '-sdk-trace', [str(consumer / 'sdk_consumer')], driver_prefix=install_prefix, sdk=sdk)
        if mode == 'default-symlink':
            record['default_sdk_trace_sha256'] = hashlib.sha256(trace.encode()).hexdigest()
        else:
            assert hashlib.sha256(trace.encode()).hexdigest() == record['default_sdk_trace_sha256'], 'SDK geometry differs'
        assert str(ROOT / 'src/') not in (consumer / 'compile_commands.json').read_text()
        binaries = [install_prefix / 'livox_ros_driver2/lib/liblivox_ros_driver2.so',
                    install_prefix / 'realsense2_camera/lib/librealsense2_camera.so']
        for binary, library in zip(binaries + [consumer / 'sdk_consumer'],
                                   ['liblivox_lidar_sdk_shared.so', 'librealsense2.so.2.56', 'librealsense2.so.2.56']):
            libs = run(mode + '-' + binary.name + '-libraries', ['ldd', str(binary)], driver_prefix=install_prefix, sdk=sdk)
            assert 'not found' not in libs
            assert str(sdk / 'lib') in next(line for line in libs.splitlines() if library in line)
        run(mode + '-components-and-messages', [str(consumer / 'driver_consumer'), *map(str, binaries)],
            driver_prefix=install_prefix, sdk=sdk)
        record['modes'].append({'mode': mode, 'sdk_prefix': str(sdk), 'driver_prefix': str(install_prefix),
                               'sdk_geometry_frames': 512, 'fov_check': True, 'message_roundtrips': 2,
                               'actual_component_libraries_loaded': 2, 'device_nodes_constructed': 0,
                               'driver_cache_sdk_resolution_verified': True, 'consumer_no_source_include': True})
        save()
    for name, digest in snapshot['protected_installed_files'].items():
        assert sha(Path(name)) == digest
    for name, digest in snapshot['json_cache']['files'].items():
        assert sha(baseline / 'json-source' / name) == digest
    assert record['source_files'] == fingerprints(snapshot), 'sources changed during validation'
    assert subprocess.check_output(['git', 'status', '--porcelain'], cwd=NESTED, text=True) == snapshot['realsense_ros']['status']
    record['active_original_pids_still_present'] = {str(pid): Path('/proc', str(pid)).exists() for pid in snapshot['active_pids']}
    record['passed'] = True
    save()
    print('ALG-8.3 matrix passed:', out, flush=True)


if __name__ == '__main__':
    main()

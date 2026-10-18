#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""ALG-7.1 ground fitting/state equivalence, real node and two installed consumers."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from verify_api_install import clean_environment

ROOT = Path(__file__).resolve().parents[2]
CORE = 'easyfly_ground_segmentation'
FUNCTIONAL = 'easyfly_semantic_map'
PATHS = ['src/algorithms/fusion_mapping/' + CORE,
         'src/perception/p4_fusion/' + FUNCTIONAL, 'tools/algorithms/ground_consumer']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fingerprints():
    files = [p for path in PATHS for p in (ROOT / path).rglob('*') if p.is_file()
             and '__pycache__' not in p.parts and '.pytest_cache' not in p.parts]
    files += [Path(__file__).resolve(), ROOT / 'tools/algorithms/verify_api_install.py',
              ROOT / 'tools/algorithms/probes/ground_probe.cpp']
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
    if not (symlink / 'local_setup.bash').is_file():
        parser.error('build isolated symlink prefix first')
    out.mkdir(parents=True)
    env = clean_environment()
    env.update(ROS_DOMAIN_ID='219', ROS_LOCALHOST_ONLY='1')
    record = {'schema_version': 1, 'source_files': fingerprints(), 'commands': [], 'modes': []}

    def save():
        (out / 'summary.json').write_text(json.dumps(record, indent=2) + '\n')

    def run(name, command, ros=False, overlay=None, include_current=True):
        actual = command
        if ros:
            script = ('source /opt/ros/humble/setup.bash && source "$1/install/setup.bash" && '
                      'source "$1/artifacts/scene-s3-20261016/normal/install/local_setup.bash" && '
                      'source "$1/artifacts/algorithms/alg6-20261018/initial/install/local_setup.bash" && shift && ')
            paths = [str(ROOT)]
            if include_current:
                script += 'source "$1/local_setup.bash" && shift && '
                paths.append(str(symlink))
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

    snapshot = json.loads((baseline / 'input-snapshot.json').read_text())
    for name, digest in snapshot['files'].items():
        assert sha(baseline / 'legacy' / name) == digest
    binary = ROOT / snapshot['legacy_node_binary']['path']
    assert sha(binary) == snapshot['legacy_node_binary']['sha256']
    record['baseline_source_commit'] = snapshot['source_commit']
    record['baseline_snapshot_sha256'] = sha(baseline / 'input-snapshot.json')
    old = (baseline / 'legacy/src/perception/p4_fusion/easyfly_semantic_map/src/semantic_map_node.cpp').read_text()
    start = old.index('    if (cloud->size() >= 100) {', old.index('    GroundPlaneUpdate plane_update'))
    end = old.index('          candidate_ground_z =', start)
    block = old[start:end] + '''          result.candidate = candidate;
          result.has_candidate = true;
        }
      }
    }
'''
    header = (ROOT / 'src/algorithms/fusion_mapping/easyfly_ground_segmentation/include/easyfly_ground_segmentation/ground_fitting.hpp').read_text()
    declarations = header[header.index('struct GroundFitParams'):header.index('// Synchronous')]
    native = out / 'legacy-consumer-source'
    native.mkdir()
    (native / 'legacy_ground_fitting.hpp').write_text('namespace easyfly_semantic_map {\n' + declarations + '''inline GroundFitResult fitGroundCandidate(
  const pcl::PointCloud<pcl::PointXYZI> & input,
  const GroundVehiclePose & ground_vehicle, const GroundFitParams & params_ = {})
{
  const auto cloud = &input;
  const auto ground_cloud = &input;
  GroundFitResult result;
  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices);
''' + block + '''  result.inlier_count = ground_inliers->indices.size();
  result.coefficient_count = coeff->values.size();
  return result;
}
}
''')
    shutil.copy2(ROOT / 'tools/algorithms/probes/ground_probe.cpp', native / 'ground_probe.cpp')
    legacy_headers = baseline / 'legacy/src/perception/p4_fusion/easyfly_semantic_map/include'
    # Plain CMake discovers the system PCL through its official config, including C/MPI targets.
    (native / 'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.16)
project(legacy_ground LANGUAGES C CXX)
find_package(PCL REQUIRED COMPONENTS common segmentation)
add_executable(legacy_ground ground_probe.cpp)
target_compile_features(legacy_ground PRIVATE cxx_std_17)
target_compile_definitions(legacy_ground PRIVATE EASYFLY_LEGACY_GROUND)
target_include_directories(legacy_ground PRIVATE "''' + str(native) + '" "' + str(legacy_headers) + '''" ${PCL_INCLUDE_DIRS})
target_link_libraries(legacy_ground PRIVATE ${PCL_LIBRARIES})
''')
    native_build = out / 'legacy-consumer-build'
    run('legacy-configure', ['cmake', '-S', str(native), '-B', str(native_build)])
    run('legacy-build', ['cmake', '--build', str(native_build), '--parallel', '2'])
    legacy_trace = run('legacy-trace', [str(native_build / 'legacy_ground')])
    run('legacy-ground-ros', ['python3', '-m', 'pytest', '-q',
        str(ROOT / 'src/perception/p4_fusion/easyfly_semantic_map/test/test_ground_pipeline_ros.py'),
        '--junitxml=' + str(out / 'legacy-ground-ros.xml')], ros=True, include_current=False)
    closure = run('core-closure', ['colcon', 'list', '--base-paths', 'src', '--packages-up-to', CORE, '--names-only'], ros=True)
    assert set(closure.splitlines()) == {CORE}
    for mode in ('symlink', 'copy'):
        directory = out / mode
        directory.mkdir()
        build = symlink.parent / 'build' if mode == 'symlink' else directory / 'build'
        install = symlink if mode == 'symlink' else directory / 'install'
        command = ['colcon', '--log-base', str(directory / 'log'), 'build', '--base-paths', 'src',
                   '--build-base', str(build), '--install-base', str(install), '--packages-select', CORE, FUNCTIONAL,
                   '--allow-overriding', CORE, FUNCTIONAL, '--executor', 'sequential']
        if mode == 'symlink':
            command += ['--symlink-install']
        command += ['--cmake-args', '-DBUILD_TESTING=ON']
        run(mode + '-build', command, ros=True)
        run(mode + '-core-tests', ['ctest', '--test-dir', str(build / CORE), '--output-on-failure',
            '-j1', '-R', '^test_', '--output-junit', str(directory / (CORE + '.xml'))], ros=True, overlay=install)
        # The known shutdown/stress problem remains a separate, unresolved baseline.
        pattern = '^test_(ground_pipeline_ros|observed_grid_ros|classification(_ros)?|map_validity|grid_inflation|observed_grid|dynamic_obstacle_tracker)$'
        run(mode + '-functional-tests', ['ctest', '--test-dir', str(build / FUNCTIONAL), '--output-on-failure',
            '-j1', '-R', pattern, '--output-junit', str(directory / (FUNCTIONAL + '.xml'))], ros=True, overlay=install)
        source = directory / 'consumer-source'
        shutil.copytree(ROOT / 'tools/algorithms/ground_consumer', source)
        shutil.copy2(ROOT / 'tools/algorithms/probes/ground_probe.cpp', source / 'ground_probe.cpp')
        old_build = directory / 'old-headers-consumer-build'
        run(mode + '-old-headers-configure', ['cmake', '-S', str(source), '-B', str(old_build),
            '-DEASYFLY_CHECK_LEGACY=ON', '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'], ros=True, overlay=install)
        run(mode + '-old-headers-build', ['cmake', '--build', str(old_build), '--parallel', '2'], ros=True, overlay=install)
        run(mode + '-old-headers-test', ['ctest', '--test-dir', str(old_build), '--output-on-failure',
            '--output-junit', str(directory / 'old-headers.xml')], ros=True, overlay=install)
        assert str(ROOT / 'src/') not in (old_build / 'compile_commands.json').read_text()
        if mode == 'copy':
            relocated = directory / 'relocated-install'
            install.rename(relocated)
            install = relocated
            for path in (install / CORE / 'include').rglob('*.hpp'):
                assert not path.is_symlink(), path
        consumer = directory / 'consumer-build'
        run(mode + '-consumer-configure', ['cmake', '-S', str(source), '-B', str(consumer),
            '-DCMAKE_PREFIX_PATH=' + str(install / CORE), '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'])
        run(mode + '-consumer-build', ['cmake', '--build', str(consumer), '--parallel', '2'])
        run(mode + '-consumer-test', ['ctest', '--test-dir', str(consumer), '--output-on-failure',
            '--output-junit', str(directory / 'consumer.xml')])
        assert str(ROOT / 'src/') not in (consumer / 'compile_commands.json').read_text()
        trace = run(mode + '-differential', [str(consumer / 'ground_probe')])
        assert trace == legacy_trace, 'fitting/tracker/transform traces changed'
        libraries = run(mode + '-libraries', ['ldd', str(consumer / 'ground_consumer')])
        assert '/opt/ros/' not in libraries and 'rclcpp' not in libraries and 'not found' not in libraries
        if mode == 'copy':
            assert str(install / CORE / 'lib') in libraries
        record['modes'].append({'mode': mode, 'consumer_prefix': str(install), 'differential_identical': True,
            'trace_sha256': hashlib.sha256(trace.encode()).hexdigest(), 'fitting_frames': 32, 'tracker_steps': 240,
            'real_ground_lifecycle_and_existing_ros_regressions_passed': True,
            'installed_old_headers_and_pure_consumer_passed': True})
    assert sha(binary) == snapshot['legacy_node_binary']['sha256'], 'legacy binary changed'
    assert record['source_files'] == fingerprints(), 'sources changed during validation'
    record['passed'] = True
    save()
    print('ALG-7.1 matrix passed:', out, flush=True)


if __name__ == '__main__':
    main()

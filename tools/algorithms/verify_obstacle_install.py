#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""ALG-7.2 obstacle geometry/state equivalence, real node and two installed consumers."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from verify_api_install import clean_environment

ROOT = Path(__file__).resolve().parents[2]
CORE = 'easyfly_obstacle_geometry'
FUNCTIONAL = 'easyfly_semantic_map'
PATHS = ['src/algorithms/fusion_mapping/' + CORE,
         'src/perception/p4_fusion/' + FUNCTIONAL, 'tools/algorithms/obstacle_consumer']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fingerprints():
    files = [p for path in PATHS for p in (ROOT / path).rglob('*') if p.is_file()
             and '__pycache__' not in p.parts and '.pytest_cache' not in p.parts]
    files += [Path(__file__).resolve(), ROOT / 'tools/algorithms/verify_api_install.py',
              ROOT / 'tools/algorithms/probes/obstacle_probe.cpp']
    return {str(p.relative_to(ROOT)): sha(p) for p in sorted(files)}


def function(text, signature):
    start = text.index(signature)
    brace = text.index('{', start)
    level = 1
    end = brace + 1
    while level:
        level += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


def write_native_geometry(old, baseline, native):
    core = ROOT / 'src/algorithms/fusion_mapping' / CORE
    legacy = baseline / 'legacy/src/perception/p4_fusion/easyfly_semantic_map'
    # Moved tracker/test must be byte-identical after namespace/include substitutions.
    moved = (core / 'include' / CORE / 'dynamic_obstacle_tracker.hpp').read_text()
    assert moved.replace('EASYFLY_OBSTACLE_GEOMETRY__', 'EASYFLY_SEMANTIC_MAP__').replace(
        'namespace easyfly::obstacle_geometry', 'namespace easyfly_semantic_map') == (
            legacy / 'include/easyfly_semantic_map/dynamic_obstacle_tracker.hpp').read_text()
    test = (core / 'test/test_dynamic_obstacle_tracker.cpp').read_text()
    assert test.replace('easyfly_obstacle_geometry/', 'easyfly_semantic_map/').replace(
        'namespace easyfly::obstacle_geometry', 'namespace easyfly_semantic_map') == (
            legacy / 'test/test_dynamic_obstacle_tracker.cpp').read_text()
    header = (core / 'include' / CORE / 'clustering.hpp').read_text().replace(
        'easyfly_obstacle_geometry/dynamic_obstacle_tracker.hpp',
        'easyfly_semantic_map/dynamic_obstacle_tracker.hpp').replace(
        'namespace easyfly::obstacle_geometry', 'namespace easyfly_semantic_map')
    (native / 'legacy_clustering.hpp').write_text(header)
    observation = function(old, '  std::vector<DynamicObstacleObservation> extractCurrentDynamicObservations(')
    observation_body = observation[observation.index('{')+1:observation.rfind('}')].replace('params_.', 'params.')
    cluster_start = old.index('    std::vector<pcl::PointIndices> clusters;', old.index('// 欧式聚类 (当前有效帧)'))
    cluster_end = old.index('\n\n    if (params_.verbose)', cluster_start)
    cluster_body = old[cluster_start:cluster_end].replace('params_.', 'params.').replace('merged', 'cloud')
    output_start = old.index('      float x1 = 1e9f, y1 = 1e9f, z1 = 1e9f, x2 = -1e9f, y2 = -1e9f, z2 = -1e9f;')
    output_end = old.index('\n\n      ObstacleInfo o;', output_start)
    output = old[output_start:output_end]
    bounds_end = output.index('      const double sx')
    bounds = output[:bounds_end].replace('merged->points', 'cloud.points').replace('c.indices', 'cluster.indices')
    math = output[bounds_end:].replace('params_.max_obstacle_size', 'max_obstacle_size')
    math = math.replace('invalidateMap("ObstacleInfo 包框坐标/尺寸非法，清除 semantic_map");\n        return;',
                        'result.status = ClusterGeometryStatus::kInvalidGeometry;\n        return result;')
    math = math.replace('continue;                                                              // 噪声链',
                        'result.status = ClusterGeometryStatus::kTooLarge;\n        return result;')
    math = math.replace('continue;                                                              // 碎片',
                        'result.status = ClusterGeometryStatus::kFragment;\n        return result;')
    math = math.replace('invalidateMap("ObstacleInfo 包框 double→float 非有限/负尺寸，清除 semantic_map");\n        return;',
                        'result.status = ClusterGeometryStatus::kInvalidFloatBox;\n        return result;')
    code = '#include "legacy_clustering.hpp"\n#include <pcl/search/kdtree.h>\n#include <pcl/segmentation/extract_clusters.h>\nnamespace easyfly_semantic_map {\n'
    code += 'std::vector<DynamicObstacleObservation> extractCurrentDynamicObservations(\nconst pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud, const ClusteringParams & params) {' + observation_body + '}\n'
    code += 'std::vector<pcl::PointIndices> clusterCloud(\nconst pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud, const ClusteringParams & params) {\n' + cluster_body + '\nreturn clusters;\n}\n'
    code += 'std::array<float, 6> clusterBounds(\nconst pcl::PointCloud<pcl::PointXYZI> & cloud, const pcl::PointIndices & cluster) {\n' + bounds + '\nreturn {x1, y1, z1, x2, y2, z2};\n}\n'
    code += 'ClusterGeometry outputClusterGeometry(\nconst pcl::PointCloud<pcl::PointXYZI> & cloud, const pcl::PointIndices & cluster, const double max_obstacle_size) {\nClusterGeometry result;\n' + bounds + '\nresult.bounds = {x1, y1, z1, x2, y2, z2};\n' + math + '\nresult.fields = {center_x, center_y, center_z, size_x, size_y, size_z};\nreturn result;\n}\n}\n'
    (native / 'legacy_clustering.cpp').write_text(code)
    # Verify each numerical block independent of whitespace / comments.
    import re
    def normalized(text):
        return re.sub(r'\s+', '', re.sub(r'//[^\n]*', '', text))
    production = (core / 'src/clustering.cpp').read_text()
    for name in ('extractCurrentDynamicObservations(', 'clusterCloud(', 'clusterBounds('):
        a = function(production, name)
        b = function(code, name)
        assert normalized(a[a.index('{'):]) == normalized(b[b.index('{'):]), name
    assert normalized(math) in normalized(production), 'output arithmetic changed'
    # Unmoved policy/body remains exactly the same after replacing extraction sites.
    current = (ROOT / 'src/perception/p4_fusion/easyfly_semantic_map/src/semantic_map_node.cpp').read_text()
    old_remaining = old.replace(observation, '@@OBSERVATIONS@@')
    new_start = current.index('  easyfly::obstacle_geometry::ClusteringParams clusteringParams() const')
    new_end = current.index('\n  void onCloud(', new_start)
    current_remaining = current[:new_start] + '@@OBSERVATIONS@@\n' + current[new_end:]
    old_remaining = old_remaining.replace(old[cluster_start:cluster_end], '@@CLUSTERS@@')
    current_remaining = current_remaining.replace('    const auto clusters = easyfly::obstacle_geometry::clusterCloud(merged, clusteringParams());', '@@CLUSTERS@@')
    old_remaining = old_remaining.replace(output, '@@OUTPUT@@')
    start = current_remaining.index('      const auto geometry = easyfly::obstacle_geometry::outputClusterGeometry(')
    end = current_remaining.index('\n\n      ObstacleInfo o;', start)
    current_remaining = current_remaining[:start] + '@@OUTPUT@@' + current_remaining[end:]
    start = old_remaining.index('        float x1 = 1e9f, x2 = -1e9f;')
    end = old_remaining.index('        stat +=', start)
    old_remaining = old_remaining[:start] + '@@BOUNDS@@' + old_remaining[end:]
    start = current_remaining.index('        const auto b = easyfly::obstacle_geometry::clusterBounds(*merged, c);')
    end = current_remaining.index('        stat +=', start)
    current_remaining = current_remaining[:start] + '@@BOUNDS@@' + current_remaining[end:]
    current_remaining = current_remaining.replace('#include "easyfly_obstacle_geometry/clustering.hpp"\n', '')
    assert normalized(old_remaining) == normalized(current_remaining), 'unmoved node policy changed'


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
    env.update(ROS_DOMAIN_ID='220', ROS_LOCALHOST_ONLY='1')
    record = {'schema_version': 1, 'source_files': fingerprints(), 'commands': [], 'modes': []}

    def save():
        (out / 'summary.json').write_text(json.dumps(record, indent=2) + '\n')

    def run(name, command, ros=False, overlay=None, include_current=True):
        actual = command
        if ros:
            script = ('source /opt/ros/humble/setup.bash && source "$1/install/setup.bash" && '
                      'source "$1/artifacts/scene-s3-20261016/normal/install/local_setup.bash" && '
                      'source "$1/artifacts/algorithms/alg6-20261018/initial/install/local_setup.bash" && source "$1/artifacts/algorithms/alg7-1-20261018/initial/install/local_setup.bash" && shift && ')
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
    native = out / 'legacy-consumer-source'
    native.mkdir()
    write_native_geometry(old, baseline, native)
    shutil.copy2(ROOT / 'tools/algorithms/probes/obstacle_probe.cpp', native / 'obstacle_probe.cpp')
    legacy_headers = baseline / 'legacy/src/perception/p4_fusion/easyfly_semantic_map/include'
    (native / 'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.16)
project(legacy_obstacle LANGUAGES C CXX)
find_package(PCL REQUIRED COMPONENTS common segmentation)
add_executable(legacy_obstacle obstacle_probe.cpp legacy_clustering.cpp)
target_compile_features(legacy_obstacle PRIVATE cxx_std_17)
target_compile_definitions(legacy_obstacle PRIVATE EASYFLY_NATIVE_LEGACY)
target_include_directories(legacy_obstacle PRIVATE "''' + str(native) + '" "' + str(legacy_headers) + '''" ${PCL_INCLUDE_DIRS})
target_link_libraries(legacy_obstacle PRIVATE ${PCL_LIBRARIES})
''')
    native_build = out / 'legacy-consumer-build'
    run('legacy-configure', ['cmake', '-S', str(native), '-B', str(native_build)])
    run('legacy-build', ['cmake', '--build', str(native_build), '--parallel', '2'])
    legacy_trace = run('legacy-trace', [str(native_build / 'legacy_obstacle')])
    run('legacy-dynamic-ros', ['python3', '-m', 'pytest', '-q',
        str(ROOT / 'src/perception/p4_fusion/easyfly_semantic_map/test/test_dynamic_geometry_ros.py'),
        '--junitxml=' + str(out / 'legacy-dynamic-ros.xml')], ros=True, include_current=False)
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
        pattern = '^test_(dynamic_geometry_ros|ground_pipeline_ros|observed_grid_ros|classification(_ros)?|map_validity|grid_inflation|observed_grid)$'
        run(mode + '-functional-tests', ['ctest', '--test-dir', str(build / FUNCTIONAL), '--output-on-failure',
            '-j1', '-R', pattern, '--output-junit', str(directory / (FUNCTIONAL + '.xml'))], ros=True, overlay=install)
        source = directory / 'consumer-source'
        shutil.copytree(ROOT / 'tools/algorithms/obstacle_consumer', source)
        shutil.copy2(ROOT / 'tools/algorithms/probes/obstacle_probe.cpp', source / 'obstacle_probe.cpp')
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
        trace = run(mode + '-differential', [str(consumer / 'obstacle_probe')])
        assert trace == legacy_trace, 'clustering/tracker/sweep traces changed'
        libraries = run(mode + '-libraries', ['ldd', str(consumer / 'obstacle_probe')])
        assert '/opt/ros/' not in libraries and 'rclcpp' not in libraries and 'not found' not in libraries
        if mode == 'copy':
            assert str(install / CORE / 'lib') in libraries
        record['modes'].append({'mode': mode, 'consumer_prefix': str(install), 'differential_identical': True,
            'trace_sha256': hashlib.sha256(trace.encode()).hexdigest(), 'pcl_frames': 80, 'tracker_steps': 680,
            'real_dynamic_geometry_and_existing_ros_regressions_passed': True,
            'installed_old_headers_and_pure_consumer_passed': True})
    assert sha(binary) == snapshot['legacy_node_binary']['sha256'], 'legacy binary changed'
    assert record['source_files'] == fingerprints(), 'sources changed during validation'
    record['passed'] = True
    save()
    print('ALG-7.2 matrix passed:', out, flush=True)


if __name__ == '__main__':
    main()

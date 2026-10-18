#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""ALG-7.3 occupancy/state equivalence, real node and two installed consumers."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from verify_api_install import clean_environment

ROOT = Path(__file__).resolve().parents[2]
CORE = 'easyfly_occupancy_mapping'
FUNCTIONAL = 'easyfly_semantic_map'
PATHS = ['src/algorithms/fusion_mapping/' + CORE,
         'src/perception/p4_fusion/' + FUNCTIONAL, 'tools/algorithms/occupancy_consumer']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fingerprints():
    files = [p for path in PATHS for p in (ROOT / path).rglob('*') if p.is_file()
             and '__pycache__' not in p.parts and '.pytest_cache' not in p.parts]
    files += [Path(__file__).resolve(), ROOT / 'tools/algorithms/verify_api_install.py',
              ROOT / 'tools/algorithms/probes/occupancy_probe.cpp']
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


def write_native_occupancy(old, baseline, native):
    import re
    def normalized(text):
        return re.sub(r'\s+', '', re.sub(r'//[^\n]*', '', text))
    core = ROOT / 'src/algorithms/fusion_mapping' / CORE
    legacy = baseline / 'legacy/src/perception/p4_fusion/easyfly_semantic_map'
    headers = core / 'include' / CORE
    inflation = (headers / 'grid_inflation.hpp').read_text()
    assert inflation.replace('namespace easyfly::occupancy', 'namespace easyfly_semantic_map') == (
        legacy / 'include/easyfly_semantic_map/grid_inflation.hpp').read_text()
    a = function((headers / 'observed_grid.hpp').read_text(), 'bool rasterizeObservedFootprints(')
    b = function((legacy / 'include/easyfly_semantic_map/observed_grid.hpp').read_text(), 'bool rasterizeObservedFootprints(')
    assert a[a.index('{'):] == b[b.index('{'):], 'observed rasterization body changed'
    for name in ('grid_inflation', 'observed_grid'):
        test = (core / f'test/test_{name}.cpp').read_text().replace(
            'easyfly_occupancy_mapping/' + name + '.hpp', 'easyfly_semantic_map/' + name + '.hpp').replace(
            'namespace easyfly::occupancy', 'namespace easyfly_semantic_map').replace(
            'easyfly::occupancy::rasterizeObservedFootprints', 'easyfly_semantic_map::rasterizeObservedFootprints').replace(
            'easyfly::ground::GroundPlaneModel', 'easyfly_semantic_map::GroundPlaneModel').replace(
            '#include "easyfly_ground_segmentation/ground_plane_tracker.hpp"\n', '')
        assert test == (legacy / f'test/test_{name}.cpp').read_text(), name
    start = old.index('    for (const auto & b : boxes) {', old.index('  void publishGrid('))
    end = old.index('\n\n    // 持久化融合', start)
    old_box = old[start:end]
    box = old_box.replace('grid.info.width', 'geometry.width').replace('grid.info.height', 'geometry.height').replace(
        'grid.info.origin.position.x', 'geometry.origin_x').replace('grid.info.origin.position.y', 'geometry.origin_y').replace(
        'params_.grid_resolution', 'geometry.resolution').replace('grid.data', 'output')
    start = old.index('    // 持久化融合', old.index('  void publishGrid('))
    end = old.index('    grid_pub_->publish(grid);', start)
    old_persistence = old[start:end]
    persistence = old_persistence.replace('grid.data', 'output').replace('params_.grid_persist_frames', 'persist_frames')
    start = old.index('      constexpr auto footprint = DynamicObstacleTracker::kPredictionFootprintPoints;')
    end = old.index('      publishGrid(arr.header, dynamic_boxes', start)
    old_prediction = old[start:end]
    prediction = old_prediction[old_prediction.index('      for (std::size_t i'):].replace(
        'merged_nonground->points[', 'points[').replace('merged_nonground->size()', 'points.size()')
    production = (headers / 'occupancy_grid.hpp').read_text()
    geometry = function(production, 'inline GridGeometry makeGridGeometry(')
    expected_geometry = '''{
      return {static_cast<uint32_t>(width_m / resolution),
        static_cast<uint32_t>(height_m / resolution), resolution, -width_m / 2.0, -height_m / 2.0};
    }'''
    assert normalized(geometry[geometry.index('{'):]) == normalized(expected_geometry)
    for block in (box, persistence, prediction):
        assert normalized(block) in normalized(production), 'moved numerical block changed'
    # Build original numerical paths independently from the saved callback.
    code = '''#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>
namespace easyfly_semantic_map {
struct GridGeometry {
  std::uint32_t width{0}, height{0};
  double resolution{0.0}, origin_x{0.0}, origin_y{0.0};
};
''' + 'inline GridGeometry makeGridGeometry(double width_m, double height_m, double resolution) ' + expected_geometry + '\n'
    code += 'inline void rasterizeClusterBoxes(const std::vector<std::array<double, 6>> & boxes, const GridGeometry & geometry, double inf, std::vector<int8_t> & output) {\n' + box + '\n}\n'
    code += 'template<typename Points> void appendPredictionFootprintBoxes(const Points & points, std::size_t published_prediction_points, std::size_t footprint, std::vector<std::array<double, 6>> & dynamic_boxes) {\n' + prediction + '\n}\n'
    code += 'class GridPersistence { public: void reset() {grid_persist_.clear(); grid_miss_.clear();} void apply(std::vector<int8_t> & output, int persist_frames) {const size_t n = output.size();\n' + persistence + '\n} private: std::vector<int8_t> grid_persist_; std::vector<uint32_t> grid_miss_; };\n}\n'
    (native / 'legacy_occupancy_grid.hpp').write_text(code)
    # Restore all extracted call sites; ROS policy/body must remain equivalent.
    current = (ROOT / 'src/perception/p4_fusion/easyfly_semantic_map/src/semantic_map_node.cpp').read_text()
    current = current.replace('#include "easyfly_occupancy_mapping/grid_inflation.hpp"', '#include "easyfly_semantic_map/grid_inflation.hpp"').replace(
        '#include "easyfly_occupancy_mapping/observed_grid.hpp"', '#include "easyfly_semantic_map/observed_grid.hpp"').replace(
        '#include "easyfly_occupancy_mapping/occupancy_grid.hpp"\n', '')
    current = re.sub(r'^using easyfly::occupancy::\w+;\n', '', current, flags=re.M)
    current = current.replace('    grid_history_.reset();', '    grid_persist_.clear();\n    grid_miss_.clear();').replace(
        '  easyfly::occupancy::GridPersistence grid_history_;', '  std::vector<int8_t> grid_persist_;\n  std::vector<uint32_t> grid_miss_;')
    start = current.index('      easyfly::occupancy::appendPredictionFootprintBoxes(')
    end = current.index('      publishGrid(arr.header, dynamic_boxes', start)
    current = current[:start] + old_prediction + current[end:]
    start = old.index('    grid.info.resolution = params_.grid_resolution;', old.index('  void publishGrid('))
    end = old.index('    const size_t n', start)
    old_geometry = old[start:end]
    start = current.index('    const auto geometry = easyfly::occupancy::makeGridGeometry(')
    end = current.index('    const size_t n', start)
    current = current[:start] + old_geometry + current[end:]
    current = current.replace('    easyfly::occupancy::rasterizeClusterBoxes(boxes, geometry, inf, grid.data);', old_box).replace(
        '    grid_history_.apply(grid.data, params_.grid_persist_frames);\n', old_persistence)
    assert normalized(current) == normalized(old), 'unmoved node policy/body changed'


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
    env.update(ROS_DOMAIN_ID='221', ROS_LOCALHOST_ONLY='1')
    record = {'schema_version': 1, 'source_files': fingerprints(), 'commands': [], 'modes': []}

    def save():
        (out / 'summary.json').write_text(json.dumps(record, indent=2) + '\n')

    def run(name, command, ros=False, overlay=None, include_current=True):
        actual = command
        if ros:
            script = ('source /opt/ros/humble/setup.bash && source "$1/install/setup.bash" && '
                      'source "$1/artifacts/scene-s3-20261016/normal/install/local_setup.bash" && '
                      'source "$1/artifacts/algorithms/alg6-20261018/initial/install/local_setup.bash" && source "$1/artifacts/algorithms/alg7-1-20261018/initial/install/local_setup.bash" && source "$1/artifacts/algorithms/alg7-2-20261018/initial/install/local_setup.bash" && shift && ')
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
    write_native_occupancy(old, baseline, native)
    shutil.copy2(ROOT / 'tools/algorithms/probes/occupancy_probe.cpp', native / 'occupancy_probe.cpp')
    legacy_headers = baseline / 'legacy/src/perception/p4_fusion/easyfly_semantic_map/include'
    ground_headers = ROOT / 'artifacts/algorithms/alg7-1-20261018/initial/install/easyfly_ground_segmentation/include'
    (native / 'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.16)
project(legacy_occupancy LANGUAGES CXX)
add_executable(legacy_occupancy occupancy_probe.cpp)
target_compile_features(legacy_occupancy PRIVATE cxx_std_17)
target_compile_definitions(legacy_occupancy PRIVATE EASYFLY_NATIVE_LEGACY)
target_include_directories(legacy_occupancy PRIVATE "''' + str(native) + '" "' + str(legacy_headers) + '" "' + str(ground_headers) + '''" )
''')
    native_build = out / 'legacy-consumer-build'
    run('legacy-configure', ['cmake', '-S', str(native), '-B', str(native_build)])
    run('legacy-build', ['cmake', '--build', str(native_build), '--parallel', '2'])
    legacy_trace = run('legacy-trace', [str(native_build / 'legacy_occupancy')])
    run('legacy-persistence-ros', ['python3', '-m', 'pytest', '-q',
        str(ROOT / 'src/perception/p4_fusion/easyfly_semantic_map/test/test_occupancy_persistence_ros.py'),
        '--junitxml=' + str(out / 'legacy-persistence-ros.xml')], ros=True, include_current=False)
    closure = run('core-closure', ['colcon', 'list', '--base-paths', 'src', '--packages-up-to', CORE, '--names-only'], ros=True)
    assert set(closure.splitlines()) == {CORE, 'easyfly_ground_segmentation'}  # test-only ground dependency
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
        pattern = '^test_(occupancy_persistence_ros|dynamic_geometry_ros|ground_pipeline_ros|observed_grid_ros|classification(_ros)?|map_validity)$'
        run(mode + '-functional-tests', ['ctest', '--test-dir', str(build / FUNCTIONAL), '--output-on-failure',
            '-j1', '-R', pattern, '--output-junit', str(directory / (FUNCTIONAL + '.xml'))], ros=True, overlay=install)
        source = directory / 'consumer-source'
        shutil.copytree(ROOT / 'tools/algorithms/occupancy_consumer', source)
        shutil.copy2(ROOT / 'tools/algorithms/probes/occupancy_probe.cpp', source / 'occupancy_probe.cpp')
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
        trace = run(mode + '-differential', [str(consumer / 'occupancy_probe')])
        assert trace == legacy_trace, 'occupancy/projection/persistence traces changed'
        libraries = run(mode + '-libraries', ['ldd', str(consumer / 'occupancy_probe')])
        assert '/opt/ros/' not in libraries and 'rclcpp' not in libraries and 'libpcl' not in libraries and 'not found' not in libraries
        record['modes'].append({'mode': mode, 'consumer_prefix': str(install), 'differential_identical': True,
            'trace_sha256': hashlib.sha256(trace.encode()).hexdigest(), 'grid_frames': 540,
            'real_occupancy_persistence_and_existing_ros_regressions_passed': True,
            'installed_old_headers_and_pure_consumer_passed': True})
    assert sha(binary) == snapshot['legacy_node_binary']['sha256'], 'legacy binary changed'
    assert record['source_files'] == fingerprints(), 'sources changed during validation'
    record['passed'] = True
    save()
    print('ALG-7.3 matrix passed:', out, flush=True)


if __name__ == '__main__':
    main()

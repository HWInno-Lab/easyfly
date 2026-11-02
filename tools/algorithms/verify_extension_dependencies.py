#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""EXT-06: fresh minimal builds, dependency failures and installed consumers.

Only /opt/ros/humble is sourced for builds. Never sources workspace overlays,
downloads SDKs, removes host dependencies or starts ROS nodes.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET

from verify_api_install import clean_environment

ROOT = Path(__file__).resolve().parents[2]
LIGHT = ['easyfly_tracking_api', 'easyfly_sort', 'easyfly_track_id_fusion',
         'easyfly_grid_planner_api', 'easyfly_astar', 'easyfly_coverage_planning']
FACTORIES = ['easyfly_multi_object_tracking', 'easyfly_global_planner',
             'easyfly_object_detection', 'easyfly_single_object_tracking']
BUILD_TAGS = {'depend', 'build_depend', 'build_export_depend', 'buildtool_depend',
              'buildtool_export_depend'}
RUN_TAGS = {'depend', 'exec_depend'}


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ort-root', type=Path, required=True)
    parser.add_argument('--reuse-symlink-from', type=Path,
                        help='reuse a completed fresh symlink build after strictly checking source/SDK hashes; consumer checks remain fresh')
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    packages = {}
    discovery = subprocess.check_output(['/usr/bin/colcon', 'list', '--base-paths',
                                         str(ROOT/'src'), '--paths-only'], text=True,
                                        env={**clean_environment(), 'PYTHONNOUSERSITE': '1'})
    (out/'discovery.txt').write_text(discovery)
    for directory in discovery.splitlines():
        p = Path(directory)/'package.xml'
        if not p.exists():
            continue  # Plain CMake SDKs have no ROS dependency metadata.
        xml = ET.parse(p).getroot()
        name = xml.findtext('name')
        if name in packages:
            raise RuntimeError('duplicate package: ' + name)
        packages[name] = (p.parent, xml)

    def closure(seeds, tags):
        internal, external = set(), set()
        todo = list(seeds)
        while todo:
            name = todo.pop()
            if name in internal or name in external:
                continue
            if name not in packages:
                external.add(name)
                continue
            internal.add(name)
            todo.extend(e.text.strip() for e in packages[name][1] if e.tag in tags)
        return {'workspace': sorted(internal), 'external': sorted(external)}

    registry = json.loads((ROOT / 'src/easyfly_common/easyfly_backend_registry/'
                                'easyfly_backend_registry/data/backends.json').read_text())
    inventory = []
    for backend in registry['backends']:
        seed = backend['entrypoint']['package']
        inventory.append({'id': backend['id'], 'capability': backend['capability'],
                          'entrypoint': backend['entrypoint'],
                          'build': closure([seed], BUILD_TAGS),
                          'runtime': closure([seed], RUN_TAGS),
                          'runtime_contract': backend['runtime']})
    (out / 'dependency-inventory.json').write_text(json.dumps(inventory, indent=2) + '\n')
    names = sorted(set(LIGHT) | set(closure(FACTORIES, BUILD_TAGS)['workspace']))
    record = {'schema_version': 1, 'commands': [], 'modes': [], 'build_packages': names,
              'scope': __doc__, 'declared_build': closure(FACTORIES, BUILD_TAGS)}
    files = [p for n in names for p in packages[n][0].rglob('*')
             if p.is_file() and '__pycache__' not in p.parts and '.pytest_cache' not in p.parts]
    files += [Path(__file__).resolve()]
    record['source_files'] = {str(p.relative_to(ROOT)): sha(p) for p in sorted(files)}
    env = clean_environment()
    env.update(PATH='/usr/bin:/bin', PYTHONNOUSERSITE='1', CMAKE_BUILD_PARALLEL_LEVEL='2',
               MAKEFLAGS='-j2', QT_QPA_PLATFORM='offscreen')
    for key in ['ORT_ROOT', 'OpenCV_DIR', 'Qt5_DIR', 'realsense2_DIR', 'PKG_CONFIG_PATH',
                'LIBRARY_PATH', 'CMAKE_MODULE_PATH']:
        env.pop(key, None)

    def save():
        (out / 'summary.json').write_text(json.dumps(record, indent=2) + '\n')

    def run(label, command, ros=False, expected=0):
        actual = list(map(str, command))
        if ros:
            actual = ['bash', '--noprofile', '--norc', '-c',
                      'source /opt/ros/humble/setup.bash && exec "$@"', 'bash', *actual]
        print(label, flush=True)
        log = out / (label + '.log')
        with log.open('w') as stream:
            result = subprocess.run(actual, cwd=out, env=env, stdout=stream,
                                    stderr=subprocess.STDOUT)
        record['commands'].append({'name': label, 'command': actual,
                                   'returncode': result.returncode, 'expected': expected,
                                   'log_sha256': sha(log)})
        save()
        if (expected == 0 and result.returncode != 0) or (expected != 0 and result.returncode == 0):
            raise RuntimeError(f'{label}: unexpected return code; evidence retained in {log}')
        return log.read_text()

    sdk = out / 'external-ort-sdk'
    shutil.copytree(args.ort_root.resolve(), sdk, symlinks=True)
    record['sdk_files'] = {str(p.relative_to(sdk)): {'sha256': sha(p),
                           'symlink': os.readlink(p) if p.is_symlink() else None}
                           for p in sorted(sdk.rglob('*')) if p.is_file()}
    record['model'] = {'path': 'models/yolo11n.onnx', 'sha256': sha(ROOT/'models/yolo11n.onnx')}
    new_sdk = sdk
    for mode in ['symlink', 'copy']:
        directory = out / mode
        build, install = directory/'build', directory/'install'
        sdk = new_sdk
        reused = mode == 'symlink' and args.reuse_symlink_from is not None
        if reused:
            previous = args.reuse_symlink_from.resolve()
            prior = json.loads((previous/'summary.json').read_text())
            verifier = str(Path(__file__).relative_to(ROOT))
            assert {k: v for k, v in prior['source_files'].items() if k != verifier} == {
                k: v for k, v in record['source_files'].items() if k != verifier}, 'build inputs changed'
            required = ['symlink-light-build', 'symlink-factory-build',
                        *['symlink-'+n+'-tests' for n in LIGHT]]
            checks = [c for c in prior['commands'] if c['name'] in required]
            assert {c['name'] for c in checks} == set(required)
            assert all(c['returncode'] == 0 for c in checks)
            assert prior['sdk_files'] == record['sdk_files'], 'SDK inputs changed'
            build, install = previous/'symlink/build', previous/'symlink/install'
            sdk = previous/'external-ort-sdk'
            assert all(sha(sdk/p) == item['sha256'] for p, item in record['sdk_files'].items())
            directory.mkdir(parents=True)
            for name in LIGHT:
                shutil.copy2(previous/'symlink'/(name+'.xml'), directory/(name+'.xml'))
            record['reused_symlink'] = {'from': str(previous), 'source_and_sdk_unchanged': True,
                                        'build_commands': checks,
                                        'verification_tool_change_only': True}
        common = ['colcon', '--log-base', directory/'log', 'build', '--base-paths',
                  *[packages[n][0] for n in names], '--build-base', build,
                  '--install-base', install, '--executor', 'sequential']
        if mode == 'symlink':
            common += ['--symlink-install']
        if not reused:
            run(mode+'-light-build', [*common, '--packages-select', *LIGHT, '--cmake-args',
            '-DBUILD_TESTING=ON', '-DCMAKE_IGNORE_PREFIX_PATH=/usr/local',
            '-DCMAKE_DISABLE_FIND_PACKAGE_OpenCV=TRUE', '-DCMAKE_DISABLE_FIND_PACKAGE_Qt5=TRUE',
            '-DORT_ROOT='+str(out/'absent-sdk')], ros=True)
            for name in LIGHT:
                run(mode+'-'+name+'-tests', ['ctest', '--test-dir', build/name,
                '--output-on-failure', '--output-junit', directory/(name+'.xml')], ros=True)
            run(mode+'-factory-build', [*common, '--packages-select', *[n for n in names if n not in LIGHT],
            '--cmake-args', '-DBUILD_TESTING=OFF', '-DCMAKE_IGNORE_PREFIX_PATH=/usr/local',
            '-DORT_ROOT='+str(sdk), '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'], ros=True)
        # Prove explicit selection of an unavailable SDK fails at configuration.
        missing = run(mode+'-missing-ort', ['cmake', '-S', packages['easyfly_yolo_onnx'][0],
            '-B', directory/'missing-ort', '-DBUILD_TESTING=OFF',
            '-DCMAKE_PREFIX_PATH='+str(install/'easyfly_detection_api'),
            '-DORT_ROOT='+str(out/'absent-sdk')], ros=True, expected=1)
        assert 'ONNX Runtime SDK not found at ORT_ROOT=' in ' '.join(missing.split())
        if mode == 'copy':
            relocated = directory/'relocated-install'
            install.rename(relocated)
            install = relocated
            assert not any(p.is_symlink() for n in names
                           for p in (install/n/'include').rglob('*.hpp'))
        # Implementation headers/configs are not made visible to generic consumers.
        public = ['easyfly_backend_registry', 'easyfly_tracking_api', 'easyfly_grid_planner_api',
                  'easyfly_detection_api', 'easyfly_single_tracking_api', *FACTORIES]
        source = directory/'consumer-source'
        shutil.copytree(ROOT/'tools/algorithms/backend_consumer', source)
        consumer = directory/'consumer-build'
        run(mode+'-consumer-configure', ['cmake', '-S', source, '-B', consumer,
            '-DCMAKE_PREFIX_PATH='+';'.join(str(install/n) for n in public),
            '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON', '-DCMAKE_IGNORE_PREFIX_PATH=/usr/local',
            '-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF', '-DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF',
            '-DCMAKE_DISABLE_FIND_PACKAGE_easyfly_sort=TRUE',
            '-DCMAKE_DISABLE_FIND_PACKAGE_easyfly_track_id_fusion=TRUE',
            '-DCMAKE_DISABLE_FIND_PACKAGE_easyfly_astar=TRUE',
            '-DCMAKE_DISABLE_FIND_PACKAGE_easyfly_csrt=TRUE',
            '-DCMAKE_DISABLE_FIND_PACKAGE_easyfly_kcf=TRUE',
            '-DCMAKE_DISABLE_FIND_PACKAGE_easyfly_yolo_onnx=TRUE'])
        run(mode+'-consumer-build', ['cmake', '--build', consumer, '--parallel', '2'])
        model = install/'easyfly_object_detection/share/easyfly_object_detection/models/yolo11n.onnx'
        assert sha(model) == record['model']['sha256']
        # Symlink installs intentionally reference build/source inputs. Only the
        # copy/relocation mode promises independence from the build-time SDK.
        if mode == 'copy':
            sdk.rename(out/'external-ort-sdk-unavailable')
        try:
            run(mode+'-consumer-run', [consumer/'backend_consumer', model])
            loaded = run(mode+'-consumer-ldd', ['ldd', consumer/'backend_consumer'])
        finally:
            if mode == 'copy':
                (out/'external-ort-sdk-unavailable').rename(sdk)
        assert not any(p in loaded for p in ['/opt/ros/', 'libpython', 'not found', str(sdk)])
        runtime_paths = [Path(p).resolve() for p in re.findall(r'=> (\S+)', loaded) if p.startswith('/')]
        assert (install/'easyfly_yolo_onnx/lib/libonnxruntime.so').resolve() in runtime_paths
        compiles = (consumer/'compile_commands.json').read_text()
        assert str(ROOT/'src') not in compiles and str(sdk) not in compiles
        for name in ['easyfly_sort', 'easyfly_astar', 'easyfly_track_id_fusion', 'easyfly_yolo_onnx', 'easyfly_csrt', 'easyfly_kcf']:
            assert str(install/name/'include') not in compiles
        # Compiler dependency files also show real resolved include paths.
        deps = '\n'.join(p.read_text() for p in consumer.rglob('*.o.d'))
        tracker_header = install/'easyfly_tracking_api/include/easyfly_tracking_api/tracker.hpp'
        assert str(tracker_header.resolve()) in deps
        if mode == 'copy':
            assert str(ROOT/'src') not in deps
        sdk_deps = '\n'.join(p.read_text() for p in (build/'easyfly_yolo_onnx').rglob('*.o.d'))
        assert str(sdk/'include/onnxruntime_cxx_api.h') in sdk_deps
        assert str(ROOT/'src/third_party/runtimes/onnxruntime/include') not in sdk_deps
        exported = '\n'.join(p.read_text() for n in public for p in (install/n/'share'/n/'cmake').glob('*.cmake'))
        assert str(ROOT/'src') not in exported and str(sdk) not in exported
        for label in ['tracking', 'planning']:
            legacy_source = ROOT/'tools/algorithms'/f'{label}_consumer'
            legacy_build = directory/(label+'-legacy-build')
            run(mode+'-'+label+'-legacy-configure', ['cmake', '-S', legacy_source, '-B', legacy_build,
                '-DCMAKE_PREFIX_PATH='+';'.join(str(install/n) for n in names)])
            run(mode+'-'+label+'-legacy-build', ['cmake', '--build', legacy_build, '--parallel', '2'])
            run(mode+'-'+label+'-legacy-run', ['ctest', '--test-dir', legacy_build, '--output-on-failure'])
        record['modes'].append({'mode': mode, 'consumer_prefix': str(install),
                                'public_packages': public, 'runtime_paths': list(map(str, runtime_paths)),
                                'passed': True})
        save()
    assert record['source_files'] == {str(p.relative_to(ROOT)): sha(p) for p in sorted(files)}
    record['passed'] = True
    save()


if __name__ == '__main__':
    main()

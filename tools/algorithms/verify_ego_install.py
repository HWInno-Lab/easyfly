#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""ALG-8.2: classify the complete EGO tree and verify installed ROS consumers."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

from verify_api_install import clean_environment

ROOT = Path(__file__).resolve().parents[2]
OLD = 'src/third_party/planners/ego_planner'
NEW = 'src/third_party/algorithms/planning/ego_planner'
ADAPTER = 'src/execution/e2_planning/easyfly_local_planner'
PACKAGES = ['plan_env', 'path_searching', 'traj_utils_so', 'traj_utils',
            'quadrotor_msgs', 'bspline_opt', 'ego_planner', 'ego_to_mavros',
            'easyfly_local_planner']
CHANGED = {'bspline_opt/CMakeLists.txt', 'path_searching/CMakeLists.txt',
           'traj_utils_so/CMakeLists.txt', 'plan_manage/CMakeLists.txt',
           'plan_manage/test/test_ego_signal_shutdown.py',
           'plan_manage/test/probe_ego_shutdown.py',
           'plan_manage/test/probe_ego_active_shutdown.py'}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fingerprints(snapshot):
    files = [ROOT / name.replace(OLD, NEW, 1) for name in snapshot['files']]
    files += list((ROOT / 'tools/algorithms/ego_consumer').glob('*'))
    files += [Path(__file__).resolve(), ROOT / 'tools/algorithms/verify_api_install.py',
              ROOT / '.gitignore', ROOT / 'src/third_party/README.md',
              ROOT / 'src/third_party/algorithms/planning/README.md',
              ROOT / 'src/third_party/algorithms/planning/ego_planner.provenance.json']
    return {str(p.relative_to(ROOT)): sha(p) for p in sorted(files) if p.is_file()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--symlink-prefix', required=True, type=Path)
    parser.add_argument('--baseline', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    out, symlink, baseline = args.output.resolve(), args.symlink_prefix.resolve(), args.baseline.resolve()
    if out.exists():
        parser.error('preserve prior evidence; output exists')
    snapshot = json.loads((baseline / 'input-snapshot.json').read_text())
    provenance = json.loads((ROOT / 'src/third_party/algorithms/planning/ego_planner.provenance.json').read_text())
    changed = set()
    for name, digest in snapshot['files'].items():
        assert sha(baseline / 'legacy' / name) == digest, name
        current = ROOT / name.replace(OLD, NEW, 1)
        if name.startswith(OLD + '/'):
            relative = name[len(OLD) + 1:]
            assert provenance['tracked_vendor_files'][relative] == {
                'before_sha256': digest, 'current_sha256': sha(current)}, relative
            if sha(current) != digest:
                changed.add(relative)
        elif name.endswith('/CMakeLists.txt'):
            assert current.read_text() == (baseline / 'legacy' / name).read_text().replace(OLD, NEW)
        else:
            assert sha(current) == digest, name
    assert changed == CHANGED, changed
    assert (ROOT / OLD).is_symlink() and (ROOT / OLD).resolve() == ROOT / NEW
    assert (ROOT / OLD).parent.joinpath('COLCON_IGNORE').is_file()
    for name, digest in snapshot['underlay_binaries'].items():
        assert sha(ROOT / name) == digest
    out.mkdir(parents=True)
    env = clean_environment()
    env.update(ROS_DOMAIN_ID='224', ROS_LOCALHOST_ONLY='1')
    record = {'schema_version': 1, 'source_commit': snapshot['source_commit'],
              'source_files': fingerprints(snapshot), 'commands': [], 'modes': [],
              'vendor_files': snapshot['vendor_tracked_count'], 'vendor_changed_files': sorted(changed),
              'legacy_trace_sha256': sha(baseline / 'legacy-spline-trace.log')}

    def save():
        (out / 'summary.json').write_text(json.dumps(record, indent=2) + '\n')

    def run(name, command, prefix=None):
        # Source only existing support underlays. Explicit package roots take precedence
        # without relying on generated colcon setup scripts remaining relocatable.
        script = ('source /opt/ros/humble/setup.bash && source "$1/install/setup.bash" && '
                  'source "$1/artifacts/scene-s3-20261016/normal/install/local_setup.bash" && '
                  'source "$1/artifacts/algorithms/alg4-20261018/initial/install/easyfly_pointcloud_filters/share/easyfly_pointcloud_filters/local_setup.bash" && '
                  'source "$1/artifacts/algorithms/alg5-20261018/initial/install/easyfly_tag_pose/share/easyfly_tag_pose/local_setup.bash" && shift && ')
        paths = [str(ROOT)]
        if prefix:
            roots = ':'.join(str(prefix / p) for p in PACKAGES)
            libraries = ':'.join(str(prefix / p / 'lib') for p in PACKAGES)
            python = ':'.join(str(prefix / p / 'local/lib/python3.10/dist-packages') + ':' +
                              str(prefix / p / 'lib/python3.10/site-packages') for p in PACKAGES)
            script += ('export AMENT_PREFIX_PATH="$1:$AMENT_PREFIX_PATH" CMAKE_PREFIX_PATH="$1:$CMAKE_PREFIX_PATH" && '
                       'export LD_LIBRARY_PATH="$2:$LD_LIBRARY_PATH" PYTHONPATH="$3:$PYTHONPATH" && shift 3 && ')
            paths += [roots, libraries, python]
        actual = ['bash', '--noprofile', '--norc', '-c', script + 'exec "$@"', 'bash', *paths, *command]
        print(name, flush=True)
        log = out / (name + '.log')
        with log.open('w') as stream:
            code = subprocess.call(actual, cwd=ROOT, env=env, stdout=stream, stderr=subprocess.STDOUT)
        record['commands'].append({'name': name, 'command': actual, 'returncode': code, 'log_sha256': sha(log)})
        save()
        if code:
            raise RuntimeError(f'{name} failed ({code}); see {log}')
        return log.read_text()

    discovery = run('discovery', ['colcon', 'list', '--base-paths', 'src'])
    before = [line.split() for line in snapshot['colcon_before'].splitlines() if line.strip()]
    after = [line.split() for line in discovery.splitlines() if line.strip()]
    assert len(before) == len(after) == 91
    assert [[part.replace(OLD, NEW) for part in row] for row in before] == after
    assert not run('legacy-root-discovery', ['colcon', 'list', '--base-paths', str((ROOT / OLD).parent)]).strip()
    for mode in ('symlink', 'copy'):
        directory = out / mode
        directory.mkdir()
        build = symlink.parent / 'build' if mode == 'symlink' else directory / 'build'
        install = symlink if mode == 'symlink' else directory / 'install'
        command = ['colcon', '--log-base', str(directory / 'log'), 'build', '--base-paths', 'src',
                   '--build-base', str(build), '--install-base', str(install),
                   '--packages-select', *PACKAGES, '--allow-overriding', *PACKAGES, '--executor', 'sequential']
        if mode == 'symlink':
            command += ['--symlink-install']
        command += ['--cmake-args', '-DBUILD_TESTING=ON']
        run(mode + '-build', command)
        original_install = install
        if mode == 'copy':
            install = directory / 'relocated-install'
            original_install.rename(install)
            assert not original_install.exists()
            assert not (install / 'bspline_opt/include/bspline_opt/uniform_bspline.h').is_symlink()
        for source_root, package in [(ROOT / NEW / 'plan_manage', 'ego_planner'),
                                     (ROOT / ADAPTER, 'easyfly_local_planner'),
                                     (ROOT / NEW / 'ego_to_mavros', 'ego_to_mavros')]:
            for group in ('config', 'launch'):
                for src in (source_root / group).rglob('*'):
                    if src.is_file() and '__pycache__' not in src.parts:
                        assert sha(src) == sha(install / package / 'share' / package / src.relative_to(source_root)), src
        # Consumer is configured after copy relocation, using only installed interfaces.
        source = directory / 'consumer-source'
        shutil.copytree(ROOT / 'tools/algorithms/ego_consumer', source)
        consumer = directory / 'consumer-build'
        targets = ['bspline_opt', 'path_searching', 'plan_env', 'traj_utils_so', 'traj_utils', 'quadrotor_msgs']
        flags = ['-D' + p + '_DIR=' + str(install / p / 'share' / p / 'cmake') for p in targets]
        run(mode + '-consumer-configure', ['cmake', '-S', str(source), '-B', str(consumer),
            '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON', *flags], prefix=install)
        run(mode + '-consumer-build', ['cmake', '--build', str(consumer), '--parallel', '2'], prefix=install)
        trace = run(mode + '-consumer-trace', [str(consumer / 'ego_consumer')], prefix=install)
        assert trace == (baseline / 'legacy-spline-trace.log').read_text(), 'spline result differs from old install'
        assert str(ROOT / 'src/') not in (consumer / 'compile_commands.json').read_text()
        libs = run(mode + '-consumer-libraries', ['ldd', str(consumer / 'ego_consumer')], prefix=install)
        assert 'not found' not in libs
        for library in ('libbspline_opt.so', 'libpath_searching.so', 'libplan_env.so',
                        'libtraj_utils__rosidl_typesupport_cpp.so',
                        'libquadrotor_msgs__rosidl_typesupport_cpp.so'):
            line = next(line for line in libs.splitlines() if library in line)
            assert str(install) in line, line
        for binary in ('ego_planner_node', 'traj_server'):
            libs = run(mode + '-' + binary + '-libraries', ['ldd',
                str(install / 'ego_planner/lib/ego_planner' / binary)], prefix=install)
            assert 'not found' not in libs
        selection = ('from ament_index_python.packages import get_package_prefix; '
                     'import traj_utils.msg._bspline as b; import quadrotor_msgs.msg._position_command as p; '
                     'print(get_package_prefix("ego_planner")); '
                     'print(get_package_prefix("easyfly_local_planner")); print(b.__file__); print(p.__file__)')
        selected = run(mode + '-runtime-selection', ['python3', '-c', selection], prefix=install)
        assert all(str(install) in line for line in selected.splitlines()), selected
        # Fixtures launch installed nodes on their original, nonzero DDS domains.
        # Preserve every original assertion, timeout and sanitizer environment.
        counts = {}
        for package in ('plan_env', 'bspline_opt', 'ego_planner'):
            junit = directory / (package + '.xml')
            run(mode + '-' + package + '-tests', ['ctest', '--test-dir', str(build / package),
                '--output-on-failure', '-j1', '-R', '^test_', '--output-junit', str(junit)], prefix=install)
            suites = list(ET.parse(junit).getroot().iter('testsuite'))
            assert suites and all(int(s.get('failures', '0')) == 0 and int(s.get('errors', '0')) == 0 for s in suites)
            counts[package] = sum(int(s.get('tests', '0')) for s in suites)
        record['modes'].append({'mode': mode, 'install': str(install), 'ctest_registered_checks': counts,
                               'spline_frames_identical_to_old_install': 200,
                               'installed_messages_cdr_roundtrip': ['traj_utils/Bspline', 'quadrotor_msgs/PositionCommand'],
                               'installed_resources_identical': True, 'consumer_no_source_include': True})
        save()
    for name, digest in snapshot['underlay_binaries'].items():
        assert sha(ROOT / name) == digest
    assert record['source_files'] == fingerprints(snapshot), 'sources changed during verification'
    record['active_original_pids_still_present'] = {
        str(pid): Path('/proc', str(pid)).exists() for pid in snapshot['active_original_pids']}
    record['passed'] = True
    save()
    print('ALG-8.2 matrix passed:', out, flush=True)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Opt-in media, native RViz and real offline-tool acceptance on a private X server.

Uses real installed/build executables and synthetic ROS peers. No simulator,
flight, hardware, existing display, or existing tool process is controlled.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import secrets
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

from extension_baseline import active_domains
from verify_api_install import clean_environment

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    parser.add_argument('--underlay', type=Path, action='append', default=[])
    parser.add_argument('--build-base', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--xvfb', type=Path, required=True)
    parser.add_argument('--qgc', type=Path, default=ROOT/'src/third_party/QGC.AppImage')
    parser.add_argument('--phase', choices=['media', 'rviz', 'tools', 'all'], default='all')
    parser.add_argument('--domain', type=int, default=181)
    parser.add_argument('--only', action='append', default=[], help='run selected named steps only')
    parser.add_argument('--debug-target', help='run this named step under batch gdb with a backtrace')
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    occupied = active_domains()
    if str(args.domain) in occupied:
        parser.error('requested fixture domain occupied')
    # Existing C++ fixtures use these exact independent domains.
    conflicts = set(occupied) & {'93', '115', '120', '124', '129', '131', '132'}
    if conflicts:
        parser.error('existing fixture domain occupied: '+str(conflicts))
    prefixes = [p.resolve() for p in args.underlay] + [args.prefix.resolve()]
    env = clean_environment()
    env.update(PATH='/usr/bin:/bin', PYTHONNOUSERSITE='1', ROS_DOMAIN_ID=str(args.domain),
               ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(out/'ros-logs'))
    for key in ['DISPLAY', 'XAUTHORITY', 'WAYLAND_DISPLAY', 'QT_PLUGIN_PATH', 'QT_QPA_PLATFORM_PLUGIN_PATH']:
        env.pop(key, None)
    shell = 'source /opt/ros/humble/setup.bash\n'
    shell += 'source "$1/local_setup.bash"\nshift\n' * len(prefixes)
    selected = subprocess.check_output(['bash', '--noprofile', '--norc', '-c',
        shell+'exec /usr/bin/env -0', 'bash', *map(str, prefixes)], env=env)
    env = dict(item.decode().split('=', 1) for item in selected.split(b'\0') if item)
    display = next(number for number in range(180, 250)
                   if not Path(f'/tmp/.X{number}-lock').exists()
                   and not Path(f'/tmp/.X11-unix/X{number}').exists())
    authority = out/'Xauthority'
    authority.touch(mode=0o600)
    subprocess.run(['xauth', '-f', str(authority), 'add', ':'+str(display), '.', secrets.token_hex(16)],
                   env=env, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    env.update(DISPLAY=':'+str(display), XAUTHORITY=str(authority), QT_QPA_PLATFORM='xcb',
               LIBGL_ALWAYS_SOFTWARE='1', __GLX_VENDOR_LIBRARY_NAME='mesa',
               EASYFLY_QGC_TEST_IMAGE=str(args.qgc.resolve()))
    commands = []
    failures = []
    server = None
    def run(name, command, timeout, command_env=None):
        print(name, flush=True)
        started = time.monotonic()
        with (out/(name+'.log')).open('w') as log:
            process = subprocess.Popen(list(map(str, command)), env=command_env or env, cwd=ROOT,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                code = process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
                code = 124
        commands.append({'name': name, 'command': list(map(str, command)), 'returncode': code,
                         'seconds': time.monotonic()-started})
        (out/'commands.json').write_text(json.dumps(commands, indent=2)+'\n')
        if code:
            failures.append(name)
    try:
        with (out/'xvfb.log').open('w') as log:
            server = subprocess.Popen([str(args.xvfb.resolve()), ':'+str(display), '-screen', '0',
                '1920x1080x24', '-nolisten', 'tcp', '-noreset', '-auth', str(authority)],
                env=env, stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic()+10
        while not Path(f'/tmp/.X11-unix/X{display}').exists() and server.poll() is None and time.monotonic()<deadline:
            time.sleep(.05)
        assert server.poll() is None, (out/'xvfb.log').read_text()
        run('display', ['xdpyinfo', '-display', env['DISPLAY']], 10)
        if failures:
            raise RuntimeError('private X11 display not available')
        (out/'graphics-environment.json').write_text(json.dumps({
            'display': env['DISPLAY'], 'authority_file': str(authority), 'server_pid': server.pid,
            'server_executable': str(args.xvfb.resolve()), 'software_gl': True,
            'prefixes': list(map(str, prefixes)),
            'qgc_sha256': hashlib.sha256(args.qgc.read_bytes()).hexdigest(),
            'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}, indent=2)+'\n')
        build = args.build_base.resolve()
        targets = []
        if args.phase in ['media', 'all']:
            targets += [('recording', build/'easyfly_desktop/test_recording',
                         '*-Recording.PrivateSmallTmpfsRealEnospcNeverResumesOrVerifiesPartialRecord', 150),
                        ('recording-enospc', build/'easyfly_desktop/test_recording',
                         'Recording.PrivateSmallTmpfsRealEnospcNeverResumesOrVerifiesPartialRecord', 60),
                        ('playback', build/'easyfly_desktop/test_playback', '*', 150),
                        ('raw-recording', build/'easyfly_desktop/test_desktop',
                         'Desktop.OverlayDisplayNeverReplacesRawRecordingPixels', 60),
                        ('parameter-media', build/'easyfly_desktop_tests/test_parameter_controls', 'ParameterMedia.*', 150),
                        ('managed-media', build/'easyfly_desktop_tests/test_managed_stack',
                         'ManagedMediaStack.RunningMediaToolsExitAndLookupOnlyReopen:ManagedMediaStack.PendingAdmissionMediaToolsExitAndLookupOnlyReopen:TimelineStack.*', 180)]
        if args.phase in ['rviz', 'all']:
            targets += [('native-rviz', build/'easyfly_desktop/test_rviz_view', '*', 240)]
        if args.phase in ['tools', 'all']:
            targets += [('real-rviz-presets', build/'easyfly_desktop/test_tools', 'Tools.DISABLED_RealRvizPresets', 90),
                        ('real-qgc', build/'easyfly_desktop/test_tools', 'Tools.DISABLED_RealQgcOfflinePreview', 60),
                        ('real-tools-managed', build/'easyfly_desktop_tests/test_managed_stack',
                         'ManagedStack.DISABLED_RealToolsUnresolvedExitAndLookupOnlyReopen', 120),
                        ('real-tools-media', build/'easyfly_desktop_tests/test_managed_stack',
                         'ManagedMediaStack.DISABLED_RealToolsMediaExitAndLookupOnlyReopen', 150)]
        if args.only:
            unknown = set(args.only) - {t[0] for t in targets}
            if unknown:
                raise ValueError('step not in selected phase: '+str(unknown))
            targets = [t for t in targets if t[0] in args.only]
        metadata = {}
        for name, executable, pattern, timeout in targets:
            if not executable.is_file():
                raise FileNotFoundError(executable)
            package = executable.parent.name
            if package not in metadata:
                info = json.loads(subprocess.check_output(['ctest', '--test-dir', str(executable.parent),
                    '--show-only=json-v1'], env=env, text=True))
                metadata[package] = {t['name']: t for t in info['tests']}
            step_env = dict(env)
            test = metadata[package].get(executable.name)
            if test:
                for prop in test.get('properties', []):
                    if prop['name'] == 'ENVIRONMENT':
                        for entry in prop['value']:
                            key, value = entry.split('=', 1)
                            if key != 'QT_QPA_PLATFORM':
                                step_env[key] = value
            if step_env.get('ROS_DOMAIN_ID') in occupied:
                raise ValueError('original CTest domain occupied: '+step_env['ROS_DOMAIN_ID'])
            command = [executable, '--gtest_also_run_disabled_tests', '--gtest_filter='+pattern,
                       '--gtest_output=xml:'+str(out/(name+'.xml'))]
            if args.debug_target == name:
                command = ['gdb', '--batch', '--return-child-result', '-ex', 'set debuginfod enabled off',
                           '-ex', 'set print thread-events off', '-ex', 'run', '-ex', 'thread apply all bt',
                           '--args', *command]
            if name == 'recording-enospc':
                evidence = out/'enospc-evidence'; evidence.mkdir()
                with tempfile.TemporaryDirectory(prefix='easyfly-c07-enospc.') as mount:
                    step_env.update(EASYFLY_C07_ENOSPC_ROOT=mount,
                        EASYFLY_C07_HOST_MOUNT_NS=os.readlink('/proc/self/ns/mnt'),
                        EASYFLY_C07_ENOSPC_EVIDENCE_ROOT=str(evidence))
                    script = 'mount -t tmpfs -o size=8m,mode=0700 tmpfs "$1"\nshift\nexec "$@"'
                    run(name, ['unshare', '--user', '--map-root-user', '--mount', '--propagation', 'private',
                              'bash', '-e', '-c', script, 'bash', mount, *command], timeout, step_env)
            else:
                run(name, command, timeout, step_env)
    finally:
        if server and server.poll() is None:
            server.terminate()
            try:
                server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server.kill()
                server.wait(timeout=5)
        (out/'display-closed.json').write_text(json.dumps({'server_exited': server is None or server.poll() is not None}, indent=2))
    results = []
    for path in sorted(out.glob('*.xml')):
        for case in ET.parse(path).getroot().iter('testcase'):
            status = 'failed' if case.find('failure') is not None or case.find('error') is not None else (
                'skipped' if case.find('skipped') is not None or case.get('status') == 'notrun'
                or case.get('result') in ['skipped', 'suppressed'] else 'passed')
            results.append({'name': case.get('name'), 'class': case.get('classname'),
                            'status': status, 'evidence': path.name})
    # Disabled tests selected by a filter must genuinely execute.
    empty = [name for name, _, _, _ in targets if not (out/(name+'.xml')).is_file()
             or not list(ET.parse(out/(name+'.xml')).getroot().iter('testcase'))]
    passed = not failures and not empty and bool(results) and all(c['status'] == 'passed' for c in results)
    (out/'summary.json').write_text(json.dumps({'passed': passed, 'phase': args.phase,
        'failed_commands': failures, 'empty_selections': empty, 'cases': results,
        'counts': {s: sum(c['status']==s for c in results) for s in ['passed','failed','skipped']},
        'scope': __doc__, 'commands': commands, 'post_run_domains': active_domains()}, indent=2)+'\n')
    print(json.dumps({'passed': passed, 'failures': failures, 'counts': len(results)}), flush=True)
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()

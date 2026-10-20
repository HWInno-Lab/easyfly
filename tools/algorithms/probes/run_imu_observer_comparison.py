#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Run native/Python read-only observers with managed static Gazebo or a bag.

Never issues flight goals. Keeps guard limits and normal launch configuration.
An optional native-executor pause affects only the independent C++ observer.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from easyfly_fastdds_compat.runtime import component_profile_environment, verified_library


HERE = Path(__file__).resolve().parent


def stage_lockstep(out, poll_trace=None):
    """Add one server CLI flag in this run's ament overlay; preserve real libraries."""
    real = Path(get_package_prefix('easyfly_gazebo_plugins')).resolve()
    server = real / 'lib/easyfly_gazebo_plugins/easyfly_gzserver'
    prefix = out / 'lockstep-overlay'
    resource = prefix / 'share/ament_index/resource_index/packages/easyfly_gazebo_plugins'
    resource.parent.mkdir(parents=True)
    resource.write_text('')
    (prefix / 'share/easyfly_gazebo_plugins').symlink_to(real / 'share/easyfly_gazebo_plugins')
    lib = prefix / 'lib'
    lib.mkdir()
    for entry in (real / 'lib').iterdir():
        if entry.name != 'easyfly_gazebo_plugins':
            (lib / entry.name).symlink_to(entry)
    executables = lib / 'easyfly_gazebo_plugins'
    executables.mkdir()
    for entry in (real / 'lib/easyfly_gazebo_plugins').iterdir():
        if entry.name != 'easyfly_gzserver':
            (executables / entry.name).symlink_to(entry)
    shim = executables / 'easyfly_gzserver'
    trace_setup = ''
    if poll_trace:
        trace_setup = (f'os.environ["LD_PRELOAD"] = {str(poll_trace)!r}\n' +
                       f'os.environ["EASYFLY_POLL_TRACE"] = {str(out / "poll.csv")!r}\n')
    shim.write_text('#!/usr/bin/python3\nimport os\nimport sys\n' +
                    trace_setup + f'os.execv({str(server)!r}, ' +
                    f'[{str(server)!r}, "--lockstep", *sys.argv[1:]])\n')
    shim.chmod(0o755)
    return dict(prefix=str(prefix), real_prefix=str(real), real_server=str(server),
                real_server_sha256=hashlib.sha256(server.read_bytes()).hexdigest(),
                shim_sha256=hashlib.sha256(shim.read_bytes()).hexdigest())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--native-executable', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=120)
    parser.add_argument('--domain-id', type=int, default=209)
    parser.add_argument('--master-port', type=int, default=11631)
    parser.add_argument('--bag', type=Path)
    parser.add_argument('--profile', type=Path)
    parser.add_argument('--pause-after', type=int, default=0)
    parser.add_argument('--pause-ms', type=int, default=0)
    parser.add_argument('--no-native-notifications', action='store_true')
    parser.add_argument('--native-camera-load', action='store_true')
    parser.add_argument('--audit-guard', action='store_true')
    parser.add_argument('--audit-freshness', action='store_true')
    parser.add_argument('--audit-clock', action='store_true')
    parser.add_argument('--gazebo-lockstep', action='store_true')
    parser.add_argument('--sensor-load', choices=['all', 'no-lidar', 'no-depth', 'rgb-imu'],
                        help='explicit diagnostic model copies, using the low-level launch')
    parser.add_argument('--stack-layer', choices=['tiny', 'world', 'sitl', 'communications',
                                                  'calibration', 'bridge', 'perception', 'full',
                                                  'mavros', 'project_bridge'])
    parser.add_argument('--mavros-experiment', choices=[
        'all_on', 'all_off', 'reference_on', 'reference_off'])
    parser.add_argument('--step-trace-plugin', type=Path)
    parser.add_argument('--poll-trace-library', type=Path)
    parser.add_argument('--process-budget', action='store_true')
    parser.add_argument('--gazebo-camera-executable', type=Path)
    parser.add_argument('--cpu-limit', type=int, default=0)
    parser.add_argument('--cpu-workers', type=int, default=0)
    parser.add_argument('--guard-pause-after', type=float, default=40)
    parser.add_argument('--guard-pause-ms', type=int, default=0)
    args = parser.parse_args()
    if not 5 <= args.seconds <= 270 or not 0 <= args.domain_id <= 232 or \
            not 1024 <= args.master_port <= 65535 or not 0 <= args.pause_ms <= 2000 or \
            args.pause_after < 0:
        parser.error('invalid bounded duration/domain/port/pause')
    if not 0 <= args.cpu_limit <= 4 or not 0 <= args.cpu_workers <= 4 or \
            (args.cpu_workers and not args.cpu_limit) or \
            not 0 <= args.guard_pause_ms <= 2000 or \
            (args.guard_pause_ms and not 15 <= args.guard_pause_after <= args.seconds-15):
        parser.error('invalid bounded CPU or guard pause experiment')
    if args.bag and (args.guard_pause_ms or args.gazebo_camera_executable or args.gazebo_lockstep):
        parser.error('guard pause, native Gazebo camera and lockstep require managed Gazebo')
    if args.sensor_load and (args.bag or not args.gazebo_lockstep or not args.profile):
        parser.error('sensor ablations require Gazebo lockstep and an explicit DDS profile')
    if args.stack_layer and ((args.sensor_load != 'rgb-imu' and not (
            args.stack_layer == 'full' and args.sensor_load)) or not args.audit_guard or
                             not args.process_budget or args.seconds < 45):
        parser.error('stack layers require rgb-imu, guard audit, process budget and >=45 seconds')
    if args.audit_freshness and (not args.audit_guard or args.bag):
        parser.error('freshness audit requires managed Gazebo and guard audit')
    if args.audit_clock and not args.audit_freshness:
        parser.error('clock audit requires freshness audit')
    if any((args.mavros_experiment, args.step_trace_plugin, args.poll_trace_library)) and not (
            args.mavros_experiment and args.step_trace_plugin and args.poll_trace_library and
            args.stack_layer == 'communications' and args.audit_clock):
        parser.error('MAVROS isolation requires communications, clock audit, both trace libraries')
    native = args.native_executable.resolve(strict=True)
    camera = (args.gazebo_camera_executable.resolve(strict=True)
              if args.gazebo_camera_executable else None)
    bag = args.bag.resolve(strict=True) if args.bag else None
    profile = args.profile.resolve(strict=True) if args.profile else None
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    library, digest = verified_library(get_package_prefix('easyfly_fastdds_compat'))
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY='1',
               ROS_LOG_DIR=str(out / 'ros'),
               GAZEBO_MASTER_URI=f'http://127.0.0.1:{args.master_port}',
               LIBGL_ALWAYS_SOFTWARE='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
               EASYFLY_IMU_TIMING_TRACE=str(out / 'producer'),
               LD_LIBRARY_PATH=str(library.parent) + ':' + os.environ.get('LD_LIBRARY_PATH', ''))
    if profile:
        for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
            env[key] = str(profile)
    if args.audit_guard:
        env['PYTHONPATH'] = str(HERE / 'guard_trace_hook') + ':' + env.get('PYTHONPATH', '')
        env['EASYFLY_GUARD_TAKE_TRACE'] = str(out / 'guard-take')
    if args.audit_freshness:
        env['EASYFLY_GUARD_FRESHNESS_TRACE'] = '1'
    if args.audit_clock:
        env['EASYFLY_GUARD_CLOCK_TRACE'] = '1'
    report = dict(scope=__doc__, error=None, forced=False, children=[],
                  scenario='recorded bag' if bag else 'managed static Gazebo',
                  native_executable_sha256=hashlib.sha256(native.read_bytes()).hexdigest(),
                  middleware=dict(path=str(library), sha256=digest),
                  profile=str(profile) if profile else None, guard_limits_changed=False,
                  native_pause_after=args.pause_after, native_pause_ms=args.pause_ms,
                  native_camera_load=args.native_camera_load,
                  native_notifications=not args.no_native_notifications)
    report['guard_take_audit'] = args.audit_guard
    report['guard_freshness_audit'] = args.audit_freshness
    report['guard_clock_audit'] = args.audit_clock
    report['gazebo_lockstep'] = args.gazebo_lockstep
    if args.sensor_load:
        if args.stack_layer:
            from stack_layer_fixture import stage as stage_layer
            if args.mavros_experiment:
                from mavros_isolation_fixture import stage as stage_isolation
                report['sensor_fixture'] = stage_isolation(
                    out, args.mavros_experiment, args.step_trace_plugin.resolve(strict=True))
                env['EASYFLY_STEP_TRACE'] = str(out/'steps.csv')
            else:
                report['sensor_fixture'] = stage_layer(out, args.stack_layer, args.sensor_load)
        else:
            from sensor_load_fixture import stage as stage_sensor_load
            report['sensor_fixture'] = stage_sensor_load(out, args.sensor_load)
        component_profile = Path(get_package_prefix('easyfly_fastdds_compat')) / \
            'share/easyfly_fastdds_compat/config/component_services.xml'
        env.update(component_profile_environment(component_profile, env))
    if args.gazebo_lockstep:
        poll_trace = (args.poll_trace_library.resolve(strict=True)
                      if args.poll_trace_library else None)
        report['lockstep_staging'] = stage_lockstep(out, poll_trace)
        env['AMENT_PREFIX_PATH'] = report['lockstep_staging']['prefix'] + ':' + \
            env.get('AMENT_PREFIX_PATH', '')
    cpus = sorted(os.sched_getaffinity(0))[:args.cpu_limit] if args.cpu_limit else []
    report.update(cpu_affinity=cpus, cpu_workers=args.cpu_workers, guard_pause=None)
    processes = []
    paused_guard = None
    budget_samples = []

    def descendants(root):
        table = [line.split(None, 2) for line in subprocess.check_output(
            ['ps', '-eo', 'pid=,ppid=,args='], text=True).splitlines()]
        owned = {root}
        for _ in range(12):
            owned.update(int(pid) for pid, parent, _ in table if int(parent) in owned)
        return [(int(pid), command) for pid, _, command in table if int(pid) in owned]

    def last_status():
        path = out / 'python/samples.jsonl'
        with path.open('rb') as stream:
            stream.seek(max(0, path.stat().st_size-65536))
            lines = stream.read().splitlines()[1:-1]
        return next((json.loads(line) for line in reversed(lines)
                     if b'"stream": "status"' in line), None)

    def start(name, command):
        if cpus:
            command = ['taskset', '-c', ','.join(map(str, cpus)), *command]
        with (out / (name + '.log')).open('w') as stream:
            process = subprocess.Popen(command, env=env, stdout=stream, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        record = dict(name=name, pid=process.pid, command=command)
        report['children'].append(record)
        processes.append((process, record))
        return process

    def ready(path):
        end = time.monotonic() + 15
        while not path.is_file():
            if any(p.poll() is not None for p, _ in processes):
                raise RuntimeError('observer exited before ready')
            if time.monotonic() > end:
                raise RuntimeError('observer readiness timed out')
            time.sleep(.02)

    try:
        for i in range(args.cpu_workers):
            start(f'load-{i}', ['python3', str(HERE / 'bounded_cpu_load.py'),
                                '--seconds', str(args.seconds+20)])
        native_command = [str(native), '--ros-args', '-p', f'output:={out / "native.csv"}',
                          '-p', f'duration:={args.seconds}', '-p',
                          f'pause_after:={args.pause_after}', '-p', f'pause_ms:={args.pause_ms}',
                          '-p', f'notifications:={str(not args.no_native_notifications).lower()}',
                          '-p', f'camera_load:={str(args.native_camera_load).lower()}']
        cpp = start('native', native_command)
        ready(out / 'native.csv.ready')
        python_command = ['python3', str(HERE / 'openvins_input_trace.py'), '--seconds',
                          str(args.seconds), '--output', str(out / 'python')]
        if args.sensor_load or args.process_budget:
            python_command.append('--sensor-topology')
        python = start('python', python_command)
        ready(out / 'python/ready')
        command = (['ros2', 'bag', 'play', str(bag), '--rate', '.5', '--clock', '100',
                    '--delay', '2']
                   if bag else ['ros2', 'launch', 'easyfly_sim_launcher', 'profile.launch.py',
                                'profile:=navigation_dev', 'algorithms_json:={"slam":"openvins"}',
                                'scene:=primitives_sparse_seed42_v1', 'headless:=true',
                                'rviz:=false'])
        if args.sensor_load:
            command = ['ros2', 'launch', report['sensor_fixture']['launch']]
        stack = start('stack', command)
        started = time.monotonic()
        captured_server = False
        next_budget = started + 5
        camera_started = False
        pause_done = False
        end = time.monotonic() + args.seconds + 10
        while cpp.poll() is None or python.poll() is None:
            if stack.poll() is not None:
                raise RuntimeError('stack/player exited before observation completed')
            if time.monotonic() > end:
                raise RuntimeError('observers exceeded deadline')
            if not captured_server and not bag:
                for pid, cmd in descendants(stack.pid):
                    if '/easyfly_gzserver' not in cmd:
                        continue
                    actual = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
                    actual = [os.fsdecode(value) for value in actual if value]
                    executable = Path(f'/proc/{pid}/exe').resolve()
                    if not actual or executable.name != 'easyfly_gzserver':
                        continue  # The staging shim has not exec'd the real server yet.
                    entries = dict(v.split(b'=', 1) for v in
                                   Path(f'/proc/{pid}/environ').read_bytes().split(b'\0')
                                   if b'=' in v)
                    process_environment = {key: os.fsdecode(entries.get(key.encode(), b''))
                                           for key in ('EASYFLY_GAZEBO_LOCKSTEP', 'LP_NUM_THREADS')}
                    report['actual_gazebo'] = dict(pid=pid, executable=str(executable),
                                                   command=actual,
                                                   lockstep_cli='--lockstep' in actual,
                                                   environment=process_environment,
                                                   lockstep=(
                                                       '--lockstep' in actual or
                                                       process_environment[
                                                           'EASYFLY_GAZEBO_LOCKSTEP'] == '1'),
                                                   sha256=hashlib.sha256(
                                                       executable.read_bytes()).hexdigest())
                    expected_lockstep = args.gazebo_lockstep or not args.sensor_load
                    if report['actual_gazebo']['lockstep'] != expected_lockstep:
                        raise RuntimeError('actual server lockstep flag does not match experiment')
                    captured_server = True
                    break
            if camera and not camera_started and captured_server:
                # Gazebo's native client may miss discovery if initialized before
                # the new master. Start after the server reports its connection.
                server_ready = 'Connected to gazebo master' in (out / 'stack.log').read_text()
                if server_ready and time.monotonic()-started >= 1:
                    start('gazebo-camera', [str(camera), str(out / 'gazebo-camera.csv')])
                    camera_started = True
                    report['gazebo_camera_start'] = dict(
                        after_server_master_log=True, delay_s=time.monotonic()-started)
            if args.guard_pause_ms and not pause_done and \
                    time.monotonic()-started >= args.guard_pause_after:
                guards = [pid for pid, cmd in descendants(stack.pid)
                          if '/lib/easyfly_openvins_integration/openvins_guard' in cmd]
                if len(guards) != 1:
                    raise RuntimeError('expected exactly one owned guard before pause')
                status = last_status()
                report['guard_pause'] = dict(pid=guards[0], before=status,
                                             requested_ms=args.guard_pause_ms)
                if not status or status['state'] not in (1, 2):
                    raise RuntimeError('guard must be admitting healthy inputs before pause')
                os.kill(guards[0], signal.SIGSTOP)
                paused_guard = guards[0]
                report['guard_pause']['start_mono_ns'] = time.monotonic_ns()
                pause_done = True
            if paused_guard and time.monotonic_ns() - \
                    report['guard_pause']['start_mono_ns'] >= args.guard_pause_ms * 1000000:
                os.kill(paused_guard, signal.SIGCONT)
                report['guard_pause']['end_mono_ns'] = time.monotonic_ns()
                paused_guard = None
            if args.mavros_experiment and captured_server and \
                    time.monotonic()-started > 20 and 'trace_libraries' not in report:
                maps = Path(f'/proc/{report["actual_gazebo"]["pid"]}/maps').read_text()
                paths = {line.split()[-1] for line in maps.splitlines() if any(
                    name in line for name in ('libgazebo_mavlink_interface', 'libmavlink_interface',
                                              'libeasyfly_step_trace', 'libeasyfly_poll_trace'))}
                report['trace_libraries'] = {p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
                                             for p in sorted(paths)}
            if args.process_budget and time.monotonic() >= next_budget:
                from process_cpu_trace import snapshot
                budget_samples.append(snapshot(descendants(stack.pid)))
                next_budget = time.monotonic() + 5
            time.sleep(.05)
        assert cpp.returncode == python.returncode == 0, 'observer failed'
        if not bag and not captured_server:
            raise RuntimeError('real Gazebo executable/arguments not captured')
        if camera and not camera_started:
            raise RuntimeError('native camera observer was not started after master readiness')
        if not bag:
            table = [line.split(None, 2) for line in subprocess.check_output(
                ['ps', '-eo', 'pid=,ppid=,args='], text=True).splitlines()]
            owned = {stack.pid}
            for _ in range(8):
                owned.update(int(pid) for pid, parent, _ in table if int(parent) in owned)
            for pid, _, command in table:
                if int(pid) not in owned or '/easyfly_gzserver' not in command:
                    continue
                entries = dict(v.split(b'=', 1) for v in Path(f'/proc/{pid}/environ').read_bytes()
                               .split(b'\0') if b'=' in v)
                path = entries.get(b'FASTRTPS_DEFAULT_PROFILES_FILE')
                if path:
                    (out / 'actual-fastdds.xml').write_bytes(Path(os.fsdecode(path)).read_bytes())
    except Exception as exc:
        report['error'] = repr(exc)
    finally:
        if paused_guard:
            os.kill(paused_guard, signal.SIGCONT)
        for process, record in reversed(processes):
            if process.poll() is None:
                if record['name'] == 'stack' and not bag:
                    process.send_signal(signal.SIGINT)
                else:
                    os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=20)
                record['forced'] = False
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
                record['forced'] = report['forced'] = True
            record['returncode'] = process.returncode
        report['clean_execution'] = report['error'] is None and not report['forced'] and all(
            r['returncode'] == 0 for r in report['children'])
        report['nested_process_errors'] = [
            line for line in (out / 'stack.log').read_text().splitlines()
            if 'process has died' in line] if (out / 'stack.log').exists() else []
        if report['nested_process_errors']:
            report['clean_execution'] = False
        if args.audit_guard and not bag:
            report['guard_audit_files'] = [str(p) for p in out.glob('guard-take.*.json')]
            if len(report['guard_audit_files']) != 1:
                report['clean_execution'] = False
                report['guard_audit_error'] = 'expected one guard audit trace after normal shutdown'
                if report['error'] is None:
                    report['error'] = report['guard_audit_error']
        if args.process_budget:
            budget = dict(scope='Owned process CPU counters, not function or lock timing',
                          clock_ticks_per_second=os.sysconf('SC_CLK_TCK'),
                          page_size=os.sysconf('SC_PAGE_SIZE'),
                          kernel_schedstats_enabled=Path(
                              '/proc/sys/kernel/sched_schedstats').read_text().strip() == '1',
                          samples=budget_samples)
            (out / 'process-budget.json').write_text(json.dumps(budget, indent=2) + '\n')
        (out / 'lifetime.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
    return 0 if report['clean_execution'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

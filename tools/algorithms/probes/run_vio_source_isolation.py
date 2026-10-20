#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Isolate real rendered camera/IMU timing without PX4, lidar or detectors.

Copies the installed sensor configuration into a static fixture. Lockstep and
bounded CPU contention are explicit experiments, never production defaults.
"""

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from easyfly_fastdds_compat.runtime import verified_library
from easyfly_px4_sitl.session_guard import SessionReservation


HERE = Path(__file__).resolve().parent


def fixture(iris, mid, plugins, output):
    root = ET.Element('sdf', version='1.6')
    world = ET.SubElement(root, 'world', name='vio_source_isolation')
    physics = ET.SubElement(world, 'physics', name='default', type='ode')
    ET.SubElement(physics, 'max_step_size').text = '0.001'
    ET.SubElement(physics, 'real_time_update_rate').text = '1000'
    ET.SubElement(world, 'gravity').text = '0 0 -9.81'
    light = ET.SubElement(world, 'light', name='sun', type='directional')
    ET.SubElement(light, 'diffuse').text = '0.8 0.8 0.8 1'
    ET.SubElement(light, 'direction').text = '-0.5 0.1 -0.9'
    model = ET.SubElement(world, 'model', name='source_fixture')
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = '0 0 1 0 0 0'
    link = ET.SubElement(model, 'link', name='base_link')
    color, = ET.parse(iris).findall(".//sensor[@name='color']")
    imu, = ET.parse(mid).findall(".//sensor[@type='imu']")
    libraries = []
    for original in (color, imu):
        sensor = copy.deepcopy(original)
        for plugin in sensor.findall('plugin'):
            library = plugins / Path(plugin.get('filename')).name
            assert library.is_file(), library
            plugin.set('filename', str(library))
            libraries.append(library)
        link.append(sensor)
    # Simple visible geometry; no resource downloads, sensor or control plugins.
    for index, (pose, size, color) in enumerate([
            ('4 0 1 0 0 0', '0.2 4 2', '0.8 0.8 0.8 1'),
            ('3 -1 0.5 0 0 0', '0.4 0.4 1', '0.8 0.1 0.1 1'),
            ('3 1 0.8 0 0 0', '0.4 0.4 1.6', '0.1 0.1 0.8 1')]):
        prop = ET.SubElement(world, 'model', name=f'box_{index}')
        ET.SubElement(prop, 'static').text = 'true'
        ET.SubElement(prop, 'pose').text = pose
        visual = ET.SubElement(ET.SubElement(prop, 'link', name='link'), 'visual', name='box')
        ET.SubElement(ET.SubElement(ET.SubElement(visual, 'geometry'), 'box'), 'size').text = size
        material = ET.SubElement(visual, 'material')
        ET.SubElement(material, 'diffuse').text = color
        ET.SubElement(material, 'ambient').text = color
    ET.indent(root)
    ET.ElementTree(root).write(output, encoding='utf-8')
    return libraries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--iris-sdf', type=Path, required=True)
    parser.add_argument('--native-executable', type=Path, required=True)
    parser.add_argument('--gazebo-camera-executable', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--domain-id', type=int, default=215)
    parser.add_argument('--master-port', type=int, default=11831)
    parser.add_argument('--lockstep', action='store_true')
    parser.add_argument('--cpu-limit', type=int, default=0)
    parser.add_argument('--cpu-workers', type=int, default=0)
    parser.add_argument('--localization', action='store_true')
    parser.add_argument('--profile', type=Path)
    args = parser.parse_args()
    if not 10 <= args.seconds <= 180 or not 0 <= args.domain_id <= 232 or \
            not 1024 <= args.master_port <= 65535 or not 0 <= args.cpu_limit <= 4 or \
            not 0 <= args.cpu_workers <= 4 or (args.cpu_workers and not args.cpu_limit):
        parser.error('bounded duration, CPU load and private domain/port required')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    native = args.native_executable.resolve(strict=True)
    camera = args.gazebo_camera_executable.resolve(strict=True)
    iris = args.iris_sdf.resolve(strict=True)
    plugins = Path(get_package_prefix('easyfly_gazebo_plugins')) / 'lib'
    server = plugins / 'easyfly_gazebo_plugins/easyfly_gzserver'
    mid = Path(get_package_share_directory('easyfly_gazebo_models')) / \
        'easyfly/runtime_models/mid360/mid360.sdf'
    world = out / 'fixture.world'
    libraries = fixture(iris, mid, plugins, world)
    library, digest = verified_library(get_package_prefix('easyfly_fastdds_compat'))
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY='1',
               ROS_LOG_DIR=str(out / 'ros'), LIBGL_ALWAYS_SOFTWARE='1',
               GAZEBO_MASTER_URI=f'http://127.0.0.1:{args.master_port}',
               GAZEBO_MODEL_DATABASE_URI='',
               GAZEBO_PLUGIN_PATH=str(plugins) + ':/opt/ros/humble/lib',
               RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
               EASYFLY_IMU_TIMING_TRACE=str(out / 'producer'),
               LD_LIBRARY_PATH=str(library.parent) + ':' + os.environ.get('LD_LIBRARY_PATH', ''))
    if args.profile:
        profile = args.profile.resolve(strict=True)
        for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
            env[key] = str(profile)
    if args.localization:
        env['PYTHONPATH'] = str(HERE / 'guard_trace_hook') + ':' + env.get('PYTHONPATH', '')
        env['EASYFLY_GUARD_TAKE_TRACE'] = str(out / 'guard-take')
    cpus = sorted(os.sched_getaffinity(0))[:args.cpu_limit] if args.cpu_limit else []
    sources = [iris, mid, world, server, *libraries, native, camera, Path(__file__)]
    report = dict(scope=__doc__, error=None, forced=False, children=[], lockstep=args.lockstep,
                  localization=args.localization, cpu_affinity=cpus, cpu_workers=args.cpu_workers,
                  middleware=dict(path=str(library), sha256=digest),
                  profile=str(args.profile) if args.profile else None,
                  source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in sources})
    processes = []
    reservation = SessionReservation()

    def start(name, command):
        if cpus:
            command = ['taskset', '-c', ','.join(map(str, cpus)), *command]
        with (out / (name + '.log')).open('w') as stream:
            process = subprocess.Popen(command, env=env, stdout=stream, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        record = dict(name=name, pid=process.pid, command=command)
        processes.append((process, record))
        report['children'].append(record)
        return process

    def ready(path):
        end = time.monotonic() + 15
        while not path.exists():
            if time.monotonic() > end or any(p.poll() is not None for p, _ in processes):
                raise RuntimeError('observer readiness failed')
            time.sleep(.02)

    try:
        reservation.acquire(env['GAZEBO_MASTER_URI'])
        for i in range(args.cpu_workers):
            start('load-' + str(i), ['python3', str(HERE / 'bounded_cpu_load.py'),
                                     '--seconds', str(args.seconds+20)])
        cpp = start('native', [str(native), '--ros-args', '-p', f'output:={out / "native.csv"}',
                               '-p', f'duration:={args.seconds}', '-p', 'notifications:=false'])
        ready(out / 'native.csv.ready')
        python = start('python', ['python3', str(HERE / 'openvins_input_trace.py'),
                                  '--output', str(out / 'python'), '--seconds', str(args.seconds)])
        ready(out / 'python/ready')
        if args.localization:
            start('localization', ['ros2', 'launch', 'easyfly_openvins_integration',
                                   'openvins.launch.py'])
        command = [str(server), '--verbose', '-s', 'libgazebo_ros_init.so', str(world)]
        if args.lockstep:
            command.insert(1, '--lockstep')
        gazebo = start('gazebo', command)
        start('gazebo-camera', [str(camera), str(out / 'gazebo-camera.csv')])
        end = time.monotonic() + args.seconds + 10
        while cpp.poll() is None or python.poll() is None:
            if gazebo.poll() is not None or time.monotonic() > end:
                raise RuntimeError('source exited or observation deadline exceeded')
            time.sleep(.05)
        if cpp.returncode or python.returncode:
            raise RuntimeError('observer failed')
    except Exception as exc:
        report['error'] = repr(exc)
    finally:
        # Stop localization before its source; EOF is outside the capture window.
        order = {'gazebo-camera': 0, 'localization': 1, 'gazebo': 2, 'python': 3, 'native': 4}
        for process, record in sorted(processes, key=lambda p: order.get(p[1]['name'], 5)):
            if process.poll() is None:
                if record['name'] == 'localization':
                    # launch owns its child shutdown; avoid duplicate SIGINT to
                    # children while they are releasing their ROS context.
                    process.send_signal(signal.SIGINT)
                else:
                    os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
                record['forced'] = False
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
                record['forced'] = report['forced'] = True
            record['returncode'] = process.returncode
        reservation.close()
        log = out / 'localization.log'
        report['nested_process_errors'] = [
            line for line in log.read_text().splitlines() if 'process has died' in line
        ] if log.exists() else []
        report['clean_execution'] = (
            not report['error'] and not report['forced'] and
            not report['nested_process_errors'] and
            all(r['returncode'] == 0 for r in report['children']))
        (out / 'lifetime.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
    return 0 if report['clean_execution'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

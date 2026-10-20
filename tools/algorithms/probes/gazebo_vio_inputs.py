#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Measure installed Gazebo camera/IMU streams in a private static fixture.

No PX4 or flight commands. Retains source timestamps and compares simultaneous
reliable/best-effort image observers. Never substitutes synthetic ROS messages.
"""

import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from easyfly_px4_sitl.session_guard import SessionReservation
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu


def summarize(items):
    stamps = [m['stamp_ns'] * 1e-9 for m in items]
    diffs = [b - a for a, b in zip(stamps, stamps[1:])]
    result = dict(count=len(items), nonincreasing=sum(d <= 0 for d in diffs))
    if diffs and min(diffs) > 0:
        result.update(source_span=stamps[-1] - stamps[0],
                      median_source_hz=1 / statistics.median(diffs),
                      mean_source_hz=len(diffs) / (stamps[-1] - stamps[0]),
                      max_source_gap=max(diffs))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--iris-sdf', required=True, type=Path)
    parser.add_argument('--seconds', type=float, default=12)
    parser.add_argument('--color-only', action='store_true')
    parser.add_argument('--lockstep', action='store_true')
    parser.add_argument('--master-port', type=int, default=11539)
    args = parser.parse_args()
    if not 3 <= args.seconds <= 120:
        parser.error('seconds must be in [3, 120]')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    plugins = Path(get_package_prefix('easyfly_gazebo_plugins')) / 'lib'
    server = plugins / 'easyfly_gazebo_plugins/easyfly_gzserver'
    camera_library = plugins / 'libeasyfly_gazebo_ros_camera.so'
    mid = Path(get_package_share_directory('easyfly_gazebo_models')) / \
        'easyfly/runtime_models/mid360/mid360.sdf'
    iris = args.iris_sdf.resolve(strict=True)
    root = ET.Element('sdf', version='1.6')
    world = ET.SubElement(root, 'world', name='vio_input_check')
    physics = ET.SubElement(world, 'physics', name='default', type='ode')
    ET.SubElement(physics, 'max_step_size').text = '0.001'
    ET.SubElement(physics, 'real_time_update_rate').text = '1000'
    ET.SubElement(world, 'gravity').text = '0 0 -9.81'
    light = ET.SubElement(world, 'light', name='sun', type='directional')
    ET.SubElement(light, 'diffuse').text = '0.8 0.8 0.8 1'
    ET.SubElement(light, 'direction').text = '-0.5 0.1 -0.9'
    model = ET.SubElement(world, 'model', name='camera_fixture')
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = '0 0 1 0 0 0'
    base = ET.SubElement(model, 'link', name='base_link')
    for original in ET.parse(iris).findall(".//link[@name='base_link']/sensor"):
        if args.color_only and original.get('name') != 'color':
            continue
        sensor = copy.deepcopy(original)
        for plugin in sensor.findall('plugin'):
            plugin.set('filename', str(camera_library))
        base.append(sensor)
    lidar = copy.deepcopy(ET.parse(mid).find('model'))
    ET.SubElement(lidar, 'static').text = 'true'
    ET.SubElement(lidar, 'pose').text = '0 0 1.08 0 0 0'
    for plugin in lidar.findall('.//plugin'):
        if plugin.get('filename') == 'liblivox_laser_simulation.so':
            plugin.set('filename', str(plugins / plugin.get('filename')))
    world.append(lidar)
    ET.indent(root)
    fixture = out / 'fixture.world'
    ET.ElementTree(root).write(fixture, encoding='utf-8')
    os.environ.update(ROS_DOMAIN_ID='232', ROS_LOCALHOST_ONLY='1',
                      ROS_LOG_DIR=str(out / 'ros'),
                      GAZEBO_MASTER_URI=f'http://127.0.0.1:{args.master_port}',
                      GAZEBO_PLUGIN_PATH=str(plugins) + ':/opt/ros/humble/lib',
                      GAZEBO_MODEL_DATABASE_URI='', LIBGL_ALWAYS_SOFTWARE='1')
    record = dict(scope=__doc__.strip(), error=None, forced=False,
                  color_only=args.color_only, lockstep=args.lockstep,
                  files_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in [iris, mid, server, camera_library, fixture]})
    samples = {k: [] for k in ['image_reliable', 'image_best_effort', 'camera_info', 'imu']}
    if not args.color_only:
        samples['depth'] = []
    node, process = None, None
    reservation = SessionReservation()
    try:
        reservation.acquire(os.environ['GAZEBO_MASTER_URI'])
        rclpy.init()
        node = rclpy.create_node('easyfly_vio_input_check')
        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        best_effort = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        subscriptions = []

        def receive(msg, key):
            item = dict(stamp_ns=msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                        receipt=time.monotonic(), frame_id=msg.header.frame_id)
            if key == 'imu':
                item.update(gyro=[msg.angular_velocity.x, msg.angular_velocity.y,
                                  msg.angular_velocity.z],
                            accel=[msg.linear_acceleration.x, msg.linear_acceleration.y,
                                   msg.linear_acceleration.z],
                            gyro_covariance=list(msg.angular_velocity_covariance),
                            accel_covariance=list(msg.linear_acceleration_covariance))
            elif key == 'camera_info':
                item.update(k=list(msg.k), d=list(msg.d), width=msg.width, height=msg.height)
            else:
                item.update(width=msg.width, height=msg.height, encoding=msg.encoding)
            samples[key].append(item)

        topics = [('image_reliable', '/camera/color/image_raw', Image, reliable),
                  ('image_best_effort', '/camera/color/image_raw', Image, best_effort),
                  ('camera_info', '/camera/color/camera_info', CameraInfo, reliable),
                  ('imu', '/livox/imu', Imu, reliable)]
        if not args.color_only:
            topics.append(('depth', '/camera/aligned_depth_to_color/depth/image_raw',
                           Image, reliable))
        for key, topic, kind, qos in topics:
            subscriptions.append(node.create_subscription(
                kind, topic, lambda msg, key=key: receive(msg, key), qos))
        command = [str(server), '--verbose', '-s', 'libgazebo_ros_init.so', str(fixture)]
        if args.lockstep:
            command.insert(1, '--lockstep')
        record['command'] = command
        with (out / 'gazebo.log').open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            deadline = time.monotonic() + 20 + args.seconds * 5
            first_source = None
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError('Gazebo exited during capture')
                rclpy.spin_once(node, timeout_sec=.01)
                if all(samples.values()):
                    if first_source is None:
                        first_source = samples['imu'][-1]['stamp_ns']
                    if (samples['imu'][-1]['stamp_ns'] - first_source) * 1e-9 >= args.seconds:
                        break
            else:
                raise RuntimeError('sensor capture timed out')
            record['publishers'] = {
                topic: [dict(name=p.node_name, reliability=str(p.qos_profile.reliability))
                        for p in node.get_publishers_info_by_topic(topic)]
                for _, topic, _, _ in topics}
            process.send_signal(signal.SIGINT)
            process.wait(timeout=12)
            # Drain already delivered samples without expensive per-sample disk writes.
            until = time.monotonic() + .3
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.01)
            record['server_returncode'] = process.returncode
    except Exception as exc:
        record['error'] = repr(exc)
    finally:
        if process is not None and process.poll() is None:
            record['forced'] = True
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        reservation.close()
    with (out / 'samples.jsonl').open('w') as stream:
        for key, items in samples.items():
            for item in items:
                stream.write(json.dumps(dict(stream=key, **item)) + '\n')
    record['streams'] = {key: summarize(items) for key, items in samples.items()}
    checks = {}
    if samples['imu']:
        imus = samples['imu']
        record['imu_stddev'] = {
            field: [statistics.pstdev(m[field][axis] for m in imus) for axis in range(3)]
            for field in ['gyro', 'accel']}
        record['imu_first'] = imus[0]
        checks['imu_frame'] = all(m['frame_id'] == 'livox_imu_frame' for m in imus)
        for field, sigma in [('gyro', .00017), ('accel', .017)]:
            checks[field + '_noise'] = all(.7 * sigma < s < 1.3 * sigma
                                           for s in record['imu_stddev'][field])
            checks[field + '_covariance'] = all(
                math.isclose(m[field + '_covariance'][i], sigma**2, rel_tol=1e-6)
                for m in imus for i in [0, 4, 8])
        checks['finite_imu'] = all(math.isfinite(v) for m in imus
                                   for field in ['gyro', 'accel'] for v in m[field])
    checks['all_streams'] = all(len(items) >= 50 for items in samples.values())
    checks['source_monotonic'] = all(s['nonincreasing'] == 0
                                     for s in record['streams'].values())
    log_path = out / 'gazebo.log'
    checks['no_noise_schema_errors'] = log_path.is_file() and \
        'Get noise index not valid' not in log_path.read_text()
    record['checks'] = checks
    record['passed'] = all(checks.values()) and record['error'] is None and \
        not record['forced'] and record.get('server_returncode') == 0
    for key in ['image_reliable', 'image_best_effort']:
        received = {m['stamp_ns'] for m in samples[key]}
        info = {m['stamp_ns'] for m in samples['camera_info']}
        record['streams'][key]['missing_vs_info'] = len(info - received)
    (out / 'summary.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps({k: record[k] for k in ['passed', 'error', 'streams', 'checks']}, indent=2))
    return 0 if record['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

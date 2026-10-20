#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Run installed guard + actual OpenVINS on an original bag, then fault/restart checks."""

import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from easyfly_msgs.msg import LocalizationStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu
from tf2_msgs.msg import TFMessage


def ns(msg):
    return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec


def message_digest(msg):
    """Compare every message field and payload; CDR alignment padding is unspecified."""
    header = [msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id]
    if isinstance(msg, Image):
        fields = [header, msg.height, msg.width, msg.encoding, msg.is_bigendian, msg.step]
        return hashlib.sha256(json.dumps(fields).encode() + bytes(msg.data)).hexdigest()
    fields = [header,
              [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
              list(msg.orientation_covariance),
              [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
              list(msg.angular_velocity_covariance),
              [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
              list(msg.linear_acceleration_covariance)]
    return hashlib.sha256(json.dumps(fields, allow_nan=False).encode()).hexdigest()


def oracle(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    camera, imu, counts = {}, {}, Counter()
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/camera/color/image_raw':
            msg = deserialize_message(data, Image)
            counts[ns(msg)] += 1
            camera[ns(msg)] = message_digest(msg)
        elif topic == '/livox/imu':
            msg = deserialize_message(data, Imu)
            imu[ns(msg)] = message_digest(msg)
    del reader
    camera = {t: digest for t, digest in camera.items() if counts[t] == 1}
    camera.pop(max(camera))  # Last group is deliberately never sealed at EOF.
    return camera, imu, sum(n for n in counts.values() if n > 1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--prefix', required=True, type=Path)
    parser.add_argument('--domain-id', type=int, default=217)
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 232:
        parser.error('domain-id must be in [0, 232]')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    prefix, bag = args.prefix.resolve(strict=True), args.bag.resolve(strict=True)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY='1',
                      ROS_LOG_DIR=str(out / 'ros'))
    record = dict(scope=__doc__, error=None, forced=False, launches=[], passed=False)
    for package in ['easyfly_msgs', 'ov_core', 'ov_init', 'ov_msckf',
                    'easyfly_openvins_integration']:
        actual = Path(get_package_prefix(package)).resolve()
        assert actual == prefix / package, (package, actual, prefix)
    expected_camera, expected_imu, rejected = oracle(bag)
    rclpy.init()
    node = rclpy.create_node('openvins_installed_observer')
    process = player = None
    latest = None
    statuses, poses, images, imus, tf_messages = [], [], {}, {}, []
    image_sequence = []
    allow_latched = False

    def status(msg):
        nonlocal latest
        latest = msg
        statuses.append(dict(state=msg.state, pose_valid=msg.pose_valid,
                             session_id=msg.session_id, generation=msg.generation,
                             camera_forwarded=msg.camera_forwarded,
                             camera_rejected=msg.camera_rejected,
                             imu_forwarded=msg.imu_forwarded, imu_rejected=msg.imu_rejected,
                             pose_rejected=msg.pose_rejected, reason=msg.reason))

    def pose(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        poses.append(dict(stamp_ns=ns(msg), frame=msg.header.frame_id,
                          position=[p.x, p.y, p.z], quaternion=[q.x, q.y, q.z, q.w],
                          covariance=list(msg.pose.covariance)))

    def image(msg):
        image_sequence.append(ns(msg))
        images[ns(msg)] = message_digest(msg)

    node.create_subscription(LocalizationStatus, '/localization/openvins/status', status,
                             QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(PoseWithCovarianceStamped, '/localization/openvins/pose', pose, 100)
    node.create_subscription(Image, '/openvins/input/camera', image, 100)
    node.create_subscription(Imu, '/openvins/input/imu', lambda msg: imus.update(
        {ns(msg): message_digest(msg)}), 1000)
    node.create_subscription(TFMessage, '/tf', tf_messages.append, 10)
    node.create_subscription(TFMessage, '/tf_static', tf_messages.append,
                             QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    clock_pub = node.create_publisher(Clock, '/clock', QoSProfile(
        depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if process is not None and process.poll() is not None:
                raise RuntimeError('localization launch exited unexpectedly')
            rclpy.spin_once(node, timeout_sec=.01)
            if not allow_latched and latest is not None and latest.state in [
                    LocalizationStatus.RESET_REQUIRED, LocalizationStatus.FAILED]:
                raise RuntimeError('input/output admission failed: ' + latest.reason)

    def until(predicate, timeout=15):
        end = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= end:
                raise RuntimeError('condition timed out; last status: ' + str(statuses[-1:]))
            spin(.02)

    def start(name):
        nonlocal process
        command = ['ros2', 'launch', 'easyfly_openvins_integration', 'openvins.launch.py']
        with (out / (name + '.log')).open('w') as stream:
            process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        record['launches'].append(dict(name=name, command=command, pid=process.pid))
        until(lambda: ('run_subscribe_msckf', '/openvins/raw') in
              node.get_node_names_and_namespaces() and latest is not None)

    def stop():
        nonlocal process
        if process is not None:
            process.send_signal(signal.SIGINT)
            record['launches'][-1]['returncode'] = process.wait(timeout=15)
            assert process.returncode == 0
            process = None

    try:
        start('launch')
        command = ['ros2', 'bag', 'play', str(bag), '--rate', '.5', '--clock', '100',
                   '--delay', '2', '--read-ahead-queue-size', '1000']
        record['replay_command'] = command
        with (out / 'player.log').open('w') as log:
            player = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        until(lambda: player.poll() is not None, timeout=150)
        assert player.returncode == 0
        spin(.15)
        assert len(poses) > 100 and any(s['pose_valid'] for s in statuses)
        assert images == expected_camera, ('camera fields/selection mismatch', len(images),
                                           len(expected_camera))
        assert len(image_sequence) == len(images)
        assert all(b > a for a, b in zip(image_sequence, image_sequence[1:]))
        assert imus and imus == {t: d for t, d in expected_imu.items() if t >= min(imus)}
        assert latest.camera_rejected == rejected == 3
        assert latest.camera_forwarded == len(images)
        assert latest.imu_forwarded == len(imus)
        assert not latest.velocity_valid and not latest.tracking_quality_available
        assert not tf_messages, 'unexpected global TF output'
        assert all(math.isfinite(v) for p in poses
                   for key in ['position', 'quaternion', 'covariance'] for v in p[key])
        assert all(p['frame'] == 'openvins_odom' for p in poses)
        assert all(b['stamp_ns'] > a['stamp_ns'] for a, b in zip(poses, poses[1:]))
        session = latest.session_id
        until(lambda: latest.state == LocalizationStatus.STALE)
        assert not latest.pose_valid
        count = len(poses)
        allow_latched = True
        message = Clock()
        message.clock.sec = 1
        clock_pub.publish(message)
        until(lambda: latest.state == LocalizationStatus.RESET_REQUIRED)
        assert latest.generation == 1 and latest.session_id == session
        spin(.1)
        assert len(poses) == count and not latest.pose_valid
        stop()
        start('restart')
        until(lambda: latest.session_id != session)
        assert latest.generation == 0 and not latest.pose_valid
        # Terminate only our recorded upstream child; verify the launch also closes the guard.
        until(lambda: re.search(r'run_subscribe_msckf[^\n]*process started with pid',
                                (out / 'restart.log').read_text()) is not None)
        log = (out / 'restart.log').read_text()
        match = re.search(r'run_subscribe_msckf[^\n]*process started with pid \[(\d+)\]', log)
        assert match, log
        child = int(match.group(1))
        command_line = Path(f'/proc/{child}/cmdline').read_bytes()
        assert str(prefix / 'ov_msckf/lib/ov_msckf/run_subscribe_msckf').encode() in command_line
        os.kill(child, signal.SIGTERM)
        record['launches'][-1]['terminated_child'] = child
        record['launches'][-1]['returncode'] = process.wait(timeout=15)
        assert process.returncode == 0
        process = None
        record.update(passed=True, accepted_camera_count=len(images),
                      accepted_imu_count=len(imus), rejected_camera_count=rejected,
                      state_count=len(poses),
                      source_duration=(poses[-1]['stamp_ns'] - poses[0]['stamp_ns']) * 1e-9,
                      original_message_fields_and_image_bytes_preserved=True, eof_stale=True,
                      rollback_latched=True, new_session_on_restart=True,
                      upstream_exit_closes_group=True, no_global_tf=True)
    except Exception as exc:
        record['error'] = repr(exc)
    finally:
        for proc in [player, process]:
            if proc is not None and proc.poll() is None:
                record['forced'] = True
                if proc is process:
                    os.killpg(proc.pid, signal.SIGTERM)
                else:
                    proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    if proc is process:
                        os.killpg(proc.pid, signal.SIGKILL)
                    else:
                        proc.kill()
                    proc.wait(timeout=3)
        node.destroy_node()
        rclpy.shutdown()
    (out / 'poses.jsonl').write_text(''.join(json.dumps(p) + '\n' for p in poses))
    (out / 'statuses.json').write_text(json.dumps(statuses, indent=2) + '\n')
    record['log_sha256'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                            for p in out.glob('*.log')}
    (out / 'summary.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(record, indent=2))
    return 0 if record['passed'] and not record['forced'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Observe raw/guarded VIO inputs without launching or controlling the stack.

Run in the stack's ROS domain. Stores source stamps, callback receipt times,
all-field image/IMU digests and admission status; never stores full image data.
This observer cannot prove what another DDS reader received.
"""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import time

from easyfly_msgs.msg import LocalizationStatus
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu

from openvins_integration_probe import message_digest, ns


TOPICS = [
    ('image', '/camera/color/image_raw', Image, 32),
    ('info', '/camera/color/camera_info', CameraInfo, 32),
    ('imu', '/livox/imu', Imu, 1000),
    ('imu_best_effort', '/livox/imu', Imu, 1000),
    ('forwarded_image', '/openvins/input/camera', Image, 32),
    ('forwarded_imu', '/openvins/input/imu', Imu, 1000),
    ('status', '/localization/openvins/status', LocalizationStatus, 100),
    ('clock', '/clock', Clock, 1),
]


def describe(items):
    source = [b['stamp_ns'] - a['stamp_ns'] for a, b in zip(items, items[1:])]
    receipt = [b['receipt'] - a['receipt'] for a, b in zip(items, items[1:])]
    return dict(count=len(items), nonincreasing=sum(d <= 0 for d in source),
                max_source_gap_ns=max(source, default=0),
                max_callback_receipt_gap_s=max(receipt, default=0))


def compare(raw, forwarded):
    counts = Counter(m['stamp_ns'] for m in raw)
    by_stamp = {m['stamp_ns']: m for m in raw}
    accepted = {m['stamp_ns']: m for m in forwarded}
    digests = {}
    for item in raw:
        digests.setdefault(item['stamp_ns'], set()).add(item['digest'])
    window = (forwarded[0]['stamp_ns'], forwarded[-1]['stamp_ns']) if forwarded else None
    expected = {t for t in by_stamp if window and window[0] <= t <= window[1] and counts[t] == 1}
    return dict(observed_window_ns=window,
                observed_unique_raw_stamps_not_forwarded=sorted(expected - accepted.keys()),
                forwarded_stamps_not_observed_raw=sorted(accepted.keys() - by_stamp.keys()),
                observed_payload_mismatches=[t for t in accepted.keys() & by_stamp.keys()
                                             if accepted[t]['digest'] not in digests[t]],
                observed_ambiguous_raw_messages=sum(n for n in counts.values() if n > 1),
                observed_raw_stamps_with_different_payloads=sum(
                    len(values) > 1 for values in digests.values()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--max-samples', type=int, default=200000)
    parser.add_argument('--sensor-topology', action='store_true',
                        help='record read-only publisher graph snapshots every ten seconds')
    args = parser.parse_args()
    if not 1 <= args.seconds <= 300 or not 100 <= args.max_samples <= 1000000:
        parser.error('seconds must be in [1, 300], max-samples in [100, 1000000]')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    records = {key: [] for key, _, _, _ in TOPICS}
    count = 0
    rclpy.init()
    node = rclpy.create_node('openvins_input_trace')
    report = dict(scope=__doc__, error=None)
    try:
        with (out / 'samples.jsonl').open('w') as stream:
            def receive(message, key):
                nonlocal count
                if count >= args.max_samples:
                    return
                stamp = (message.clock.sec * 10**9 + message.clock.nanosec
                         if key == 'clock' else ns(message))
                item = dict(stamp_ns=stamp, receipt=time.monotonic())
                if key in ('image', 'imu', 'imu_best_effort', 'forwarded_image', 'forwarded_imu'):
                    item['digest'] = message_digest(message)
                    if isinstance(message, Image):
                        item.update(pixel_sha256=hashlib.sha256(bytes(message.data)).hexdigest(),
                                    width=message.width, height=message.height, step=message.step,
                                    encoding=message.encoding)
                    if isinstance(message, Imu):
                        item['values'] = [message.linear_acceleration.x,
                                          message.linear_acceleration.y,
                                          message.linear_acceleration.z,
                                          message.angular_velocity.x,
                                          message.angular_velocity.y,
                                          message.angular_velocity.z]
                elif key == 'info':
                    item.update(frame=message.header.frame_id, width=message.width,
                                height=message.height, k=list(message.k), d=list(message.d),
                                distortion_model=message.distortion_model)
                elif key == 'status':
                    for name in ('state', 'reason', 'session_id', 'generation', 'pose_valid',
                                 'camera_forwarded', 'camera_rejected', 'imu_forwarded',
                                 'imu_rejected', 'pose_rejected', 'last_rejection'):
                        item[name] = getattr(message, name)
                    for name in ('pose_stamp', 'camera_stamp', 'imu_stamp'):
                        value = getattr(message, name)
                        item[name + '_ns'] = value.sec * 10**9 + value.nanosec
                records[key].append(item)
                stream.write(json.dumps(dict(stream=key, **item), allow_nan=False) + '\n')
                count += 1

            subscriptions = []
            for key, topic, kind, depth in TOPICS:
                qos = QoSProfile(depth=depth)
                if key == 'status':
                    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
                if key in ('clock', 'imu_best_effort'):
                    qos.reliability = ReliabilityPolicy.BEST_EFFORT
                subscriptions.append(node.create_subscription(
                    kind, topic, lambda m, key=key: receive(m, key), qos))
            (out / 'ready').write_text('subscriptions ready\n')
            deadline = time.monotonic() + args.seconds
            next_topology = time.monotonic() + 20
            if args.sensor_topology:
                report['sensor_topology'] = []
            while time.monotonic() < deadline and count < args.max_samples:
                rclpy.spin_once(node, timeout_sec=.01)
                if args.sensor_topology and time.monotonic() >= next_topology:
                    report['sensor_topology'].append(dict(
                        receipt=time.monotonic(), publishers={
                            topic: [dict(name=p.node_name, namespace=p.node_namespace)
                                    for p in node.get_publishers_info_by_topic(topic)]
                            for topic in ['/livox/lidar', '/livox/imu', '/camera/color/image_raw',
                                          '/camera/aligned_depth_to_color/image_raw']},
                        nodes=sorted(namespace.rstrip('/') + '/' + name
                                     for name, namespace in node.get_node_names_and_namespaces())))
                    next_topology += 10
            report['sample_limit_reached'] = count >= args.max_samples
            report['publishers'] = {
                topic: [dict(name=p.node_name, namespace=p.node_namespace,
                             graph_depth=p.qos_profile.depth,
                             reliability=str(p.qos_profile.reliability))
                        for p in node.get_publishers_info_by_topic(topic)]
                for _, topic, _, _ in TOPICS}
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        report['interrupted'] = True
    except Exception as exc:
        report['error'] = repr(exc)
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        report.setdefault('sample_limit_reached', count >= args.max_samples)
        report['streams'] = {k: describe(v) for k, v in records.items()}
        report['comparison'] = {k: compare(records[k], records['forwarded_' + k])
                                for k in ('image', 'imu')}
        info_stamps = {m['stamp_ns'] for m in records['info']}
        report['observed_image_stamps_without_info'] = sorted(
            {m['stamp_ns'] for m in records['image']} - info_stamps)
        report['states'] = sorted({(m['state'], m['reason']) for m in records['status']})
        (out / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""SIGINT the actual estimator during recorded-image processing; never command a vehicle."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--repeats', type=int, default=12)
    parser.add_argument('--domain-id', type=int, default=225)
    args = parser.parse_args()
    if not 1 <= args.repeats <= 24 or not 0 <= args.domain_id <= 232:
        parser.error('invalid bounded repeat/domain count')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY='1')
    executable = Path(get_package_prefix('ov_msckf'))/'lib/ov_msckf/run_subscribe_msckf'
    report = dict(error=None, forced=False, cases=[], scope=__doc__)
    rclpy.init()
    node = rclpy.create_node('shutdown_stress_observer')
    try:
        for index in range(args.repeats):
            namespace = '/shutdown_'+str(index)
            estimator = player = subscription = None
            row = dict(index=index, namespace=namespace, processing_observed=False,
                       phase='processing' if index < 6 else 'estimating', pose_count=0)
            report['cases'].append(row)
            with (out/f'{index}.estimator.log').open('w') as elog, \
                    (out/f'{index}.player.log').open('w') as plog:
                try:
                    estimator = subprocess.Popen([
                        str(executable), str(args.config.resolve()), '--ros-args',
                        '-p', 'use_sim_time:=true', '-p', 'publish_global_to_imu_tf:=false',
                        '-p', 'publish_calibration_tf:=false', '-r', '__ns:='+namespace],
                        stdout=elog, stderr=subprocess.STDOUT)
                    row['estimator_pid'] = estimator.pid
                    deadline = time.monotonic()+15
                    while time.monotonic() < deadline:
                        if estimator.poll() is not None:
                            raise RuntimeError('estimator exited before input discovery')
                        rclpy.spin_once(node, timeout_sec=.02)
                        names = node.get_node_names_and_namespaces()
                        if ('run_subscribe_msckf', namespace) in names:
                            topics = dict(node.get_subscriber_names_and_types_by_node(
                                'run_subscribe_msckf', namespace))
                            if '/livox/imu' in topics and '/camera/color/image_raw' in topics:
                                break
                    else:
                        raise TimeoutError('estimator discovery deadline')

                    def on_pose(message):
                        row['pose_count'] += 1

                    subscription = node.create_subscription(
                        PoseWithCovarianceStamped, namespace+'/poseimu', on_pose, 100)
                    player = subprocess.Popen([
                        'ros2', 'bag', 'play', str(args.bag.resolve()), '--rate', '1',
                        '--clock', '100', '--delay', '.5'], stdout=plog, stderr=subprocess.STDOUT)
                    row['player_pid'] = player.pid
                    deadline = time.monotonic()+20
                    while time.monotonic() < deadline:
                        if estimator.poll() is not None:
                            raise RuntimeError('estimator exited during processing')
                        rclpy.spin_once(node, timeout_sec=.01)
                        if '[TIME]:' in (out/f'{index}.estimator.log').read_text():
                            row['processing_observed'] = True
                            if row['phase'] == 'processing' or row['pose_count']:
                                break
                    else:
                        raise TimeoutError(
                            'required actual processing/estimating phase not observed')
                    row['post_processing_delay_s'] = [0., .002, .01, .05, .2, .5][index % 6]
                    until = time.monotonic()+row['post_processing_delay_s']
                    while time.monotonic() < until:
                        rclpy.spin_once(node, timeout_sec=.001)
                    # Input continues during teardown, matching the original race's ownership.
                    estimator.send_signal(signal.SIGINT)
                    row['estimator_returncode'] = estimator.wait(timeout=15)
                    if estimator.returncode:
                        raise RuntimeError('estimator abnormal shutdown')
                    player.send_signal(signal.SIGINT)
                    row['player_returncode'] = player.wait(timeout=10)
                    if player.returncode:
                        raise RuntimeError('player abnormal shutdown')
                finally:
                    for process in (player, estimator):
                        if process is not None and process.poll() is None:
                            report['forced'] = True
                            process.terminate()
                            try:
                                process.wait(timeout=10)
                            except subprocess.TimeoutExpired:
                                process.kill()
                                process.wait(timeout=10)
                    if subscription is not None:
                        node.destroy_subscription(subscription)
            (out/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    except Exception as error:
        report['error'] = repr(error)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    report['passed'] = not report['error'] and not report['forced'] and \
        len(report['cases']) == args.repeats and all(
            row.get('processing_observed') and row.get('estimator_returncode') == 0 and
            row.get('player_returncode') == 0 and
            (row['phase'] == 'processing' or row['pose_count'] > 0) for row in report['cases'])
    (out/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""Exercise an installed FAST-LIO process with no sensor data or flight actions."""
import argparse
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    prefix, out = args.prefix.resolve(), args.output.resolve()
    if out.exists():
        parser.error('preserve existing runtime evidence')
    out.mkdir(parents=True)
    executable = prefix / 'fast_lio/lib/fast_lio/fastlio_mapping'
    config = prefix / 'fast_lio/share/fast_lio/config/mid360.yaml'
    record = {'cases': []}
    rclpy.init()
    peer = rclpy.create_node('alg81_fastlio_runtime_probe')
    process = None
    try:
        with (out / 'valid.log').open('w') as log:
            process = subprocess.Popen([str(executable), '--ros-args', '--params-file', str(config),
                                        '-p', 'use_sim_time:=false'], stdout=log, stderr=subprocess.STDOUT)
            client = peer.create_client(Trigger, '/map_save')
            deadline = time.monotonic() + 10.
            while True:
                assert process.poll() is None, (out / 'valid.log').read_text()
                assert time.monotonic() < deadline, (out / 'valid.log').read_text()
                rclpy.spin_once(peer, timeout_sec=.02)
                if client.service_is_ready() and ('laser_mapping', '/') in peer.get_node_names_and_namespaces():
                    topics = dict(peer.get_subscriber_names_and_types_by_node('laser_mapping', '/'))
                    if '/livox/lidar' in topics and '/livox/imu' in topics:
                        assert topics['/livox/lidar'] == ['livox_ros_driver2/msg/CustomMsg']
                        assert topics['/livox/imu'] == ['sensor_msgs/msg/Imu']
                        break
            # Do not call map_save, publish IMU/LiDAR, or contact the vehicle.
            process.send_signal(signal.SIGINT)
            assert process.wait(timeout=5.) == 0, (out / 'valid.log').read_text()
            assert 'Node init finished.' in (out / 'valid.log').read_text()
            record['cases'].append({'name': 'installed_config_topics_service_and_normal_exit', 'returncode': 0})
        for name, key, values, expected in [
            ('short_translation', 'T', '[0.0, 0.0]', 'exactly 3'),
            ('short_rotation', 'R', '[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]', 'exactly 9'),
            ('nonfinite_rotation', 'R', '[1.0, 0.0, 0.0, 0.0, .nan, 0.0, 0.0, 0.0, 1.0]', 'non-finite'),
        ]:
            params = out / (name + '.yaml')
            params.write_text('/**:\n  ros__parameters:\n    mapping.extrinsic_' + key + ': ' + values + '\n')
            with (out / (name + '.log')).open('w') as log:
                process = subprocess.Popen([str(executable), '--ros-args', '--params-file', str(config),
                                            '--params-file', str(params), '-p', 'use_sim_time:=false'],
                                           stdout=log, stderr=subprocess.STDOUT)
                code = process.wait(timeout=8.)
            output = (out / (name + '.log')).read_text()
            assert code == 1 and expected in output and 'failed to start' in output, output
            record['cases'].append({'name': name, 'returncode': code, 'expected_rejection': True})
        record['passed'] = True
        record['logs_sha256'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in out.glob('*.log')}
        (out / 'summary.json').write_text(json.dumps(record, indent=2) + '\n')
        print('PASS installed FAST-LIO initialization and three original extrinsic guards')
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait(timeout=2.)
        peer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

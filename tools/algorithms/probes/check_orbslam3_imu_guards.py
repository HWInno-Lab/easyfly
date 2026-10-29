#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Actual ROS guard contract tests with explicitly synthetic invalid inputs.

These cases never certify estimator accuracy, initialization, navigation or loops.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time



def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID='228', ROS_LOCALHOST_ONLY='1')
    from easyfly_fastdds_compat.runtime import activate
    activate(openvins=True)
    # Activation must precede imports that load the DDS shared library.
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from easyfly_msgs.msg import LocalizationStatus
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    executable = Path(get_package_prefix('easyfly_orbslam3_integration')) / 'lib/easyfly_orbslam3_integration/orbslam3_rgbd'
    template = Path(get_package_share_directory('easyfly_orbslam3_integration')) / 'config/gazebo_rgbd_imu.yaml'
    vocabulary = Path(get_package_share_directory('easyfly_orbslam3_core')) / 'vocabulary/ORBvoc.txt'
    rclpy.init()
    observer = rclpy.create_node('orbslam3_imu_guard_contract_test')
    statuses = []
    observer.create_subscription(LocalizationStatus, '/localization/orbslam3/status',
                                 statuses.append, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    clock = observer.create_publisher(Clock, '/clock', 16)
    imu = observer.create_publisher(Imu, '/livox/imu', 256)

    def wait(predicate, seconds=15):
        end = time.monotonic()+seconds
        while not predicate():
            if time.monotonic() >= end: raise TimeoutError('guard contract observation deadline')
            rclpy.spin_once(observer, timeout_sec=.02)

    def tick():
        message = Clock(); message.clock.sec = 100; clock.publish(message)

    records = []
    try:
        for case in ['nonzero_k3', 'wrong_extrinsic', 'wrong_frame', 'nonfinite', 'duplicate', 'source_gap']:
            folder = args.output/case; folder.mkdir(); runtime = folder/'runtime'; runtime.mkdir()
            settings = template.read_text()
            if case == 'nonzero_k3': settings = settings.replace('Camera1.k3: 0.0', 'Camera1.k3: 0.1')
            if case == 'wrong_extrinsic': settings = settings.replace('0.29350', '0.39350')
            config = folder/'settings.yaml'; config.write_text(settings)
            command = [str(executable), '--ros-args', '-p', 'use_sim_time:=true', '-p', 'sensor_mode:=rgbd_imu',
                       '-p', 'settings_template:='+str(config.resolve()), '-p', 'vocabulary:='+str(vocabulary),
                       '-p', 'runtime_directory:='+str(runtime.resolve())]
            row = dict(case=case, passed=False, synthetic_guard_test_only=True, command=command)
            console = (folder/'console.log').open('x'); proc = subprocess.Popen(command, stdout=console, stderr=subprocess.STDOUT)
            try:
                if case in ['nonzero_k3', 'wrong_extrinsic']:
                    proc.wait(timeout=15); console.flush()
                    reason = 'estimator/input calibration mismatch' if case=='nonzero_k3' else 'IMU/camera extrinsic mismatch'
                    row.update(expected_rejection=reason, passed=proc.returncode==1 and reason in (folder/'console.log').read_text())
                else:
                    previous_sessions = {s.session_id for s in statuses}
                    wait(lambda: any(s.session_id not in previous_sessions for s in statuses))
                    session = next(s.session_id for s in reversed(statuses) if s.session_id not in previous_sessions)
                    wait(lambda: observer.count_subscribers('/livox/imu')==1)
                    for _ in range(5):
                        tick(); rclpy.spin_once(observer, timeout_sec=.03)
                    message = Imu(); message.header.stamp.sec = 100
                    message.header.frame_id = 'livox_imu_frame'; message.linear_acceleration.z = 9.81
                    if case in ['duplicate', 'source_gap']:
                        imu.publish(message)
                        wait(lambda: any(s.session_id==session and s.imu_stamp.sec==100 for s in statuses))
                    if case=='wrong_frame': message.header.frame_id='incorrect_frame'
                    if case=='nonfinite': message.angular_velocity.x=float('nan')
                    if case=='source_gap': message.header.stamp.nanosec=25000000
                    imu.publish(message)
                    expected = dict(wrong_frame='invalid_imu_frame_or_values', nonfinite='invalid_imu_frame_or_values',
                                    duplicate='imu_rollback_or_duplicate', source_gap='imu_source_gap')[case]
                    wait(lambda: any(s.session_id==session and s.state==4 for s in statuses))
                    last = next(s for s in reversed(statuses) if s.session_id==session and s.state==4)
                    row.update(expected_rejection=expected, observed_reason=last.reason,
                               passed=last.reason==expected and not last.pose_valid and not last.covariance_available)
            except Exception as error:
                row['error'] = repr(error)
            finally:
                if proc.poll() is None: proc.send_signal(signal.SIGINT)
                proc.wait(timeout=30); console.close()
                row['returncode'] = proc.returncode
                if case not in ['nonzero_k3', 'wrong_extrinsic']:
                    atlas = json.loads((runtime/'atlas-final.json').read_text())
                    row['workers_joined'] = atlas['workers_joined']
                    row['passed'] = row['passed'] and proc.returncode==0 and atlas['workers_joined']
                (folder/'result.json').write_text(json.dumps(row, indent=2)+'\n')
                records.append(row)
                # Drain previous transient samples before the next fresh session.
                end = time.monotonic()+.5
                while time.monotonic()<end: rclpy.spin_once(observer, timeout_sec=.02)
    finally:
        observer.destroy_node(); rclpy.try_shutdown()
    result = dict(passed=all(row['passed'] for row in records) and len(records)==6, cases=records,
                  scope='synthetic ROS guard contract only, no algorithm accuracy or flight claim',
                  executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (args.output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Exercise real ORB startup, no image input, empty atlas save and normal worker shutdown."""
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
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    output = args.output.resolve()
    runtime = output / 'runtime'
    runtime.mkdir()
    from easyfly_fastdds_compat.runtime import activate
    activate(openvins=True)
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    import rclpy
    from easyfly_msgs.msg import LocalizationStatus
    from rclpy.qos import QoSProfile, DurabilityPolicy
    os.environ.update(ROS_DOMAIN_ID='228', ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(output / 'ros'))
    rclpy.init()
    node = rclpy.create_node('orb_empty_atlas_observer')
    samples = []
    node.create_subscription(LocalizationStatus, '/localization/orbslam3/status',
                             lambda m: samples.append(dict(state=m.state, pose_valid=m.pose_valid)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    exe = Path(get_package_prefix('easyfly_orbslam3_integration')) / 'lib/easyfly_orbslam3_integration/orbslam3_rgbd'
    share = Path(get_package_share_directory('easyfly_orbslam3_integration'))
    core = Path(get_package_share_directory('easyfly_orbslam3_core'))
    command = [str(exe), '--ros-args']
    for key, value in dict(use_sim_time='true', runtime_directory=runtime,
                           save_atlas=runtime / 'atlas',
                           vocabulary=core / 'vocabulary/ORBvoc.txt',
                           settings_template=share / 'config/gazebo_rgbd.yaml').items():
        command += ['-p', key + ':=' + str(value)]
    error = None
    forced = False
    with (output / 'console.log').open('x') as log:
        process = subprocess.Popen(command, env=dict(os.environ), stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 90
            while len(samples) < 3:
                if process.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError('real empty atlas startup deadline or early exit')
                rclpy.spin_once(node, timeout_sec=.05)
        except Exception as exc:
            error = repr(exc)
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    forced = True
                    process.kill()
                    process.wait(timeout=5)
    node.destroy_node()
    rclpy.try_shutdown()
    final = runtime / 'atlas-final.json'
    atlas = json.loads(final.read_text()) if final.exists() else {}
    checks = dict(normal_exit=process.returncode == 0 and not forced,
                  no_public_pose=bool(samples) and all(not s['pose_valid'] for s in samples),
                  no_fabricated_map=atlas.get('keyframes') == atlas.get('map_points') == 0,
                  workers_joined=atlas.get('workers_joined') is True,
                  empty_atlas_saved=(runtime / 'atlas.osa').is_file(), startup_error_absent=error is None)
    result = dict(passed=all(checks.values()), checks=checks, error=error, forced=forced,
                  returncode=process.returncode, atlas=atlas, command=command,
                  executable_sha256=hashlib.sha256(exe.read_bytes()).hexdigest(), hardware=False)
    (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

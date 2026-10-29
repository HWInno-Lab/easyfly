#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Replay original synchronized Gazebo sensors into real FAST-LIVO2; no flight commands."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.parameter import Parameter
from easyfly_openvins_integration.geometry import rotation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--flight", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    from easyfly_fastdds_compat.runtime import activate

    middleware = activate(openvins=True)
    env = dict(
        os.environ,
        ROS_DOMAIN_ID="228",
        ROS_LOCALHOST_ONLY="1",
        **middleware["profile_environment"]
    )
    os.environ.update(env)
    rclpy.init()
    node = rclpy.create_node(
        "fastlivo_replay_observer", parameter_overrides=[Parameter("use_sim_time", value=True)]
    )
    rows = []

    def pose(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        base = rotation(quat) @ [0.011, 0.02329, -0.12412] + [p.x, p.y, p.z]
        rows.append(
            dict(
                stream="local_body_pose",
                stamp_ns=m.header.stamp.sec * 10 ** 9 + m.header.stamp.nanosec,
                position=base.tolist(),
                quaternion=quat,
                receipt=time.monotonic(),
                clock_ns=node.get_clock().now().nanoseconds,
                frame="fastlivo_odom",
            )
        )

    def tracks(m):
        values = {v.key: v.value for v in m.status[0].values}
        rows.append(
            dict(
                stream="visual_tracking",
                stamp_ns=m.header.stamp.sec * 10 ** 9 + m.header.stamp.nanosec,
                tracked_features=int(values["tracked_features"]),
                initialized=values["initialized"] == "1",
                receipt=time.monotonic(),
            )
        )

    node.create_subscription(PoseWithCovarianceStamped, "/localization/fast_livo2/pose", pose, 100)
    node.create_subscription(DiagnosticArray, "/fastlivo/raw/visual_tracking", tracks, 32)
    log = (args.output / "console.log").open("x")
    launch = subprocess.Popen(
        [
            "ros2",
            "launch",
            "easyfly_fast_livo2_integration",
            "fastlivo.launch.py",
            "enable_navigation:=false",
        ],
        env=env,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    player = None
    error = None
    forced = False
    try:
        deadline = time.monotonic() + 15
        while not all(k in node.get_node_names() for k in ["fastlivo_guard", "laserMapping"]):
            if launch.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError("replay estimator startup failed")
            rclpy.spin_once(node, timeout_sec=0.1)
        player_log = (args.output / "player.log").open("x")
        player = subprocess.Popen(
            [
                "ros2",
                "bag",
                "play",
                str((args.flight / "input-bag").resolve()),
                "--rate",
                "0.5",
                "--topics",
                "/clock",
                "/livox/lidar",
                "/livox/imu",
                "/camera/color/image_raw",
                "/camera/color/camera_info",
            ],
            env=env,
            stdout=player_log,
            stderr=subprocess.STDOUT,
        )
        deadline = time.monotonic() + 900
        while player.poll() is None:
            if launch.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError("replay algorithm exited or playback deadline expired")
            rclpy.spin_once(node, timeout_sec=0.05)
        if player.returncode:
            raise RuntimeError("original sensor bag playback failed")
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    except Exception as exc:
        error = repr(exc)
    finally:
        for proc in [player, launch]:
            if proc is not None and proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    forced = True
                    os.killpg(launch.pid, signal.SIGTERM) if proc is launch else proc.terminate()
                    proc.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
        log.close()
        if player is not None:
            player_log.close()
    data = args.output / "vio.jsonl"
    data.write_text("".join(json.dumps(row) + "\n" for row in rows))
    (args.output / "truth.csv").symlink_to((args.flight / "truth.csv").resolve())
    limits = json.loads((args.flight.parent / "acceptance-limits.json").read_text())
    report = dict(
        scope="offline_replay_not_flight",
        error=error,
        forced=forced,
        launch_returncode=launch.returncode,
        player_returncode=player.returncode if player else None,
        passed=error is None
        and not forced
        and launch.returncode == 0
        and player is not None
        and player.returncode == 0,
        acceptance_limits=limits,
        original_flight=str(args.flight.resolve()),
        input_bag_manifest_sha256=hashlib.sha256(
            (args.flight / "input-bag/metadata.yaml").read_bytes()
        ).hexdigest(),
        estimator_executable_sha256=hashlib.sha256(
            (Path(get_package_prefix("fast_livo")) / "lib/fast_livo/fastlivo_mapping").read_bytes()
        ).hexdigest(),
    )
    (args.output / "result.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

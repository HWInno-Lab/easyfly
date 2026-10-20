#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""Observe actual OpenVINS subscriptions and normal exit without sensor input."""

import argparse
import json
from pathlib import Path
import signal
import subprocess
import time

import rclpy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--installation", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    installation = args.installation.resolve(strict=True)
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    executable = installation / "ov_msckf/lib/ov_msckf/run_subscribe_msckf"
    config = installation / "ov_msckf/share/ov_msckf/config/euroc_mav/estimator_config.yaml"
    command = [str(executable), str(config), "--ros-args", "-r", "__ns:=/openvins_build_probe"]
    record = dict(
        command=command, config=str(config), error=None, forced=False,
        scope=("EuRoC example for executable startup only; "
               "no sensor input or localization validation."),
    )
    process = None
    node = None
    try:
        rclpy.init()
        node = rclpy.create_node("openvins_build_observer")
        with (out / "subscriber.log").open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 20
            expected = {"/imu0": ["sensor_msgs/msg/Imu"],
                        "/cam0/image_raw": ["sensor_msgs/msg/Image"],
                        "/cam1/image_raw": ["sensor_msgs/msg/Image"]}
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError("OpenVINS exited before subscription discovery")
                rclpy.spin_once(node, timeout_sec=.05)
                expected_node = ("run_subscribe_msckf", "/openvins_build_probe")
                if expected_node not in node.get_node_names_and_namespaces():
                    continue
                subscriptions = dict(node.get_subscriber_names_and_types_by_node(
                    "run_subscribe_msckf", "/openvins_build_probe"))
                if all(subscriptions.get(topic) == types for topic, types in expected.items()):
                    record["subscriptions"] = subscriptions
                    record["publishers"] = dict(node.get_publisher_names_and_types_by_node(
                        "run_subscribe_msckf", "/openvins_build_probe"))
                    break
            else:
                raise RuntimeError("OpenVINS subscription discovery timed out")
            process.send_signal(signal.SIGINT)
            record["returncode"] = process.wait(timeout=10)
            if process.returncode != 0:
                raise RuntimeError("OpenVINS normal shutdown failed")
    except Exception as exc:
        record["error"] = repr(exc)
    finally:
        if process is not None and process.poll() is None:
            record["forced"] = True
            process.kill()
            process.wait(timeout=5)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    record["passed"] = record["error"] is None and not record["forced"]
    (out / "summary.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps(record, indent=2))
    return 0 if record["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

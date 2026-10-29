#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Acceptance-owned transparent live sensor relay; no synthetic messages or retiming."""
import argparse
import json
import signal
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image, Imu
from livox_ros_driver2.msg import CustomMsg
from rclpy.serialization import serialize_message
import hashlib


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kind", choices=["camera", "depth", "imu", "lidar"])
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    kind, topic = {
        "camera": (Image, "/camera/color/image_raw"),
        "depth": (Image, "/camera/aligned_depth_to_color/depth/image_raw"),
        "imu": (Imu, "/livox/imu"),
        "lidar": (CustomMsg, "/livox/lidar"),
    }[args.kind]
    requested = False

    def stop(*unused):
        nonlocal requested
        requested = True

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node = Node("acceptance_" + args.kind + "_relay")
    publisher = node.create_publisher(
        kind, "/acceptance/" + args.kind, 16 if args.kind != "imu" else 512
    )
    stream = args.output.open("x")

    def forward(m):
        data = serialize_message(m)
        publisher.publish(m)
        stamp = m.header.stamp.sec * 10 ** 9 + m.header.stamp.nanosec
        stream.write(
            json.dumps(dict(stamp_ns=stamp, sha256=hashlib.sha256(data).hexdigest())) + "\n"
        )

    node.create_subscription(kind, topic, forward, qos_profile_sensor_data)
    try:
        while not requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

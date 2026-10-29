#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Simulation-only checks for RC relay and odometry interfaces lacking behavior tests."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from easyfly_msgs.msg import UAVState
from mavros_msgs.msg import RCIn
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage

import tempfile

os.environ["ROS_DOMAIN_ID"] = "224"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

TEMP = tempfile.TemporaryDirectory(prefix="easyfly-interface-regression-")
ROOT = Path(TEMP.name)
out = ROOT / "uncovered-interfaces"
out.mkdir(exist_ok=False)
records = []
processes = []
logs = []
rclpy.init()
node = rclpy.create_node("easyfly_full_test_interfaces")
odometry = []
transforms = []
channels = []
subscriptions = [
    node.create_subscription(Odometry, "/odom", odometry.append, 20),
    node.create_subscription(TFMessage, "/tf", transforms.append, 20),
    node.create_subscription(Float32MultiArray, "/rc_channels", channels.append, 20),
]
state_pub = node.create_publisher(UAVState, "/uav_state", 10)
rc_pub = node.create_publisher(RCIn, "/mavros/rc/in", qos_profile_sensor_data)


def spin(seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)


def start(package, executable):
    log = (out / (executable + ".log")).open("w")
    logs.append(log)
    command = [
        str(Path(get_package_prefix(package)) / "lib" / package / executable),
        "--ros-args",
        "-p",
        "use_sim_time:=false",
    ]
    p = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append((executable, p))


def check(name, passed, **evidence):
    records.append(dict(name=name, passed=bool(passed), **evidence))
    (out / "result.json").write_text(json.dumps(records, indent=2, allow_nan=False) + "\n")
    print(name, "PASS" if passed else "FAIL", flush=True)


def send_state(s):
    s.header.stamp = node.get_clock().now().to_msg()
    for _ in range(8):
        state_pub.publish(s)
        spin(0.04)
    return [m for m in odometry if m.header.stamp == s.header.stamp]


try:
    start("easyfly_coord_transform", "odom_tf_publisher")
    start("easyfly_rc_receiver", "rc_receiver_node")
    spin(1.0)
    check("no_odom_before_state", not odometry)
    for values in ([1000, 1500, 2000], [], [0, 65535] + [1500] * 16):
        channels.clear()
        message = RCIn()
        message.channels = list(values)
        for _ in range(10):
            rc_pub.publish(message)
            spin(0.04)
        received = list(channels[-1].data) if channels else None
        check(
            "rc_relay_" + str(len(values)),
            received == [float(x) for x in values],
            expected=list(values),
            actual=received,
        )
    cases = [
        ("identity", (0.0, 0.0, 0.0), (1.0, 2.0, 3.0), (1.0, 2.0, 3.0)),
        ("yaw_90", (0.0, 0.0, math.pi / 2), (1.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
        ("pitch_90", (0.0, math.pi / 2, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
        ("roll_90", (math.pi / 2, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0)),
    ]
    for name, angles, velocity, expected in cases:
        s = UAVState()
        s.header.frame_id = "odom"
        s.connected = s.pose_valid = s.velocity_valid = s.attitude_valid = True
        s.position_x, s.position_y, s.position_z = 1.0, 2.0, 3.0
        s.roll, s.pitch, s.yaw = angles
        s.velocity_x, s.velocity_y, s.velocity_z = velocity
        replies = send_state(s)
        actual = None
        if replies:
            v = replies[-1].twist.twist.linear
            actual = [v.x, v.y, v.z]
        check(
            "odom_velocity_" + name,
            actual is not None and max(abs(a - b) for a, b in zip(actual, expected)) < 1e-6,
            input_rpy=angles,
            input_world_velocity=velocity,
            expected_body_velocity=expected,
            actual_body_velocity=actual,
        )
    for kind in [
        "pose_invalid",
        "attitude_invalid",
        "nan_position",
        "wrong_frame",
        "velocity_invalid",
        "disconnected",
        "nan_velocity",
        "nan_attitude",
        "nan_rate",
    ]:
        s = UAVState()
        s.header.frame_id = "odom"
        s.connected = s.pose_valid = s.velocity_valid = s.attitude_valid = True
        s.position_x, s.position_y, s.position_z = 7.0, 8.0, 9.0
        if kind == "pose_invalid":
            s.pose_valid = False
        elif kind == "attitude_invalid":
            s.attitude_valid = False
        elif kind == "nan_position":
            s.position_x = float("nan")
        elif kind == "velocity_invalid":
            s.velocity_valid = False
        elif kind == "disconnected":
            s.connected = False
        elif kind == "nan_velocity":
            s.velocity_x = float("nan")
        elif kind == "nan_attitude":
            s.yaw = float("nan")
        elif kind == "nan_rate":
            s.yawspeed = float("nan")
        else:
            s.header.frame_id = "unexpected_map"
        replies = send_state(s)
        tf_count = sum(
            1
            for msg in transforms
            for t in msg.transforms
            if t.header.stamp == s.header.stamp and t.child_frame_id == "base_link"
        )
        check(
            "reject_" + kind,
            not replies and tf_count == 0,
            odom_count=len(replies),
            tf_count=tf_count,
            output_frame=replies[-1].header.frame_id if replies else None,
            contains_nonfinite=bool(
                replies and not math.isfinite(replies[-1].pose.pose.position.x)
            ),
        )
    s = UAVState()
    s.header.frame_id = "odom"
    s.connected = s.pose_valid = s.velocity_valid = s.attitude_valid = True
    check("valid_input_recovers_output", bool(send_state(s)))
    spin(0.7)
    odometry.clear()
    transforms.clear()
    spin(0.2)
    check("stopped_state_stops_odom_and_tf", not odometry and not transforms)
finally:
    for name, process in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            rc = process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            rc = process.wait(timeout=5)
        check(name + "_shutdown", rc == 0, returncode=rc)
    node.destroy_node()
    rclpy.shutdown()
    for log in logs:
        log.close()
raise SystemExit(0 if all(r["passed"] for r in records) else 1)

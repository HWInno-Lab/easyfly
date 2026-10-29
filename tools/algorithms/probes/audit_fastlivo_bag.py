#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Compare every forwarded observation to original raw fields, excluding CDR padding."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import struct
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def fingerprint(m):
    stamp = m.header.stamp.sec * 10 ** 9 + m.header.stamp.nanosec
    h = hashlib.sha256()
    if hasattr(m, "points"):
        metadata = [m.header.frame_id, stamp, m.timebase, m.point_num, m.lidar_id, [int(v) for v in m.rsvd]]
        for p in m.points:
            h.update(
                struct.pack(
                    "<IfffBBB", p.offset_time, p.x, p.y, p.z, p.reflectivity, p.tag, p.line
                )
            )
    elif hasattr(m, "encoding"):
        metadata = [
            m.header.frame_id,
            stamp,
            m.width,
            m.height,
            m.encoding,
            m.is_bigendian,
            m.step,
        ]
        h.update(bytes(m.data))
    else:
        metadata = [m.header.frame_id, stamp]
        values = [getattr(m.orientation, k) for k in ("x", "y", "z", "w")] + list(
            m.orientation_covariance
        )
        values += [getattr(m.angular_velocity, k) for k in ("x", "y", "z")] + list(
            m.angular_velocity_covariance
        )
        values += [getattr(m.linear_acceleration, k) for k in ("x", "y", "z")] + list(
            m.linear_acceleration_covariance
        )
        h.update(struct.pack("<" + "d" * len(values), *values))
    h.update(json.dumps(metadata, separators=(",", ":")).encode())
    return stamp, h.hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("flight", type=Path)
    a = p.parse_args()
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str((a.flight / "input-bag").resolve()), storage_id="sqlite3"
        ),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {v.name: get_message(v.type) for v in reader.get_all_topics_and_types()}
    topics = {
        "/camera/color/image_raw": ("camera", False),
        "/livox/imu": ("imu", False),
        "/livox/lidar": ("lidar", False),
        "/fastlivo/input/camera": ("camera", True),
        "/fastlivo/input/imu": ("imu", True),
        "/fastlivo/input/lidar": ("lidar", True),
    }
    raw = {}
    forwarded = {}
    counts = Counter()
    duplicates = []
    while reader.has_next():
        topic, data, record_time = reader.read_next()
        if topic not in topics:
            continue
        kind, is_forwarded = topics[topic]
        m = deserialize_message(data, types[topic])
        stamp, digest = fingerprint(m)
        key = (kind, stamp)
        target = forwarded if is_forwarded else raw
        if key in target:
            duplicates.append(dict(topic=topic, stamp_ns=stamp, conflicting=target[key] != digest))
        target[key] = digest
        counts[topic] += 1
    mismatches = [
        dict(kind=k[0], stamp_ns=k[1], raw_missing=k not in raw)
        for k, d in forwarded.items()
        if raw.get(k) != d
    ]
    report = dict(
        passed=not mismatches and not duplicates and all(counts[k] > 100 for k in topics),
        raw_observations=len(raw),
        forwarded_observations=len(forwarded),
        counts=dict(counts),
        mismatches=mismatches,
        duplicates=duplicates,
        comparison="all message fields, original header/source stamps; CDR padding excluded",
    )
    (a.flight / "input-fidelity.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

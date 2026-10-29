#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Audit original native IMU samples after a completed, stopped capture."""
import argparse
import json
from pathlib import Path
import sqlite3
import math
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('flight', type=Path)
    args = parser.parse_args()
    result = json.loads((args.flight / 'result.json').read_text())
    if not result['passed'] or result['recorder_returncode'] != 0:
        raise ValueError('successful completed recording required')
    connection = sqlite3.connect('file:' + str(args.flight / 'rgbd-bag/rgbd-bag_0.db3') + '?mode=ro', uri=True)
    tid = connection.execute('select id from topics where name=?', ('/livox/imu',)).fetchone()
    stamps = []; invalid = []; maximum_gap = 0
    if tid:
        for raw, in connection.execute('select data from messages where topic_id=? order by id', tid):
            message = deserialize_message(raw, Imu)
            source = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            values = [getattr(v, axis) for v in [message.linear_acceleration, message.angular_velocity] for axis in ['x', 'y', 'z']]
            if message.header.frame_id != 'livox_imu_frame' or not all(math.isfinite(v) for v in values) or source <= 0:
                invalid.append(source)
            if stamps:
                if source <= stamps[-1]: invalid.append(source)
                maximum_gap = max(maximum_gap, source - stamps[-1])
            stamps.append(source)
    connection.close()
    report = dict(passed=len(stamps) >= 1000 and not invalid and maximum_gap <= 20000000,
                  samples=len(stamps), invalid=invalid, maximum_source_gap_ns=maximum_gap,
                  source_topic='/livox/imu', frame='livox_imu_frame',
                  measured_hz=(len(stamps)-1)*1e9/(stamps[-1]-stamps[0]) if len(stamps)>1 else None,
                  method='all original messages; no interpolation, retiming, deduplication or input substitution')
    (args.flight / 'imu-input-audit.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

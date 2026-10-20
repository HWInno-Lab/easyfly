#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Compare native Gazebo and ROS camera timestamps and pixel identities.

Independent observers have different start/end coverage. Native image transport
and ROS DDS use separate paths; this is an identity comparison, not wire timing.
"""

import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native', type=Path, required=True)
    parser.add_argument('--python', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with args.native.open() as stream:
        metadata = stream.readline().strip()
        native = list(csv.DictReader(stream))
    assert metadata.startswith('# CLOCK_MONOTONIC ')
    ros = [json.loads(line) for line in args.python.read_text().splitlines()]
    raw = [r for r in ros if r['stream'] == 'image']
    forwarded = [r for r in ros if r['stream'] == 'forwarded_image']

    def identity(row, native=False):
        return (int(row['stamp_ns']), row['sha256' if native else 'pixel_sha256'],
                int(row['width']), int(row['height']), int(row['step']))

    def ambiguity(rows, native=False):
        groups = defaultdict(set)
        counts = Counter()
        for row in rows:
            stamp, digest, *_ = identity(row, native)
            groups[stamp].add(digest)
            counts[stamp] += 1
        return dict(count=len(rows), unique_stamps=len(groups),
                    duplicate_stamp_groups=sum(n > 1 for n in counts.values()),
                    different_pixel_groups=sum(len(v) > 1 for v in groups.values()))

    n = Counter(identity(row, True) for row in native)
    r = Counter(identity(row) for row in raw)
    f = Counter(identity(row) for row in forwarded)
    report = dict(scope=__doc__, metadata=metadata, native=ambiguity(native, True),
                  raw_ros=ambiguity(raw), forwarded_ros=ambiguity(forwarded),
                  native_formats=dict(Counter(row['format'] for row in native)),
                  ros_encodings=dict(Counter(row['encoding'] for row in raw)),
                  native_raw_ros_matched_messages=sum((n & r).values()),
                  raw_ros_without_native_identity=sum((r-n).values()),
                  native_without_raw_ros_identity=sum((n-r).values()),
                  forwarded_without_raw_ros_identity=sum((f-r).values()),
                  forwarded_without_native_identity=sum((f-n).values()))
    common_ambiguous = defaultdict(set)
    for stamp, digest, *_ in n.keys() & r.keys():
        common_ambiguous[stamp].add(digest)
    report['common_stamps_with_multiple_identical_native_ros_pixels'] = sum(
        len(v) > 1 for v in common_ambiguous.values())
    report['source_sha256'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in (args.native, args.python)}
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()

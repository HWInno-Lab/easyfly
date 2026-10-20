#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Reject ambiguous duplicate-stamp groups from a recorded camera/IMU bag.

Keeps the original bag and every retained CDR message and storage timestamp intact.
Rejects all messages in a duplicate group rather than choosing an arbitrary image.
This bounded data preparation step is not a production sensor adapter.
"""

import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


def reader(path):
    result = rosbag2_py.SequentialReader()
    result.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    return result


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(chunk)
    return result.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--max-drop-fraction', type=float, default=.01)
    args = parser.parse_args()
    if not 0 <= args.max_drop_fraction <= .01:
        parser.error('max-drop-fraction must be in [0, .01]')
    source = args.input.resolve(strict=True)
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    first = reader(source)
    topics = first.get_all_topics_and_types()
    types = {t.name: get_message(t.type) for t in topics}
    counts = defaultdict(Counter)
    total = Counter()
    # First pass records only header identities; does not retain large image arrays.
    while first.has_next():
        topic, data, stamp = first.read_next()
        msg = deserialize_message(data, types[topic])
        header = msg.header.stamp
        source_stamp = header.sec * 10**9 + header.nanosec
        if stamp != source_stamp:
            raise ValueError('this profile requires bag storage time to equal source time')
        counts[topic][source_stamp] += 1
        total[topic] += 1
    del first
    rejected = {topic: {stamp for stamp, n in values.items() if n > 1}
                for topic, values in counts.items()}
    dropped = {topic: sum(counts[topic][t] for t in stamps)
               for topic, stamps in rejected.items()}
    record = dict(scope=__doc__.strip(), source=str(source), passed=False,
                  input_counts=dict(total), dropped=dropped,
                  rejected_stamp_groups={t: sorted(v) for t, v in rejected.items()},
                  max_drop_fraction=args.max_drop_fraction)
    for topic, number in dropped.items():
        if number > args.max_drop_fraction * total[topic]:
            record['error'] = 'excessive duplicate-stamp rejection: ' + topic
            (out / 'summary.json').write_text(json.dumps(record, indent=2) + '\n')
            raise ValueError(record['error'])
    second = reader(source)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(out / 'bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for topic in topics:
        writer.create_topic(topic)
    last = {}
    kept = Counter()
    # Digest includes topic, timestamp and exact serialized message bytes in order.
    retained_digest = hashlib.sha256()
    while second.has_next():
        topic, data, stamp = second.read_next()
        if stamp in rejected[topic]:
            continue
        if stamp <= last.get(topic, -1):
            raise ValueError('nonincreasing retained source time: ' + topic)
        writer.write(topic, data, stamp)
        retained_digest.update(topic.encode() + str(stamp).encode() + data)
        last[topic] = stamp
        kept[topic] += 1
    del writer, second
    check = reader(out / 'bag')
    output_digest = hashlib.sha256()
    while check.has_next():
        topic, data, stamp = check.read_next()
        output_digest.update(topic.encode() + str(stamp).encode() + data)
    del check
    assert retained_digest.digest() == output_digest.digest(), 'retained CDR identity changed'
    record.update(passed=True, kept=dict(kept),
                  retained_cdr_digest=retained_digest.hexdigest(),
                  original_files_sha256={
                      p.name: digest(p) for p in source.iterdir() if p.is_file()},
                  output_files_sha256={
                      p.name: digest(p) for p in (out / 'bag').iterdir() if p.is_file()})
    (out / 'summary.json').write_text(json.dumps(record, indent=2) + '\n')
    fields = ['passed', 'input_counts', 'dropped', 'kept']
    print(json.dumps({k: record[k] for k in fields}, indent=2))


if __name__ == '__main__':
    main()

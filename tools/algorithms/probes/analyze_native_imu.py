#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Compare native middleware timestamps/notifications with executor receipts.

Reception metadata is middleware-specific; notification is data availability,
not a wire capture. All traces must belong to one host and session.
"""

import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path

from analyze_imu_delivery import analyze, distribution


NAMES = {0: 'imu', 1: 'imu_best_effort', 2: 'forwarded_imu'}


def table(path):
    with path.open() as stream:
        metadata = stream.readline().strip()
        rows = list(csv.DictReader(stream))
    assert metadata.startswith('# CLOCK_MONOTONIC '), metadata
    attributes = dict(part.split('=', 1) for part in metadata.split() if '=' in part)
    return attributes, rows


def guard_audit(path):
    """Keep reception, take and callback latency distinct; no wire-time claim."""
    trace = json.loads(path.read_text())
    records = trace['records']
    result = dict(capacity=trace['capacity'], discarded=trace['discarded'], topics={},
                  callback_duration_s={})

    def sample(row):
        return dict(stamp_ns=row['stamp_ns'], source_ns=row['source_timestamp'],
                    received_ns=row['received_timestamp'], take_real_ns=row['real_ns'],
                    source_to_reception_s=(
                        row['received_timestamp']-row['source_timestamp'])/1e9,
                    reception_to_take_s=(row['real_ns']-row['received_timestamp'])/1e9)

    topics = sorted({r['topic'] for r in records if r['kind'] == 'take'})
    for topic in topics:
        takes = [r for r in records if r['kind'] == 'take' and r['topic'] == topic]
        valid = [r for r in takes if r['source_timestamp'] > 0 and r['received_timestamp'] > 0]
        item = dict(count=len(takes), supported_timestamp_count=len(valid),
                    source_to_rmw_reception_s=distribution([
                        (r['received_timestamp']-r['source_timestamp'])/1e9 for r in valid]),
                    rmw_reception_to_take_s=distribution([
                        (r['real_ns']-r['received_timestamp'])/1e9 for r in valid]),
                    take_duration_s=distribution([
                        (r['end_ns']-r['begin_ns'])/1e9 for r in takes]))
        late = next((i for i, r in enumerate(valid)
                     if r['received_timestamp']-r['source_timestamp'] > 500000000), None)
        item['first_reception_over_500ms_and_following'] = (
            [sample(r) for r in valid[late:late+2]] if late is not None else [])
        if valid:
            worst = max(range(len(valid)), key=lambda i:
                        valid[i]['received_timestamp']-valid[i]['source_timestamp'])
            item['worst_reception_and_following'] = [sample(r) for r in valid[worst:worst+2]]
        result['topics'][topic] = item
    for kind in sorted({r['kind'] for r in records if r['kind'] != 'take'}):
        result['callback_duration_s'][kind] = distribution([
            (r['end_ns']-r['begin_ns'])/1e9 for r in records if r['kind'] == kind])
    result['first_failure'] = next((r for r in records if r.get('failure')), None)
    result['interpretation'] = (
        'RMW reception metadata is not packet capture. Reception-to-take includes '
        'middleware availability/ordering and executor scheduling, not just Python cost.')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native', type=Path, required=True)
    parser.add_argument('--python', type=Path)
    parser.add_argument('--producer', type=Path)
    parser.add_argument('--guard', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    metadata, raw = table(args.native)
    event_metadata, events_raw = table(Path(str(args.native) + '.events.csv'))
    rows = []
    for item in raw:
        row = {k: int(v) for k, v in item.items()
               if k not in ('gid', 'ax', 'ay', 'az', 'wx', 'wy', 'wz')}
        row['gid'] = item['gid']
        row['values'] = [float(item[k]) for k in ('ax', 'ay', 'az', 'wx', 'wy', 'wz')]
        rows.append(row)
    events = [{k: int(v) for k, v in item.items()} for item in events_raw]
    python = {name: [] for name in NAMES.values()}
    if args.python:
        for line in args.python.read_text().splitlines():
            item = json.loads(line)
            if item['stream'] in python:
                python[item['stream']].append(item)
    report = dict(scope=__doc__, metadata=metadata, event_metadata=event_metadata, streams={})
    callbacks = {}
    for stream, name in NAMES.items():
        samples = [r for r in rows if r['stream'] == stream]
        notices = [e for e in events if e['stream'] == stream]
        gids = Counter(r['gid'] for r in samples)
        sequence = [r['sequence'] for r in samples]
        valid = [r for r in samples if r['rmw_source_ns'] > 0 and r['rmw_received_ns'] > 0]
        # Expand bounded notification counts only for a single writer, complete
        # trace and ordered sequence. A trailing shutdown notice is not a sample.
        announced = sum(e['count'] for e in notices)
        ordered = all(b > a for a, b in zip(sequence, sequence[1:]))
        association = len(gids) == 1 and ordered and int(metadata['discarded']) == 0 and \
            int(event_metadata['discarded']) == 0 and \
            len(samples) <= announced <= len(samples) + 4
        notify_times = []
        if association:
            for event in notices:
                notify_times.extend([event['mono_ns']] * min(event['count'], len(samples)))
            notify_times = notify_times[:len(samples)]
        notification_lag = [(r['mono_ns']-e)/1e9 for r, e in zip(samples, notify_times)]
        item = dict(count=len(samples), publisher_gids=dict(gids),
                    supported_rmw_timestamp_count=len(valid),
                    source_to_rmw_reception_s=distribution([
                        (r['rmw_received_ns']-r['rmw_source_ns'])/1e9 for r in valid]),
                    rmw_reception_to_cpp_callback_s=distribution([
                        (r['real_ns']-r['rmw_received_ns'])/1e9 for r in valid]),
                    realtime_minus_monotonic_span_ns=(
                        max(r['real_ns']-r['mono_ns'] for r in samples) -
                        min(r['real_ns']-r['mono_ns'] for r in samples) if samples else 0),
                    notification_count=len(notices), notification_announced_samples=announced,
                    notification_to_callback_association=association,
                    notification_to_cpp_callback_s=distribution(notification_lag),
                    notification_gap_s=distribution([
                        (b['mono_ns']-a['mono_ns'])/1e9 for a, b in zip(notices, notices[1:])]),
                    callback_gap_s=distribution([(b['mono_ns']-a['mono_ns'])/1e9
                                                 for a, b in zip(samples, samples[1:])]),
                    publication_sequence_nonincreasing=sum(
                        b <= a for a, b in zip(sequence, sequence[1:])))
        py = {m['stamp_ns']: m for m in python[name]}
        native = {r['stamp_ns']: r for r in samples}
        common = py.keys() & native.keys()
        item['python_common_samples'] = len(common)
        item['python_minus_cpp_callback_s'] = distribution([
            py[t]['receipt']-native[t]['mono_ns']/1e9 for t in common])
        item['python_values_compared'] = sum('values' in py[t] for t in common)
        item['python_value_mismatches'] = [t for t in common if 'values' in py[t] and
                                           py[t]['values'] != native[t]['values']]
        report['streams'][name] = item
        callbacks['cpp_' + name] = [
            dict(stamp_ns=r['stamp_ns'], receipt=r['mono_ns']/1e9) for r in samples]
    if args.producer:
        producer_meta, producer = table(args.producer)
        report['producer_metadata'] = producer_meta
        report['producer_comparison'] = analyze(
            [{k: int(v) for k, v in row.items()} for row in producer],
            {**callbacks, **{'python_' + k: v for k, v in python.items()}})
    if args.guard:
        report['guard_audit'] = guard_audit(args.guard)
    paths = [args.native, Path(str(args.native) + '.events.csv')]
    paths += [p for p in (args.python, args.producer, args.guard) if p]
    report['source_sha256'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    print(json.dumps(report['streams'], indent=2))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Match opt-in Gazebo IMU producer timings with independent observer receipts.

Both traces must be from the same session and host CLOCK_MONOTONIC clock.
Accepts openvins_input_trace JSONL or
legacy capture_timing JSON. Publication return does not certify DDS delivery.
"""

import argparse
from bisect import bisect_left, bisect_right
import csv
import hashlib
import json
from pathlib import Path


STREAMS = ('imu', 'imu_best_effort', 'forwarded_imu')


def distribution(values):
    ordered = sorted(values)
    if not ordered:
        return dict(count=0)
    return dict(count=len(values), minimum=min(values), maximum=max(values),
                p50=ordered[round((len(values)-1)*.5)],
                p95=ordered[round((len(values)-1)*.95)],
                p99=ordered[round((len(values)-1)*.99)])


def analyze(producer, receivers):
    for sample in producer:
        assert 0 < sample['entry_ns'] <= sample['snapshot_end_ns']
        if sample['result'] == 2:
            assert sample['snapshot_end_ns'] <= sample['publish_begin_ns'] <= \
                sample['publish_end_ns']
    sent = [m for m in producer if m['result'] == 2]
    assert sent, 'no published samples in trace'
    assert all(b['entry_ns'] >= a['entry_ns'] for a, b in zip(sent, sent[1:]))
    stamps = [m['source_ns'] for m in sent]
    assert len(stamps) == len(set(stamps)), 'ambiguous producer stamps; cannot pair by stamp'
    by_stamp = {m['source_ns']: m for m in sent}
    begins = [m['publish_begin_ns'] for m in sent]
    report = dict(producer=dict(
        callback_count=len(producer), published_count=len(sent),
        source_nonincreasing=sum(b <= a for a, b in zip(stamps, stamps[1:])),
        source_gap_s=distribution([(b-a)/1e9 for a, b in zip(stamps, stamps[1:])]),
        entry_gap_s=distribution([(b['entry_ns']-a['entry_ns'])/1e9
                                  for a, b in zip(sent, sent[1:])]),
        snapshot_duration_s=distribution([(m['snapshot_end_ns']-m['entry_ns'])/1e9 for m in sent]),
        publish_duration_s=distribution([(m['publish_end_ns']-m['publish_begin_ns'])/1e9
                                         for m in sent])), receivers={})
    for key, items in receivers.items():
        matched = [(m, by_stamp[m['stamp_ns']]) for m in items if m['stamp_ns'] in by_stamp]
        delays = [m['receipt']-p['publish_begin_ns']/1e9 for m, p in matched]
        gaps = [(b['receipt']-a['receipt'], a, b) for a, b in zip(items, items[1:])]
        events = []
        for gap, a, b in sorted(gaps, key=lambda v: v[0], reverse=True)[:20]:
            lo, hi = bisect_left(begins, round(a['receipt']*1e9)), \
                bisect_right(begins, round(b['receipt']*1e9))
            events.append(dict(receipt_gap_s=gap, previous_stamp_ns=a['stamp_ns'],
                               next_stamp_ns=b['stamp_ns'],
                               producer_publish_calls_during_gap=hi-lo,
                               next_receipt_minus_publish_begin_s=(
                                   b['receipt']-by_stamp[b['stamp_ns']]['publish_begin_ns']/1e9
                                   if b['stamp_ns'] in by_stamp else None)))
        report['receivers'][key] = dict(
            count=len(items), matched_count=len(matched),
            unmatched_stamps=[m['stamp_ns'] for m in items if m['stamp_ns'] not in by_stamp][:20],
            receipt_minus_publish_begin_s=distribution(delays),
            receipt_before_publish_begin=sum(d < -.000001 for d in delays),
            delay_over_100ms=sum(d > .1 for d in delays),
            delay_over_500ms=sum(d > .5 for d in delays),
            largest_receipt_gaps=events)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--producer', type=Path, required=True)
    parser.add_argument('--receivers', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with args.producer.open() as stream:
        metadata = stream.readline().strip()
        assert metadata.startswith('# CLOCK_MONOTONIC '), metadata
        producer = [{k: int(v) for k, v in row.items()} for row in csv.DictReader(stream)]
    receivers = {k: [] for k in STREAMS}
    if args.receivers.suffix == '.jsonl':
        for line in args.receivers.read_text().splitlines():
            item = json.loads(line)
            if item['stream'] in receivers:
                receivers[item['stream']].append(item)
    else:
        source = json.loads(args.receivers.read_text())
        for key in receivers:
            receivers[key] = [dict(stamp_ns=m['t'], receipt=m['wall'])
                              for m in source.get(key, [])]
    report = analyze(producer, receivers)
    report['trace_metadata'] = metadata
    report['trace_truncated'] = int(metadata.rsplit('discarded=', 1)[1]) > 0
    report['scope'] = __doc__
    report['source_sha256'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in (args.producer, args.receivers)}
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    print(json.dumps(dict(producer=report['producer'], receivers={
        k: {field: value for field, value in v.items() if field != 'largest_receipt_gaps'}
        for k, v in report['receivers'].items()}), indent=2))


if __name__ == '__main__':
    main()

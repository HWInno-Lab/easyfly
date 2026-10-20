#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Aggregate managed rendering/recovery comparisons and source production budgets.

Finite stationary simulations only. Timeline correlation does not establish
which component caused a stall, and receipt metadata is not a packet capture.
"""

import argparse
from collections import defaultdict
import csv
import hashlib
import json
from pathlib import Path
import re
import statistics

from analyze_imu_delivery import distribution
from analyze_vio_source import analyze


def timeline(directory, result):
    with (directory / 'native.csv').open() as stream:
        next(stream)
        offsets = [int(r['real_ns'])-int(r['mono_ns']) for r in csv.DictReader(stream)]
    offset = int(statistics.median(offsets))
    producer, = directory.glob('producer.*.csv')
    with producer.open() as stream:
        next(stream)
        samples = [{k: int(v) for k, v in r.items()} for r in csv.DictReader(stream)
                   if r['result'] == '2']
    origin = samples[0]['entry_ns']
    bins = defaultdict(lambda: dict(samples=0, source_gaps=[], wall_gaps=[], publish=[]))
    for i, row in enumerate(samples):
        bucket = bins[int((row['entry_ns']-origin)/5e9)]
        bucket['samples'] += 1
        bucket['publish'].append((row['publish_end_ns']-row['publish_begin_ns'])/1e9)
        if i:
            bucket['source_gaps'].append((row['source_ns']-samples[i-1]['source_ns'])/1e9)
            bucket['wall_gaps'].append((row['entry_ns']-samples[i-1]['entry_ns'])/1e9)
    table = [dict(start_s=index*5, sample_count=r['samples'],
                  source_gap_s=distribution(r['source_gaps']),
                  production_wall_gap_s=distribution(r['wall_gaps']),
                  publish_duration_s=distribution(r['publish']))
             for index, r in sorted(bins.items())]
    events = []
    for path in (directory / 'ros').glob('*/launch.log'):
        for line in path.read_text().splitlines():
            match = re.match(r'([0-9.]+) \[INFO\] \[([^]]+)\]: process started with pid', line)
            if match:
                elapsed = (float(match[1])*1e9-offset-origin)/1e9
                events.append(dict(process=match[2], since_first_production_s=elapsed))
    guard = result['native'].get('guard_audit', {})
    fault = guard.get('first_failure')
    return dict(reference='first actual IMU production callback, CLOCK_MONOTONIC',
                realtime_minus_monotonic_ns=offset,
                measured_clock_offset_span_ns=max(offsets)-min(offsets),
                source_bins_5s=table, process_start_events=events,
                first_guard_fault_since_production_s=(
                    (fault['begin_ns']-origin)/1e9 if fault else None))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matrix', type=Path, required=True)
    parser.add_argument('--extra-case', type=Path, action='append', default=[])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    matrix = json.loads((args.matrix / 'matrix.json').read_text())
    if not matrix['cases']:
        parser.error('matrix has no completed cases')
    report = dict(scope=__doc__, matrix_error=matrix['error'], cases={},
                  default_launch_changed=False, navigation_enabled=False,
                  dynamic_pose_validated=False, controlled_packet_loss=False)
    directories = [args.matrix / item['name'] for item in matrix['cases']] + args.extra_case
    for directory in directories:
        if directory.name in report['cases']:
            parser.error('duplicate case name: ' + directory.name)
        result = analyze(directory)
        result['timeline'] = timeline(directory, result)
        load = {}
        for path in directory.glob('load-*.log'):
            values = json.loads(path.read_text())
            load[path.name] = dict(**values, cpu_per_wall_s=values['cpu_s']/values['wall_s'])
        result['load_workers'] = load
        result['evidence_sha256'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in [
            directory / 'analysis.json', directory / 'camera-analysis.json',
            directory / 'lifetime.json', directory / 'python/summary.json']}
        report['cases'][directory.name] = result
    report['complete_case_evidence'] = all(
        r['evidence_complete'] for r in report['cases'].values())
    report['healthy_input_cases'] = [name for name, r in report['cases'].items()
                                     if r['localized_input_healthy']]
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    for name, result in report['cases'].items():
        print(name, 'healthy=' + str(result['localized_input_healthy']),
              'states=' + str(result['states']), 'quality=' + str(result['source_quality']))
    return 0 if report['complete_case_evidence'] and not matrix['error'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

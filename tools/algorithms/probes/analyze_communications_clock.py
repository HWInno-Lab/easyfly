#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Audit communication subgroups and the guard's own Clock delivery, not wire loss."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

from analyze_bridge_split import freshness
from analyze_imu_delivery import distribution
from analyze_stack_layers import inspect


LAYERS = ('sitl', 'mavros', 'project_bridge', 'communications')


def clock_audit(directory):
    path, = directory.glob('guard-take.*.json')
    trace = json.loads(path.read_text())
    takes = [r for r in trace['records'] if r['kind'] == 'take' and r['topic'] == '/clock']
    callbacks = [r for r in trace['records'] if r['kind'] == 'clock_callback']
    pairs_match = len(takes) == len(callbacks) and all(
        a['stamp_ns'] == b['stamp_ns'] and a['end_ns'] <= b['begin_ns']
        for a, b in zip(takes, callbacks))
    applied = all(r['cached_clock_ns'] == r['stamp_ns'] and
                  r['associated_clock_ns'] and
                  all(v == r['stamp_ns'] for v in r['associated_clock_ns']) for r in callbacks)
    cached = 0
    gate_mismatches = []
    for row in trace['records']:
        if row['kind'] == 'clock_callback':
            cached = row['cached_clock_ns']
        elif row['kind'] in ('on_imu', 'on_camera', 'on_info', 'report') and \
                row['clock_ns'] != cached:
            gate_mismatches.append(row)
    clock = trace['clock_audit']
    publishers = [e for s in clock['endpoint_snapshots'] for e in s['publishers']]
    unique_qos = {json.dumps(e['qos'], sort_keys=True) for e in publishers}
    python_clocks = [json.loads(line) for line in (directory / 'python/samples.jsonl').open()
                     if '"stream": "clock"' in line]
    common = None
    if takes and python_clocks:
        lower = max(takes[0]['stamp_ns'], python_clocks[0]['stamp_ns'])
        upper = min(takes[-1]['stamp_ns'], python_clocks[-1]['stamp_ns'])
        own = {r['stamp_ns'] for r in takes if lower <= r['stamp_ns'] <= upper}
        independent = {r['stamp_ns'] for r in python_clocks if lower <= r['stamp_ns'] <= upper}
        common = dict(source_window_ns=[lower, upper], guard_unique=len(own),
                      independent_unique=len(independent), shared=len(own & independent),
                      independent_not_in_guard=len(independent-own),
                      guard_not_in_independent=len(own-independent),
                      interpretation='Different readers observed different samples; not wire loss')
    metadata_keys = sorted(set.intersection(*(set(r) for r in takes))) if takes else []
    worst = None
    if len(takes) >= 2:
        index = max(range(1, len(takes)), key=lambda i:
                    takes[i]['begin_ns']-takes[i-1]['begin_ns'])
        a, b = takes[index-1:index+1]
        worst = dict(before=a, after=b,
                     take_wall_gap_s=(b['begin_ns']-a['begin_ns'])/1e9,
                     payload_progress_s=(b['stamp_ns']-a['stamp_ns'])/1e9,
                     writer_metadata_gap_s=(b['source_timestamp']-a['source_timestamp'])/1e9,
                     reception_gap_s=(b['received_timestamp']-a['received_timestamp'])/1e9)
    return dict(
        enabled=clock is not None, take_count=len(takes), callback_count=len(callbacks),
        pairs_match=pairs_match, all_callbacks_applied=applied,
        gate_clock_mismatches=gate_mismatches, reader=clock['reader'],
        endpoint_snapshot_count=len(clock['endpoint_snapshots']),
        publisher_nodes=sorted({e['node_namespace']+'/'+e['node_name'] for e in publishers}),
        publisher_gids=[list(g) for g in sorted({tuple(e['gid']) for e in publishers})],
        publisher_qos=[json.loads(q) for q in sorted(unique_qos)],
        endpoint_snapshots=clock['endpoint_snapshots'],
        time_source_sha256=clock['time_source_sha256'], metadata_keys=metadata_keys,
        sequence_metadata_available=any('sequence' in k for k in metadata_keys),
        take_to_callback_s=distribution([(b['begin_ns']-a['end_ns'])/1e9
                                        for a, b in zip(takes, callbacks)]),
        source_to_reception_s=distribution([
            (r['received_timestamp']-r['source_timestamp'])/1e9 for r in takes]),
        reception_to_take_s=distribution([
            (r['real_ns']-r['received_timestamp'])/1e9 for r in takes]),
        worst_take_gap=worst, independent_common_window=common,
        discarded=trace['discarded'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matrix', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    matrix = json.loads((args.matrix/'matrix.json').read_text())
    expected = {layer+'-'+str(n) for layer in LAYERS for n in (1, 2)}
    assert len(matrix['cases']) == 8 and {c['name'] for c in matrix['cases']} == expected
    report = dict(cases={}, failed_attempts=matrix.get('failed_attempts', []),
                  matrix_error=matrix['error'], navigation_enabled=False,
                  dynamic_localization_validated=False, regular_profile_registered=False)
    controls = []
    counts = Counter()
    for item in matrix['cases']:
        directory = args.matrix/item.get('directory', item['name'])
        result = inspect(directory)
        f = freshness(directory)
        clock = clock_audit(directory)
        result.update(freshness_audit=f, clock_audit=clock)
        result['evidence_complete'] &= (
            item['returncode'] == 0 and f['enabled'] and f['status_count'] >= 100 and
            f['limits_consistent'] and not f['model_mismatches'] and
            f['all_status_receipt_deadlines'] == [.5] and
            f['all_status_source_age_limits'] == [.5] and
            clock['take_count'] >= 100 and clock['pairs_match'] and
            clock['all_callbacks_applied'] and not clock['gate_clock_mismatches'] and
            clock['endpoint_snapshot_count'] >= 3 and clock['publisher_qos'] and
            clock['discarded'] == 0)
        life = result['lifetime']
        counts[item['layer']] += 1
        controls.append(dict(
            sources=life['sensor_fixture']['source_sha256'], server=life['actual_gazebo']['sha256'],
            dds=hashlib.sha256((directory/'actual-fastdds.xml').read_bytes()).hexdigest(),
            native=life['native_executable_sha256'], middleware=life['middleware'],
            cpu_affinity=life['cpu_affinity'], limits=f['actual_limits'],
            gate=f['gate_source_sha256'], time_source=clock['time_source_sha256']))
        report['cases'][item['name']] = result
        print(item['name'], 'healthy', result['localized_input_healthy'],
              'evidence', bool(result['evidence_complete']), 'clock', clock['take_count'],
              'stale', f['stale_status_count'], flush=True)
    report['controlled_fingerprints_identical'] = all(c == controls[0] for c in controls)
    report['control_fingerprints'] = controls[0]
    report['two_runs_each'] = counts == Counter({layer: 2 for layer in LAYERS})
    report['repeat_agreement'] = {
        layer: report['cases'][layer+'-1']['localized_input_healthy'] ==
        report['cases'][layer+'-2']['localized_input_healthy'] for layer in LAYERS}
    report['complete_case_evidence'] = bool(
        not matrix['error'] and report['two_runs_each'] and
        report['controlled_fingerprints_identical'] and
        all(c['evidence_complete'] for c in report['cases'].values()))
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    return 0 if report['complete_case_evidence'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

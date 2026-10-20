#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Audit plugin/alignment controls and observed update/poll intervals, not CPU attribution."""

import argparse
import bisect
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import re

from analyze_bridge_split import freshness
from analyze_communications_clock import clock_audit
from analyze_imu_delivery import distribution
from analyze_stack_layers import inspect
from mavros_isolation_fixture import MODES, REFERENCE_PLUGINS


def table(path):
    with path.open() as stream:
        metadata = next(stream).strip()
        rows = [{k: int(v) for k, v in row.items()} for row in csv.DictReader(stream)]
    assert 'CLOCK_MONOTONIC' in metadata and 'discarded=0' in metadata
    return rows


def timing(directory, first_stale):
    steps = table(directory/'steps.csv')
    polls = table(directory/'poll.csv')
    trace, = directory.glob('guard-take.*.json')
    callbacks = [r for r in json.loads(trace.read_text())['records'] if r['kind'] == 'on_imu']
    lower, upper = callbacks[0]['begin_ns'], callbacks[-1]['end_ns']
    active = [r for r in steps if lower <= r['begin_ns'] and r['end_ns'] <= upper]
    starts = [r['begin_ns'] for r in steps]
    matched = Counter()
    outside = []
    for p in polls:
        i = bisect.bisect_right(starts, p['begin_ns'])-1
        if i >= 0 and p['end_ns'] <= steps[i]['end_ns']:
            matched[i] += p['end_ns']-p['begin_ns']
        else:
            outside.append(p)
    observed_polls = [p for p in polls if lower <= p['begin_ns'] and p['end_ns'] <= upper]
    durations = [r['end_ns']-r['begin_ns'] for r in active]
    gaps = [b['begin_ns']-a['end_ns'] for a, b in zip(active, active[1:])]
    worst = max(active, key=lambda r: r['end_ns']-r['begin_ns'])
    index = bisect.bisect_left(starts, worst['begin_ns'])
    related = [p for p in polls if worst['begin_ns'] <= p['begin_ns'] <= worst['end_ns']]
    long = [(i, r) for i, r in enumerate(steps) if lower <= r['begin_ns'] and
            r['end_ns'] <= upper and r['end_ns']-r['begin_ns'] >= 25000000]
    context = None
    if first_stale:
        at = first_stale['begin_ns']
        i = bisect.bisect_right(starts, at)-1
        if i >= 0:
            row = steps[i]
            context = dict(step=row, phase='inside_update' if at <= row['end_ns'] else
                           'after_update_end', status_minus_step_begin_s=(at-row['begin_ns'])/1e9,
                           poll_duration_s=matched[i]/1e9)
    return dict(
        scope='WorldUpdate event span includes callbacks and physics, not physics-only timing',
        steps=len(steps), polls=len(polls), active_window_ns=[lower, upper],
        active_steps=len(active), active_polls=len(observed_polls),
        active_step_span_s=distribution([v/1e9 for v in durations]),
        active_between_steps_s=distribution([v/1e9 for v in gaps]),
        active_poll_s=distribution([(r['end_ns']-r['begin_ns'])/1e9 for r in observed_polls]),
        active_poll_settings=dict(Counter(str((r['timeout_ms'], r['result']))
                                          for r in observed_polls)),
        poll_rows_inside_steps=len(polls)-len(outside), poll_rows_outside_steps=outside,
        negative_step_spans=sum(v < 0 for v in durations),
        negative_between_step_gaps=sum(v < 0 for v in gaps),
        worst_step=worst, worst_step_poll_duration_s=matched[index]/1e9,
        worst_step_polls=related, long_steps_over_25ms=len(long),
        long_step_duration_s=sum(r['end_ns']-r['begin_ns'] for _, r in long)/1e9,
        poll_duration_inside_long_steps_s=sum(matched[i] for i, _ in long)/1e9,
        first_stale_step=context)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matrix', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    matrix = json.loads((args.matrix/'matrix.json').read_text())
    assert len(matrix['cases']) == 8
    expected = {m+'-'+str(n) for m in MODES for n in (1, 2)}
    assert {c['name'] for c in matrix['cases']} == expected
    report = dict(cases={}, matrix_error=matrix['error'],
                  failed_attempts=matrix.get('failed_attempts', []),
                  retry_rule=matrix.get('retry_rule'), navigation_enabled=False,
                  dynamic_localization_validated=False, regular_profile_registered=False)
    controls = []
    plugin_sets = {}
    for item in matrix['cases']:
        directory = args.matrix/item.get('directory', item['name'])
        result = inspect(directory)
        f, clock = freshness(directory), clock_audit(directory)
        t = timing(directory, f['first_stale'])
        life = result['lifetime']
        fixture = life['sensor_fixture']
        mode = fixture['mavros_isolation']['mode']
        actual = sorted(re.findall(r'Plugin (\S+) initialized',
                                   (directory/'stack.log').read_text()))
        plugin_sets[item['name']] = actual
        plugins_match = actual == sorted(REFERENCE_PLUGINS) if mode.startswith('reference') else \
            len(actual) > 40 and len(actual) == len(set(actual))
        libraries = life['trace_libraries']
        library_names = [Path(p).name for p in libraries]
        libraries_match = all(any(prefix in p for p in library_names) for prefix in (
            'libgazebo_mavlink_interface', 'libeasyfly_poll_trace', 'libeasyfly_step_trace'))
        result.update(freshness_audit=f, clock_audit=clock, timing_audit=t,
                      initialized_plugins=actual)
        result['evidence_complete'] &= bool(
            item['returncode'] == 0 and f['enabled'] and f['status_count'] >= 100 and
            f['limits_consistent'] and not f['model_mismatches'] and
            clock['pairs_match'] and clock['all_callbacks_applied'] and
            not clock['gate_clock_mismatches'] and clock['discarded'] == 0 and
            plugins_match and libraries_match and not t['poll_rows_outside_steps'] and
            not t['negative_step_spans'] and not t['negative_between_step_gaps'] and
            t['active_steps'] > 100 and t['active_polls'] > 100)
        controls.append(dict(
            sources=fixture['source_sha256'], server=life['actual_gazebo']['sha256'],
            dds=hashlib.sha256((directory/'actual-fastdds.xml').read_bytes()).hexdigest(),
            cpu_affinity=life['cpu_affinity'], middleware=life['middleware'],
            limits=f['actual_limits'], gate=f['gate_source_sha256'], libraries=libraries))
        report['cases'][item['name']] = result
        print(item['name'], 'healthy', result['localized_input_healthy'],
              'evidence', result['evidence_complete'], 'plugins', len(actual),
              'step_max_ms', round(t['active_step_span_s']['maximum']*1000, 3),
              'poll_max_ms', round(t['active_poll_s']['maximum']*1000, 3), flush=True)
    report['all_plugin_sets_identical'] = all(
        plugin_sets[name] == plugin_sets['all_on-1']
        for name in plugin_sets if name.startswith('all'))
    report['controlled_fingerprints_identical'] = all(c == controls[0] for c in controls)
    report['control_fingerprints'] = controls[0]
    report['repeat_agreement'] = {
        m: report['cases'][m+'-1']['localized_input_healthy'] ==
        report['cases'][m+'-2']['localized_input_healthy'] for m in MODES}
    report['complete_case_evidence'] = bool(
        not matrix['error'] and report['controlled_fingerprints_identical'] and
        report['all_plugin_sets_identical'] and
        all(c['evidence_complete'] for c in report['cases'].values()))
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    return 0 if report['complete_case_evidence'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

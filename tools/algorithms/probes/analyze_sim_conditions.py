#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Locate measured long update spans in the pinned MAVLink IMU condition wait."""

import argparse
import bisect
from collections import Counter
import hashlib
import json
from pathlib import Path

from analyze_bridge_split import freshness
from analyze_communications_clock import clock_audit
from analyze_imu_delivery import distribution
from analyze_mavros_isolation import table, timing
from analyze_stack_layers import inspect


BINARY_SHA = '4a18f6e953f2632b95570b0fb2eabb3f1df5f134a7784125181167ff456c4adc'
# Reviewed objdump: call at 0x5b028 in GazeboMavlinkInterface::OnUpdate.
CALLER_OFFSET = 0x5b02d


def conditions(directory, first_stale):
    steps = table(directory/'steps.csv')
    rows = table(directory/'poll.csv.conditions.csv')
    starts = [r['begin_ns'] for r in steps]
    trace, = directory.glob('guard-take.*.json')
    callbacks = [r for r in json.loads(trace.read_text())['records'] if r['kind'] == 'on_imu']
    lower, upper = callbacks[0]['begin_ns'], callbacks[-1]['end_ns']
    active = [r for r in rows if lower <= r['begin_ns'] and r['end_ns'] <= upper]
    indices = Counter()
    outside = []
    for r in rows:
        index = bisect.bisect_right(starts, r['begin_ns'])-1
        if index >= 0 and r['end_ns'] <= steps[index]['end_ns']:
            indices[index] += r['end_ns']-r['begin_ns']
        else:
            outside.append(r)
    long = [(i, s) for i, s in enumerate(steps) if lower <= s['begin_ns'] and
            s['end_ns'] <= upper and s['end_ns']-s['begin_ns'] >= 25000000]
    step_ns = sum(s['end_ns']-s['begin_ns'] for _, s in long)
    condition_ns = sum(indices[i] for i, _ in long)
    worst = max((s for s in steps if lower <= s['begin_ns'] and s['end_ns'] <= upper),
                key=lambda s: s['end_ns']-s['begin_ns'])
    worst_index = bisect.bisect_left(starts, worst['begin_ns'])
    context = None
    if first_stale:
        i = bisect.bisect_right(starts, first_stale['begin_ns'])-1
        context = dict(step=steps[i], condition_duration_s=indices[i]/1e9,
                       status_inside_step=first_stale['begin_ns'] <= steps[i]['end_ns'])
    return dict(
        count=len(rows), active_count=len(active),
        offsets=sorted({r['caller_offset'] for r in rows}),
        clock_ids=sorted({r['clock_id'] for r in rows}),
        active_duration_s=distribution([(r['end_ns']-r['begin_ns'])/1e9 for r in active]),
        active_deadline_minus_begin_s=distribution([
            (r['deadline_ns']-r['begin_ns'])/1e9 for r in active]),
        active_return_codes=dict(Counter(str(r['result']) for r in active)),
        timed_out_return_minus_deadline_s=distribution([
            (r['end_ns']-r['deadline_ns'])/1e9 for r in active if r['result'] == 110]),
        long_step_count=len(long), long_step_span_s=step_ns/1e9,
        condition_duration_inside_long_steps_s=condition_ns/1e9,
        long_step_condition_fraction=condition_ns/step_ns if step_ns else None,
        worst_step=worst, worst_step_condition_duration_s=indices[worst_index]/1e9,
        outside_step_rows=outside, first_stale_step=context,
        interpretation='clockwait spans include sleep, wake scheduling and mutex reacquisition')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matrix', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    matrix = json.loads((args.matrix/'conditions.json').read_text())
    assert len(matrix['cases']) == 2
    report = dict(cases={}, error=matrix['error'], scope=matrix['scope'])
    controls = []
    for item in matrix['cases']:
        directory = args.matrix/item['name']
        r = inspect(directory)
        f, clock = freshness(directory), clock_audit(directory)
        t, c = timing(directory, f['first_stale']), conditions(directory, f['first_stale'])
        libraries = r['lifetime']['trace_libraries']
        binary, = [digest for path, digest in libraries.items()
                   if Path(path).name == 'libgazebo_mavlink_interface.so']
        r.update(freshness_audit=f, clock_audit=clock, timing_audit=t, condition_audit=c)
        r['evidence_complete'] &= bool(
            item['returncode'] == 0 and binary == BINARY_SHA and
            c['offsets'] == [CALLER_OFFSET] and c['clock_ids'] == [1] and
            not c['outside_step_rows'] and c['active_count'] > 100 and
            clock['pairs_match'] and clock['all_callbacks_applied'] and
            not clock['gate_clock_mismatches'] and not f['model_mismatches'])
        controls.append(dict(sources=r['lifetime']['sensor_fixture']['source_sha256'],
                             limits=f['actual_limits'], libraries=libraries,
                             dds=hashlib.sha256(
                                 (directory/'actual-fastdds.xml').read_bytes()).hexdigest()))
        report['cases'][item['name']] = r
        print(item['name'], 'evidence', r['evidence_complete'],
              'healthy', r['localized_input_healthy'],
              'condition share of long steps', c['long_step_condition_fraction'], flush=True)
    report['controls_identical'] = controls[0] == controls[1]
    report['complete_case_evidence'] = not matrix['error'] and report['controls_identical'] and \
        all(r['evidence_complete'] for r in report['cases'].values())
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    return 0 if report['complete_case_evidence'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

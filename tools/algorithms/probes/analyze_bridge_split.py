#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Audit bridge factorial controls and the real guard's first stale status."""

import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path

from analyze_stack_layers import inspect


def freshness(directory):
    path, = directory.glob('guard-take.*.json')
    trace = json.loads(path.read_text())
    statuses = [r for r in trace['records'] if r['kind'] == 'freshness_status']
    assert statuses, 'freshness audit is missing'
    errors = []
    for index, row in enumerate(statuses):
        f = row['freshness']
        limits = f['limits']
        checks = dict(
            camera_stamp_positive=f['camera_ns'] > 0, imu_stamp_positive=f['imu_ns'] > 0,
            camera_receipt_deadline=f['camera_receipt_age_s'] is not None and
            f['camera_receipt_age_s'] <= limits['receipt_timeout'],
            imu_receipt_deadline=f['imu_receipt_age_s'] is not None and
            f['imu_receipt_age_s'] <= limits['receipt_timeout'],
            camera_source_age=f['clock_ns']-f['camera_ns'] <= limits['max_source_age']*1e9,
            imu_source_age=f['clock_ns']-f['imu_ns'] <= limits['max_source_age']*1e9)
        failed = [k for k, value in checks.items() if not value]
        if checks != f['checks'] or failed != f['failed_checks']:
            errors.append(dict(index=index, kind='snapshot_condition_mismatch'))
        if row['state'] in (1, 2) and failed:
            errors.append(dict(index=index, kind='healthy_state_with_failed_condition'))
        if row['state'] == 3 and row['reason'] == 'input_stale' and not failed:
            errors.append(dict(index=index, kind='stale_without_failed_condition'))
    stale = [r for r in statuses if r['state'] == 3 and r['reason'] == 'input_stale']
    first_failure = next((r for r in trace['records'] if r.get('failure')), None)
    failure_context = {}
    if first_failure:
        fault_ns = first_failure['begin_ns']
        clocks = [json.loads(line) for line in (directory / 'python/samples.jsonl').open()
                  if '"stream": "clock"' in line]
        clocks = [r for r in clocks if r['receipt']*1e9 <= fault_ns]
        if clocks:
            latest = clocks[-1]
            failure_context['independent_clock'] = dict(
                stamp_ns=latest['stamp_ns'], callback_age_s=fault_ns/1e9-latest['receipt'])
        takes = [r for r in trace['records'] if r['kind'] == 'take' and
                 r['topic'] == '/clock' and r['begin_ns'] <= fault_ns]
        if takes:
            take = takes[-1]
            failure_context['guard_clock_take'] = dict(
                age_s=(fault_ns-take['begin_ns'])/1e9,
                source_to_rmw_reception_s=(
                    take['received_timestamp']-take['source_timestamp'])/1e9,
                rmw_reception_to_take_s=(take['real_ns']-take['received_timestamp'])/1e9,
                payload_clock_stamp_recorded=bool(trace.get('clock_audit')))
            if trace.get('clock_audit'):
                failure_context['guard_clock_take']['payload_stamp_ns'] = take['stamp_ns']
        all_clock_takes = [r for r in trace['records'] if r['kind'] == 'take' and
                           r['topic'] == '/clock']
        nearby = takes[-2:] + [r for r in all_clock_takes if r['begin_ns'] > fault_ns][:2]
        failure_context['clock_takes_near_failure'] = [dict(
            take_relative_fault_s=(r['begin_ns']-fault_ns)/1e9,
            source_to_rmw_reception_s=(r['received_timestamp']-r['source_timestamp'])/1e9,
            rmw_reception_to_take_s=(r['real_ns']-r['received_timestamp'])/1e9)
            for r in nearby]
        if trace.get('clock_audit'):
            for entry, take in zip(failure_context['clock_takes_near_failure'], nearby):
                entry['payload_stamp_ns'] = take['stamp_ns']
        if first_failure['reason'] == 'invalid_source_time':
            failure_context['incoming_minus_guard_clock_s'] = (
                first_failure['stamp_ns']-first_failure['clock_ns'])/1e9
            limits = statuses[0]['freshness']['limits']
            checks = dict(
                positive_stamp=first_failure['stamp_ns'] > 0,
                future_tolerance=first_failure['stamp_ns'] <=
                first_failure['clock_ns']+limits['future_tolerance']*1e9,
                source_age=first_failure['stamp_ns'] >=
                first_failure['clock_ns']-limits['max_source_age']*1e9)
            failure_context['time_valid_failed_checks'] = [
                k for k, value in checks.items() if not value]
    transitions = [row for i, row in enumerate(statuses)
                   if i == 0 or (row['state'], row['reason']) !=
                   (statuses[i-1]['state'], statuses[i-1]['reason'])]
    context = {}
    if stale:
        first = stale[0]
        f = first['freshness']
        requests = dict(camera=('/camera/color/image_raw', f['camera_ns']),
                        imu=('/livox/imu', f['imu_ns']),
                        info=('/camera/color/camera_info', f['info_ns']))
        for name, (topic, stamp) in requests.items():
            takes = [r for r in trace['records'] if r['kind'] == 'take' and
                     r['topic'] == topic and r['stamp_ns'] == stamp and
                     r['begin_ns'] <= first['begin_ns']]
            if takes:
                take = takes[-1]
                context[name] = dict(
                    stamp_ns=stamp,
                    source_to_rmw_reception_s=(
                        take['received_timestamp']-take['source_timestamp'])/1e9,
                    rmw_reception_to_take_s=(take['real_ns']-take['received_timestamp'])/1e9,
                    take_to_status_s=f['now_s']-take['begin_ns']/1e9)
        with (directory / 'gazebo-camera.csv').open() as stream:
            next(stream)
            native = [r for r in csv.DictReader(stream)
                      if int(r['mono_ns']) <= first['begin_ns']]
        if native:
            latest = max(native, key=lambda r: int(r['mono_ns']))
            context['latest_native_camera_receipt'] = dict(
                stamp_ns=int(latest['stamp_ns']),
                age_s=f['now_s']-int(latest['mono_ns'])/1e9,
                interpretation='Independent native receipt, not sensor exposure time')
    return dict(enabled=trace.get('freshness_enabled', False),
                gate_source_sha256=trace['gate_source_sha256'],
                actual_limits=statuses[0]['freshness']['limits'],
                limits_consistent=all(r['freshness']['limits'] ==
                                      statuses[0]['freshness']['limits'] for r in statuses),
                status_count=len(statuses), model_mismatches=errors,
                stale_status_count=len(stale), first_stale=stale[0] if stale else None,
                first_failure=first_failure, first_failure_context=failure_context,
                first_stale_take_context=context,
                stale_failed_check_counts=dict(Counter(
                    check for row in stale for check in row['freshness']['failed_checks'])),
                status_transitions=transitions,
                all_status_receipt_deadlines=sorted({r['freshness']['limits']['receipt_timeout']
                                                    for r in statuses}),
                all_status_source_age_limits=sorted({r['freshness']['limits']['max_source_age']
                                                     for r in statuses}),
                trace_sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matrix', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    matrix = json.loads((args.matrix / 'matrix.json').read_text())
    expected_names = {layer+'-'+str(repeat)
                      for layer in ('sitl', 'communications', 'calibration', 'bridge')
                      for repeat in (1, 2)}
    if len(matrix['cases']) != 8 or {item['name'] for item in matrix['cases']} != expected_names:
        parser.error('expected two cases each for sitl, communications, calibration and bridge')
    report = dict(scope=__doc__, matrix_error=matrix['error'], cases={},
                  navigation_enabled=False, dynamic_localization_validated=False,
                  regular_profile_registered=False)
    controls = []
    counts = Counter()
    for item in matrix['cases']:
        directory = args.matrix / item.get('directory', item['name'])
        result = inspect(directory)
        audit = freshness(directory)
        result['freshness_audit'] = audit
        result['evidence_complete'] &= (
            item['returncode'] == 0 and audit['enabled'] and audit['status_count'] >= 100 and
            audit['limits_consistent'] and
            not audit['model_mismatches'] and audit['all_status_receipt_deadlines'] == [.5] and
            audit['all_status_source_age_limits'] == [.5])
        life = result['lifetime']
        fixture = life['sensor_fixture']
        layer = fixture['stack_layer']['name']
        counts[layer] += 1
        controls.append(dict(
            original_sources=fixture['source_sha256'], server=life['actual_gazebo']['sha256'],
            dds=hashlib.sha256((directory / 'actual-fastdds.xml').read_bytes()).hexdigest(),
            native=life['native_executable_sha256'], middleware=life['middleware'],
            cpu_affinity=life['cpu_affinity'], limits=result['limits_unchanged'],
            gate_source=audit['gate_source_sha256'], actual_limits=audit['actual_limits']))
        report['cases'][item['name']] = result
        first = audit['first_stale']
        print(item['name'], 'healthy', result['localized_input_healthy'],
              'evidence', result['evidence_complete'],
              'first_stale_failed_checks', first['freshness']['failed_checks'] if first else [])
    report['controlled_fingerprints_identical'] = all(c == controls[0] for c in controls)
    report['failed_attempts'] = matrix.get('failed_attempts', [])
    report['control_fingerprints'] = controls[0]
    report['two_runs_each'] = counts == Counter(
        {'sitl': 2, 'communications': 2, 'calibration': 2, 'bridge': 2})
    report['repeat_agreement'] = {
        layer: report['cases'][layer+'-1']['localized_input_healthy'] ==
        report['cases'][layer+'-2']['localized_input_healthy']
        for layer in counts}
    report['complete_case_evidence'] = (
        not matrix['error'] and report['two_runs_each'] and
        report['controlled_fingerprints_identical'] and
        all(c['evidence_complete'] for c in report['cases'].values()))
    report['healthy_input_cases'] = [name for name, c in report['cases'].items()
                                     if c['localized_input_healthy']]
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    return 0 if report['complete_case_evidence'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

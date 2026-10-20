#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Aggregate static Gazebo recovery experiments, preserving failures and scope."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess


HERE = Path(__file__).resolve().parent


def case(directory):
    lifetime = json.loads((directory / 'lifetime.json').read_text())
    guard, = directory.glob('guard-take.*.json')
    producer, = directory.glob('producer.*.csv')
    for tool, output, arguments in [
            ('analyze_native_imu.py', 'analysis.json',
             ['--native', str(directory / 'native.csv'), '--python',
              str(directory / 'python/samples.jsonl'), '--producer', str(producer),
              '--guard', str(guard)]),
            ('analyze_gazebo_camera.py', 'camera-analysis.json',
             ['--native', str(directory / 'gazebo-camera.csv'), '--python',
              str(directory / 'python/samples.jsonl')])]:
        path = directory / output
        if not path.exists():
            with (directory / (output + '.log')).open('w') as log:
                subprocess.run(['python3', str(HERE / tool), *arguments, '--output', str(path)],
                               stdout=log, stderr=subprocess.STDOUT, check=True)
    native = json.loads((directory / 'analysis.json').read_text())
    camera = json.loads((directory / 'camera-analysis.json').read_text())
    summary_path = directory / 'python/summary.json'
    reconstructed = not summary_path.exists()
    if reconstructed:
        summary_path = directory / 'python/summary.recovered.json'
    python = json.loads(summary_path.read_text())
    launch_log = directory / 'stack.log'
    nested_errors = [line for line in launch_log.read_text().splitlines()
                     if 'process has died' in line] if launch_log.exists() else []
    states = [json.loads(line) for line in (directory / 'python/samples.jsonl').open()
              if '"stream": "status"' in line]
    faults = [r for r in states if r['state'] == 4 or r['state'] == 5]
    first = faults[0] if faults else None
    stale = [r for r in states if r['state'] == 3]
    callbacks = json.loads(guard.read_text())['records']
    receipt = {}
    for row in callbacks:
        if row['kind'] in ('on_camera', 'on_imu'):
            receipt.setdefault((row['kind'], row['stamp_ns']), row['begin_ns']/1e9)
    stale_examples = []
    for row in stale[:5]:
        example = dict(observer_receipt=row['receipt'], reason=row['reason'])
        for kind, key in [('on_camera', 'camera_stamp_ns'), ('on_imu', 'imu_stamp_ns')]:
            time = receipt.get((kind, row[key]))
            example[kind + '_original_callback_age_s'] = (
                row['receipt']-time if time is not None else None)
        stale_examples.append(example)
    result = dict(directory=str(directory), lifetime=lifetime,
                  nested_process_errors=nested_errors,
                  all_layers_clean=lifetime['clean_execution'] and not nested_errors,
                  python_summary_reconstructed=reconstructed,
                  states=python['states'],
                  forwarded_imu=python['streams']['forwarded_imu']['count'],
                  forwarded_images=python['streams']['forwarded_image']['count'],
                  comparisons=python['comparison'], first_latched_fault=first,
                  observed_stale_status_count=len(stale), stale_examples=stale_examples,
                  healthy_admission=(not any(r['state'] >= 3 for r in states) and
                                     python['streams']['forwarded_imu']['count'] > 100),
                  native=native['streams'], guard=native['guard_audit'], camera=camera,
                  post_latch_forwarded_imu_increase=(
                      max(r['imu_forwarded'] for r in states[states.index(first):]) -
                      first['imu_forwarded']
                      if faults else 0))
    pause = lifetime.get('guard_pause')
    if pause and 'end_mono_ns' in pause:
        result['actual_guard_pause_ms'] = (pause['end_mono_ns']-pause['start_mono_ns'])/1e6
        result['first_fault_after_pause_resume_s'] = (
            first['receipt']-pause['end_mono_ns']/1e9 if first else None)
        after = [r for r in states if r['receipt'] >= pause['end_mono_ns']/1e9]
        result['admission_resumed_after_pause'] = (
            any(r['state'] in (1, 2) for r in after) and
            any(r['imu_forwarded'] > pause['before']['imu_forwarded'] for r in after))
    result['evidence_complete'] = (
        lifetime['clean_execution'] and not nested_errors and not python.get('error') and
        not python['sample_limit_reached'] and int(native['metadata']['discarded']) == 0 and
        int(native['event_metadata']['discarded']) == 0 and
        native['guard_audit']['discarded'] == 0 and
        'discarded=0' in camera['metadata'] and camera['native']['count'] > 100)
    result['source_sha256'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in [
        directory / 'lifetime.json', directory / 'analysis.json',
        directory / 'camera-analysis.json', summary_path]}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matrix', type=Path, required=True)
    parser.add_argument('--long-pause', type=Path, required=True)
    parser.add_argument('--short-pause-retry', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    matrix = json.loads((args.matrix / 'matrix.json').read_text())
    assert len(matrix['cases']) == 7, 'incomplete matrix'
    report = dict(scope=__doc__, default_launch_changed=False, navigation_enabled=False,
                  controlled_packet_loss=False, matrix_execution_error=matrix['error'], cases={})
    for item in matrix['cases']:
        report['cases'][item['name']] = case(args.matrix / item['name'])
    report['cases']['pause-candidate-800ms'] = case(args.long_pause)
    if args.short_pause_retry:
        report['cases']['pause-candidate-200ms-retry'] = case(args.short_pause_retry)
    report['all_evidence_complete'] = all(r['evidence_complete'] for r in report['cases'].values())
    report['unsuccessful_executions'] = [
        n for n, r in report['cases'].items() if not r['all_layers_clean']]
    report['completed_execution_evidence_complete'] = all(
        r['evidence_complete'] for r in report['cases'].values()
        if r['all_layers_clean'])
    report['candidate_healthy_cases'] = [n for n, r in report['cases'].items()
                                         if 'candidate' in n and r['healthy_admission']]
    report['candidate_faulted_cases'] = [n for n, r in report['cases'].items()
                                         if 'candidate' in n and not r['healthy_admission']]
    report['candidate_uninjected_unhealthy_cases'] = [
        n for n in report['candidate_faulted_cases']
        if not (report['cases'][n]['lifetime'].get('guard_pause') or {}).get('start_mono_ns')]
    report['limits'] = [
        'Finite static simulated runs; no new dynamic alignment, map or flight acceptance.',
        'CPU affinity and worker load affect only this test and its child processes.',
        'SIGSTOP/SIGCONT pauses the complete guard process, including middleware threads; '
        'this is not network packet-loss injection.',
        'A clean candidate case does not prove the intermittent delivery issue is fixed.']
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    for name, result in report['cases'].items():
        print(name, 'healthy=' + str(result['healthy_admission']),
              'imu=' + str(result['forwarded_imu']), 'states=' + str(result['states']))
    return 0 if report['completed_execution_evidence_complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

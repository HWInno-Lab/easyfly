#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Compare stationary four-CPU sensor ablations, preserving failures as evidence.

Sensor removal measures the total stack effect, including downstream consumers;
it does not measure a specific function's CPU time or physics-lock wait.
"""

import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from analyze_imu_delivery import distribution
from analyze_managed_sync import timeline
from analyze_vio_source import analyze


def structure(element):
    return [element.tag, dict(element.attrib), (element.text or '').strip(),
            [structure(child) for child in element]]


def inspect_case(directory):
    result = analyze(directory)
    result['timeline'] = timeline(directory, result)
    lifetime = result['lifetime']
    fixture = lifetime['sensor_fixture']
    python = json.loads((directory / 'python/summary.json').read_text())
    snapshots = python['sensor_topology']
    topology_matches = len(snapshots) >= 3 and all(
        {k: len(v) for k, v in snapshot['publishers'].items()} == fixture['expected_publishers']
        for snapshot in snapshots[-3:])
    sources_unchanged = all(hashlib.sha256(Path(p).read_bytes()).hexdigest() == digest
                            for p, digest in fixture['source_sha256'].items())
    staged_unchanged = all(hashlib.sha256(Path(p).read_bytes()).hexdigest() == digest
                           for p, digest in fixture['staged_sha256'].items())
    # Recheck the actual staged XML rather than trusting the staging boolean.
    exact_removal = True
    for original in fixture['source_sha256']:
        path = Path(original)
        staged, = [Path(p) for p in fixture['staged_sha256'] if Path(p).name == path.name]
        expected = ET.parse(path)
        for parent in expected.iter():
            for sensor in parent.findall('sensor'):
                if sensor.get('name') in fixture['disabled_sensors']:
                    parent.remove(sensor)
        exact_removal &= structure(expected.getroot()) == structure(ET.parse(staged).getroot())
    producer, = directory.glob('producer.*.csv')
    with producer.open() as stream:
        next(stream)
        rows = [{k: int(v) for k, v in r.items()} for r in csv.DictReader(stream)
                if r['result'] == '2']
    gaps = [(b['entry_ns']-a['entry_ns'])/1e9 for a, b in zip(rows, rows[1:])]
    source_gaps = [(b['source_ns']-a['source_ns'])/1e9 for a, b in zip(rows, rows[1:])]
    wall_span = (rows[-1]['entry_ns']-rows[0]['entry_ns'])/1e9
    sim_span = (rows[-1]['source_ns']-rows[0]['source_ns'])/1e9
    callback = [(r['publish_end_ns']-r['entry_ns'])/1e9 for r in rows]
    result['sensor_load'] = dict(
        variant=fixture['variant'], graph_snapshots=snapshots,
        last_three_graph_snapshots_match=topology_matches,
        original_model_files_unchanged=sources_unchanged,
        staged_files_unchanged=staged_unchanged, exact_requested_sensor_removal=exact_removal,
        production_samples=len(rows), simulated_span_s=sim_span, production_wall_span_s=wall_span,
        simulated_per_wall_s=sim_span/wall_span, production_wall_gap_s=distribution(gaps),
        producer_source_gap_s=distribution(source_gaps),
        callback_body_s=distribution(callback),
        snapshot_duration_s=distribution([
            (r['snapshot_end_ns']-r['entry_ns'])/1e9 for r in rows]),
        publish_duration_s=distribution([
            (r['publish_end_ns']-r['publish_begin_ns'])/1e9 for r in rows]),
        wall_gaps_over_25ms=sum(g > .025 for g in gaps),
        wall_gaps_over_100ms=sum(g > .1 for g in gaps),
        wall_gaps_over_500ms=sum(g > .5 for g in gaps))
    result['evidence_complete'] &= (
        topology_matches and sources_unchanged and staged_unchanged and exact_removal and
        lifetime['gazebo_lockstep'] and lifetime['actual_gazebo']['lockstep'] and
        len(lifetime['cpu_affinity']) == 4 and lifetime['cpu_workers'] == 0)
    pids = {child['pid'] for child in lifetime['children']}
    for log in (directory / 'ros').glob('*/launch.log'):
        started = re.findall(r'process started with pid \[(\d+)\]', log.read_text())
        pids.update(int(pid) for pid in started)
    result['remaining_owned_pids'] = [pid for pid in sorted(pids) if Path(f'/proc/{pid}').exists()]
    result['owned_pids_checked'] = len(pids)
    result['evidence_complete'] &= not result['remaining_owned_pids']
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matrix', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    matrix = json.loads((args.matrix / 'matrix.json').read_text())
    if not matrix['cases']:
        parser.error('matrix is empty')
    report = dict(scope=__doc__, matrix_error=matrix['error'], cases={},
                  navigation_enabled=False, dynamic_localization_validated=False,
                  regular_profile_registered=False)
    fingerprints = []
    variants = Counter()
    for item in matrix['cases']:
        name = item['name']
        if name in report['cases']:
            parser.error('duplicate case name: ' + name)
        directory = args.matrix / name
        result = inspect_case(directory)
        result['evidence_complete'] &= item['returncode'] == 0
        lifetime = result['lifetime']
        fixture = lifetime['sensor_fixture']
        variants[fixture['variant']] += 1
        arguments = {k: v for k, v in fixture['launch_arguments'].items()
                     if k not in ('model_sdf', 'debug_model_path')}
        fingerprints.append(dict(
            actual_server=lifetime['actual_gazebo']['sha256'],
            actual_dds_xml=hashlib.sha256(
                (directory / 'actual-fastdds.xml').read_bytes()).hexdigest(),
            native_observer=lifetime['native_executable_sha256'],
            middleware=lifetime['middleware'], cpu_affinity=lifetime['cpu_affinity'],
            original_scene=fixture['original_scene'], sources=fixture['source_sha256'],
            arguments=arguments, limits=result['limits_unchanged']))
        report['cases'][name] = result
        load = result['sensor_load']
        print(name, 'healthy=' + str(result['localized_input_healthy']),
              'max_wall_ms=' + str(round(load['production_wall_gap_s']['maximum']*1000, 3)),
              'sim/wall=' + str(round(load['simulated_per_wall_s'], 4)),
              'states=' + str(result['states']))
    report['controlled_inputs_identical'] = all(f == fingerprints[0] for f in fingerprints)
    report['control_fingerprints'] = fingerprints[0]
    report['variant_counts'] = dict(variants)
    report['two_runs_per_variant'] = variants == Counter(
        {'all': 2, 'no-lidar': 2, 'no-depth': 2, 'rgb-imu': 2})
    report['complete_case_evidence'] = (
        not report['matrix_error'] and report['controlled_inputs_identical'] and
        report['two_runs_per_variant'] and
        all(r['evidence_complete'] for r in report['cases'].values()))
    report['healthy_input_cases'] = [name for name, r in report['cases'].items()
                                     if r['localized_input_healthy']]
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    return 0 if report['complete_case_evidence'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Audit real stack-layer runs; CPU counters do not identify function/lock waits."""

import argparse
import ast
from collections import Counter, defaultdict
import csv
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from analyze_managed_sync import timeline
from analyze_sensor_load import structure
from analyze_vio_source import analyze


def cpu_budget(directory, labels):
    budget = json.loads((directory / 'process-budget.json').read_text())
    samples = budget['samples']
    previous = {}
    groups = defaultdict(lambda: dict(cpu_s=0., observed_wall_s=0., max_threads=0,
                                      max_rss_bytes=0, intervals=0))
    for sample in samples:
        for row in sample['processes']:
            key = (row['pid'], row['start_ticks'])
            group = groups[labels.get(row['pid'], row['name'])]
            group['max_threads'] = max(group['max_threads'], row['threads'])
            group['max_rss_bytes'] = max(group['max_rss_bytes'],
                                         row['rss_pages']*budget['page_size'])
            ticks = row['user_ticks']+row['system_ticks']
            if key in previous:
                before, before_ticks = previous[key]
                assert ticks >= before_ticks
                group['cpu_s'] += (ticks-before_ticks)/budget['clock_ticks_per_second']
                group['observed_wall_s'] += (sample['begin_ns']-before)/1e9
                group['intervals'] += 1
            previous[key] = sample['begin_ns'], ticks
    for group in groups.values():
        group['cpu_per_observed_wall_s'] = (
            group['cpu_s']/group['observed_wall_s'] if group['observed_wall_s'] else None)
    span = (samples[-1]['begin_ns']-samples[0]['begin_ns'])/1e9
    for group in groups.values():
        group['cpu_per_sampled_wall_s'] = group['cpu_s']/span
    return dict(scope=budget['scope'], sample_count=len(samples), sampled_span_s=span,
                kernel_schedstats_enabled=budget['kernel_schedstats_enabled'],
                max_sampling_duration_s=max(s['end_ns']-s['begin_ns'] for s in samples)/1e9,
                process_cpu_s=sum(g['cpu_s'] for g in groups.values()),
                stack_cpu_per_sampled_wall_s=sum(g['cpu_s'] for g in groups.values())/span,
                max_stack_threads=max(sum(p['threads'] for p in s['processes']) for s in samples),
                max_stack_rss_bytes=max(sum(p['rss_pages']*budget['page_size']
                                            for p in s['processes']) for s in samples),
                groups=dict(groups),
                observed_process_masks=sorted({tuple(p['allowed_cpus']) for s in samples
                                               for p in s['processes']}))


def inspect(directory, expected_cpu_workers=0):
    result = analyze(directory)
    result['timeline'] = timeline(directory, result)
    lifetime = result['lifetime']
    fixture = lifetime['sensor_fixture']
    layer = fixture['stack_layer']['name']
    labels = {}
    for log in (directory / 'ros').glob('*/launch.log'):
        starts = re.findall(r'\[([^]]+)\]: process started with pid \[(\d+)\]', log.read_text())
        for name, pid in starts:
            labels[int(pid)] = name.rsplit('-', 1)[0]
    processes = Counter(labels.values())
    core = {'gzserver', 'spawn_model', 'openvins_guard',
            'run_subscribe_msckf', 'openvins_alignment'}
    expected = Counter(core)
    if fixture.get('mavros_isolation', {}).get('alignment') is False:
        del expected['openvins_alignment']
    if layer not in ('tiny', 'world'):
        expected.update(['px4_sitl'])
    if layer in ('communications', 'bridge', 'perception', 'full'):
        expected.update(['mavros_node', 'mavlink_bridge_node', 'rc_receiver_node'])
    if layer == 'mavros':
        expected.update(['mavros_node'])
    if layer == 'project_bridge':
        expected.update(['mavlink_bridge_node', 'rc_receiver_node'])
    if layer in ('calibration', 'bridge', 'perception', 'full'):
        expected.update(['component_container', 'component_container'])
    if layer in ('perception', 'full'):
        expected.update(['component_container', 'component_container', 'component_container',
                         'pointcloud_filter_node', 'semantic_map_node'])
    if layer == 'full':
        expected.update(['flight_control_node', 'mission_fsm_node', 'search_area_node',
                         'navigation_coordinator_node',
                         'parameter_manager', 'perception_services', 'mission_services'])
    python = json.loads((directory / 'python/summary.json').read_text())
    snapshots = python['sensor_topology']
    graph_matches = len(snapshots) >= 3 and all(
        {k: len(v) for k, v in s['publishers'].items()} == fixture['expected_publishers']
        for s in snapshots[-3:])
    fingerprints_unchanged = all(
        hashlib.sha256(Path(p).read_bytes()).hexdigest() == digest
        for entries in ['source_sha256', 'staged_sha256']
        for p, digest in fixture[entries].items())
    original = ast.parse(Path(fixture['stack_layer']['source_launch']).read_text())
    staged = ast.parse(Path(fixture['stack_layer']['generated_launch']).read_text())
    isolation_matches = True
    if fixture.get('mavros_isolation'):
        from mavros_isolation_fixture import audit_trees
        isolation_matches = audit_trees(fixture, original, staged)
    sites = ('actions', 'p2_nodes', 'mavlink_bridge') if layer in (
        'mavros', 'project_bridge') else ('actions', 'p2_nodes')
    for tree in (original, staged):
        for node in ast.walk(tree):
            if isinstance(node, ast.Assign) and len(node.targets) == 1 and \
                    isinstance(node.targets[0], ast.Name) and \
                    node.targets[0].id in sites:
                node.value = ast.Constant(None)
    ast_matches = ast.dump(original, include_attributes=False) == \
        ast.dump(staged, include_attributes=False)
    communication_matches = True
    if layer in ('mavros', 'project_bridge'):
        meta = fixture['stack_layer']
        comm_original = ast.parse(Path(meta['communication_source']).read_text())
        comm_staged = ast.parse(Path(meta['communication_generated']).read_text())
        removed = {'bridge_node', 'rc_node'} if layer == 'mavros' else {'mavros_node'}
        returns = [n for n in ast.walk(comm_original) if isinstance(n, ast.Return) and
                   isinstance(n.value, ast.Call) and isinstance(n.value.func, ast.Name) and
                   n.value.func.id == 'LaunchDescription']
        returned, = returns
        returned.value.args[0].elts = [
            n for n in returned.value.args[0].elts
            if not isinstance(n, ast.Name) or n.id not in removed]
        communication_matches = ast.dump(comm_original, include_attributes=False) == \
            ast.dump(comm_staged, include_attributes=False)
        # Independently require that only the include source path changed at this site.
        orig = ast.parse(Path(meta['source_launch']).read_text())
        staged_full = ast.parse(Path(meta['generated_launch']).read_text())

        def assignment(tree):
            return next(n for n in ast.walk(tree) if isinstance(n, ast.Assign) and
                        isinstance(n.targets[0], ast.Name) and
                        n.targets[0].id == 'mavlink_bridge')
        before, after = assignment(orig), assignment(staged_full)
        communication_matches &= (
            after.value.args[0].args[0].value == meta['communication_generated'])
        after.value.args[0].args[0] = before.value.args[0].args[0]
        communication_matches &= ast.dump(before, include_attributes=False) == \
            ast.dump(after, include_attributes=False)
    ast_matches &= communication_matches and isolation_matches
    model_path = Path(fixture['launch_arguments']['model_sdf'])
    color, = ET.parse(model_path).findall(".//sensor[@name='color']")
    original_color, = ET.parse(fixture['original_scene']['model_sdf']).findall(
        ".//sensor[@name='color']")
    color_matches = structure(color) == structure(original_color)
    original_mid, = [Path(p) for p in fixture['source_sha256'] if Path(p).name == 'mid360.sdf']
    staged_mid, = [Path(p) for p in fixture['staged_sha256'] if Path(p).name == 'mid360.sdf']
    mid_expected = ET.parse(original_mid)
    for parent in mid_expected.iter():
        for sensor in parent.findall("sensor[@name='laser_livox']"):
            if fixture['variant'] in ('rgb-imu', 'no-lidar'):
                parent.remove(sensor)
    mid_matches = structure(mid_expected.getroot()) == structure(ET.parse(staged_mid).getroot())
    producer, = directory.glob('producer.*.csv')
    with producer.open() as stream:
        next(stream)
        rows = [{k: int(v) for k, v in r.items()} for r in csv.DictReader(stream)
                if r['result'] == '2']
    ratio = (rows[-1]['source_ns']-rows[0]['source_ns']) / \
        (rows[-1]['entry_ns']-rows[0]['entry_ns'])
    pids = set(labels) | {r['pid'] for r in lifetime['children']}
    remaining = [pid for pid in pids if Path(f'/proc/{pid}').exists()]
    budget = cpu_budget(directory, labels)
    with (directory / 'gazebo-camera.csv').open() as stream:
        next(stream)
        native_images = list(csv.DictReader(stream))
    raw_images = [json.loads(line) for line in (directory / 'python/samples.jsonl').open()
                  if '"stream": "image"' in line]
    lower = max(min(int(r['stamp_ns']) for r in native_images),
                min(r['stamp_ns'] for r in raw_images))
    upper = min(max(int(r['stamp_ns']) for r in native_images),
                max(r['stamp_ns'] for r in raw_images))
    native_identities = Counter(
        (int(r['stamp_ns']), r['sha256'], int(r['width']), int(r['height']), int(r['step']))
        for r in native_images if lower <= int(r['stamp_ns']) <= upper)
    raw_identities = Counter(
        (r['stamp_ns'], r['pixel_sha256'], r['width'], r['height'], r['step'])
        for r in raw_images if lower <= r['stamp_ns'] <= upper)
    identity_count = sum(raw_identities.values())
    unmatched_identities = sum((raw_identities-native_identities).values())
    identity_verified = identity_count >= 100 and unmatched_identities == 0
    result['layer_audit'] = dict(
        name=layer, actual_process_labels=dict(processes), expected_process_labels=dict(expected),
        process_set_matches=processes == expected, topology_matches=graph_matches,
        graph_snapshots=snapshots, fingerprints_unchanged=fingerprints_unchanged,
        launch_changes_limited_to_two_assignments=ast_matches if layer not in (
            'mavros', 'project_bridge') else False,
        reviewed_launch_changes_match=ast_matches,
        communication_split_matches=communication_matches,
        mavros_isolation_matches=isolation_matches,
        color_sensor_unchanged=color_matches,
        mid_only_ray_removed=mid_matches if fixture['variant'] in (
            'rgb-imu', 'no-lidar') else False,
        mid_changes_match_variant=mid_matches,
        source_time_per_production_wall_s=ratio, budget=budget,
        raw_image_native_identity_verified_in_common_window=identity_verified,
        identity_common_window_ns=[lower, upper], common_window_raw_images=identity_count,
        common_window_raw_without_native_identity=unmatched_identities,
        raw_images_outside_common_window=len(raw_images)-identity_count,
        owned_pids_checked=len(pids), remaining_owned_pids=remaining)
    result['evidence_complete'] &= (
        processes == expected and graph_matches and fingerprints_unchanged and ast_matches and
        color_matches and mid_matches and identity_verified and not remaining and
        budget['sample_count'] >= 5 and
        budget['observed_process_masks'] == [tuple(lifetime['cpu_affinity'])] and
        lifetime['actual_gazebo']['command'].count('--lockstep') == 1 and
        len(lifetime['cpu_affinity']) == 4 and lifetime['cpu_workers'] == expected_cpu_workers)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matrix', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    matrix = json.loads((args.matrix / 'matrix.json').read_text())
    report = dict(scope=__doc__, matrix_error=matrix['error'], cases={},
                  navigation_enabled=False, dynamic_localization_validated=False,
                  regular_profile_registered=False)
    controls = []
    for item in matrix['cases']:
        directory = args.matrix / item['name']
        result = inspect(directory)
        result['evidence_complete'] &= item['returncode'] == 0
        lifetime = result['lifetime']
        fixture = lifetime['sensor_fixture']
        controls.append(dict(
            sources=fixture['source_sha256'], server=lifetime['actual_gazebo']['sha256'],
            dds=hashlib.sha256((directory / 'actual-fastdds.xml').read_bytes()).hexdigest(),
            native=lifetime['native_executable_sha256'], middleware=lifetime['middleware'],
            cpu_affinity=lifetime['cpu_affinity'],
            limits=result['limits_unchanged']))
        report['cases'][item['name']] = result
        producer = result['native']['producer_comparison']['producer']
        print(item['name'], 'healthy', result['localized_input_healthy'],
              'evidence', result['evidence_complete'],
              'source_max_ms', round(producer['source_gap_s']['maximum']*1000, 3),
              'wall_max_ms', round(producer['entry_gap_s']['maximum']*1000, 3),
              'stack_cpu/wall', round(result['layer_audit']['budget'][
                  'stack_cpu_per_sampled_wall_s'], 3))
    report['controlled_fingerprints_identical'] = all(c == controls[0] for c in controls)
    report['control_fingerprints'] = controls[0]
    report['selected_repeat_pair'] = matrix['selected_repeat_pair']
    report['repeat_agreement'] = {
        layer: report['cases'][layer+'-1']['localized_input_healthy'] ==
        report['cases'][layer+'-2']['localized_input_healthy']
        for layer in matrix['selected_repeat_pair']}
    report['complete_case_evidence'] = (
        len(report['cases']) == 8 and not matrix['error'] and
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

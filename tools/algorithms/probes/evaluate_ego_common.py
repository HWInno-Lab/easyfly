#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Score actual EGO output against the frozen SUPER comparison constraints."""
import argparse
import csv
import gzip
import hashlib
import json
from pathlib import Path
import numpy as np
from bspline_metrics import polynomial_pieces, value, derivative_maximum, MapCertificate, jerk_energy


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line]


def audit(case):
    runtime = json.loads((case/'result.json').read_text())
    limits = runtime['acceptance_limits']
    splines = rows(case/'ego-bsplines.jsonl')
    commands = rows(case/'commands.jsonl')
    with gzip.open(case/'voxel-grid.jsonl.gz', 'rt') as stream:
        maps = [json.loads(line) for line in stream]
    checks = dict(actual_ego_runtime=runtime['passed'] and runtime['strategy'] == 'ego',
                  actual_common_ray_map=runtime.get('comparison_map', {}).get('same_source_ray_map') is True,
                  comparison_map_normal_exit=runtime.get('comparison_map_returncode') == 0,
                  actual_configured_acceleration=runtime.get('actual_ego_comparison_parameters') ==
                    {'manager/max_acc': 2., 'optimization/max_acc': 2., 'manager/max_jerk': 4.},
                  original_native_splines=bool(splines), actual_control_stream=bool(commands))
    lookup = {}
    certificates = []
    for spline in splines:
        if not spline['session']:
            continue
        pieces = polynomial_pieces(spline)
        key = (spline['session'], spline['trajectory_id'])
        lookup[key] = (spline, pieces)
        available = [m for m in maps if m['valid'] and m['receipt'] <= spline['receipt']]
        observed = available[-1] if available else None
        fresh = observed is not None and -.2e9 <= spline['stamp_ns'] - observed['stamp_ns'] <= 1e9
        certificate = fresh and MapCertificate(observed).trajectory(pieces)
        maximum = [derivative_maximum(pieces, n) for n in [1, 2, 3]]
        certificates.append(dict(session=spline['session'], trajectory_id=spline['trajectory_id'],
            original_start_stamp_ns=spline['stamp_ns'],
            maximum_derivatives=maximum, common_map_fresh=bool(fresh),
            common_map_continuously_free=bool(certificate),
            common_map_identity={k:observed[k] for k in ['session','generation','revision','stamp_ns']} if observed else None,
            duration_s=pieces[-1]['start']+pieces[-1]['duration'], jerk_energy=jerk_energy(pieces)))
    error = 0.
    matched = []
    unmatched = []
    for command in commands:
        key = (command['session'], command['trajectory_id'])
        if key not in lookup:
            unmatched.append(key)
            continue
        spline, pieces = lookup[key]
        relative = (command['stamp_ns']-spline['stamp_ns'])*1e-9
        for derivative, field in enumerate(['position','velocity','acceleration']):
            expected = value(pieces, relative, derivative)
            duration = pieces[-1]['start']+pieces[-1]['duration']
            if derivative and relative >= duration:
                expected = np.zeros(3)  # Actual traj_server end-hold branch.
            error = max(error, float(np.linalg.norm(expected-command[field])))
        matched.append(command)
    checks['original_spline_command_identity'] = bool(matched) and not unmatched
    checks['native_command_polynomial_consistency'] = bool(matched) and error <= 1e-6
    checks['same_unknown_blocked_map_certificate'] = bool(certificates) and all(c['common_map_continuously_free'] for c in certificates)
    checks['same_independent_dynamic_bounds'] = bool(certificates) and all(
        all(x <= bound+1e-6 for x,bound in zip(c['maximum_derivatives'],
            [limits['planned_speed_mps'], limits['max_acceleration_mps2'], limits['max_jerk_mps3']]))
        for c in certificates)
    tracking = []
    states = rows(case/'states.jsonl')
    for key, (spline,pieces) in lookup.items():
        own = [c for c in matched if (c['session'],c['trajectory_id']) == key]
        if not own:
            continue
        low,high = min(c['stamp_ns'] for c in own),max(c['stamp_ns'] for c in own)
        tracking += [float(np.linalg.norm(value(pieces,(s['source_ns']-spline['stamp_ns'])*1e-9)-s['position']))
                     for s in states if low <= s['source_ns'] <= high]
    checks['same_tracking_bound'] = bool(tracking) and max(tracking) <= limits['tracking_error_m']+1e-6
    checks['same_arrival_bound'] = bool(runtime.get('navigation_terminal_errors_m')) and max(runtime['navigation_terminal_errors_m']) <= limits['arrival_error_m']
    with (case/'truth.csv').open() as stream:
        truth = list(csv.DictReader(line for line in stream if not line.startswith('#')))
    native_clearance = None
    native_altitude_min = None
    if matched:
        low,high=min(c['stamp_ns'] for c in matched),max(c['stamp_ns'] for c in matched)
        flight=[t for t in truth if low<=int(t['stamp_ns'])<=high]
        if flight:
            points=np.asarray([[float(t[k]) for k in ['x','y','z']] for t in flight])
            native_altitude_min=float(points[:,2].min())
            for obstacle in runtime.get('obstacles',[]):
                distance=float(np.linalg.norm(np.maximum(
                    np.abs(points-np.asarray(obstacle['center']))-np.asarray(obstacle['size'])*.5,0.),axis=1).min())
                native_clearance=distance if native_clearance is None else min(native_clearance,distance)
            checks['same_native_floor_clearance']=native_altitude_min>=limits['collision_body_bounding_sphere_m']
            checks['same_native_obstacle_clearance']=native_clearance is None or native_clearance>=limits['independent_model_center_to_obstacle_aabb_m']
            if runtime['scenario']=='narrow':
                crossing=points[(points[:,0]>=.5)&(points[:,0]<=3.5)]
                checks['actual_native_narrow_traversal']=(bool(len(crossing)) and
                    float(np.abs(crossing[:,1]).max())<=1.1-.35 and
                    float(points[:,0].min())<=.5 and float(points[:,0].max())>=3.5)
        else:
            checks['native_truth_navigation_window']=False
    else:
        checks['native_truth_navigation_window']=False
    latency = []
    for request in runtime.get('planning_requests', []):
        own = [s for s in splines if s['session'].startswith(request['goal_uuid']+':') and s['receipt'] >= request['receipt']]
        if own:
            latency.append(min(s['receipt'] for s in own)-request['receipt'])
    report = dict(passed=all(checks.values()), checks=checks, certificates=certificates,
                  evaluator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  native_command_error_m=error, unmatched_command_keys=list(dict.fromkeys(unmatched)),
                  source_time_tracking_max_m=max(tracking) if tracking else None,
                  actual_spline_count=len(splines), matched_command_count=len(matched),
                  native_obstacle_clearance_min_m=native_clearance,
                  native_navigation_altitude_min_m=native_altitude_min,
                  navigation_to_first_spline_wall_seconds=latency,
                  case_result_sha256=hashlib.sha256((case/'result.json').read_bytes()).hexdigest(),
                  method='original native EGO knots/control points; exact Cox-de-Boor polynomial expansion, independent roots, continuous common-ray-map checks',
                  qualification='An EGO constraint failure remains a measured comparison result; it cannot be relabeled a passing SUPER flight.',
                  hardware=False)
    (case/'common-constraint-evaluation.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case',type=Path)
    result=audit(parser.parse_args().case)
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['passed'] else 1)

#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Compare measured body poses with independent native Gazebo truth, at source time."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def yaw(q):
    x, y, z, w = q
    return np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def evaluate(directory):
    result = json.loads((directory/'result.json').read_text())
    with (directory/'truth.csv').open() as stream:
        metadata = stream.readline().strip()
        truth = list(csv.DictReader(stream))
    rows = [json.loads(row) for row in (directory/'vio.jsonl').read_text().splitlines()]
    poses = [r for r in rows if r['stream'] == 'aligned_pose']
    if len(poses) < 100 or len(truth) < 100:
        raise ValueError('insufficient actual aligned poses or independent Gazebo truth')
    times = np.array([int(p['stamp_ns']) for p in truth], dtype=np.int64)
    # Gazebo can republish unchanged pose groups at the same source stamp.
    unique = np.r_[True, np.diff(times) > 0]
    if np.any(np.diff(times) < 0):
        raise ValueError('Gazebo truth source time rolled back')
    times = times[unique]
    positions = np.array([[float(p[k]) for k in ['x', 'y', 'z']] for p in truth])[unique]
    quaternions = np.array([[float(p[k]) for k in ['qx', 'qy', 'qz', 'qw']]
                            for p in truth])[unique]
    receipts = np.array([int(p['mono_ns']) for p in truth], dtype=np.int64)[unique]/1e9
    measured, matched, orientation_errors, delays = [], [], [], []
    rotation = shift = None
    initial_yaw = None
    missed = 0
    for p in poses:
        stamp = p['stamp_ns']
        index = np.searchsorted(times, stamp, side='right')
        if not 0 < index < len(times) or times[index]-times[index-1] > .15e9:
            missed += 1
            continue
        fraction = (stamp-times[index-1])/(times[index]-times[index-1])
        position = positions[index-1]+fraction*(positions[index]-positions[index-1])
        q0, q1 = quaternions[index-1], quaternions[index]
        if np.dot(q0, q1) < 0:
            q1 = -q1
        quaternion = q0+fraction*(q1-q0)
        quaternion /= np.linalg.norm(quaternion)
        if rotation is None:
            initial_yaw = yaw(quaternion)-yaw(p['quaternion'])
            c, s = np.cos(initial_yaw), np.sin(initial_yaw)
            rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
            shift = position-rotation@p['position']
        measured.append(rotation@p['position']+shift)
        matched.append(position)
        angle = yaw(p['quaternion'])+initial_yaw-yaw(quaternion)
        orientation_errors.append(abs(np.arctan2(np.sin(angle), np.cos(angle))))
        receipt = receipts[index-1]+fraction*(receipts[index]-receipts[index-1])
        delays.append(p['receipt']-receipt)
    measured, matched = np.asarray(measured), np.asarray(matched)
    error = np.linalg.norm(measured-matched, axis=1)
    limits = result['acceptance_limits']
    checks = dict(actual_flight_passed=result['passed'],
                  truth_not_discarded='discarded=0' in metadata,
                  sufficient_matches=len(measured) >= 100,
                  coverage=(len(measured)/len(poses)) >= .95,
                  position_rmse=float(np.sqrt(np.mean(error**2))) <= limits['position_rmse_m'],
                  position_max=float(error.max()) <= limits['position_max_m'],
                  yaw_max=max(orientation_errors) <= limits['attitude_error_rad'])
    obstacle = None
    if result.get('scenario') == 'obstacle':
        config = result['obstacle']
        center = np.asarray(config['center_world'][:2])
        active = (times >= config['first_source_ns']) & (positions[:, 2] > .6)
        route = positions[active]
        direction = np.asarray(config.get('nominal_route_direction', [3., 0.]))
        direction = direction/np.linalg.norm(direction)
        perpendicular = np.asarray([-direction[1], direction[0]])
        relative = route[:, :2]-center
        along = relative @ direction
        minimum = float(np.linalg.norm(route[:, :2]-center, axis=1).min())
        obstacle = dict(minimum_horizontal_center_distance_m=minimum,
                        maximum_lateral_detour_m=float(np.abs(relative @ perpendicular).max()),
                        minimum_along_route_m=float(along.min()),
                        maximum_along_route_m=float(along.max()),
                        maximum_altitude_m=float(route[:, 2].max()),
                        minimum_x_m=float(route[:, 0].min()),
                        maximum_x_m=float(route[:, 0].max()))
        checks.update(
            actual_obstacle_clearance=minimum >=
            config['required_horizontal_model_center_clearance_m'],
            actual_bypass_both_sides=obstacle['minimum_along_route_m'] < -.8 and
            obstacle['maximum_along_route_m'] > .8,
            actual_horizontal_detour=obstacle['maximum_lateral_detour_m'] >= .7,
            stayed_below_obstacle_top=obstacle['maximum_altitude_m'] <= 2.2)
    checks = {key: bool(value) for key, value in checks.items()}
    report = dict(passed=all(checks.values()), checks=checks, samples=len(measured),
                  unmatched=missed, position_rmse_m=float(np.sqrt(np.mean(error**2))),
                  position_p95_m=float(np.percentile(error, 95)), position_max_m=float(error.max()),
                  yaw_max_rad=float(max(orientation_errors)),
                  observed_truth_receipt_to_pose_wall_s=dict(
                      p50=float(np.percentile(delays, 50)), p95=float(np.percentile(delays, 95)),
                      maximum=float(max(delays))),
                  alignment='fixed scale 1; one yaw from first matched attitudes and one '
                            'translation from first matched positions; frozen thereafter; '
                            'no trajectory fit and no time shift',
                  reference='Native Gazebo model easyfly_iris truth, read-only; source-time '
                            'bracket interpolation only, max bracket .15 seconds',
                  initial_yaw_rad=float(initial_yaw), initial_translation_m=shift.tolist(),
                  limits=limits, obstacle=obstacle)
    (directory/'evaluation.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', type=Path, required=True)
    args = parser.parse_args()
    report = evaluate(args.case)
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

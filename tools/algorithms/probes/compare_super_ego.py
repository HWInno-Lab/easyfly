#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Describe paired actual EGO/SUPER flights without hiding constraint failures."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import numpy as np
from bspline_metrics import polynomial_pieces, jerk_energy
from summarize_openvins_resources import summarize


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line]


def distribution(values):
    return dict(count=len(values),minimum=min(values),median=float(np.median(values)),
                p95=float(np.percentile(values,95)),maximum=max(values)) if values else None


def geometry(pieces):
    # Numerical quadrature is a descriptive length metric, never admission.
    nodes, weights=np.polynomial.legendre.leggauss(48)
    length=0.
    for piece in pieces:
        times=(nodes+1)*piece['duration']/2
        velocities=np.asarray([np.polyval(np.polyder(c),times) for c in piece['coefficients']])
        length+=float(np.dot(weights,np.linalg.norm(velocities,axis=0))*piece['duration']/2)
    return dict(duration_s=sum(p['duration'] for p in pieces),
                length_m_48point_quadrature=length,integrated_squared_jerk=jerk_energy(pieces))


def describe(case):
    result=json.loads((case/'result.json').read_text())
    strategy=result['strategy']
    if strategy=='super':
        evaluation=json.loads((case/'independent-evaluation.json').read_text())
        proposals=rows(case/'polynomials.jsonl')
        trajectories=[]
        for proposal in proposals:
            offset=0.;pieces=[]
            for index,duration in enumerate(proposal['durations']):
                pieces.append(dict(start=offset,duration=duration,
                    coefficients=np.asarray(proposal['coefficients'][index*24:(index+1)*24]).reshape(3,8)))
                offset+=duration
            trajectories.append(dict(session=proposal['session'],trajectory_id=proposal['trajectory_id'],**geometry(pieces)))
        latency=[min(p['receipt'] for p in proposals
                      if p['session'].startswith(request['goal_uuid']+':') and p['receipt']>=request['receipt'])-request['receipt']
                 for request in result['planning_requests']]
    else:
        evaluation=json.loads((case/'common-constraint-evaluation.json').read_text())
        proposals=rows(case/'ego-bsplines.jsonl')
        trajectories=[dict(session=p['session'],trajectory_id=p['trajectory_id'],**geometry(polynomial_pieces(p)))
                      for p in proposals if p['session']]
        latency=evaluation['navigation_to_first_spline_wall_seconds']
    resources=summarize(case)
    names=['super_navigation','gcopter_voxel_map'] if strategy=='super' else ['ego_planner_node','traj_server']
    process_resources=[p for p in resources['processes']
                       if any(p['executable'].endswith('/'+name) for name in names)]
    budget=json.loads((case/'process-budget.json').read_text())
    sampled_peak=max(sum(p['rss_pages']*budget['page_size']/2**20 for p in sample['processes']
                         if Path(p['executable']).name in names+(['super_worker'] if strategy=='super' else []))
                     for sample in budget['samples'])
    worker_resources=[]
    if strategy=='super':
        for log in sorted(case.glob('super-*/job-*.log')):
            match=re.search(r'worker_resource cpu_seconds=([\d.e+-]+) peak_rss_kib=(\d+)',log.read_text())
            if match:
                worker_resources.append(dict(log=str(log.relative_to(case)),cpu_seconds=float(match[1]),peak_rss_mib=int(match[2])/1024))
    commands=rows(case/'commands.jsonl')
    states=rows(case/'states.jsonl')
    if commands:
        low,high=min(c['stamp_ns'] for c in commands),max(c['stamp_ns'] for c in commands)
        actual=[s for s in states if low<=s['source_ns']<=high]
        actual_length=sum(float(np.linalg.norm(np.asarray(a['position'])-b['position'])) for a,b in zip(actual,actual[1:]))
    else:
        actual_length=None
    actions=[a for a in result['actions'] if a['name']=='/navigate']
    first_proposals=[]
    goal_times=[]
    for request in result['planning_requests']:
        own=[p for p in proposals if p['session'].startswith(request['goal_uuid']+':') and p['receipt']>=request['receipt']]
        if own:
            first=min(own,key=lambda p:p['receipt'])
            matched_geometry=next(g for g in trajectories if g['session']==first['session'] and g['trajectory_id']==first['trajectory_id'])
            first_proposals.append(dict(goal_uuid=request['goal_uuid'],wall_latency_s=first['receipt']-request['receipt'],
                target=request['target'],proposal=matched_geometry))
        terminal_action=next((a for a in actions if a['goal_uuid']==request['goal_uuid']),None)
        if terminal_action and terminal_action.get('terminal_state'):
            terminal=terminal_action['terminal_state']
            goal_times.append(dict(goal_uuid=request['goal_uuid'],
                status=terminal_action['status'],success=terminal_action['success'],
                source_seconds=(terminal['source_ns']-request['source_ns'])*1e-9,
                wall_seconds=terminal['steady_now']-request['receipt']))
    return dict(case=str(case),strategy=strategy,scenario=result['scenario'],runtime_passed=result['passed'],
        runtime_error=result['error'],expected_route_goal_count=2,
        unissued_route_goals_after_abort=max(0,2-len(actions)),
        common_constraint_passed=evaluation['passed'],
        constraint_failures=evaluation.get('failures') or [k for k,v in evaluation.get('checks',{}).items() if not v],
        completed_goals=sum(a.get('success',False) for a in actions),requested_goals=len(actions),
        navigation_to_first_proposal_wall_s=distribution(latency),proposals=trajectories,
        first_proposals_by_goal=first_proposals,actual_goal_elapsed=goal_times,
        actual_measured_route_length_m=actual_length,
        tracking_max_m=evaluation.get('source_time_tracking_max_m'),
        terminal_errors_m=result.get('navigation_terminal_errors_m'),
        source_per_wall=resources['source_per_wall'],backend_processes=process_resources,
        sampled_simultaneous_backend_rss_sum_peak_mib=sampled_peak,
        bounded_worker_kernel_resources=worker_resources,
        evidence_result_sha256=hashlib.sha256((case/'result.json').read_bytes()).hexdigest())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['super-smoke','ego-smoke','super-narrow','ego-narrow']:
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    checks={};pairs=[]
    native_file=args.ego_narrow/'native-loaded-libraries.json'
    native_capture=json.loads(native_file.read_text())
    native_ego=next(p for p in native_capture['processes'] if Path(p['executable']).name=='ego_planner_node')
    checks['actual_owned_native_ego_libraries']=native_capture['passed'] and all(
        any(Path(path).name==library for path in native_ego['libraries'])
        for library in ['libbspline_opt.so','libplan_env.so','libpath_searching.so'])
    for scenario in ['smoke','narrow']:
        left=getattr(args,'super_'+scenario);right=getattr(args,'ego_'+scenario)
        a=json.loads((left/'result.json').read_text());b=json.loads((right/'result.json').read_text())
        equal_keys=['scene','server_sha256','px4_sha256','truth_executable_sha256','acceptance_limits']
        checks[scenario+'_physical_server_sensors_limits']=all(a[k]==b[k] for k in equal_keys)
        checks[scenario+'_physical_obstacles']=([(o['center'],o['size']) for o in a.get('obstacles',[]) ] ==
                                                [(o['center'],o['size']) for o in b.get('obstacles',[]) ])
        producer_hashes={sha for path,sha in a['runtime_executables_sha256'].items()
                         if path.endswith('/gcopter_voxel_map')}
        checks[scenario+'_common_native_ray_lineage']=(a.get('native_rays',{}).get('count',0)>0 and
            b['comparison_map']['same_source_ray_map'] and b['voxel_samples']>0 and
            b['comparison_map']['executable_sha256'] in producer_hashes)
        checks[scenario+'_actual_sensor_plugins']=all(
            bool({sha for path,sha in a['runtime_libraries_sha256'].items() if Path(path).name==library}) and
            {sha for path,sha in a['runtime_libraries_sha256'].items() if Path(path).name==library}==
            {sha for path,sha in b['runtime_libraries_sha256'].items() if Path(path).name==library}
            for library in ['liblivox_laser_simulation.so','libeasyfly_native_imu.so'])
        checks[scenario+'_actual_ego_executable_identity']=b['runtime_executables_sha256'][native_ego['executable']]==native_ego['executable_sha256']
        descriptions=[describe(p) for p in [left,right]]
        checks[scenario+'_super_actual_two_goals']=descriptions[0]['runtime_passed'] and descriptions[0]['completed_goals']==descriptions[0]['requested_goals']==2
        ego_actions=[action for action in b['actions'] if action['name']=='/navigate']
        # An actual failed reference trial is a measured reliability result.
        # Startup/instrumentation failures and unfinished cleanup are not.
        checks[scenario+'_ego_measurement_complete']=(b['strategy']=='ego' and b['selected_roles_started'] and
            b['comparison_map_returncode']==b['launch_returncode']==b['truth_returncode']==0 and
            b['sessions_closed'] and not b['forced'] and not b['leftovers'] and
            not b['abnormal_lines'] and b['transport_drained'] and not b['discarded'] and
            not b['final_disarmed_state']['armed'] and set(b['normal_exits'])==set(b['started_processes']) and
            bool(ego_actions) and all(a['status'] in [4,6] for a in ego_actions) and
            all(a['success'] or a['message']=='planner_emergency_stop' for a in ego_actions) and
            bool(descriptions[1]['proposals']))
        checks[scenario+'_super_common_constraints']=descriptions[0]['common_constraint_passed']
        pairs.append(dict(scenario=scenario,results=descriptions))
    report=dict(passed=all(checks.values()),checks=checks,pairs=pairs,
        evaluator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        actual_ego_native_library_capture=dict(case=str(args.ego_narrow),
            sha256=hashlib.sha256(native_file.read_bytes()).hexdigest(),
            qualification='Mappings were observed during the actual narrow flight. Both comparison flights separately recorded the identical native planner executable hash.'),
        interpretation='passed certifies a complete comparison measurement and SUPER gates; it does not mark every EGO trial successful. Actual reference failures retain their original false runtime/common evaluation and unissued return goals. No claim that SUPER is universally faster.',
        map_representation='SUPER imports the exact source-time inflated ray grid into native ROG-Map and blocks unknown. EGO uses its native sensor map; an observer-only copy of the same ray producer certifies its original proposals against the common grid.',
        sensor_lineage='Both flights record the identical actually loaded lidar/IMU plugin hashes. SUPER also records raw ray packets/counts; EGO records the actual source-time ray-map observer with identical producer executable hash and valid integrated maps. Raw EGO ray packet counts were not recorded and are not inferred.',
        dynamics='Both request .6 m/s; EGO manager/optimizer acceleration 2 and native jerk 4 (stricter than common 8) are read back. SUPER native soft targets .3/1.5/6; independent hard common limits .6/2/8.',
        resources='Owned PID/start-tick 1 Hz samples. SUPER backend includes its required ray-map producer, owner and bounded workers; EGO includes planner/internal map and traj_server. Short accepted SUPER workers additionally report kernel total CPU and peak RSS. Sampled simultaneous backend RSS can miss short worker peaks and counts shared pages repeatedly. Owner CPU and whole-worker CPU must not be summed as instantaneous utilization. EGO observer-only common map is listed in raw resources and excluded from backend totals, but its CPU overhead remains part of the actual host conditions.',
        statistics='Two finite physical routes, one run per planner per route. Quantiles describe these samples only. Proposal metrics include superseded EGO proposals; they are not total executed distance. Actual measured route length includes sensor noise.',
        hardware=False)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(passed=report['passed'],checks=checks),indent=2))
    raise SystemExit(0 if report['passed'] else 1)


if __name__=='__main__':
    main()

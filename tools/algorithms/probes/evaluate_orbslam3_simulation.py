#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Fixed-scale native-truth pose evaluation and actual ORB atlas/input audits."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

spec=importlib.util.spec_from_file_location('native_pose_metrics',Path(__file__).with_name('evaluate_fastlivo_simulation.py'))
metrics=importlib.util.module_from_spec(spec);spec.loader.exec_module(metrics)


def main():
    p=argparse.ArgumentParser();p.add_argument('case',type=Path);a=p.parse_args()
    result=json.loads((a.case/'result.json').read_text())
    ev=metrics.evaluate(a.case);ev['backend']='orbslam3'
    rows=[json.loads(x) for x in (a.case/'vio.jsonl').read_text().splitlines()]
    tracks=[r['tracked_features'] for r in rows if r['stream']=='visual_tracking' and r['initialized']]
    ev['visual_updates_ge30']=sum(v>=30 for v in tracks)
    ev['checks']['actual_visual_updates']=ev['visual_updates_ge30']>=100
    fingerprints=[json.loads(x) for x in (a.case/'input-fingerprints.jsonl').read_text().splitlines()]
    original={(r['kind'],r['stamp_ns']):r['sha256'] for r in fingerprints if not r['kind'].startswith('forwarded_')}
    forwarded=[r for r in fingerprints if r['kind'].startswith('forwarded_')]
    failures=[];seen=set()
    for r in forwarded:
        key=(r['kind'].removeprefix('forwarded_'),r['stamp_ns'])
        if key in seen or original.get(key)!=r['sha256']:failures.append(dict(kind=r['kind'],stamp_ns=r['stamp_ns']))
        seen.add(key)
    ev['checks']['original_input_field_fidelity']=len(forwarded)>=200 and not failures
    if result.get('sensor_mode')=='rgbd_imu':
        ev['checks']['original_imu_forwarded']=sum(r['kind']=='forwarded_imu' for r in forwarded)>=1000
        ready=[r for r in rows if r['stream']=='map_lifecycle' and r['values'].get('inertial_ready')=='1']
        ev['checks']['completed_inertial_initialization']=bool(ready)
        public=[r for r in rows if r['stream']=='local_body_pose']
        ev['checks']['no_pose_before_inertial_ready']=bool(ready) and bool(public) and min(r['stamp_ns'] for r in public)>=min(r['stamp_ns'] for r in ready)
    ev['input_audit']=dict(forwarded_observed=len(forwarded),original_observed=len(original),mismatches=failures)
    statuses=[r for r in rows if r['stream']=='status' and r['pose_valid']]
    ev['checks']['covariance_explicitly_unavailable']=bool(statuses) and not any(r['covariance_available'] for r in statuses)
    runtime=a.case/('orb-runtime' if (a.case/'orb-runtime').is_dir() else 'runtime')
    atlas=json.loads((runtime/'atlas-final.json').read_text());ev['atlas']=atlas
    ev['checks']['workers_joined_before_map_save']=atlas['workers_joined']
    if result.get('load_atlas'):
        ev['checks']['actual_saved_map_relocalized']=atlas['loaded'] and atlas['relocalizations']>=1 and atlas['map_id']==atlas['loaded_map_id'] and atlas['loaded_keyframes']>=2 and atlas['keyframes']==atlas['loaded_keyframes']
    elif result.get('scenario') not in ['visual-loss', 'clock-reset']:
        path=runtime/'atlas.osa';ev['checks']['actual_atlas_saved']=path.is_file() and path.stat().st_size>10000 and atlas['keyframes']>=2
    else:
        # Deliberate loss/reset may legitimately empty the map. This is a fault
        # isolation case, never proof of successful map persistence or reuse.
        ev['map_persistence_scope']='not scored after intentional tracking/clock reset'
    if result.get('scenario') in ['visual-loss', 'localization-loss', 'backend-exit', 'map-loss', 'cancel']:
        injection=result.get('injection',{})
        ev['checks']['actual_task_stop_and_stream_quiet']=injection.get('actual_trajectory_stream_quiet') is True
        if result['scenario'] in ['visual-loss','localization-loss','backend-exit']:
            ev['checks']['new_task_refused_after_source_failure']=injection.get('new_navigation_rejected') is True
    if result.get('scenario')=='long-run':
        ev['checks']['full_120_source_second_hold']=result.get('long_hold_source_seconds',0)>=120
    # Actual loop evidence remains a separate gate for the final whole-scope verdict.
    ev['actual_loop_closure_observed']=atlas['loops']>0
    console=(a.case/'console.log').read_text();ev['checks']['no_failed_launch_child']='process has died' not in console and 'Traceback (most recent call last)' not in console
    ev['evaluator_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest();ev['passed']=all(ev['checks'].values())
    (a.case/'evaluation.json').write_text(json.dumps(ev,indent=2)+'\n');print(json.dumps(ev,indent=2));return 0 if ev['passed'] else 1


if __name__=='__main__':raise SystemExit(main())

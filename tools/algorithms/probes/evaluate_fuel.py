#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Independent post-run coverage, native decision, polynomial and physical audits."""
import argparse
import base64
import csv
import gzip
import hashlib
import json
import math
import re
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from evaluate_gcopter import maximum, evaluate, rows
from audit_fuel_sensor import audit as audit_sensor


def revisit_metrics(truth, begin, end):
    """Distance within .25m of a point at least 2m earlier along the path.

    Use native physical positions, sampled after .05m movement to suppress hover
    jitter. This is a descriptive benchmark metric, not an acceptance threshold.
    """
    points=[]
    for row in truth:
        if not begin <= int(row['stamp_ns']) <= end:continue
        p=np.array([float(row[k]) for k in ['x','y','z']])
        if not points or np.linalg.norm(p-points[-1])>=.05:points.append(p)
    if len(points)<2:return dict(distance_m=0.,revisited_distance_m=0.,revisited_fraction=0.)
    points=np.array(points);steps=np.linalg.norm(np.diff(points,axis=0),axis=1)
    arc=np.r_[0.,np.cumsum(steps)];revisited=0.
    for i,step in enumerate(steps,1):
        earlier=points[arc<=arc[i]-2.]
        if len(earlier) and np.linalg.norm(earlier-points[i],axis=1).min()<=.25:revisited+=step
    return dict(distance_m=float(arc[-1]),revisited_distance_m=float(revisited),
                revisited_fraction=float(revisited/arc[-1]),spatial_radius_m=.25,
                minimum_prior_arc_distance_m=2.,movement_sampling_m=.05)


def resource_metrics(directory):
    data=json.loads((directory/'process-budget.json').read_text());roles={};processes={};previous={}
    launched={int(pid):role for role,pid in re.findall(
        r'\[([^\]\n]+)\]: process started with pid \[(\d+)\]',(directory/'console.log').read_text())}
    peak_total=0
    for sample in data['samples']:
        peak_total=max(peak_total,sum(p['rss_pages']*data['page_size'] for p in sample['processes']))
        for p in sample['processes']:
            key=(p['pid'],p['start_ticks']);ticks=p['user_ticks']+p['system_ticks']
            role=roles.setdefault(p['executable'],dict(peak_rss_mib=0.,observed_cpu_sec=0.,peak_one_core_percent=0.))
            individual=processes.setdefault(str(key),dict(pid=p['pid'],start_ticks=p['start_ticks'],
                executable=p['executable'],launch_role=launched.get(p['pid']),peak_rss_mib=0.,
                observed_cpu_sec=0.,peak_one_core_percent=0.,first_sample_ns=sample['begin_ns']))
            individual['last_sample_ns']=sample['begin_ns'];individual['executable']=p['executable']
            for value in [role,individual]:
                value['peak_rss_mib']=max(value['peak_rss_mib'],p['rss_pages']*data['page_size']/2**20)
            if key in previous:
                old_time,old_ticks=previous[key];delta=(ticks-old_ticks)/data['clock_ticks_per_second']
                elapsed=(sample['begin_ns']-old_time)/1e9
                if delta>=0 and elapsed>0:
                    for value in [role,individual]:
                        value['observed_cpu_sec']+=delta
                        value['peak_one_core_percent']=max(value['peak_one_core_percent'],100*delta/elapsed)
            previous[key]=(sample['begin_ns'],ticks)
    for value in processes.values():
        elapsed=(value['last_sample_ns']-value['first_sample_ns'])/1e9
        value['mean_one_core_percent']=100*value['observed_cpu_sec']/elapsed if elapsed>0 else None
    return dict(sample_count=len(data['samples']),peak_sum_process_rss_mib=peak_total/2**20,
                note='1Hz owned processes; RSS sum includes shared pages; short-lived work may fall between samples',
                roles=roles,processes=list(processes.values()))


def eligible_mask(key, mission):
    nx,ny,nz,res,ox,oy,oz=key[6:]
    z,y,x=np.indices((nz,ny,nx))
    centers=np.column_stack([x.ravel(),y.ravel(),z.ravel()])*res+np.array([ox,oy,oz])+res/2
    low=np.array(mission['min_bound']);high=np.array(mission['max_bound'])
    eligible=((centers-res/2>=low-1e-8)&(centers+res/2<=high+1e-8)).all(axis=1)
    navigable=eligible.copy()
    boxes=np.array(mission['forbidden_boxes']).reshape((-1,2,3))
    for lo,hi in boxes:
        eligible &= ~((centers+res/2>lo)&(centers-res/2<hi)).all(axis=1)
        navigable &= ~((centers+res/2>lo-.65)&(centers-res/2<hi+.65)).all(axis=1)
    return eligible,navigable,centers


def audit(directory):
    result=json.loads((directory/'result.json').read_text());failures=[];details=[]
    def require(value,reason):
        if not value:failures.append(reason)
    attachment=audit_sensor(directory)
    require(attachment['passed'],'native sensor attachment audit failed')
    limits=result['acceptance_limits']
    require(result['passed'],'managed run failed')
    require(result['shutdown_seconds']<=limits['normal_shutdown_s'],'normal stack shutdown deadline')
    native_ray_count=result.get('native_rays',{}).get('count',0)
    require(native_ray_count>20,'missing real native ray stream')
    for p in directory.glob('rays-*.bin'):
        cloud=np.frombuffer(p.read_bytes(),dtype=np.dtype([('xyz','<f4',(3,)),('type','u1'),('padding','u1',(3,)),('min','<f4'),('max','<f4')]))
        require(np.all(cloud['max']==3.) and np.all(np.linalg.norm(cloud['xyz'],axis=1)<=3.011),'physical ray range exceeded')
    plans=rows(directory/'polynomials.jsonl');commands=rows(directory/'commands.jsonl');states=rows(directory/'states.jsonl')
    with gzip.open(directory/'voxel-grid.jsonl.gz','rt') as stream:
        maps={(m['session'],m['generation'],m['revision']):m for m in map(json.loads,stream)}
    lookup={(p['session'],p['trajectory_id']):p for p in plans}
    dynamics=[];tracking=[];command_error=0.
    for plan in plans:
        for revision,stamp_name in [('map_revision','map_stamp_ns'),('validated_map_revision','validated_map_stamp_ns')]:
            source=maps.get((plan['map_session'],plan['map_generation'],plan[revision]))
            require(source and source['valid'] and source['stamp_ns']==plan[stamp_name], 'trajectory source map identity mismatch')
        maxima=[maximum(plan,k) for k in (1,2,3)];dynamics.append(maxima)
        require(all(v<=limit+1e-6 for v,limit in zip(maxima,[.6,2.,8.])), 'independent continuous dynamics exceeded')
        selected=[c for c in commands if (c['session'],c['trajectory_id'])==(plan['session'],plan['trajectory_id'])]
        require(bool(selected),'trajectory without recorded commands')
        if selected:
            begin,end=min(c['stamp_ns'] for c in selected),max(c['stamp_ns'] for c in selected)
            tracking.extend(float(np.linalg.norm(evaluate(plan,s['source_ns'])-s['position']))
                            for s in states if begin<=s['source_ns']<=end)
    for command in commands:
        key=command['session'],command['trajectory_id'];require(key in lookup,'command without trajectory')
        if key in lookup:
            for derivative,field in enumerate(['position','velocity','acceleration']):
                command_error=max(command_error,float(np.linalg.norm(evaluate(lookup[key],command['stamp_ns'],derivative)-command[field])))
    require(command_error<=1e-6,'command differs from native polynomial')
    require(not tracking or max(tracking)<=.2+1e-6,'source-time tracking exceeded .2m')
    with (directory/'truth.csv').open() as stream:
        truth=list(csv.DictReader(line for line in stream if not line.startswith('#')))
    clearance=None;altitude=None;path_length=0.
    if commands:
        begin,end=min(c['stamp_ns'] for c in commands),max(c['stamp_ns'] for c in commands)
        flight=[t for t in truth if begin<=int(t['stamp_ns'])<=end]
        require(bool(flight),'missing native physical truth')
        require(all(.5<=s['position'][2]<=2.5 for s in states if begin<=s['source_ns']<=end),'measured flight altitude out of bounds')
        if flight:
            points=np.array([[float(t[k]) for k in ['x','y','z']] for t in flight])
            altitude=float(points[:,2].min());require(altitude>=.35,'physical floor collision clearance')
            path_length=float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum())
            world=directory/'source-snapshot/src/simulation/easyfly_gazebo_worlds/worlds/benchmark/exploration'/f"{result['scene']}.world"
            for model in ET.parse(world).getroot().findall('world/model'):
                if model.attrib['name']=='arena_floor':continue
                pose=np.array(list(map(float,model.find('pose').text.split()))[:3])
                size=np.array(list(map(float,model.find('link/collision/geometry/box/size').text.split())))
                distance=float(np.linalg.norm(np.maximum(np.abs(points-pose)-size/2,0.),axis=1).min())
                clearance=distance if clearance is None else min(clearance,distance)
                require(distance>=.35,'native obstacle clearance below .35m: '+model.attrib['name'])
    for mission_dir in sorted((directory/'fuel').glob('*/')):
        mission_path=mission_dir/'mission.json'
        if not mission_path.exists():continue
        mission=json.loads(mission_path.read_text());events=rows(mission_dir/'events.jsonl')
        with gzip.open(mission_dir/'source-maps.jsonl.gz','rt') as stream:
            source_maps=list(map(json.loads,stream))
        require(bool(source_maps),'task missing real observation snapshots')
        key=source_maps[0]['identity'];eligible,navigable,centers=eligible_mask(key,mission)
        require(hashlib.sha256(eligible.tobytes()).hexdigest()==mission['eligible_sha'],'independent task volume differs')
        coverages=[];raw_lookup={}
        for entry in source_maps:
            raw=np.frombuffer(base64.b64decode(entry['raw']),dtype=np.int8)
            inflated=np.frombuffer(base64.b64decode(entry['inflated']),dtype=np.int8)
            require(entry['identity'][:2]==key[:2] and entry['identity'][6:]==key[6:],'task silently changed source map epoch')
            require(entry['inflation']>=.65 and not ((inflated==0)&(raw!=0)).any(),'inflated free without observation')
            coverage=float(np.count_nonzero((raw!=-1)&eligible)/eligible.sum());coverages.append(coverage)
            require(abs(coverage-entry['coverage'])<1e-12,'coverage counted policy/inflation as observation')
            raw_lookup[tuple(entry['identity'])]=(raw,inflated)
            nav_map=maps.get((mission['mission_id'],entry['identity'][1],entry['identity'][2]))
            if nav_map:
                policy_data=inflated.copy();policy_data[~navigable]=100
                actual=np.frombuffer(base64.b64decode(nav_map['data_b64']),dtype=np.int8)
                require(np.array_equal(policy_data,actual),'motion map bypassed policy mask')
        initial_path=mission_dir/'initial-source.json'
        if initial_path.exists():
            initial=json.loads(initial_path.read_text());initial_raw=np.array(initial['raw'],dtype=np.int8)
            actual_initial=float(np.count_nonzero((initial_raw!=-1)&eligible)/eligible.sum())
            raw_lookup[tuple(initial['identity'])]=(initial_raw,np.array(initial['inflated'],dtype=np.int8))
            require(abs(actual_initial-mission['initial_coverage'])<1e-12,'initial coverage is not actual observations')
        else:
            # Early development run: only an equal first actual scan can substantiate its initial count.
            require(abs(coverages[0]-mission['initial_coverage'])<1e-12,'missing initial observation snapshot')
        arrivals=[e for e in events if e['phase']=='ARRIVED']
        require(all(e['error']<=.11 for e in arrivals),'viewpoint arrival error exceeded')
        actual_decisions=[]
        for source in sorted(mission_dir.glob('*.json')):
            answer=Path(str(source)+'.result.yaml')
            if not answer.exists():continue
            req=json.loads(source.read_text());decision=yaml.safe_load(answer.read_text())
            require(req['request_id']==decision['request_id'],'native request/result mismatch')
            require(decision['wall_sec']<=4. and decision['cpu_total_sec']<=120. and decision['max_rss_kib']<=768*1024,'native resource limits exceeded')
            actual_decisions.append(decision)
            original=raw_lookup.get(tuple(req['source_identity']))
            if original is not None:
                a,b=original[0].copy(),original[1].copy();a[~eligible]=100;b[~navigable]=100
                require(np.array_equal(a,req['raw']) and np.array_equal(b,req['inflated']),'native input differs from source observations/policy')
            else:
                require(not initial_path.exists(),'missing native source snapshot')
            if decision['reason']=='NEXT_VIEWPOINT':
                goal=np.array(decision['goal']);cell=np.floor((goal-np.array(req['origin']))/.2).astype(int)
                nx,ny,nz=req['size'];index=int(cell[0]+nx*(cell[1]+ny*cell[2]))
                require((cell>=0).all() and (cell<np.array(req['size'])).all() and req['raw'][index]==req['inflated'][index]==0,'unsafe native viewpoint')
                require(any(np.linalg.norm(goal-e['target'])<1e-9 for e in events if e['phase']=='VIEWPOINT'),'native selected view not handed to navigation')
        require([d['updates'] for d in actual_decisions]==list(range(1,len(actual_decisions)+1)),'native FIS not persistent')
        require(any(e['phase']=='WORKER_REAPED' for e in events),'native worker not reaped before task terminal')
        terminal=[e for e in events if e['phase']=='TERMINAL'][-1]
        require(not terminal['fault_lock'],'task left unresolved navigation ownership')
        measured=np.array([s['position'] for s in states if events[0]['source']<=s['source_ns']<=terminal['source']])
        require(len(measured)>0,'mission missing measured task-volume positions')
        if len(measured):
            require(((measured>=np.array(mission['min_bound'])) &
                     (measured<np.array(mission['max_bound']))).all(),
                    'measured aircraft center left declared task volume')
        if terminal['success']:
            require(terminal['coverage']>=.9 and max(coverages)>=.9,'completed task below coverage target')
            if mission['return_home']:
                homes=[e for e in events if e['phase']=='RETURN_HOME']
                require(len(homes)==1 and any(e.get('child_id')==homes[0]['child_id'] for e in arrivals),'return-home did not actually arrive')
        else:
            require(terminal['reason']!='COVERAGE_COMPLETE','failed task claimed completion')
        targets=[e['target'] for e in events if e['phase']=='VIEWPOINT_COMPLETE']
        repeats=sum(any(math.dist(p,q)<.15 for q in targets[:i]) for i,p in enumerate(targets))
        require(repeats==0,'repeated viewpoint without completion')
        home_events=[e for e in events if e['phase']=='RETURN_HOME']
        split=home_events[0]['source'] if home_events else terminal['source']
        details.append(dict(mission_id=mission['mission_id'],source_maps=len(source_maps),
            initial_coverage=mission['initial_coverage'],final_coverage=terminal['coverage'],
            gain=terminal['coverage']-mission['initial_coverage'],completed=terminal['completed'],
            decision_count=len(actual_decisions),repeat_viewpoints=repeats,reason=terminal['reason'],
            source_duration_s=(terminal['source']-events[0]['source'])/1e9,
            wall_duration_s=terminal['receipt']-events[0]['receipt'],
            remaining_unknown_cells=int(round((1-terminal['coverage'])*eligible.sum())),
            peak_rss_kib=max([d['max_rss_kib'] for d in actual_decisions],default=0),
            native_cpu_sec=max([d['cpu_total_sec'] for d in actual_decisions],default=0),
            measured_position_min=measured.min(axis=0).tolist() if len(measured) else None,
            measured_position_max=measured.max(axis=0).tolist() if len(measured) else None,
            mission_path=revisit_metrics(truth,events[0]['source'],terminal['source']),
            exploration_path=revisit_metrics(truth,events[0]['source'],split),
            return_path=revisit_metrics(truth,split,terminal['source']) if home_events else None,
            max_decision_wall_sec=max([d['wall_sec'] for d in actual_decisions],default=0)))
    if result['scenario']=='complete':
        require(len(details)==1 and details[0]['gain']>=.15 and details[0]['completed']>=2,'complete benchmark misses frozen gain/viewpoint gates')
    reasons=[m['reason'] for m in details]
    expected={'cancel':'CANCELED','map-loss':'SOURCE_STATE_OR_MAP_STALE',
              'worker-exit':'NATIVE_WORKER_EXIT','worker-timeout':'NATIVE_DECISION_DEADLINE',
              'timeout':'MISSION_DEADLINE','land-takeover':'CANCELED'}
    scenario=result['scenario']
    if scenario in expected:
        require(any(reason.startswith(expected[scenario]) for reason in reasons),
                'fault did not terminate for expected cause: '+expected[scenario])
    if scenario in ('cancel','map-loss'):
        require(result.get('explicit_recovery_completed') and len(details)==2 and
                'COVERAGE_COMPLETE' in reasons,'explicit new mission recovery missing')
    if scenario=='land-takeover':
        lands=[a for a in result['actions'] if a['name']=='/land']
        require(result.get('concurrent_land_rejected') and result.get('controlled_land_handover_completed') and
                len(lands)==2 and not lands[0]['accepted'] and lands[1]['accepted'] and
                lands[1].get('success') and not result['final_disarmed_state']['armed'],
                'parallel Land exclusion and explicit cancel/release/Land handover missing')
    if scenario=='unreachable':
        require(len(details)==1 and details[0]['final_coverage']<.9 and
                details[0]['remaining_unknown_cells']>0 and
                reasons[0] in ('NO_FRONTIER','NO_REACHABLE_FRONTIER','SELECTED_FRONTIER_UNREACHABLE'),
                'unreachable region falsely completed or failed for unrelated cause')
    if scenario=='no-fly':
        require(len(details)==1 and reasons==['COVERAGE_COMPLETE'],'no-fly mission did not complete')
        for mission_dir in (directory/'fuel').glob('*/'):
            mission=json.loads((mission_dir/'mission.json').read_text())
            require(bool(mission['forbidden_boxes']),'no-fly case lacks declared boxes')
            events=rows(mission_dir/'events.jsonl');begin=events[0]['source'];end=events[-1]['source']
            points=np.array([s['position'] for s in states if begin<=s['source_ns']<=end])
            require(len(points)>20,'no-fly measured motion missing')
            for lo,hi in np.array(mission['forbidden_boxes']).reshape((-1,2,3)):
                distance=np.linalg.norm(np.maximum(np.maximum(lo-points,points-hi),0.),axis=1)
                require(float(distance.min())>=.35,'actual no-fly clearance below physical .35m gate')
    output=dict(passed=not failures,failures=failures,sensor_attachment=attachment,missions=details,native_truth_samples=len(truth),
        path_length_m=path_length,independent_clearance_min_m=clearance,native_altitude_min_m=altitude,
        polynomial_count=len(plans),command_count=len(commands),command_polynomial_error=command_error,
        source_time_tracking_max_m=max(tracking,default=0.),continuous_dynamic_maxima=dynamics,
        resources=resource_metrics(directory),
        evaluator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    target=directory/'independent-evaluation.json'
    if target.exists():
        old=target.read_bytes();archive=directory/('independent-evaluation.'+hashlib.sha256(old).hexdigest()[:12]+'.json')
        if not archive.exists():archive.write_bytes(old)
    target.write_text(json.dumps(output,indent=2)+'\n')
    return output


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    value=audit(parser.parse_args().directory);print(json.dumps(value,indent=2));raise SystemExit(0 if value['passed'] else 1)

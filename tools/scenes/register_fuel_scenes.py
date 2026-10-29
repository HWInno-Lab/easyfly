#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Register only measured FUEL benchmark scenes; never edits failed run evidence."""
from pathlib import Path
import csv
import hashlib
import json
import math
import copy
import struct
import yaml

ROOT=Path(__file__).resolve().parents[2]
R=ROOT/'artifacts/new-algorithms/new5-fuel-20261023'
W=ROOT/'src/simulation/easyfly_gazebo_worlds'


def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    excluded=json.loads((R/'evidence-exclusions.json').read_text())
    for scene,bootstrap,flight in [('fuel_room_v1','bootstrap-room-5','room-flight-4'),
                                   ('fuel_warehouse_v1','bootstrap-warehouse-2','warehouse-flight-3')]:
        boot=R/bootstrap;run=R/flight
        b=json.loads((boot/'result.json').read_text());r=json.loads((run/'result.json').read_text())
        check=json.loads((run/'independent-evaluation.json').read_text())
        assert b['passed'] and r['passed'] and check['passed']
        for path in [boot,run]:
            assert json.loads((path/'sensor-attachment-audit.json').read_text())['passed']
            assert path.name not in str(excluded), 'excluded scene qualification evidence'
        state=b['bootstrap_state'];assert not state['armed'] and state['pose_valid'] and 'yaw' in state
        with (boot/'truth.csv').open() as stream:truth=list(csv.DictReader(line for line in stream if not line.startswith('#')))
        actual=min(truth,key=lambda row:abs(int(row['stamp_ns'])-state['source_ns']))
        dt=abs(int(actual['stamp_ns'])-state['source_ns'])/1e9;assert dt<.1
        x,y,z=(float(actual[k]) for k in ['x','y','z'])
        qx,qy,qz,qw=(float(actual[k]) for k in ['qx','qy','qz','qw'])
        yaw=state['yaw']-math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
        c,s=math.cos(yaw),math.sin(yaw)
        translation=[state['position'][0]-(c*x-s*y),state['position'][1]-(s*x+c*y),state['position'][2]-z]
        evidence=W/'evidence'/scene
        sensors=[]
        for ray in sorted(boot.glob('rays-*.bin')):
            data=ray.read_bytes();ranges=[struct.unpack_from('<f',data,i+20)[0] for i in range(0,len(data),24)]
            assert ranges and min(ranges)==max(ranges)==3.
            sensors.append(dict(file=str(ray.relative_to(ROOT)),sha256=digest(ray),actual_range_max_m=3.,rays=len(ranges)))
        assert sensors
        world=W/'worlds/benchmark/exploration'/f'{scene}.world'
        copied=run/'source-snapshot'/world.relative_to(ROOT)
        assert digest(copied)==digest(world)
        (evidence/'bootstrap.json').write_text(json.dumps(dict(passed=True,
            evidence=str((boot/'result.json').relative_to(ROOT)),sha256=digest(boot/'result.json'),
            disarmed_state=state,native_pose=actual,source_stamp_difference_s=dt,
            sensor_attachment_audit=str((boot/'sensor-attachment-audit.json').relative_to(ROOT)),
            sensor_attachment_sha256=digest(boot/'sensor-attachment-audit.json'),
            world_to_navigation=dict(translation=translation,yaw=yaw),native_sensors=sensors,
            world_geometry_witnesses=b['world_geometry_witnesses']),indent=2)+'\n')
        (evidence/'navigation.json').write_text(json.dumps(dict(passed=True,
            evidence=str((run/'result.json').relative_to(ROOT)),result_sha256=digest(run/'result.json'),
            independent_evaluation=str((run/'independent-evaluation.json').relative_to(ROOT)),
            evaluation_sha256=digest(run/'independent-evaluation.json'),
            sensor_attachment_audit=str((run/'sensor-attachment-audit.json').relative_to(ROOT)),
            sensor_attachment_sha256=digest(run/'sensor-attachment-audit.json'),
            mission=check['missions'],scope='scene qualification; final current FUEL build qualification recorded separately'),indent=2)+'\n')
        entry=copy.deepcopy(yaml.safe_load((W/'scenes/gcopter_3d_lane_v1.yaml').read_text()))
        entry.update(id=scene,name='FUEL bounded '+('room with partition' if 'room' in scene else 'warehouse with racks'),
            category='indoor/office' if 'room' in scene else 'indoor/warehouse')
        entry['world'].update(path=str(world.relative_to(W)),sha256=digest(world))
        entry['provenance'].update(revision=scene,evidence=[dict(package='easyfly_gazebo_worlds',path='licenses/PROJECT-LICENSE'),
            dict(package='easyfly_gazebo_worlds',path=f'evidence/{scene}/generation.json')],
            note='Project static geometry present before the first scan. Native disarmed alignment and real 3m physical lidar qualified; default sensor unchanged.')
        entry['navigation'].update(world_to_navigation=dict(translation=translation,yaw=yaw),
            recommended_region=dict(min=[-1.4,-1.8,1.4],max=[6.4,1.8,2.4]))
        entry['runtime']['entrypoint']['path']='models/easyfly_iris_fuel/easyfly_iris_fuel.sdf'
        entry['runtime']['asset_sets'].append('easyfly_fuel_sensor_v1')
        entry['runtime']['notes']=['Explicit short-range native physical ray benchmark variant, horizontal omni CSV FOV; no post-sensor cropping or fabricated free space.']
        entry['verification']['evidence_refs']=[dict(package='easyfly_gazebo_worlds',path=f'evidence/{scene}/{n}.json') for n in ['generation','bootstrap','navigation']]
        (W/'scenes'/f'{scene}.yaml').write_text(yaml.safe_dump(entry,sort_keys=False))


if __name__=='__main__':main()

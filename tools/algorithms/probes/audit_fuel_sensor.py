#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Native physical laser origin must follow the actual aircraft, never inferred from TF."""
import argparse
import csv
import collections
import struct
import hashlib
import json
from pathlib import Path
import numpy as np


def read(path):
    with path.open() as stream:
        return list(csv.DictReader(line for line in stream if not line.startswith('#')))


def audit(directory):
    failures=[]
    result=json.loads((directory/'result.json').read_text())
    body=read(directory/'truth.csv');sensor=read(directory/'sensor-truth.csv')
    assert body and sensor
    times=np.array([int(r['stamp_ns']) for r in body]);deltas=[];errors=[]
    for r in sensor:
        t=int(r['stamp_ns']);i=int(np.searchsorted(times,t))
        candidates=[j for j in [i-1,i] if 0<=j<len(times)]
        j=min(candidates,key=lambda k:abs(times[k]-t));dt=abs(times[j]-t)/1e9
        deltas.append(dt)
        if dt>.03:continue
        b=body[j];p=np.array([float(b[k]) for k in ['x','y','z']])
        x,y,z,w=[float(b[k]) for k in ['qx','qy','qz','qw']]
        offset=.13*np.array([2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)])
        actual=np.array([float(r[k]) for k in ['x','y','z']])
        errors.append(float(np.linalg.norm(actual-p-offset)))

    # LaserScanStamped has both Gazebo legacy and custom-shape publishers; its
    # range metadata is not the custom physics-ray contract. Check actual cached
    # physics ray endpoints and max fields published by the Livox adapter instead.
    ray_files=list(directory.glob('rays-*.bin'))
    if not ray_files:failures.append('native physics rays missing')
    for ray in ray_files:
        cloud=np.frombuffer(ray.read_bytes(),dtype=np.dtype([('xyz','<f4',(3,)),('type','u1'),('pad','u1',(3,)),('min','<f4'),('max','<f4')]))
        if not np.all(cloud['max']==3.) or not np.all(np.linalg.norm(cloud['xyz'],axis=1)<=3.011):
            failures.append('actual cached physical rays exceed 3m')
    if not result.get('native_sensor_joint_observed'):failures.append('native joint not observed')
    if result.get('sensor_truth_returncode')!=0:failures.append('native sensor observer failed')
    if len(errors)<20 or len(errors)<.95*len(sensor):failures.append('native model/sensor source-time overlap insufficient')
    if errors and max(errors)>.03:failures.append('native physical sensor detached or wrong extrinsics')
    console=(directory/'console.log').read_text(errors='replace')
    if 'LoadJoint Failed' in console or "Couldn't Find Child Link" in console:failures.append('native joint construction error')
    output=dict(passed=not failures,failures=failures,sensor_samples=len(sensor),matched=len(errors),
        legacy_scan_range_metadata=dict(collections.Counter(str((r['range_min'],r['range_max'])) for r in sensor)),
        physical_range_from_actual_cached_rays=True,
        max_source_stamp_difference_s=max(deltas),max_physical_origin_error_m=max(errors) if errors else None,
        expected_body_to_sensor_m=[0.,0.,.13],attachment_limit_m=.03,
        sensor_csv_sha256=hashlib.sha256((directory/'sensor-truth.csv').read_bytes()).hexdigest(),
        read_only_native_truth_not_algorithm_input=True)
    (directory/'sensor-attachment-audit.json').write_text(json.dumps(output,indent=2)+'\n')
    return output


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path)
    result=audit(p.parse_args().directory);print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)

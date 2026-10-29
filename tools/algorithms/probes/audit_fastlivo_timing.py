#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Read original CDR headers, verify fused image/IMU bracketing and physical extrinsics."""
import argparse
import bisect
import json
from pathlib import Path
import sqlite3
import statistics
import struct
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT=Path(__file__).resolve().parents[3]


def header(data):
    representation=int.from_bytes(data[:2],'big')
    if representation not in (0,1):raise ValueError('unsupported non-CDR sensor encoding')
    sec,nanosec=struct.unpack_from('<iI' if representation==1 else '>iI',data,4)
    if sec<0 or nanosec>=10**9:raise ValueError('invalid original source time')
    return sec*10**9+nanosec


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('flight',type=Path);a=p.parse_args()
    times={};valid=True;decode_checks={}
    for db in sorted((a.flight/'input-bag').glob('*.db3')):
        conn=sqlite3.connect('file:'+str(db.resolve())+'?mode=ro',uri=True)
        topics={r[0]:(r[1],r[2]) for r in conn.execute('select id,name,type from topics')}
        for tid,(name,kind) in topics.items():
            if kind not in ('sensor_msgs/msg/Imu','sensor_msgs/msg/Image','livox_ros_driver2/msg/CustomMsg'):continue
            sample=conn.execute('select data from messages where topic_id=? order by timestamp,id limit 1',(tid,)).fetchone()
            if sample:
                m=deserialize_message(sample[0],get_message(kind))
                decode_checks[name]=header(sample[0])==m.header.stamp.sec*10**9+m.header.stamp.nanosec
            stamps=[header(row[0]) for row in conn.execute('select substr(data,1,12) from messages where topic_id=? order by timestamp,id',(tid,))]
            times.setdefault(name,[]).extend(stamps)
        conn.close()
    timing={}
    for name,stamps in times.items():
        gaps=[(b-a)/1e6 for a,b in zip(stamps,stamps[1:])]
        valid=valid and bool(gaps) and all(v>0 for v in gaps)
        timing[name]=dict(count=len(stamps),source_span_s=(stamps[-1]-stamps[0])/1e9,
            gap_min_ms=min(gaps),gap_median_ms=statistics.median(gaps),gap_max_ms=max(gaps))
    camera=times['/fastlivo/input/camera'];imu=times['/fastlivo/input/imu']
    forwarded=set(imu)
    missing_imu=[t for t in times['/livox/imu'] if imu[0]<=t<=imu[-1] and t not in forwarded]
    poses=[json.loads(l) for l in (a.flight/'vio.jsonl').read_text().splitlines()]
    poses=[r['stamp_ns'] for r in poses if r['stream']=='pose']
    failures=[];brackets=[];camera_error=[]
    for t in poses:
        c=bisect.bisect_left(camera,t)
        delta=min(abs(camera[i]-t) for i in (c-1,c) if 0<=i<len(camera))
        i=bisect.bisect_left(imu,t)
        if delta>1 or not 0<i<len(imu) or imu[i]-imu[i-1]>20e6:
            failures.append(dict(stamp_ns=t,camera_error_ns=delta,imu_index=i))
        else:brackets.append((imu[i]-imu[i-1])/1e6)
        camera_error.append(delta)
    conf=yaml.safe_load((ROOT/'src/perception/p3_tracking/easyfly_fast_livo2_integration/config/gazebo.yaml').read_text())['/**']['ros__parameters']['extrin_calib']
    chain=yaml.safe_load((ROOT/'src/perception/p3_tracking/easyfly_openvins_integration/config/gazebo_mono/kalibr_imucam_chain.yaml').read_text().replace('%YAML:1.0',''))['cam0']['T_imu_cam']
    tic=np.array(chain);ril=np.array(conf['extrinsic_R']).reshape(3,3);til=np.array(conf['extrinsic_T'])
    rcl=tic[:3,:3].T@ril;tcl=tic[:3,:3].T@(til-tic[:3,3])
    rotation_error=float(np.max(np.abs(rcl-np.array(conf['Rcl']).reshape(3,3))))
    translation_error=float(np.max(np.abs(tcl-conf['Pcl'])))
    report=dict(passed=bool(valid and all(decode_checks.values()) and not failures and not missing_imu and len(poses)>=100 and rotation_error<1e-12 and translation_error<1e-12),
        cdr_decoder_checked_against_real_message=decode_checks,timing=timing,pose_count=len(poses),
        max_pose_camera_time_difference_ns=max(camera_error),max_admitted_pose_imu_bracket_ms=max(brackets) if brackets else None,
        missing_imu_inside_forwarded_window=missing_imu,
        failures=failures,extrinsic_rotation_error=rotation_error,extrinsic_translation_error_m=translation_error,
        method='original header time; no interpolation retiming, synthetic IMU or pixel modification')
    (a.flight/'input-timing-and-extrinsics.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));return 0 if report['passed'] else 1

if __name__=='__main__':raise SystemExit(main())

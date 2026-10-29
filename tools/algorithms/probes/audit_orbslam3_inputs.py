#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Verify original RGB-D message semantics, source pairing and physical calibration."""
import argparse
import hashlib
import json
from pathlib import Path
import sqlite3
import struct
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, CameraInfo


def main():
    p=argparse.ArgumentParser();p.add_argument('flight',type=Path);a=p.parse_args()
    database=a.flight/'rgbd-bag/rgbd-bag_0.db3'
    c=sqlite3.connect('file:'+str(database)+'?mode=ro',uri=True)
    topics=dict(c.execute('select name,id from topics'));records={};times={};errors=[]
    expected=[('/camera/color/image_raw',Image,'rgb8','camera_color_optical_frame'),
              ('/camera/color/camera_info',CameraInfo,None,'camera_color_optical_frame'),
              ('/camera/aligned_depth_to_color/depth/image_raw',Image,'32FC1','camera_depth_optical_frame'),
              ('/camera/aligned_depth_to_color/depth/camera_info',CameraInfo,None,'camera_depth_optical_frame')]
    for name,cls,encoding,frame in expected:
        if name not in topics:errors.append('missing '+name);continue
        tid=topics[name];stamps=[]
        for raw, in c.execute('select substr(data,1,12) from messages where topic_id=? order by id',(tid,)):
            sec,ns=struct.unpack_from('<iI',raw,4);stamps.append(sec*10**9+ns)
        if len(stamps)<100:errors.append('insufficient '+name);continue
        m=deserialize_message(c.execute('select data from messages where topic_id=? order by id limit 1',(tid,)).fetchone()[0],cls)
        assert stamps[0]==m.header.stamp.sec*10**9+m.header.stamp.nanosec
        checks=dict(width=m.width==640,height=m.height==480,frame=m.header.frame_id==frame,monotonic=bool((np.diff(stamps)>0).all()))
        fields=dict(count=len(stamps),frame=m.header.frame_id,width=m.width,height=m.height,gap_max_ms=float(np.diff(stamps).max()/1e6))
        if cls is Image:
            checks.update(encoding=m.encoding==encoding,endian=m.is_bigendian==0,stride=m.step==640*(3 if encoding=='rgb8' else 4))
            fields['encoding']=m.encoding
            if m.encoding=='32FC1':
                pixels=np.frombuffer(bytes(m.data),dtype='<f4');fields['finite_positive_fraction']=float((np.isfinite(pixels)&(pixels>0)).mean())
        else:
            checks['calibration']=bool(np.allclose(m.k,[337.3570508552852,0,320.5,0,337.3570508552852,240.5,0,0,1],rtol=0,atol=1e-6) and np.allclose(m.d,0,rtol=0,atol=1e-9))
            fields['K']=list(m.k)
        records[name]=dict(**fields,checks=checks)
        errors.extend(name+': '+k for k,v in checks.items() if not v)
        times[name]=np.array(stamps,dtype=np.int64)
    if len(times)==4:
        rgb=times[expected[0][0]];depth=times[expected[2][0]]
        paired=bool(np.array_equal(rgb,depth))
        if not paired:errors.append('RGB and depth source stamps differ')
    else:paired=False
    sdf=Path('src/simulation/easyfly_px4_sitl/models/easyfly_iris/easyfly_iris.sdf')
    text=sdf.read_text();coincident=text.count('<pose>0.2825 0 0.22 0 0 0</pose>')==2
    if not coincident:errors.append('native RGB/depth physical mounts differ')
    result=dict(passed=not errors,errors=errors,topics=records,identical_rgb_depth_source_stamps=paired,native_coincident_mounts=coincident,native_model_sha256=hashlib.sha256(sdf.read_bytes()).hexdigest(),method='original source headers and unmodified native messages; no approximate image registration or generated depth')
    (a.flight/'rgbd-input-audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())

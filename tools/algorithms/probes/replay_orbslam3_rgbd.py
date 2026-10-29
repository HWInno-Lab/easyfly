#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Run real ORB-SLAM3 on original RGB-D/clock data, optionally reuse a saved atlas."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from diagnostic_msgs.msg import DiagnosticArray
from easyfly_msgs.msg import LocalizationStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image, Imu
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from audit_fastlivo_bag import fingerprint


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--flight',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--load-atlas',type=Path)
    p.add_argument('--settings-template',type=Path)
    p.add_argument('--debug-shutdown',action='store_true',help='diagnostic gdb target; never an acceptance pass')
    p.add_argument('--debug-stop-after-loop',action='store_true',help='diagnostic only, stop owned player after actual loop')
    p.add_argument('--sensor-mode',choices=['rgbd','rgbd_imu'],default='rgbd')
    p.add_argument('--rate',type=float,default=.5)
    a=p.parse_args()
    if a.debug_stop_after_loop and not a.debug_shutdown:raise ValueError('early stop is diagnostic only')
    if not .1<=a.rate<=1:raise ValueError('replay rate must be .1..1; no restamping')
    if not json.loads((a.flight/'result.json').read_text())['passed']:
        raise ValueError('source flight/recorder did not complete successfully')
    if not json.loads((a.flight/'rgbd-input-audit.json').read_text())['passed']:
        raise ValueError('original source RGB-D input audit did not pass')
    if a.sensor_mode=='rgbd_imu' and not json.loads((a.flight/'imu-input-audit.json').read_text())['passed']:
        raise ValueError('original IMU source audit did not pass')
    a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'replay-source.py').write_bytes(Path(__file__).read_bytes())
    runtime=a.output/'runtime';runtime.mkdir()
    from easyfly_fastdds_compat.runtime import activate
    activate(openvins=True)
    os.environ.update(ROS_DOMAIN_ID='229',ROS_LOCALHOST_ONLY='1')
    env=dict(os.environ)
    rclpy.init()
    node=rclpy.create_node('orbslam3_replay_observer',parameter_overrides=[Parameter('use_sim_time',value=True)])
    log=(a.output/'vio.jsonl').open('x');hashlog=(a.output/'input-fingerprints.jsonl').open('x')
    counts={}
    resources=[];last_resource=0.
    loop_seen=[None];complete_playback=True
    def record(kind,data):
        counts[kind]=counts.get(kind,0)+1
        log.write(json.dumps(dict(stream=kind,receipt=time.monotonic(),**data))+'\n')
    def pose(m,raw=False):
        pose=m.pose if raw else m.pose.pose;q=pose.orientation;x=pose.position
        record('raw_body_pose' if raw else 'local_body_pose',dict(stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,position=[x.x,x.y,x.z],quaternion=[q.x,q.y,q.z,q.w],frame=m.header.frame_id))
    def diagnostic(m,kind):
        values={v.key:v.value for v in m.status[0].values}
        stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        if kind=='atlas_event' and int(values.get('loop_closures',0))>0 and loop_seen[0] is None:loop_seen[0]=time.monotonic()
        if kind=='visual_tracking':record(kind,dict(stamp_ns=stamp,tracked_features=int(values['tracked_features']),initialized=values['initialized']=='1'))
        else:record(kind,dict(stamp_ns=stamp,values=values))
    def status(m):record('status',dict(stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,state=m.state,reason=m.reason,generation=m.generation,pose_valid=m.pose_valid,covariance_available=m.covariance_available))
    node.create_subscription(PoseWithCovarianceStamped,'/localization/orbslam3/pose',pose,128)
    node.create_subscription(PoseStamped,'/orbslam3/raw/body_pose',lambda m:pose(m,True),128)
    node.create_subscription(LocalizationStatus,'/localization/orbslam3/status',status,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    for kind,topic in [('visual_tracking','/orbslam3/raw/visual_tracking'),('map_lifecycle','/orbslam3/raw/map_status'),('atlas_event','/orbslam3/raw/atlas_events')]:
        node.create_subscription(DiagnosticArray,topic,lambda m,k=kind:diagnostic(m,k),128)
    for kind,topic in [('rgb','/camera/color/image_raw'),('depth','/camera/aligned_depth_to_color/depth/image_raw'),('forwarded_rgb','/orbslam3/input/rgb'),('forwarded_depth','/orbslam3/input/depth')]:
        def image(m,k=kind):
            stamp,h=fingerprint(m)
            hashlog.write(json.dumps(dict(kind=k,stamp_ns=stamp,sha256=h,encoding=m.encoding,receipt=time.monotonic()))+'\n')
        node.create_subscription(Image,topic,image,QoSProfile(depth=64,reliability=ReliabilityPolicy.RELIABLE))
    if a.sensor_mode=='rgbd_imu':
        for kind,topic in [('imu','/livox/imu'),('forwarded_imu','/orbslam3/input/imu')]:
            def imu(m,k=kind):
                stamp,h=fingerprint(m)
                hashlog.write(json.dumps(dict(kind=k,stamp_ns=stamp,sha256=h,receipt=time.monotonic()))+'\n')
            node.create_subscription(Imu,topic,imu,QoSProfile(depth=2000,reliability=ReliabilityPolicy.RELIABLE))
    console=(a.output/'console.log').open('x');playerlog=(a.output/'player.log').open('x')
    command=['ros2','launch','easyfly_orbslam3_integration','orbslam3.launch.py','runtime_directory:='+str(runtime.resolve()),'enable_navigation:=false']
    command+=['sensor_mode:='+a.sensor_mode]
    if (a.sensor_mode=='rgbd_imu' or a.debug_shutdown) and not a.settings_template:
        from ament_index_python.packages import get_package_share_directory
        a.settings_template=Path(get_package_share_directory('easyfly_orbslam3_integration'))/('config/gazebo_rgbd_imu.yaml' if a.sensor_mode=='rgbd_imu' else 'config/gazebo_rgbd.yaml')
    if a.settings_template:command+=['settings_template:='+str(a.settings_template.resolve(strict=True))]
    if a.load_atlas:command+=['load_atlas:='+str(a.load_atlas.resolve())]
    else:command+=['save_atlas:='+str((runtime/'atlas').resolve())]
    if a.debug_shutdown:
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        executable=Path(get_package_prefix('easyfly_orbslam3_integration'))/'lib/easyfly_orbslam3_integration/orbslam3_rgbd'
        vocabulary=Path(get_package_share_directory('easyfly_orbslam3_core'))/'vocabulary/ORBvoc.txt'
        params=dict(use_sim_time='true',sensor_mode=a.sensor_mode,runtime_directory=str(runtime.resolve()),
                    vocabulary=str(vocabulary),settings_template=str(a.settings_template.resolve()))
        if a.load_atlas:params['load_atlas']=str(a.load_atlas.resolve())
        else:params['save_atlas']=str((runtime/'atlas').resolve())
        command=['gdb','-q','-batch','-ex','set pagination off','-ex','handle SIGINT nostop pass',
                 '-ex','run','-ex','thread apply all bt','--args',str(executable),'--ros-args']
        for key,value in params.items():command+=['-p',key+':='+value]
    launch=subprocess.Popen(command,env=env,stdout=console,stderr=subprocess.STDOUT,start_new_session=True)
    player=None;forced=False;error=None;provenance=[]
    try:
        deadline=time.monotonic()+90
        while node.count_publishers('/localization/orbslam3/status')!=1:
            if launch.poll() is not None or time.monotonic()>deadline:raise RuntimeError('estimator startup failed')
            rclpy.spin_once(node,timeout_sec=.1)
        for proc in Path('/proc').iterdir():
            if not proc.name.isdigit():continue
            try:
                exe=(proc/'exe').resolve(strict=True)
                if exe.name!='orbslam3_rgbd' or b'ROS_DOMAIN_ID=229' not in (proc/'environ').read_bytes().split(b'\0'):continue
                libs=sorted(set(line.split()[-1] for line in (proc/'maps').read_text().splitlines() if any(k in line for k in ['libeasyfly_orbslam3','libpangolin'])))
                provenance.append(dict(pid=int(proc.name),start_ticks=(proc/'stat').read_text().rsplit(')',1)[1].split()[19],executable=str(exe),executable_sha256=hashlib.sha256(exe.read_bytes()).hexdigest(),libraries_sha256={x:hashlib.sha256(Path(x).read_bytes()).hexdigest() for x in libs}))
            except (OSError,ProcessLookupError):pass
        assert len(provenance)==1
        imu_topics=['/livox/imu'] if a.sensor_mode=='rgbd_imu' else []
        player=subprocess.Popen(['ros2','bag','play',str((a.flight/'rgbd-bag').resolve()),'--rate',str(a.rate),'--topics','/clock','/camera/color/image_raw','/camera/color/camera_info','/camera/aligned_depth_to_color/depth/image_raw','/camera/aligned_depth_to_color/depth/camera_info']+imu_topics,env=env,stdout=playerlog,stderr=subprocess.STDOUT)
        deadline=time.monotonic()+900
        while player.poll() is None:
            if launch.poll() is not None or time.monotonic()>deadline:raise RuntimeError('estimator exited or playback deadline exceeded')
            rclpy.spin_once(node,timeout_sec=.02)
            if a.debug_stop_after_loop and loop_seen[0] is not None and time.monotonic()-loop_seen[0]>2.:
                complete_playback=False;player.send_signal(signal.SIGINT);player.wait(timeout=15);break
            if time.monotonic()-last_resource>=1.:
                last_resource=time.monotonic()
                proc=Path('/proc')/str(provenance[0]['pid'])
                try:
                    fields=(proc/'stat').read_text().rsplit(')',1)[1].split()
                    if fields[19]!=provenance[0]['start_ticks']:raise RuntimeError('owned estimator PID identity changed')
                    resources.append(dict(steady=last_resource,source_ns=node.get_clock().now().nanoseconds,
                                          cpu_seconds=(int(fields[11])+int(fields[12]))/os.sysconf('SC_CLK_TCK'),
                                          rss_bytes=int(fields[21])*os.sysconf('SC_PAGE_SIZE')))
                except FileNotFoundError:raise RuntimeError('owned estimator disappeared')
        if player.returncode:raise RuntimeError('original RGB-D playback failed')
        deadline=time.monotonic()+1
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
    except Exception as exc:error=repr(exc)
    finally:
        for proc in [player,launch]:
            if proc is not None and proc.poll() is None:
                if proc is launch and a.debug_shutdown and provenance:
                    target=Path('/proc')/str(provenance[0]['pid'])
                    if target.exists() and (target/'stat').read_text().rsplit(')',1)[1].split()[19]==provenance[0]['start_ticks']:
                        os.kill(provenance[0]['pid'],signal.SIGINT)
                    else:proc.send_signal(signal.SIGINT)
                else:proc.send_signal(signal.SIGINT)
                try:proc.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    forced=True
                    os.killpg(launch.pid,signal.SIGTERM) if proc is launch else proc.terminate()
                    proc.wait(timeout=5)
        node.destroy_node();rclpy.try_shutdown()
        for f in [log,hashlog,console,playerlog]:f.close()
    (a.output/'truth.csv').symlink_to((a.flight/'truth.csv').resolve())
    forced=forced or 'escalating to' in (a.output/'console.log').read_text()
    child_errors=[line for line in (a.output/'console.log').read_text().splitlines() if 'process has died' in line or 'Traceback (most recent call last)' in line]
    if child_errors and error is None:error='child process failed: '+child_errors[0]
    result=dict(backend='orbslam3',sensor_mode=a.sensor_mode,diagnostic_debugger=a.debug_shutdown,complete_playback=complete_playback,scope='offline_replay_not_flight',passed=not a.debug_shutdown and complete_playback and error is None and not forced and player is not None and player.returncode==launch.returncode==0,error=error,child_errors=child_errors,forced=forced,launch_returncode=launch.returncode,player_returncode=player.returncode if player else None,counts=counts,command=command,load_atlas=str(a.load_atlas) if a.load_atlas else None,original_flight=str(a.flight.resolve()),runtime_provenance=provenance,hardware=False,playback_wall_rate=a.rate,script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),input_audit_qos=dict(depth=64,reliability='RELIABLE'))
    (a.output/'resources.json').write_text(json.dumps(resources,indent=2)+'\n')
    (a.output/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())

#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Explicit scene bootstrap or bounded mission; never executed by colcon test."""
import argparse
import hashlib
import importlib.machinery
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix
from easyfly_msgs.action import ExecuteMission
from easyfly_px4_sitl.scene_environment import installed_catalog
from sensor_msgs.msg import Image, Imu, PointCloud2
from livox_ros_driver2.msg import CustomMsg
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import yaml


def load_navigation_probe():
    installed=Path(get_package_prefix('easyfly_sim_launcher'))/'lib/easyfly_sim_launcher/navigation_sitl'
    source=Path(__file__).with_name('navigation_sitl.py')
    path=source if source.exists() else installed
    loader=importlib.machinery.SourceFileLoader('_easyfly_navigation_probe',str(path))
    spec=importlib.util.spec_from_loader(loader.name,loader)
    module=importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


navigation=load_navigation_probe()


class SceneProbe(navigation.Probe):
    def __init__(self,args):
        super().__init__(args)
        self.catalog,_=installed_catalog(px4_dir=Path(self.report['staging']['staging']))
        if args.scene not in self.catalog.scenes:
            raise ValueError('unknown scene: '+args.scene)
        self.scene=self.catalog.scenes[args.scene]
        audit=self.catalog.validate(args.scene).as_dict()
        if not audit['resources_complete'] or not audit['licenses_complete']:
            raise ValueError('scene resources/licenses incomplete: '+json.dumps(audit['issues']))
        self.report.update(scene=args.scene,scene_audit=audit,bootstrap=args.bootstrap,
            world_sha256=self.scene['world']['sha256'],sensors={},alignment_samples=[],
            scene_probe_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        if not args.bootstrap:
            mission=args.mission.resolve(strict=True)
            allowed=[self.catalog.resolve(loc) for loc in self.scene['recommended_missions']]
            if mission not in allowed:
                raise ValueError('flight mission must be the registered scene reference mission')
            data=yaml.safe_load(mission.read_text())
            targets=[[step['x'],step['y'],step['z']] for step in data['steps'] if step['type']=='navigate']
            if data['frame_id']!='odom' or data['failure_policy']!='abort' or targets!=[[1.5,0,1.5],[0,0,1.5]]:
                raise ValueError('this probe supports only the reviewed bounded aisle out-and-back mission')
        for topic,kind in [('/camera/color/image_raw',Image),
                           ('/camera/aligned_depth_to_color/depth/image_raw',Image),
                           ('/livox/imu',Imu),('/livox/lidar',CustomMsg)]:
            self.node.create_subscription(kind,topic,lambda msg,name=topic:self.sensor(name,msg),qos_profile_sensor_data)
        self.ground_cloud=None
        self.node.create_subscription(PointCloud2,'/pointcloud_filtered',
            lambda msg:setattr(self,'ground_cloud',msg),qos_profile_sensor_data)

    def sensor(self,name,msg):
        now=time.monotonic()
        source=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        if source<=0:
            return
        entry=self.report['sensors'].setdefault(name,dict(count=0,first_source_ns=source))
        if entry['count'] and source<=entry['last_source_ns']:
            entry['non_increasing_stamps']=entry.get('non_increasing_stamps',0)+1
        entry.update(count=entry['count']+1,last_source_ns=source,receipt=now,frame=msg.header.frame_id)
        if isinstance(msg,Image):
            entry.update(width=msg.width,height=msg.height,encoding=msg.encoding)
            if 'depth' in name and entry.get('finite_positive_pixels',0)<100:
                dtype={'32FC1':np.dtype('>f4' if msg.is_bigendian else '<f4'),
                       '16UC1':np.dtype('>u2' if msg.is_bigendian else '<u2')}.get(msg.encoding)
                if dtype is not None:
                    data=np.frombuffer(bytes(msg.data),dtype=dtype).reshape(msg.height,-1)[:,:msg.width]
                    values=data.astype(float)*(0.001 if msg.encoding=='16UC1' else 1)
                    finite=values[np.isfinite(values)&(values>0)]
                    entry.update(finite_positive_pixels=int(finite.size),
                                 min_m=float(finite.min()) if finite.size else None,
                                 max_m=float(finite.max()) if finite.size else None)
                    np.save(self.directory/'depth-first.npy',values)
            elif name=='/camera/color/image_raw' and 'sample_saved' not in entry:
                np.save(self.directory/'color-first.npy',np.frombuffer(bytes(msg.data),dtype=np.uint8))
                entry['sample_saved']=True
        elif isinstance(msg,CustomMsg):
            entry['points']=len(msg.points)
            if 'sample_xyz' not in entry and len(msg.points)>100:
                sample=msg.points[::max(1,len(msg.points)//256)]
                entry['sample_xyz']=[[p.x,p.y,p.z] for p in sample]
        elif isinstance(msg,Imu):
            entry['finite']=all(math.isfinite(v) for v in [msg.angular_velocity.x,msg.angular_velocity.y,
                msg.angular_velocity.z,msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])

    def launch_command(self):
        command=super().launch_command()
        if getattr(self.args,'ego_debugger',None):
            if not self.args.bootstrap:
                raise ValueError('debugger is only allowed for disarmed inspection')
            script=self.args.ego_debugger.resolve(strict=True)
            command[2:2]=['--launch-prefix',f'gdb -q -batch --return-child-result -x {script} --args',
                          '--launch-prefix-filter','.*ego_planner_node.*']
            self.report['ego_debugger']=str(script)
        if self.args.bootstrap:
            sets=dict.fromkeys(self.scene['asset_sets']+self.scene['runtime']['asset_sets'])
            roots=list(dict.fromkeys(str(self.catalog.resolve(self.catalog.assets[k]['model_root'])) for k in sets))
            spawn=self.scene['spawn']
            if spawn['position']!=[0.0,0.0,0.83] or spawn['yaw']!=0.0:
                raise ValueError('bootstrap harness currently requires the unchanged default spawn')
            command.extend(['world:='+str(self.catalog.resolve(self.scene['world'])),
                            'debug_model_path:='+':'.join(roots),
                            'ground_model_frame:='+self.scene['navigation'].get('ground_model_frame','odom')])
            self.report['bootstrap_only_raw_debug_inputs']=True
        else:
            command.extend(['scene:='+self.args.scene,'scene_profile:=navigation_dev'])
        return command

    def send(self,*args,**kwargs):
        if self.args.bootstrap:
            raise RuntimeError('bootstrap is disarmed inspection; flight goals forbidden')
        return super().send(*args,**kwargs)

    def startup_ready(self):
        if self.args.bootstrap:
            # Disarmed resource/sensor inspection is separate from navigation
            # readiness. It records a missing map rather than requiring flight
            # admission before the scene can be inspected.
            return (self.ready(False) and len(self.report['sensors'])==4 and
                    'ego_planner_node' in self.node.get_node_names())
        return super().startup_ready()

    def inspect(self):
        expected={
            'libeasyfly_semantic_map_nodes.so':Path(get_package_prefix('easyfly_semantic_map'))/'lib/libeasyfly_semantic_map_nodes.so',
            'liblivox_laser_simulation.so':Path(get_package_prefix('easyfly_gazebo_plugins'))/'lib/liblivox_laser_simulation.so',
            'libplan_env.so':Path(get_package_prefix('plan_env'))/'lib/libplan_env.so',
        }
        loaded={}
        for pid in self.owned:
            maps=Path(f'/proc/{pid}/maps')
            if not maps.exists():
                continue
            for line in maps.read_text().splitlines():
                path=Path(line.split()[-1])
                if path.name in expected:
                    if path.resolve()!=expected[path.name].resolve():
                        raise RuntimeError('loaded library differs from selected overlay: '+str(path))
                    loaded[path.name]=dict(pid=pid,path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        self.report['loaded_libraries']=loaded
        if set(loaded)!=set(expected):
            raise RuntimeError('selected map and lidar libraries not confirmed in owned processes')
        # Let the unchanged ground confirmation and map pipeline accumulate
        # observations before recording readiness; inspection still sends no goals.
        deadline=time.monotonic()+30
        while time.monotonic()<deadline and not (
                self.counts.get('/occupancy_grid',0)>=5 and
                self.counts.get('/pointcloud_nonground',0)>=5):
            navigation.rclpy.spin_once(self.node,timeout_sec=0.1)
        if self.args.bootstrap and self.ground_cloud is not None:
            msg=self.ground_cloud
            fields={field.name:field for field in msg.fields}
            if all(name in fields and fields[name].datatype==7 for name in ['x','y','z']):
                endian='>' if msg.is_bigendian else '<'
                xyz=np.stack([np.ndarray((msg.height,msg.width),dtype=endian+'f4',
                    buffer=msg.data,offset=fields[name].offset,
                    strides=(msg.row_step,msg.point_step)).reshape(-1) for name in ['x','y','z']],axis=1)
                np.save(self.directory/'ground-cloud.npy',xyz)
                self.report['ground_cloud_snapshot']=dict(points=len(xyz),frame=msg.header.frame_id,
                    source_ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,state=self.state())
        def streams_ready():
            values=self.report['sensors']
            return len(values)==4 and all(v['count']>=5 and time.monotonic()-v['receipt']<1.0
                and v['last_source_ns']>v['first_source_ns']>0 for v in values.values())
        self.wait(streams_ready,30)
        depth=self.report['sensors']['/camera/aligned_depth_to_color/depth/image_raw']
        lidar=self.report['sensors']['/livox/lidar']
        if lidar.get('non_increasing_stamps',0):
            raise RuntimeError('lidar measurement stamps repeat or roll back')
        if depth.get('finite_positive_pixels',0)<100 or lidar.get('points',0)<100:
            raise RuntimeError('real finite depth/lidar geometry not observed')
        if not self.report['sensors']['/livox/imu'].get('finite'):
            raise RuntimeError('finite lidar IMU observations missing')
        # PX4 pose-valid can precede estimator convergence after startup. Do
        # not calibrate from a disarmed estimate that still reports motion.
        settled_since=[None]
        def stationary_disarmed():
            state=self.state()
            stationary=(self.ready(False) and state is not None and
                        math.sqrt(sum(v*v for v in state['velocity']))<=0.05)
            if not stationary:
                settled_since[0]=None
                return False
            now=time.monotonic()
            if settled_since[0] is None:
                settled_since[0]=now
            return now-settled_since[0]>=3.0
        self.wait(stationary_disarmed,60)
        self.report['settled_preflight_state']=self.state()
        samples=[]
        for index in range(3):
            before=self.state()
            result=subprocess.run(['gz','model','-m','easyfly_iris','-p'],
                env=self.environment,capture_output=True,text=True,timeout=8)
            (self.directory/f'gazebo-pose-{index}.txt').write_text(result.stdout+result.stderr)
            if result.returncode!=0:
                raise RuntimeError('private Gazebo model pose query failed')
            lines=[line.strip() for line in result.stdout.splitlines() if line.strip()]
            pose=[float(v) for v in lines[-1].split()]
            if len(pose)!=6 or not all(math.isfinite(v) for v in pose):
                raise RuntimeError('unexpected Gazebo model pose output')
            self.wait(lambda:self.ready(False),3)
            state=self.latest['/uav_state']
            yaw=math.atan2(math.sin(state.yaw-pose[5]),math.cos(state.yaw-pose[5]))
            c,s=math.cos(yaw),math.sin(yaw)
            rotated=[c*pose[0]-s*pose[1],s*pose[0]+c*pose[1],pose[2]]
            translation=[a-b for a,b in zip([state.position_x,state.position_y,state.position_z],rotated)]
            samples.append(dict(world_pose=pose,odom_position=[state.position_x,state.position_y,state.position_z],
                                yaw=yaw,translation=translation,state=self.state(),state_before=before))
            until=time.monotonic()+0.5
            self.wait(lambda:time.monotonic()>=until,3)
        translation=np.mean([s['translation'] for s in samples],axis=0).tolist()
        yaw=math.atan2(sum(math.sin(s['yaw']) for s in samples),sum(math.cos(s['yaw']) for s in samples))
        self.report['alignment_samples']=samples
        if max(math.dist(s['translation'],translation) for s in samples)>0.15 or max(
                abs(math.atan2(math.sin(s['yaw']-yaw),math.cos(s['yaw']-yaw))) for s in samples)>0.1:
            raise RuntimeError('world/odom stationary alignment is unstable')
        self.report.update(alignment_samples=samples,world_to_navigation=dict(translation=translation,yaw=yaw),
                           loaded_and_sensors_observed=True,
                           mapping_ready=(self.counts.get('/occupancy_grid',0)>=3 and
                                          self.counts.get('/pointcloud_nonground',0)>=3))

    def run(self):
        self.start()
        self.inspect()
        if self.args.bootstrap:
            self.report['final_disarmed_state']=self.state()
            self.report['inspection_only']=True
            return
        registered=self.scene['navigation']['world_to_navigation']
        measured=self.report['world_to_navigation']
        if registered is None or math.dist(registered['translation'],measured['translation'])>0.2 or abs(
                math.atan2(math.sin(registered['yaw']-measured['yaw']),
                           math.cos(registered['yaw']-measured['yaw'])))>0.1:
            raise RuntimeError('current alignment differs from scene reference; no flight goal sent')
        mission=self.args.mission.resolve(strict=True)
        self.report['mission_sha256']=hashlib.sha256(mission.read_bytes()).hexdigest()
        action=self.send('/execute_mission',ExecuteMission,ExecuteMission.Goal(mission_file=str(mission)))
        self.finish(action,seconds=300)
        self.wait(lambda:self.ready(False),15)
        self.report['final_disarmed_state']=self.state()
        if len(self.navigation_goals)!=2 or not navigation.sessions_closed(self.report['sessions'],self.navigation_goals):
            raise RuntimeError('two genuine navigation UUIDs with completed sessions required')
        entries=self.report['action_status']['/navigate']
        terminals=sorted([entries[k]['first_terminal'] for k in self.navigation_goals
                          if entries[k]['status']==4 and 'first_terminal' in entries[k]],key=lambda v:v['steady'])
        if len(terminals)!=2:
            raise RuntimeError('successful child navigation terminals missing')
        errors=[math.dist(v['state']['position'],goal) for v,goal in zip(terminals,[[1.5,0.0,1.5],[0.0,0.0,1.5]])]
        if max(errors)>0.15:
            raise RuntimeError('actual navigation terminal arrival error: '+str(errors))
        for identity in sorted(self.navigation_goals):
            self.ownership('/navigation/ownership',identity)
        self.report.update(navigation_terminal_errors_m=errors,sessions_closed=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene',required=True)
    parser.add_argument('--bootstrap',action='store_true',help='Disarmed raw-world inspection only; no flight goals')
    parser.add_argument('--ego-debugger',type=Path,help='GDB command file, only for disarmed diagnostic runs')
    parser.add_argument('--mission',type=Path)
    parser.add_argument('--px4-dir',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--domain',type=int,required=True)
    parser.add_argument('--gazebo-port',type=int,required=True)
    args=parser.parse_args()
    if not 0<=args.domain<=232 or not 1024<=args.gazebo_port<=65535:
        parser.error('invalid explicit domain or port')
    if not args.bootstrap and args.mission is None:
        parser.error('flight mode requires an explicit --mission; bootstrap sends no goals')
    args.scenario='scene-inspection' if args.bootstrap else 'scene-mission'
    args.strategy='combined'
    probe=SceneProbe(args)
    try:
        probe.run()
    except (Exception,KeyboardInterrupt) as error:
        probe.report['error']=type(error).__name__+': '+str(error)
        print(probe.report['error'],flush=True)
    finally:
        probe.close()
    print(json.dumps({'passed':probe.report['passed'],'result':str(probe.directory/'result.json')}),flush=True)
    raise SystemExit(0 if probe.report['passed'] else 1)


if __name__=='__main__':
    main()

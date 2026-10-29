#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Serial FUEL Gazebo tasks with original source maps and read-only native truth."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from easyfly_msgs.action import Explore
from easyfly_msgs.msg import VoxelGrid
from easyfly_msgs.srv import ReserveNavigation
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

import gcopter_simulation as base
from process_cpu_trace import snapshot

ROOT=base.ROOT


class Probe(base.Probe):
    def __init__(self,args):
        self.sensor_truth=None;self.sensor_log=None
        self.raw=None;self.raw_received=0.;self.fuel_events=[];self.cpu=[];self.cpu_at=0.
        args.acceptance_limits_file = ROOT/'artifacts/new-algorithms/new5-fuel-20261023/acceptance-limits.json'
        super().__init__(args)
        self.report['acceptance_limits']=json.loads((ROOT/'artifacts/new-algorithms/new5-fuel-20261023/acceptance-limits.json').read_text())
        self.report['scene']=args.scene
        self.report['fuel_selected_prefixes']={p:get_package_prefix(p) for p in ['easyfly_fuel_native','easyfly_fuel_core','easyfly_fuel_exploration']}
        (self.directory/'motion').mkdir()
        (self.directory/'fuel').mkdir()
        self.node.create_subscription(VoxelGrid,'/planning/fuel_raw_grid',self.on_raw,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.fuel_log=(self.directory/'exploration-events.jsonl').open('x')
        self.node.create_subscription(String,'/exploration/status',self.on_event,100)
        sources=self.directory/'source-snapshot'
        for root in ['src/execution/e2_planning/easyfly_fuel_exploration',
                     'src/algorithms/planning/easyfly_fuel_core',
                     'src/third_party/algorithms/planning/fuel']:
            for source in (ROOT/root).rglob('*'):
                if source.is_file() and source.suffix!='.pyc' and not any(
                        part in ('__pycache__','.pytest_cache') for part in source.parts):
                    relative=source.relative_to(ROOT);target=sources/relative
                    target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(source.read_bytes())
                    self.report['source_hashes'][str(relative)]=hashlib.sha256(source.read_bytes()).hexdigest()
        for source in [Path(__file__),ROOT/'src/simulation/easyfly_gazebo_plugins/safe_transport/src/sensor_witness.cpp',
                       ROOT/'tools/algorithms/probes/fuel_sensor_witness/CMakeLists.txt',
                       ROOT/'src/simulation/easyfly_gazebo_worlds/worlds/benchmark/exploration'/f'{args.scene}.world',
                       ROOT/'src/simulation/easyfly_gazebo_models/easyfly/fuel_models/mid360_fuel/mid360.sdf',
                       ROOT/'src/simulation/easyfly_px4_sitl/models/easyfly_iris_fuel/easyfly_iris_fuel.sdf']:
            relative=source.relative_to(ROOT);target=sources/relative;target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes(source.read_bytes());self.report['source_hashes'][str(relative)]=hashlib.sha256(source.read_bytes()).hexdigest()

    def startup_ready(self):
        ready=super().startup_ready()
        if self.truth is not None and self.sensor_truth is None:
            self.sensor_log=(self.directory/'sensor-truth.log').open('x')
            executable=self.args.sensor_witness_executable.resolve(strict=True)
            self.sensor_truth=subprocess.Popen([str(executable),str(self.directory/'sensor-truth.csv')],
                stdout=self.sensor_log,stderr=subprocess.STDOUT,env=self.environment)
            self.report['sensor_witness_executable_sha256']=hashlib.sha256(executable.read_bytes()).hexdigest()
        return ready

    def start(self):
        super().start()
        text=(self.directory/'console.log').read_text(errors='replace')
        if 'LoadJoint Failed' in text or "Couldn't Find Child Link" in text:
            raise RuntimeError('native vehicle sensor joint failed; no flight allowed')
        info=subprocess.run(['gz','model','--model-name=easyfly_iris','--info'],
            env=self.environment,capture_output=True,text=True,timeout=10)
        (self.directory/'native-vehicle-info.txt').write_text(info.stdout+info.stderr)
        if info.returncode or 'mid360_joint' not in info.stdout or 'mid360_fuel::lidar_link' not in info.stdout:
            raise RuntimeError('native sensor joint identity not observed')
        self.report['native_sensor_joint_observed']=True

    def on_raw(self,msg):
        self.raw=msg;self.raw_received=time.monotonic()

    def on_event(self,msg):
        row=json.loads(msg.data);self.fuel_events.append(row)
        self.fuel_log.write(json.dumps(row)+'\n');self.fuel_log.flush()
        print(json.dumps(row),flush=True)

    def map_ready(self):
        return bool(self.raw and self.raw.valid and self.raw.header.frame_id=='odom' and
                    time.monotonic()-self.raw_received<=1. and
                    -.2e9<=self.node.get_clock().now().nanoseconds-base.stamp(self.raw.header.stamp)<=1e9)

    def wait(self,predicate,seconds):
        def observed():
            if self.process and self.process.poll() is None:
                for pid,data in base.navigation.children(self.process.pid).items():
                    if pid in self.owned and self.owned[pid]['start_ticks']!=data['start_ticks']:
                        raise RuntimeError('owned PID was reused')
                    if pid in self.owned and self.owned[pid]['executable']!=data['executable']:
                        self.report.setdefault('owned_exec_transitions',[]).append(dict(pid=pid,
                            previous=self.owned[pid],current=data,receipt=time.monotonic()))
                    self.owned[pid]=data
                if time.monotonic()-self.cpu_at>=1.:
                    self.cpu.append(snapshot(self.owned.items()));self.cpu_at=time.monotonic()
            return predicate()
        return super().wait(observed,seconds)

    def launch_command(self):
        args=['headless:=true','rviz:=false','detection_use_gpu:=false',
              'fuel_evidence_directory:='+str(self.directory/'fuel'),
              'gcopter_evidence_directory:='+str(self.directory/'motion')]
        if self.args.managed_profile:
            return ['ros2','launch','easyfly_sim_launcher','profile.launch.py','profile:=navigation_dev',
                'algorithms_json:='+json.dumps(dict(slam='fast_lio',strategy='gcopter',exploration='fuel',provider='cpu')),
                'scene:='+self.args.scene,*args]
        models=ROOT/'src/simulation/easyfly_gazebo_models'
        roots=[ROOT/'src/simulation/easyfly_px4_sitl/models',models/'easyfly/runtime_models',models/'easyfly/fuel_models']
        return ['ros2','launch','easyfly_px4_sitl','perception_sitl.launch.py','navigation_strategy:=gcopter',
            'exploration_backend:=fuel','world:='+str(ROOT/'src/simulation/easyfly_gazebo_worlds/worlds/benchmark/exploration'/f'{self.args.scene}.world'),
            'model_sdf:='+str(ROOT/'src/simulation/easyfly_px4_sitl/models/easyfly_iris_fuel/easyfly_iris_fuel.sdf'),
            'debug_model_path:='+':'.join(map(str,roots)),'debug_spawn_pose:=0 0 .1 0',
            'debug_heartbeat_rate_hz:=5','ground_model_frame:=map','grid_footprint_mode:=observed_cells',*args]

    def task(self,timeout=900.):
        return self.send('/explore',Explore,Explore.Goal(
            min_bound=Point(x=-1.4,y=-1.8,z=1.4),max_bound=Point(x=6.4,y=1.8,z=2.4),
            forbidden_boxes=[Point(x=4.0,y=.8,z=1.4),Point(x=4.4,y=1.2,z=2.4)] if self.args.scenario=='no-fly' else [],
            return_home=True,timeout_sec=timeout))

    def finish_task(self,active,expected=4):
        self.wait(active['result'].done,1800.)
        reply=active['result'].result();entry=active['entry']
        entry.update(status=reply.status,success=reply.result.success,message=reply.result.reason,
                     coverage=reply.result.coverage,completed_viewpoints=reply.result.completed_viewpoints,
                     terminal_state=self.state())
        print(json.dumps({k:v for k,v in entry.items() if k!='feedback'}),flush=True)
        if not base.navigation.terminal_matches(reply,expected):
            raise RuntimeError('unexpected FUEL task terminal: '+str(entry))
        terminals=[e for e in self.fuel_events if e['phase']=='TERMINAL']
        self.wait(lambda:any(e['phase']=='TERMINAL' for e in self.fuel_events),3.)
        self.ownership('/exploration/ownership',entry['goal_uuid'])
        if self.fuel_events[-1].get('fault_lock'):
            raise RuntimeError('task did not release its navigation lease')
        return entry

    def run(self):
        self.start()
        if not self.truth:
            raise RuntimeError('independent native truth missing')
        self.observe(3.)
        if self.args.scenario=='bootstrap':
            self.report['bootstrap_state']={**self.state(),'yaw':self.latest['/uav_state'].yaw}
            self.report['world_geometry_witnesses']=[self.model_witness(name,pos,require_collision=True)
                for name,pos in [('west',[-2.9,0,2]),('east',[7.3,0,2])]]
            self.report['final_disarmed_state']=self.state()
            return
        self.configure_simulated_battery();failure=None
        try:
            takeoff=self.send('/takeoff',base.navigation.Takeoff,
                base.navigation.Takeoff.Goal(target_altitude=1.5,climb_speed=.3))
            self.finish(takeoff,seconds=90);self.stable_hover(1.5);self.wait(self.map_ready,20)
            home=list(self.state()['position'])
            action=self.task(5. if self.args.scenario=='timeout' else 900.)
            if self.args.scenario in ('cancel','map-loss','worker-exit','worker-timeout','land-takeover'):
                self.wait(lambda:bool(self.command_events) and math.dist(self.state()['position'],home)>=.25,60.)
                if self.args.scenario=='land-takeover':
                    try:self.send('/land',base.navigation.Land,base.navigation.Land.Goal())
                    except RuntimeError as exc:
                        if str(exc)!='/land rejected':raise
                        self.report['concurrent_land_rejected']=True
                    else:raise RuntimeError('parallel Land unexpectedly bypassed existing action exclusion')
                if self.args.scenario in ('cancel','land-takeover'):
                    cancel=action['handle'].cancel_goal_async();self.wait(cancel.done,5.)
                    if not cancel.result().goals_canceling:raise RuntimeError('cancel rejected')
                else:
                    target='/gcopter_voxel_map' if self.args.scenario=='map-loss' else '/fuel_worker'
                    matches=[(pid,data) for pid,data in self.owned.items() if data['executable'].endswith(target)]
                    if len(matches)!=1:raise RuntimeError('exact injection target missing')
                    pid,data=matches[0]
                    if base.navigation.children(self.process.pid).get(pid)!=data:raise RuntimeError('injection identity changed')
                    sig=signal.SIGTERM if self.args.scenario=='worker-exit' else signal.SIGSTOP
                    os.kill(pid,sig)
                    if sig==signal.SIGSTOP:self.paused.append((pid,data))
                    self.report['fault_injection']=dict(pid=pid,identity=data,signal=int(sig),receipt=time.monotonic())
                self.finish_task(action,5 if self.args.scenario in ('cancel','land-takeover') else 6)
                if self.args.scenario=='land-takeover':
                    takeover=self.send('/land',base.navigation.Land,base.navigation.Land.Goal())
                    self.finish(takeover,seconds=120)
                    self.report['controlled_land_handover_completed']=True
                for pid,data in self.paused:
                    if base.navigation.children(self.process.pid).get(pid)==data:os.kill(pid,signal.SIGCONT)
                self.paused.clear()
                self.observe(2.);self.wait(self.map_ready,20)
                if self.args.scenario in ('cancel','map-loss'):
                    recovered=self.task();self.finish_task(recovered)
                    self.report['explicit_recovery_completed']=True
            else:
                entry=self.finish_task(action,6 if self.args.scenario in ('timeout','unreachable') else 4)
                if self.args.scenario=='complete':
                    mission=json.loads((self.directory/'fuel'/entry['goal_uuid']/'mission.json').read_text())
                    if entry['coverage']<.9 or entry['coverage']-mission['initial_coverage']<.15 or entry['completed_viewpoints']<2:
                        raise RuntimeError('complete mission below frozen coverage/gain/viewpoint metrics')
        except Exception as exc:
            failure=exc;self.cleanup_actions()
        if self.state() and self.state()['armed']:
            land=self.send('/land',base.navigation.Land,base.navigation.Land.Goal());self.finish(land,seconds=120)
        self.wait(lambda:self.ready(False),20)
        self.report['final_disarmed_state']=self.state()
        if failure:raise failure

    def close(self):
        if self.sensor_truth:
            if self.sensor_truth.poll() is None:
                self.sensor_truth.send_signal(signal.SIGINT)
                try:self.sensor_truth.wait(timeout=12.)
                except subprocess.TimeoutExpired:
                    self.sensor_truth.kill();self.sensor_truth.wait(timeout=3.);self.report['forced']=True
            self.report['sensor_truth_returncode']=self.sensor_truth.returncode
            self.sensor_log.close()
        super().close();self.fuel_log.close()
        self.report['passed']=self.report['passed'] and self.report.get('native_sensor_joint_observed',False) and self.report.get('sensor_truth_returncode')==0
        (self.directory/'process-budget.json').write_text(json.dumps(dict(samples=self.cpu,
            clock_ticks_per_second=os.sysconf('SC_CLK_TCK'),page_size=os.sysconf('SC_PAGE_SIZE')),indent=2)+'\n')
        # Base acceptance still requires all launched roles to exit normally, native transport
        # drain, disarm and no leftovers. Extra task evidence adds requirements.
        worker_ids=[json.loads(p.read_text()) for p in (self.directory/'fuel').glob('*/worker-identity.json')]
        reaped=all(not Path('/proc',str(w['pid'])).exists() for w in worker_ids)
        selected={str((Path(get_package_prefix(pkg))/'lib'/lib).resolve())
            for pkg,lib in [('easyfly_fuel_native','libfuel_native.so'),('easyfly_fuel_native','libfuel_lkh.so'),('easyfly_fuel_core','libfuel_core.so')]}
        actual=[]
        for path in (self.directory/'fuel').glob('*/loaded-maps.txt'):
            loaded={line.split()[-1] for line in path.read_text().splitlines() if any(name in line
                for name in ['libfuel_native.so','libfuel_lkh.so','libfuel_core.so'])}
            actual.append(dict(file=str(path.relative_to(self.directory)),loaded=sorted(loaded),matches=loaded==selected))
        library_match=all(v['matches'] for v in actual) and (bool(actual) or self.args.scenario=='bootstrap')
        self.report.update(fuel_workers_reaped=reaped,fuel_event_count=len(self.fuel_events),fuel_native_libraries=actual)
        self.report['passed']=self.report['passed'] and library_match
        self.report['passed']=self.report['passed'] and reaped
        (self.directory/'result.json').write_text(json.dumps(self.report,indent=2)+'\n')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output-dir',required=True,type=Path);p.add_argument('--px4-dir',required=True,type=Path)
    p.add_argument('--truth-executable',required=True,type=Path);p.add_argument('--domain',type=int,default=223)
    p.add_argument('--gazebo-port',type=int,default=12623);p.add_argument('--scene',default='fuel_room_v1')
    p.add_argument('--scenario',choices=['bootstrap','complete','cancel','map-loss','worker-exit','worker-timeout','timeout','unreachable','no-fly','land-takeover'],default='complete')
    p.add_argument('--sensor-witness-executable',type=Path,default=Path(
        get_package_prefix('easyfly_gazebo_plugins'))/'lib/easyfly_gazebo_plugins/easyfly_sensor_witness')
    p.add_argument('--managed-profile',action='store_true');p.add_argument('--execute-simulation',action='store_true',required=True)
    args=p.parse_args();args.strategy='gcopter';args.shutdown_stack_diagnostic=False;args.shutdown_stack_delay=2.
    args.battery_drain_seconds=1800.
    os.environ['ROS_LOCALHOST_ONLY']='1'
    args.output_dir.mkdir(parents=True)
    probe=Probe(args)
    try:probe.run()
    except (Exception,KeyboardInterrupt) as exc:
        probe.report['error']=str(exc);traceback.print_exc()
    finally:probe.close()
    print(json.dumps(dict(passed=probe.report['passed'],error=probe.report['error'])),flush=True)
    return 0 if probe.report['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())

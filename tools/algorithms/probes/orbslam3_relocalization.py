#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Actual loaded-map tracking loss, restoration and post-admission relocalization.

Uses a native visual occluder, never edited images or synthetic pose corrections.
"""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
from PIL import Image
from audit_fastlivo_bag import fingerprint
from orbslam3_natural_fixture import NaturalFixtureMixin
import subprocess
import sys
import time

spec = importlib.util.spec_from_file_location('orb_relocalization_fixture', Path(__file__).with_name('orbslam3_simulation.py'))
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)
ATLAS = None


class RelocalizationProbe(NaturalFixtureMixin,base.Probe):
    def __init__(self, args):
        self.native_rgb = None; self.native_rgb_receipt = 0.
        super().__init__(args)
        self.report['load_atlas'] = str(ATLAS)
        self.report['test_variant'] = 'loaded_map_post_admission_relocalization_isolation'
        self.report['relocalization_probe_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        (self.directory/'relocalization-source.py').write_bytes(Path(__file__).read_bytes())

    def on_image(self, message):
        self.native_rgb = message;self.native_rgb_receipt=time.monotonic()
        super().on_image(message)

    def image_witness(self, name):
        message=self.native_rgb
        if message is None or message.encoding not in ['rgb8','bgr8']:
            raise RuntimeError('original native RGB witness unavailable')
        image=Image.frombytes('RGB',(message.width,message.height),bytes(message.data),
                              'raw','RGB' if message.encoding=='rgb8' else 'BGR',message.step,1)
        path=self.directory/(name+'.png');image.save(path)
        stamp,digest=fingerprint(message)
        record=dict(source_ns=stamp,original_message_sha256=digest,png_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    encoding=message.encoding,frame=message.header.frame_id,receipt=self.native_rgb_receipt,
                    method='lossless original RGB witness only; never fed back to estimator')
        (self.directory/(name+'.json')).write_text(json.dumps(record,indent=2)+'\n')
        return record

    def launch_command(self):
        return super().launch_command()+['orbslam3_load_atlas:='+str(ATLAS)]

    def lifecycle(self):
        for row in reversed(self.vio_samples):
            if row['stream']=='map_status': return row['values'][0]
        return {}

    def reject_navigation(self):
        state=self.state()['position'];before=len(self.report['actions'])
        try:
            self.send('/navigate',base.navigation.NavigateTo,base.navigation.NavigateTo.Goal(
                target_x=state[0]+.3,target_y=state[1],target_z=state[2],max_speed=.6))
        except RuntimeError as error:
            if str(error)!='/navigate rejected':raise
        if len(self.report['actions'])!=before+1 or self.report['actions'][-1].get('accepted') is not False:
            raise RuntimeError('navigation admitted after source loss/relocalization')

    def _run_fault_scenario(self):
        initial=self.lifecycle()
        if int(initial.get('relocalizations',0))<1 or initial.get('loaded_atlas')!='1':
            raise RuntimeError('real old-map initial relocalization required before fault')
        active=self.navigation(1.5);self.fault_active=active;origin=self.state()['position']
        self.wait(lambda:math.dist(origin,self.state()['position'])>=.25,25)
        if active['result'].done():raise RuntimeError('navigation ended before physical occlusion')
        event=dict(scenario='post_admission_relocalization',steady=time.monotonic(),state=self.state(),
                   goal_uuid=active['entry']['goal_uuid'],initial_map=initial,
                   input_counts_before=dict(self.original_counts))
        self.report['injection']=event
        event['before_occlusion_rgb']=self.image_witness('before-occlusion-native')
        model='orb_relocalization_occluder';inserted=False
        panel=self.directory/'relocalization-occluder.sdf'
        panel.write_text('''<sdf version="1.6"><model name="orb_relocalization_occluder">
<static>true</static><link name="panel"><visual name="blackout"><cast_shadows>false</cast_shadows>
<geometry><box><size>0.02 10 10</size></box></geometry><material><lighting>false</lighting>
<ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse>
<emissive>0.02 0.02 0.02 1</emissive></material></visual></link></model></sdf>''')
        def remove():
            nonlocal inserted
            if not inserted:return
            reply=subprocess.run(['gz','model','-m',model,'-d'],env=self.environment,
                                 capture_output=True,text=True,timeout=10)
            if reply.returncode:raise RuntimeError('owned native occluder removal failed')
            inserted=False;event['native_occluder_removed']=dict(steady=time.monotonic(),returncode=reply.returncode)
        try:
            start=self.scene_origin();position=[start[0]+.9,start[1],1.8]
            command=['gz','model','--spawn-file='+str(panel),'--model-name='+model,
                     '-x',str(position[0]),'-y',str(position[1]),'-z',str(position[2])]
            reply=subprocess.run(command,env=self.environment,capture_output=True,text=True,timeout=10)
            if reply.returncode:raise RuntimeError('native occluder insertion failed')
            inserted=True
            event['occluder_native_witness']=self.model_witness(model,position)
            event['occluder_sdf_sha256']=hashlib.sha256(panel.read_bytes()).hexdigest()
            self.finish(active,expected=6,seconds=30)
            event['terminal_steady']=time.monotonic()
            self.wait(lambda:base.navigation.takeover_streams_quiet(
                [r for r in self.report['streams'] if r['topic']!='/uav_command' or
                 r.get('command_mode')==base.navigation.UAVCommand.TRAJECTORY],
                event['terminal_steady'],time.monotonic()),5)
            event['actual_trajectory_stream_quiet']=True
            self.wait(lambda:int(self.lifecycle().get('tracking_state',-1)) in (3,4) and int(self.lifecycle().get('tracked_features',999999))<30,10)
            lost=self.lifecycle();event['actual_lost_map']=lost
            event['occluded_rgb']=self.image_witness('occluded-native')
            if lost.get('map_id')!=initial.get('map_id') or lost.get('keyframes')!=initial.get('keyframes'):
                raise RuntimeError('loaded map was reset instead of retained while lost')
            self.reject_navigation();event['new_navigation_rejected_while_lost']=True
            # Restore the view by a verified physical move, rather than
            # trusting a delete-request return code as proof of scene removal.
            restored=[position[0],position[1]+20.,position[2]]
            reply=subprocess.run(['gz','model','-m',model,'-x',str(restored[0]),'-y',str(restored[1]),'-z',str(restored[2])],
                                 env=self.environment,capture_output=True,text=True,timeout=10)
            if reply.returncode:raise RuntimeError('native shutter move failed')
            event['native_occluder_restored']=self.model_witness(model,restored)
            restored_wall=time.monotonic()
            self.wait(lambda:self.native_rgb_receipt>restored_wall+1. and
                      self.node.get_clock().now().nanoseconds-(self.native_rgb.header.stamp.sec*10**9+self.native_rgb.header.stamp.nanosec)<.2e9,10)
            event['restored_rgb']=self.image_witness('restored-native')
            self.wait(lambda:int(self.lifecycle().get('relocalizations',0))>int(initial['relocalizations']),30)
            final=self.lifecycle();event['actual_relocalized_map']=final
            self.wait(lambda:self.vio.get('status') is not None and self.vio['status'].state==4,5)
            self.reject_navigation();event['new_navigation_rejected']=True
            event['input_counts_after']=dict(self.original_counts)
            if any(event['input_counts_after'][k]<=event['input_counts_before'][k]+2 for k in self.original_counts):
                raise RuntimeError('original sensor streams did not continue during occlusion')
            if final.get('map_id')!=initial.get('map_id') or final.get('keyframes')!=initial.get('keyframes'):
                raise RuntimeError('relocalization created a new map instead of using old keyframes')
            events=[r for r in self.vio_samples if r['stream']=='atlas_events' and
                    int(r['values'][0].get('correction_epoch',0))>int(initial['correction_epoch'])]
            if not events:raise RuntimeError('actual relocalization epoch event not observed')
            epoch=events[0]['values'][0]
            if epoch.get('reset_latched')!='1' or int(epoch['public_pose_count'])<100:
                raise RuntimeError('post-admission relocalization did not latch before public output')
            start=self.node.get_clock().now().nanoseconds
            self.wait(lambda:self.node.get_clock().now().nanoseconds-start>=3e9,15)
            public=[r for r in self.vio_samples if r['stream']=='pose']
            later=[r for r in self.vio_samples if r['stream']=='status' and r['receipt']>=events[0]['receipt']]
            if len(public)!=int(epoch['public_pose_count']) or \
               any(r['stamp_ns']>int(epoch['last_public_pose_stamp_ns']) for r in public) or \
               not later or any(r['state']!=4 for r in later):
                raise RuntimeError('public localization resumed after actual relocalization')
            event['public_counter_frozen']=True;event['reset_maintained']=True
            event['first_actual_relocalization_epoch']=epoch
        finally:
            remove()


if __name__=='__main__':
    parser=argparse.ArgumentParser(add_help=False)
    parser.add_argument('--load-atlas',type=Path,required=True)
    options,remaining=parser.parse_known_args()
    ATLAS=options.load_atlas.resolve()
    if not Path(str(ATLAS)+'.osa').is_file():raise ValueError('actual saved atlas stem required')
    if '--scenario' not in remaining:remaining+=['--scenario','visual-loss']
    if remaining[remaining.index('--scenario')+1]!='visual-loss':raise ValueError('use visual-loss fixture dispatch')
    sys.argv=[sys.argv[0]]+remaining
    base.Probe=RelocalizationProbe
    raise SystemExit(base.main())

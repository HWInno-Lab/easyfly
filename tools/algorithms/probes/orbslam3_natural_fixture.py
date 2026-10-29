# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""A native photo-textured physical wall; no rendered-input substitutions."""
import hashlib
import importlib.util
import subprocess
import sys
from pathlib import Path

spec=importlib.util.spec_from_file_location('orb_natural_wall_builder',Path(__file__).with_name('capture_orbslam3_loop.py'))
wall=importlib.util.module_from_spec(spec);spec.loader.exec_module(wall)


class NaturalFixtureMixin:
    def __init__(self,args):
        self.native_rgb=None
        super().__init__(args)
        self.report['feature_fixture_family']='native physical wall with untouched pinned AWS photo'
        data=Path(__file__).read_bytes()
        (self.directory/'natural-fixture-source.py').write_bytes(data)
        self.report['natural_fixture_sha256']=hashlib.sha256(data).hexdigest()

    def on_image(self,message):
        self.native_rgb=message
        return super().on_image(message)

    def startup_ready(self):
        # Estimation waits for the completed physical fixture, so startup uses
        # actual FCU readiness and the native truth observer, not an early map.
        if self.truth is None and self.process is not None:
            console=self.directory/'console.log'
            if console.exists() and 'Connected to gazebo master' in console.read_text():
                self.start_truth()
        # A map cannot exist until fixture admission opens. This is disarmed
        # setup readiness only; inherited mapping_ready/flight gates still run
        # after original RGB-D forwarding starts and before navigation.
        return self.ready(False) and 'ego_planner_node' in self.node.get_node_names()

    def launch_command(self):
        return ['ros2','launch','easyfly_px4_sitl','perception_sitl.launch.py',
                'localization_backend:=orbslam3','orbslam3_navigation:=true',
                'orbslam3_runtime_directory:='+str(self.runtime),
                'orbslam3_save_atlas:='+str(self.runtime/'atlas'),
                'orbslam3_rgb_topic:=/fixture/rgb','orbslam3_depth_topic:=/fixture/depth',
                'scene_profile:=navigation_dev','scene:='+self.args.scene,
                'navigation_strategy:='+self.args.strategy,'headless:=true','rviz:=false',
                'detection_use_gpu:=false']

    def wall_layout(self):
        origin=self.scene_origin()
        return [(origin[0]+4.,origin[1],2.,0.,8.,4.8)]

    def spawn_feature_board(self):
        wall.Capture.spawn_walls(self)
        # Original sensor streams already run, but the estimator has received
        # zero RGB-D pairs during physical scene construction. Start forwarding
        # the original messages only now; do not clear a map/reset or edit data.
        if any(row['stream']=='pose' for row in self.vio_samples):
            raise RuntimeError('estimator published a pose before static fixture admission')
        self.report['public_poses_before_fixture_admission'] = 0
        relay=Path(__file__).with_name('orbslam3_fixture_relay.py')
        log=(self.directory/'fixture-relay.log').open('x')
        process=subprocess.Popen([sys.executable,str(relay)],env=self.environment,
                                 stdout=log,stderr=subprocess.STDOUT)
        self.relays.append(('fixture_rgbd',process,log))
        self.report['fixture_input_admission']=dict(
            original_messages_unmodified=True, relay_sha256=hashlib.sha256(relay.read_bytes()).hexdigest(),
            reason='start RGB-D admission after native static fixture construction; no previous public pose')
        self.synchronize_fixture_observer()

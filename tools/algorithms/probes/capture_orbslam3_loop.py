#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Collect a real closed-view Gazebo RGB-D route on the existing FAST-LIO controller."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import shutil
import signal
import subprocess
import time

SPEC = importlib.util.spec_from_file_location('orb_loop_fixture', Path(__file__).with_name('orbslam3_simulation.py'))
orb = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(orb)
common = orb.common
navigation = common.navigation


class Capture(common.Probe):
    configure_simulated_battery = orb.Probe.configure_simulated_battery
    scene_origin = orb.Probe.scene_origin

    def __init__(self, args):
        from easyfly_fastdds_compat.runtime import activate
        activate(openvins=True)
        self.native_rgb = None
        self.capture_active = None
        super().__init__(args)
        self.recorder = None
        self.recorder_log = None
        self.recording_required = False
        self.report.pop('openvins_provenance_sha256', None)
        self.report['backend'] = 'fast_lio'
        self.report['scope'] = 'original Gazebo RGB-D loop acquisition; not ORB flight certification'
        source = Path(__file__).read_bytes()
        (self.directory / 'capture-source.py').write_bytes(source)
        self.report['capture_source_sha256'] = hashlib.sha256(source).hexdigest()

    def send(self, name, action_type, goal):
        active = super().send(name, action_type, goal)
        if name in ['/takeoff', '/goto_waypoint']:
            self.capture_active = active
        return active

    def launch_command(self):
        return ['ros2', 'launch', 'easyfly_px4_sitl', 'perception_sitl.launch.py',
                'localization_backend:=fast_lio', 'gazebo_lockstep:=true',
                'scene_profile:=navigation_dev', 'navigation_strategy:=combined',
                'scene:=' + self.args.scene, 'headless:=true', 'rviz:=false',
                'detection_use_gpu:=false']

    def start_recording_static_scene(self):
        if self.state()['armed']:
            raise RuntimeError('recording boundary requires disarmed static scene')
        def pause(value):
            reply = subprocess.run(['gz', 'world', '--pause', str(value)], env=self.environment,
                                   capture_output=True, text=True, timeout=10)
            if reply.returncode:
                raise RuntimeError('native recording-boundary pause command failed')
        pause(1)
        previous = [self.node.get_clock().now().nanoseconds, time.monotonic()]
        def paused():
            current = self.node.get_clock().now().nanoseconds
            if current != previous[0]:
                previous[:] = [current, time.monotonic()]
            return time.monotonic()-previous[1] >= .5
        try:
            self.wait(paused, 10)
            self.report['recording_boundary'] = dict(
                source_ns=previous[0], disarmed=True, physics_pause_observed=True,
                reason='all physical fixture insertion completed before recording; original source stamps retained')
            self.start_recorder()
            self.wait(lambda: 'All requested topics are subscribed.' in
                      (self.directory / 'recorder.log').read_text(), 20)
        finally:
            pause(0)
        start = self.node.get_clock().now().nanoseconds
        self.wait(lambda: self.ready(False) and self.node.get_clock().now().nanoseconds-start >= 3e9, 30)

    def waypoint_speed(self, label):
        return .3

    def waypoint_timeout(self, label):
        return 60

    def start_recorder(self):
        qos = self.directory / 'recorder-qos.yaml'
        qos.write_text('/livox/imu:\n  history: keep_last\n  depth: 2048\n  reliability: reliable\n  durability: volatile\n')
        self.report['recorder_imu_qos'] = dict(history='keep_last', depth=2048, reliability='reliable')
        self.recorder_log = (self.directory / 'recorder.log').open('x')
        self.recorder = subprocess.Popen([
            'ros2', 'bag', 'record', '-o', str(self.directory / 'rgbd-bag'),
            '--qos-profile-overrides-path', str(qos),
            '/clock', '/livox/imu', '/camera/color/image_raw', '/camera/color/camera_info',
            '/camera/aligned_depth_to_color/depth/image_raw',
            '/camera/aligned_depth_to_color/depth/camera_info'] + self.recorder_arguments(),
            env=self.environment, stdout=self.recorder_log, stderr=subprocess.STDOUT)
        self.recording_required = True

    def recorder_arguments(self):
        return []

    def wait(self, predicate, seconds):
        def observed():
            if self.recording_required and self.recorder is not None and self.recorder.poll() is not None:
                raise RuntimeError('original RGB-D recorder exited during acquisition')
            return predicate()
        return super().wait(observed, seconds)

    def on_image(self, message):
        self.native_rgb = message
        return super().on_image(message)

    def wall_layout(self):
        # x, y, z, yaw, width, height; source geometry is retained per case.
        return [(4*math.cos(n*math.pi/2), 4*math.sin(n*math.pi/2), 2.,
                 n*math.pi/2, 8., 4.8) for n in range(4)]

    def route_waypoints(self, origin, yaw0):
        for degrees in range(30, 721, 30):
            theta = math.radians(degrees)
            heading = math.atan2(math.sin(yaw0+theta), math.cos(yaw0+theta))
            yield degrees, [origin[0]+1.2*math.sin(theta), origin[1]+1.2*(1-math.cos(theta)),
                            origin[2]+.25*math.sin(theta)], heading

    def startup_ready(self):
        if self.truth is None and self.process is not None:
            console = self.directory / 'console.log'
            if console.exists() and 'Connected to gazebo master' in console.read_text():
                self.start_truth()
        return navigation.Probe.startup_ready(self)

    def spawn_walls(self):
        if self.state()['armed']:
            raise RuntimeError('physical loop fixture requires disarmed insertion')
        # Project-authored geometry. Existing pinned model photos are surfaces,
        # with real collision/depth, never an image overlay or feature injection.
        import numpy as np
        from PIL import Image
        assets = Path(__file__).resolve().parents[3] / 'src/simulation/easyfly_gazebo_models'
        photo_paths = [
            'third_party/aws/house/aws_robomaker_residential_PortraitD_01/materials/textures/PortraitD_01.jpg',
            'third_party/aws/house/aws_robomaker_residential_PortraitA_01/materials/textures/PortraitA_01.jpg',
            'third_party/aws/house/aws_robomaker_residential_PortraitC_01/materials/textures/PortraitC_01.jpg',
            'third_party/aws/house/aws_robomaker_residential_DeskPortraitD_04/materials/textures/DeskPortraitD_04.jpg',
        ]
        materials = self.directory / 'wall-materials'
        materials.mkdir()
        witnesses = []
        for wall, (x, y, z, theta, width, height) in enumerate(self.wall_layout()):
            position = [x, y, z]
            name = 'orb_loop_wall_' + str(wall)
            # Ogre scans a resource location once. Give each wall a complete,
            # distinct location before its first native model insertion.
            scripts = materials / name / 'scripts'
            textures = materials / name / 'textures'
            scripts.mkdir(parents=True)
            textures.mkdir()
            original_photo = assets / photo_paths[wall % len(photo_paths)]
            texture = textures / original_photo.name
            shutil.copyfile(original_photo, texture)
            material = scripts / (name + '.material')
            material.write_text('material ' + name + ''' {
  technique { pass {
    ambient 1 1 1 1
    diffuse 1 1 1 1
    specular 0 0 0 1 0
    texture_unit { texture ''' + texture.name + '''
      filtering bilinear
    }
  } }
}
''')
            visual = ('<visual name="textured_surface"><cast_shadows>false</cast_shadows>'
                      f'<geometry><box><size>.046 {width} {height}</size></box></geometry>'
                      '<material><script><uri>file://' + str(scripts) + '</uri>'
                      '<uri>file://' + str(textures) + '</uri><name>' + name +
                      '</name></script></material></visual>')
            sdf = self.directory / (name + '.sdf')
            sdf.write_text('<sdf version="1.6"><model name="' + name + '"><static>true</static>'
                           '<link name="wall"><collision name="physical_backing"><geometry>'
                           f'<box><size>.04 {width} {height}</size></box></geometry></collision>' +
                           visual + '</link></model></sdf>')
            command = ['gz', 'model', '--spawn-file=' + str(sdf), '--model-name=' + name,
                       '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]),
                       '-Y', str(theta)]
            reply = subprocess.run(command, env=self.environment, capture_output=True,
                                   text=True, timeout=15)
            if reply.returncode:
                raise RuntimeError('native wall insertion failed: ' + reply.stderr)
            witnesses.append(dict(name=name, sdf_sha256=hashlib.sha256(sdf.read_bytes()).hexdigest(),
                                  texture_sha256=hashlib.sha256(texture.read_bytes()).hexdigest(),
                                  original_texture_path=str(original_photo), original_texture_bytes_preserved=True,
                                  material_sha256=hashlib.sha256(material.read_bytes()).hexdigest(),
                                  native=self.model_witness(name, position, require_collision=True)))
        self.report['physical_loop_fixture'] = dict(
            variant='distinct original photographs from existing pinned AWS house model assets',
            asset_provenance=str(assets / 'licenses/aws_house/README.md'),
            asset_manifest_sha256=hashlib.sha256((assets / 'licenses/aws_house/origins.json').read_bytes()).hexdigest(),
            walls=witnesses, image_or_depth_substitution=False)
        start = self.node.get_clock().now().nanoseconds
        def settled():
            message = self.native_rgb
            source = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec if message else 0
            now = self.node.get_clock().now().nanoseconds
            return self.ready(False) and now-start >= 3e9 and message is not None and 0 <= now-source <= .5e9
        self.wait(settled, 30)
        message = self.native_rgb
        source = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec if message else 0
        if message is None or not 0 <= self.node.get_clock().now().nanoseconds-source <= .5e9:
            raise RuntimeError('fresh original native texture witness unavailable')
        mode = 'RGB' if message.encoding == 'rgb8' else 'BGR'
        witness = Image.frombytes('RGB', (message.width, message.height), bytes(message.data),
                                  'raw', mode, message.step)
        witness.save(self.directory / 'physical-texture-native.png')
        contrast = float(np.asarray(witness.convert('L'))[:240].std())
        self.report['physical_loop_fixture']['native_image_witness'] = dict(
            source_ns=source, top_half_grayscale_std=contrast,
            original_data_sha256=hashlib.sha256(bytes(message.data)).hexdigest())
        console = (self.directory / 'console.log').read_text()
        if any('Unable to get Material' in line and 'orb_loop_wall' in line for line in console.splitlines()):
            raise RuntimeError('native renderer could not load a physical wall material')
        # Global contrast is diagnostic, not a feature/readiness metric: a
        # correctly rendered natural photograph can have low global variance.
        # Material loading is checked above; actual ORB initialization/inliers
        # and native pose accuracy remain the independent acceptance gates.

    def run(self):
        navigation.Probe.start(self)
        if self.truth is None:
            raise RuntimeError('independent native truth missing')
        self.configure_simulated_battery()
        self.spawn_walls()
        self.start_recording_static_scene()
        failure = None
        try:
            active = self.send('/takeoff', navigation.Takeoff,
                               navigation.Takeoff.Goal(target_altitude=1.5, climb_speed=.3))
            self.finish(active, seconds=90)
            self.stable_hover(1.5)
            state = self.latest['/uav_state']
            origin = [state.position_x, state.position_y, state.position_z]
            yaw0 = state.yaw
            self.report['closed_view_route'] = []
            for label, target, heading in self.route_waypoints(origin, yaw0):
                action = self.send('/goto_waypoint', navigation.GoToWaypoint,
                                   navigation.GoToWaypoint.Goal(
                                       target_x=target[0], target_y=target[1], target_z=target[2],
                                       target_yaw=heading, max_speed=self.waypoint_speed(label), acceptance_radius=.08))
                self.finish(action, seconds=self.waypoint_timeout(label))
                self.wait(lambda: abs(math.atan2(
                    math.sin(self.latest['/uav_state'].yaw-heading),
                    math.cos(self.latest['/uav_state'].yaw-heading))) <= .06, 20)
                self.stable_hover()
                observation = dict(route_step=label, target=target, target_yaw=heading,
                                   observed_yaw=self.latest['/uav_state'].yaw, state=self.state())
                self.report['closed_view_route'].append(observation)
                print(json.dumps(observation), flush=True)
            start = self.node.get_clock().now().nanoseconds
            self.wait(lambda: self.node.get_clock().now().nanoseconds-start >= 5e9, 20)
        except Exception as exc:
            failure = exc
            self.report['capture_primary_error'] = repr(exc)
            self.recording_required = False
            # Failure of the recorder/observer does not cancel the FCU action.
            # Cancel the exact owned goal before requesting a normal landing.
            active = self.capture_active
            if active is not None and not active['result'].done():
                response = active['handle'].cancel_goal_async()
                navigation.Probe.wait(self, response.done, 10)
                navigation.Probe.wait(self, active['result'].done, 30)
                self.report['failed_capture_cancel'] = dict(
                    goal_uuid=active['entry']['goal_uuid'],
                    return_code=response.result().return_code,
                    terminal_status=active['result'].result().status)
                first = self.node.get_clock().now().nanoseconds
                navigation.Probe.wait(self, lambda:self.node.get_clock().now().nanoseconds-first >= .5e9, 10)
        if self.state() is not None and self.state()['armed']:
            land = self.send('/land', navigation.Land, navigation.Land.Goal())
            self.finish(land, seconds=120)
        self.wait(lambda: self.ready(False), 20)
        self.report['final_disarmed_state'] = self.state()
        if failure:
            raise failure

    def close(self):
        self.recording_required = False
        super().close()
        if self.recorder is not None:
            if self.recorder.poll() is None:
                self.recorder.send_signal(signal.SIGINT)
                try:
                    self.recorder.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    self.report['forced'] = True
                    self.recorder.kill()
                    self.recorder.wait(timeout=5)
            self.recorder_log.close()
            self.report['recorder_returncode'] = self.recorder.returncode
            self.report['passed'] = self.report['passed'] and self.recorder.returncode == 0 and not self.report['forced']
        (self.directory / 'result.json').write_text(json.dumps(self.report, indent=2) + '\n')


if __name__ == '__main__':
    orb.fixture.Probe = Capture
    raise SystemExit(orb.fixture.main())

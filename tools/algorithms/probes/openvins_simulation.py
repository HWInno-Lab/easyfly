#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Explicit managed PX4/Gazebo OpenVINS acceptance; sends simulation flight goals."""

import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml

from ament_index_python.packages import get_package_share_directory
from easyfly_msgs.action import Land, NavigateTo, Takeoff
from easyfly_msgs.msg import LocalizationStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from PIL import Image as RawImage
from livox_ros_driver2.msg import CustomMsg
from tf2_ros import Buffer, TransformListener
from rcl_interfaces.msg import Parameter as ParameterMessage, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters

from process_cpu_trace import snapshot


ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT/'src/application/a3_mission/easyfly_sim_launcher/scripts/navigation_sitl.py'
SPEC = importlib.util.spec_from_file_location('openvins_navigation_probe_base', SOURCE)
navigation = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(navigation)


def freeze_vio_profile(unused, destination):
    """Use the previously measured VIO XML, not D5's different transport policy."""
    source = Path(get_package_share_directory('easyfly_fastdds_compat')) / \
        'config/openvins_simulation.xml'
    data = source.read_bytes()
    with destination.open('xb') as stream:
        stream.write(data)
    return dict(source=str(source), frozen=str(destination),
                sha256=hashlib.sha256(data).hexdigest(),
                env={key: str(destination) for key in (
                    'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE')})


class Probe(navigation.Probe):
    def __init__(self, args):
        navigation.freeze_fastdds = freeze_vio_profile
        super().__init__(args)
        sources = self.directory/'source-snapshot'
        sources.mkdir()
        for name, path in [
                ('openvins_simulation.py', Path(__file__)), ('navigation_sitl.py', SOURCE),
                ('traj_server.cpp', ROOT/'src/third_party/algorithms/planning/ego_planner/'
                 'plan_manage/src/traj_server.cpp'),
                ('planner_manager.cpp', ROOT/'src/third_party/algorithms/planning/ego_planner/'
                 'plan_manage/src/planner_manager.cpp'),
                ('navigation_node.py', ROOT/'src/perception/p3_tracking/'
                 'easyfly_openvins_integration/easyfly_openvins_integration/navigation_node.py'),
                ('shutdown.py', ROOT/'src/perception/p3_tracking/'
                 'easyfly_openvins_integration/easyfly_openvins_integration/shutdown.py'),
                ('guard_node.py', ROOT/'src/perception/p3_tracking/'
                 'easyfly_openvins_integration/easyfly_openvins_integration/node.py'),
                ('navigation_coordinator_node.cpp', ROOT/'src/execution/e2_planning/'
                 'easyfly_global_planner/src/navigation_coordinator_node.cpp'),
                ('route_clearance.hpp', ROOT/'src/execution/e2_planning/'
                 'easyfly_global_planner/include/easyfly_global_planner/route_clearance.hpp')]:
            (sources/name).write_bytes(path.read_bytes())
        self.vio = {}
        self.vio_receipts = {}
        self.vio_samples = []
        self.truth = None
        self.truth_log = None
        self.pressure_workers = []
        self.pressure_logs = []
        self.first_estimating = None
        self.discarded = 0
        self.cpu_samples = []
        self.last_cpu_sample = 0.
        self.tracking = None
        self.mapping_inputs = []
        self.images_saved = set()
        self.node.create_subscription(Image, '/camera/color/image_raw', self.on_image,
                                      qos_profile_sensor_data)
        self.tf_buffer = Buffer(node=self.node)
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        for topic, kind in [('/livox/lidar', CustomMsg), ('/pointcloud_filtered', PointCloud2)]:
            self.node.create_subscription(
                kind, topic, lambda m, t=topic: self.on_map_input(t, m), qos_profile_sensor_data)
        for topic, kind in [('/uav_command', navigation.UAVCommand),
                            ('/mavros/setpoint_raw/local', navigation.PositionTarget)]:
            self.node.create_subscription(
                kind, topic, lambda m, t=topic: self.stream(t, m), qos_profile_sensor_data)
        self.node.create_subscription(DiagnosticArray, '/openvins/raw/visual_tracking',
                                      self.on_tracking, 32)
        self.report['acceptance_limits'] = dict(
            dynamic_source_span_s=10., minimum_poses=100, position_rmse_m=.3,
            position_max_m=1., attitude_error_rad=.35)
        self.report['probe_source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        provenance = ROOT/'src/third_party/algorithms/localization/open_vins.provenance.json'
        self.report['openvins_provenance_sha256'] = hashlib.sha256(
            provenance.read_bytes()).hexdigest()
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic, kind in (
                ('status', LocalizationStatus), ('alignment_status', LocalizationStatus),
                ('navigation_status', LocalizationStatus),
                ('pose', PoseWithCovarianceStamped), ('aligned_pose', PoseStamped)):
            self.node.create_subscription(
                kind, '/localization/openvins/'+topic,
                lambda msg, key=topic: self.on_vio(key, msg),
                status_qos if 'status' in topic else 100)
        hook = ROOT/'tools/algorithms/probes/guard_trace_hook'
        self.environment.update(
            EASYFLY_GUARD_TAKE_TRACE=str(self.directory/'guard-take'),
            EASYFLY_GUARD_CLOCK_TRACE='1', EASYFLY_GUARD_FRESHNESS_TRACE='1',
            PYTHONPATH=str(hook)+':'+self.environment.get('PYTHONPATH', ''))
        if args.no_input_audit:
            for key in ['EASYFLY_GUARD_TAKE_TRACE', 'EASYFLY_GUARD_CLOCK_TRACE',
                        'EASYFLY_GUARD_FRESHNESS_TRACE']:
                self.environment.pop(key, None)

    def wait(self, predicate, seconds):
        def observed():
            now = time.monotonic()
            if self.owned and now-self.last_cpu_sample >= 1.:
                self.last_cpu_sample = now
                record = snapshot(list(self.owned.items()))
                record['clock_ns'] = self.node.get_clock().now().nanoseconds
                self.cpu_samples.append(record)
            return predicate()
        return super().wait(observed, seconds)

    def on_map_input(self, topic, msg):
        if len(self.mapping_inputs) < 50000:
            self.mapping_inputs.append(dict(
                topic=topic, stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                frame=msg.header.frame_id,
                count=getattr(msg, 'width', getattr(msg, 'point_num', 0)),
                receipt=time.monotonic()))

    def on_image(self, msg):
        # Three read-only raw receipts for diagnosis, never fed back into VIO.
        phase = ('ground' if self.state() is None or not self.state()['armed'] else
                 'outbound' if len(self.report.get('navigation_targets', [])) < 2 else 'return')
        if phase not in self.images_saved:
            path = self.directory/('raw-camera-'+phase+'.png')
            if msg.encoding not in ('rgb8', 'bgr8'):
                raise RuntimeError('unexpected raw camera encoding')
            raw_mode = 'RGB' if msg.encoding == 'rgb8' else 'BGR'
            RawImage.frombytes('RGB', (msg.width, msg.height), bytes(msg.data),
                               'raw', raw_mode, msg.step).save(path)
            self.images_saved.add(phase)
            self.report.setdefault('raw_camera_evidence', []).append(dict(
                phase=phase, stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                file=path.name, image_data_sha256=hashlib.sha256(bytes(msg.data)).hexdigest()))

    def on_tracking(self, msg):
        if len(msg.status) == 1:
            data = {v.key: v.value for v in msg.status[0].values}
            self.tracking = dict(stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                                 receipt=time.monotonic(),
                                 tracked_features=int(data['tracked_features']),
                                 initialized=data['initialized'] == '1', stream='visual_tracking')
            if len(self.vio_samples) < 200000:
                self.vio_samples.append(self.tracking)
            else:
                self.discarded += 1

    def on_vio(self, key, msg):
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        self.vio[key] = msg
        self.vio_receipts[key] = time.monotonic()
        row = dict(stream=key, stamp_ns=stamp, receipt=time.monotonic(),
                   frame=msg.header.frame_id, clock_ns=self.node.get_clock().now().nanoseconds)
        if 'status' in key:
            row.update(state=msg.state, reason=msg.reason, session=msg.session_id,
                       generation=msg.generation, pose_valid=msg.pose_valid,
                       pose_ns=msg.pose_stamp.sec*10**9+msg.pose_stamp.nanosec,
                       camera_ns=msg.camera_stamp.sec*10**9+msg.camera_stamp.nanosec,
                       camera_forwarded=msg.camera_forwarded)
            if msg.state == LocalizationStatus.ESTIMATING and self.first_estimating is None:
                self.first_estimating = row['receipt']
        else:
            pose = msg.pose.pose if key == 'pose' else msg.pose
            p, q = pose.position, pose.orientation
            row.update(position=[p.x, p.y, p.z], quaternion=[q.x, q.y, q.z, q.w])
        if len(self.vio_samples) < 200000:
            self.vio_samples.append(row)
        else:
            self.discarded += 1

    def startup_ready(self):
        if self.truth is None and self.process is not None:
            path = self.directory/'console.log'
            if path.exists() and 'Connected to gazebo master' in path.read_text():
                self.start_truth()
        status = self.vio.get('status')
        return self.ready(False) and status is not None and status.state in (1, 2)

    def mapping_ready(self):
        grid = self.latest.get('/occupancy_grid')
        ready = all(self.vio.get(k) is not None and self.vio[k].state == 2
                    for k in ['status', 'alignment_status', 'navigation_status'])
        return ready and \
            grid is not None and grid.info.width > 0 and self.counts.get('/occupancy_grid', 0) >= 3

    def start_truth(self):
        self.truth_log = (self.directory/'truth.log').open('x')
        self.truth = subprocess.Popen(
            [str(self.args.truth_executable.resolve(strict=True)),
             str(self.directory/'truth.csv')], env=self.environment,
            stdout=self.truth_log, stderr=subprocess.STDOUT)
        self.report['truth_pid'] = self.truth.pid
        self.report['truth_executable_sha256'] = hashlib.sha256(
            self.args.truth_executable.read_bytes()).hexdigest()

    def start(self):
        super().start()
        libraries = {}
        for pid in self.owned:
            try:
                lines = Path('/proc', str(pid), 'maps').read_text().splitlines()
                paths = {line.split()[-1] for line in lines if any(
                    name in line for name in [
                        'libov_msckf', 'libfastrtps', 'libsemantic_map',
                        'libeasyfly_global_planner',
                        'libeasyfly_native_imu', 'libeasyfly_imu_antialias',
                        'libeasyfly_gazebo_transport_compat', 'libgazebo_mavlink_interface'])}
                for path in paths:
                    if Path(path).is_file():
                        libraries[path] = hashlib.sha256(Path(path).read_bytes()).hexdigest()
            except OSError:
                pass
        self.report['runtime_libraries_sha256'] = libraries
        self.report['runtime_control_executables_sha256'] = {
            d['executable']: hashlib.sha256(Path(d['executable']).read_bytes()).hexdigest()
            for d in self.owned.values() if d['executable'].endswith(
                ('/traj_server', '/ego_planner_node', '/navigation_coordinator_node',
                 '/run_subscribe_msckf'))}
        share = Path(get_package_share_directory('easyfly_openvins_integration'))
        config = share/'config/gazebo_mono'/(
            'estimator_config.yaml' if self.args.scenario == 'dynamic'
            else 'estimator_config_navigation.yaml')
        self.report['estimator_configuration'] = dict(
            path=str(config), sha256=hashlib.sha256(config.read_bytes()).hexdigest())

    def launch_command(self):
        if self.args.managed_profile:
            if self.args.scenario == 'dynamic':
                raise ValueError('managed-profile acceptance requires explicit navigation')
            return ['ros2', 'launch', 'easyfly_sim_launcher', 'profile.launch.py',
                    'profile:=navigation_dev', 'openvins_navigation:=true',
                    'algorithms_json:='+json.dumps(dict(
                        slam='openvins', strategy=self.args.strategy, provider='cpu')),
                    'scene:='+self.args.scene, 'headless:=true', 'rviz:=false']
        return ['ros2', 'launch', 'easyfly_px4_sitl', 'perception_sitl.launch.py',
                'localization_backend:=openvins', 'scene:='+self.args.scene,
                'scene_profile:=navigation_dev',
                'openvins_navigation:='+str(self.args.scenario != 'dynamic').lower(),
                'navigation_strategy:='+self.args.strategy,
                'headless:=true', 'rviz:=false', 'detection_use_gpu:=false']

    def run(self):
        self.start()
        if self.truth is None:
            raise RuntimeError('independent native truth observer did not start')
        if self.args.battery_drain_seconds != 60:
            self.configure_simulated_battery()
        if self.args.visual_feature_board:
            self.spawn_feature_board()
        if self.args.scenario == 'obstacle':
            self.spawn_obstacle()
        start = self.node.get_clock().now().nanoseconds
        # The real static initializer needs its full source-time history before jerk.

        def initialized_input_history():
            status = self.vio.get('status')
            if status is None or status.state not in (1, 2) or \
                    time.monotonic()-self.vio_receipts.get('status', 0) > .5:
                raise RuntimeError('localization group stopped or failed before takeoff')
            return self.node.get_clock().now().nanoseconds-start >= 3e9

        self.wait(initialized_input_history, 30)
        self.report['initialization_bootstrap'] = (
            'one explicit PX4 takeoff triggers measured motion initialization; '
            'no truth initialization; navigation waits for actual VIO and mapping')
        self.report['bootstrap_climb_speed_mps'] = self.args.bootstrap_climb_speed
        failure = None
        try:
            active = self.send('/takeoff', Takeoff,
                               Takeoff.Goal(target_altitude=1.5,
                                            climb_speed=self.args.bootstrap_climb_speed))
            self.finish(active, seconds=90)
            self.stable_hover(1.5)
            bootstrap_origin = None
            if self.args.initialization_translation:
                state = self.latest['/uav_state']
                bootstrap_origin = [state.position_x, state.position_y, state.position_z,
                                    state.yaw]
                self.initialization_waypoint(
                    [bootstrap_origin[0]+self.args.initialization_translation,
                     *bootstrap_origin[1:]])

            def localization_initialized():
                status = self.vio.get('status')
                if status is None or status.state not in (1, 2) or \
                        time.monotonic()-self.vio_receipts.get('status', 0) > .5:
                    raise RuntimeError('localization group stopped or failed during initialization')
                return all(self.vio.get(k) is not None and self.vio[k].state == 2
                           for k in ['status', 'alignment_status'])

            self.wait(localization_initialized, 90)
            if bootstrap_origin is not None:
                self.initialization_waypoint(bootstrap_origin)
                self.report['initialization_translation_m'] = self.args.initialization_translation
            if self.args.heading_degrees:
                self.set_view_heading()
            if self.args.scenario != 'dynamic':
                # GroundPlaneTracker deliberately initializes only while disarmed.
                # A motion-initialized VIO therefore needs one explicit measured
                # initialization flight and landing before the navigation flight.
                if not self.mapping_ready():
                    bootstrap_land = self.send('/land', Land, Land.Goal())
                    self.finish(bootstrap_land, seconds=120)
                    self.wait(lambda: self.ready(False), 20)
                    self.wait(self.mapping_ready, 30)
                    self.report['mapping_bootstrap_disarmed_state'] = self.state()
                    self.report['mapping_bootstrap'] = (
                        'real motion initialization, normal landing, disarmed ground model; '
                        'no armed ground-model initialization or injected plane')
                    second_takeoff = self.send('/takeoff', Takeoff, Takeoff.Goal(
                        target_altitude=1.5, climb_speed=.3))
                    self.finish(second_takeoff, seconds=90)
                    self.stable_hover(1.5)
                self.wait(self.mapping_ready, 60)
            if self.args.cpu_workers:
                self.start_pressure()
            if self.args.scenario in ['mission', 'long-run', 'obstacle']:
                distance = (getattr(self.args, 'obstacle_distance', 3.0)
                            if self.args.scenario == 'obstacle' else 1.5)
                home = self.state()['position']
                first_nav = self.navigation(distance)
                self.finish(first_nav, seconds=120)
                self.stable_hover()
                back = (self.navigation_target(home) if self.args.scenario == 'obstacle'
                        else self.navigation(-distance))
                self.finish(back, seconds=120)
                errors = [math.dist(a['entry']['terminal_state']['position'], target)
                          for a, target in zip([first_nav, back],
                                               self.report['navigation_targets'])]
                self.report['navigation_terminal_errors_m'] = errors
                if any(error > .11 for error in errors):
                    raise RuntimeError('actual navigation did not meet existing .11m arrival limit')
                if self.args.scenario == 'long-run':
                    first = self.node.get_clock().now().nanoseconds
                    (self.directory/'long-hold-start.json').write_text(json.dumps(dict(
                        source_start_ns=first, steady_start=time.monotonic(),
                        required_source_seconds=self.args.hold_source_seconds))+'\n')
                    self.healthy_hold(self.args.hold_source_seconds)
                    self.report['long_hold_source_seconds'] = (
                        self.node.get_clock().now().nanoseconds-first)/1e9
                    for leg in [1.5, -1.5]:
                        action = self.navigation(leg)
                        self.finish(action, seconds=120)
                        error = math.dist(action['entry']['terminal_state']['position'],
                                          self.report['navigation_targets'][-1])
                        self.report['navigation_terminal_errors_m'].append(error)
                        if error > .11:
                            raise RuntimeError('post-hold navigation actual arrival error')
            elif self.args.scenario in ['cancel', 'map-loss', 'localization-loss', 'backend-exit',
                                        'visual-loss']:
                self.run_fault_scenario()
            else:
                self.healthy_hold(self.args.hold_source_seconds)
            latest = self.vio['alignment_status']
            expected_fault = self.args.scenario in [
                'map-loss', 'localization-loss', 'backend-exit', 'visual-loss']
            if latest.state != 2 and not expected_fault:
                raise RuntimeError('alignment not estimating after dynamic observation')
            after = [r for r in self.vio_samples if self.first_estimating is not None and
                     r['receipt'] >= self.first_estimating]
            if not expected_fault and any(r.get('state', 0) >= 3 for r in after):
                raise RuntimeError('dynamic VIO/alignment became stale or latched')
            poses = [r for r in after if r['stream'] == 'pose']
            if not expected_fault and (len(poses) < 100 or
                                       (poses[-1]['stamp_ns']-poses[0]['stamp_ns'])/1e9 < 10):
                raise RuntimeError('insufficient actual dynamic VIO pose coverage')
            self.report['dynamic_poses'] = len(poses)
            if self.args.cpu_workers:
                if any(process.poll() is not None for process in self.pressure_workers):
                    raise RuntimeError('bounded CPU pressure ended before navigation completion')
                self.report['cpu_pressure_covered_navigation'] = True
        except Exception as error:
            failure = error
        # Planned simulation landing is also performed after a failed observation.
        # Its success never converts the failed VIO observation into a pass.
        state = self.state()
        if state is not None and state['armed']:
            land = self.send('/land', Land, Land.Goal())
            self.finish(land, seconds=120)
        self.wait(lambda: self.ready(False), 20)
        self.report['final_disarmed_state'] = self.state()
        if failure is not None:
            raise failure
        if self.args.scenario == 'clock-reset':
            self.reset_clock()

    def navigation(self, distance):
        if self.args.scenario != 'obstacle':
            return super().navigation(distance)
        self.stable_hover()
        state = self.state()['position']
        return self.navigation_target([state[0]+distance,
                                       state[1]+self.args.obstacle_lateral_offset, state[2]])

    def navigation_target(self, target):
        self.stable_hover()
        index = len(self.report.get('navigation_targets', []))
        self.navigation_snapshot(target, f'navigation-input-{index}.json')
        self.report.setdefault('navigation_targets', []).append(target)
        return self.send('/navigate', NavigateTo, NavigateTo.Goal(
            target_x=target[0], target_y=target[1], target_z=target[2], max_speed=.6))

    def navigation_snapshot(self, target, filename):
        grid = self.latest['/occupancy_grid']
        pose = self.vio['aligned_pose']
        snapshot = dict(
            frame=grid.header.frame_id,
            stamp_ns=grid.header.stamp.sec*10**9+grid.header.stamp.nanosec,
            width=grid.info.width, height=grid.info.height, resolution=grid.info.resolution,
            origin=[grid.info.origin.position.x, grid.info.origin.position.y],
            data=list(grid.data), fcu=self.state(), target=target,
            vio_position=[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
        (self.directory/filename).write_text(json.dumps(snapshot)+'\n')

    def finish(self, active, *args, **kwargs):
        try:
            return super().finish(active, *args, **kwargs)
        except Exception:
            if self.args.scenario == 'obstacle' and self.report.get('navigation_targets'):
                try:
                    if not (self.directory/'navigation-failure-input.json').exists():
                        self.navigation_snapshot(self.report['navigation_targets'][-1],
                                                 'navigation-failure-input.json')
                except Exception as snapshot_error:
                    self.report['failure_snapshot_error'] = repr(snapshot_error)
            raise

    def initialization_waypoint(self, target):
        """A measured, explicitly requested FCU manoeuvre, never an estimator reset."""
        action = self.send('/goto_waypoint', navigation.GoToWaypoint,
                           navigation.GoToWaypoint.Goal(
                               target_x=target[0], target_y=target[1], target_z=target[2],
                               target_yaw=target[3], max_speed=.6, acceptance_radius=.1))
        self.finish(action, seconds=60)
        self.stable_hover()

    def spawn_feature_board(self):
        """Explicit physical scene fixture; sensor messages remain untouched."""
        if self.state()['armed']:
            raise RuntimeError('feature board must be inserted before arming')
        origin = self.scene_origin()
        center = [origin[0]+getattr(self.args, 'feature_board_distance', 4.), origin[1], 1.6]
        visuals = []
        for row in range(6):
            for column in range(8):
                color = '.03 .03 .03 1' if (row+column) % 2 else '.95 .95 .95 1'
                visuals.append(
                    f'<visual name="tile_{row}_{column}">'
                    f'<pose>-.022 {(column-3.5)*.4} {(row-2.5)*.4} 0 0 0</pose>'
                    '<geometry><box><size>.004 .4 .4</size></box></geometry>'
                    f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse>'
                    '</material></visual>')
        model = self.directory/'visual-feature-board.sdf'
        model.write_text(
            '<sdf version="1.6"><model name="vio_feature_board"><static>true</static>'
            '<link name="board"><collision name="backing"><geometry><box>'
            '<size>.04 3.2 2.4</size></box></geometry></collision>'
            + ''.join(visuals) + '</link></model></sdf>')
        command = ['gz', 'model', '--spawn-file='+str(model),
                   '--model-name=vio_feature_board', '-x', str(center[0]),
                   '-y', str(center[1]), '-z', str(center[2])]
        result = subprocess.run(command, env=self.environment, capture_output=True,
                                text=True, timeout=10)
        if result.returncode:
            raise RuntimeError('native Gazebo feature board insertion failed')
        self.report['visual_feature_board'] = dict(
            variant='explicit physical checkerboard in otherwise unchanged registered world',
            native_witness=self.model_witness('vio_feature_board', center,
                                              require_collision=True),
            sdf_sha256=hashlib.sha256(model.read_bytes()).hexdigest(), command=command)
        # Native model commands block this observer, not the running guard.
        # Drain discovery callbacks and establish an actual static image history
        # before inspecting receipt freshness or requesting motion initialization.
        start = self.node.get_clock().now().nanoseconds

        def settled():
            status = self.vio.get('status')
            return self.startup_ready() and status is not None and \
                time.monotonic()-self.vio_receipts.get('status', 0) <= .5 and \
                self.node.get_clock().now().nanoseconds-start >= 3e9

        self.wait(settled, 30)

    def spawn_obstacle(self):
        """A real Gazebo collision/visual object, independent of all ROS map inputs."""
        if self.state()['armed']:
            raise RuntimeError('obstacle variant requires a disarmed reviewed scene')
        origin = self.scene_origin()
        distance = getattr(self.args, 'obstacle_distance', 3.0)
        center = [origin[0]+distance/2, origin[1]+self.args.obstacle_lateral_offset/2, 1.2]
        model = self.directory/'navigation-obstacle.sdf'
        model.write_text('''<sdf version="1.6"><model name="vio_navigation_obstacle">
<static>true</static><link name="post">
<collision name="collision"><geometry><cylinder><radius>0.1</radius><length>2.4</length>
</cylinder></geometry></collision>
<visual name="visual"><geometry><cylinder><radius>0.1</radius><length>2.4</length>
</cylinder></geometry><material><ambient>0.15 0.3 0.6 1</ambient>
<diffuse>0.15 0.3 0.6 1</diffuse></material></visual></link></model></sdf>''')
        command = ['gz', 'model', '--spawn-file='+str(model),
                   '--model-name=vio_navigation_obstacle',
                   '-x', str(center[0]), '-y', str(center[1]), '-z', str(center[2])]
        result = subprocess.run(command, env=self.environment, capture_output=True,
                                text=True, timeout=10)
        self.report['obstacle'] = dict(
            center_world=center, radius_m=.1, height_m=2.4,
            required_horizontal_model_center_clearance_m=.7,
            nominal_route_direction=[distance, self.args.obstacle_lateral_offset],
            sdf_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
            command=command, returncode=result.returncode,
            first_source_ns=self.node.get_clock().now().nanoseconds,
            variant='controlled real visual/collision insertion into unchanged registered world')
        if result.returncode:
            raise RuntimeError('native Gazebo obstacle insertion failed')
        self.report['obstacle']['native_witness'] = self.model_witness(
            'vio_navigation_obstacle', center, require_collision=True)
        first = self.node.get_clock().now().nanoseconds
        self.wait(lambda: self.node.get_clock().now().nanoseconds-first >= 3e9 and
                  self.startup_ready(), 30)

    def scene_origin(self):
        if self.args.scene not in ['primitives_sparse_seed42_v1', 'office_willowgarage_v1',
                                   'forest_pine_v1']:
            raise RuntimeError('fault/obstacle geometry requires a reviewed scene')
        path = Path(get_package_share_directory('easyfly_gazebo_worlds'))/(
            'scenes/'+self.args.scene+'.yaml')
        record = yaml.safe_load(path.read_text())
        if record['spawn']['yaw'] != 0.:
            raise RuntimeError('this injection geometry requires the reviewed zero-yaw spawn')
        self.report['injection_scene_manifest_sha256'] = hashlib.sha256(
            path.read_bytes()).hexdigest()
        return record['spawn']['position']

    def model_witness(self, name, expected_position, require_collision=False):
        from ament_index_python.packages import get_package_prefix
        query = str(Path(get_package_prefix('easyfly_gazebo_plugins')) /
                    'lib/easyfly_gazebo_plugins/easyfly_model_query')
        info = subprocess.run([query, name, '--info'],
                              env=self.environment, capture_output=True, text=True, timeout=10)
        pose = subprocess.run([query, name, '--pose'],
                              env=self.environment, capture_output=True, text=True, timeout=10)
        (self.directory/(name+'.native-info.txt')).write_text(info.stdout+info.stderr)
        (self.directory/(name+'.native-pose.txt')).write_text(pose.stdout+pose.stderr)
        if info.returncode or pose.returncode or name not in info.stdout or \
                (require_collision and 'collision {' not in info.stdout):
            raise RuntimeError('native injected model identity/collision was not observed')
        coordinates = [float(value) for value in pose.stdout.strip().split()]
        if len(coordinates) != 6 or math.dist(coordinates[:3], expected_position) > .02:
            raise RuntimeError('native injected model pose differs from requested geometry')
        return dict(model=name, actual_world_pose=coordinates,
                    collision_observed='collision {' in info.stdout,
                    info_sha256=hashlib.sha256(info.stdout.encode()).hexdigest())

    def set_view_heading(self):
        state = self.latest['/uav_state']
        desired = state.yaw+math.radians(self.args.heading_degrees)
        desired = math.atan2(math.sin(desired), math.cos(desired))
        goal = navigation.GoToWaypoint.Goal(
            target_x=state.position_x, target_y=state.position_y, target_z=state.position_z,
            target_yaw=desired, max_speed=.2, acceptance_radius=.1)
        action = self.send('/goto_waypoint', navigation.GoToWaypoint, goal)
        self.finish(action, seconds=30)
        self.wait(lambda: abs(math.atan2(
            math.sin(self.latest['/uav_state'].yaw-desired),
            math.cos(self.latest['/uav_state'].yaw-desired))) <= .05 and self.hover(), 30)
        self.stable_hover()
        self.report['view_heading'] = dict(
            requested_offset_degrees=self.args.heading_degrees, target_yaw_rad=desired,
            measured_fcu_yaw_rad=self.latest['/uav_state'].yaw,
            purpose='explicit real flight heading toward scene features, retained by yaw policy')

    def healthy_hold(self, seconds):
        start = self.node.get_clock().now().nanoseconds

        def complete():
            now = time.monotonic()
            clock = self.node.get_clock().now().nanoseconds
            keys = ['status', 'alignment_status']
            if self.args.scenario != 'dynamic':
                keys.append('navigation_status')
            for key in keys:
                msg = self.vio.get(key)
                if msg is None or msg.state != 2 or now-self.vio_receipts.get(key, 0) > .5:
                    raise RuntimeError('localization invalid/stopped during hold: '+key)
                stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
                if not -.2e9 <= clock-stamp <= .5e9:
                    raise RuntimeError('localization source time stale during hold: '+key)
            return clock-start >= seconds*1e9

        self.wait(complete, seconds*8+30)

    def start_pressure(self):
        self.report['cpu_pressure'] = dict(
            started_steady=time.monotonic(), workers=self.args.cpu_workers,
            allowed_cpus=sorted(os.sched_getaffinity(0)),
            worker_bound_wall_seconds=300, duty_cycle=.75)
        for index in range(self.args.cpu_workers):
            log = (self.directory/f'cpu-worker-{index}.log').open('x')
            process = subprocess.Popen([
                '/usr/bin/python3', str(ROOT/'tools/algorithms/probes/bounded_cpu_load.py'),
                '--seconds', '300'], env=self.environment, stdout=log, stderr=subprocess.STDOUT)
            self.pressure_logs.append(log)
            self.pressure_workers.append(process)
        self.report['cpu_pressure']['pids'] = [p.pid for p in self.pressure_workers]

    def configure_simulated_battery(self):
        candidates = [(pid, identity) for pid, identity in self.owned.items()
                      if identity['executable'] == self.report['staging']['binary']]
        if len(candidates) != 1 or navigation.children(self.process.pid).get(
                candidates[0][0]) != candidates[0][1] or self.state()['armed']:
            raise RuntimeError('refuse battery configuration without an owned disarmed simulator')
        get = self.node.create_client(GetParameters, '/mavros/param/get_parameters')
        set_values = self.node.create_client(SetParameters, '/mavros/param/set_parameters')
        self.wait(lambda: get.service_is_ready() and set_values.service_is_ready(), 8)
        request = GetParameters.Request(names=['SIM_BAT_DRAIN'])
        before = get.call_async(request)
        self.wait(before.done, 5)
        self.report['simulated_battery'] = dict(
            drain_seconds=self.args.battery_drain_seconds, previous=[
                dict(type=v.type, real=v.double_value) for v in before.result().values],
            owned_px4_pid=candidates[0][0], battery_failsafe_thresholds_changed=False)
        future = set_values.call_async(SetParameters.Request(parameters=[ParameterMessage(
            name='SIM_BAT_DRAIN', value=ParameterValue(
                type=3, double_value=float(self.args.battery_drain_seconds)))]))
        self.wait(future.done, 8)
        if len(future.result().results) != 1 or not future.result().results[0].successful:
            raise RuntimeError('native simulated battery model parameter not confirmed')
        after = get.call_async(request)
        self.wait(after.done, 5)
        values = after.result().values
        if len(values) != 1 or values[0].type != 3 or abs(
                values[0].double_value-self.args.battery_drain_seconds) > .01:
            raise RuntimeError('simulated battery model readback mismatch')
        self.report['simulated_battery']['readback_seconds'] = values[0].double_value

    def reset_clock(self):
        event = dict(scenario='actual_gazebo_clock_reset', steady=time.monotonic(),
                     clock_before_ns=self.node.get_clock().now().nanoseconds,
                     disarmed_state=self.state())
        self.report['injection'] = event
        result = subprocess.run(['gz', 'world', '--reset-time'], env=self.environment,
                                capture_output=True, text=True, timeout=10)
        event.update(command_returncode=result.returncode,
                     stdout=result.stdout, stderr=result.stderr)
        if result.returncode:
            raise RuntimeError('native Gazebo reset-time command failed')
        self.wait(lambda: all(self.vio.get(k) is not None and self.vio[k].state >= 4
                              for k in ['status', 'alignment_status', 'navigation_status']), 12)

        def invalid_map_protocol():
            grid = self.latest.get('/occupancy_grid')
            cloud = self.latest.get('/pointcloud_nonground')
            # Existing semantic_map contract: retain dimensions, publish zero
            # source stamp and all UNKNOWN cells, plus an empty trusted cloud.
            return grid is not None and grid.header.stamp.sec == 0 and \
                grid.header.stamp.nanosec == 0 and len(grid.data) == \
                grid.info.width*grid.info.height and bool(grid.data) and \
                all(cell == -1 for cell in grid.data) and cloud is not None and cloud.width == 0

        self.wait(invalid_map_protocol, 5)
        event['clock_after_ns'] = self.node.get_clock().now().nanoseconds
        if event['clock_after_ns'] >= event['clock_before_ns']:
            raise RuntimeError('native simulation clock rollback not observed')
        event['all_localization_consumers_latched'] = True
        event['map_invalidated'] = True
        event['map_invalidation_protocol'] = 'zero stamp, all UNKNOWN cells, empty trusted cloud'
        client = navigation.ActionClient(self.node, NavigateTo, '/navigate')
        self.clients.append(client)
        self.wait(client.server_is_ready, 5)
        future = client.send_goal_async(NavigateTo.Goal(
            target_x=1., target_y=0., target_z=1.5, max_speed=.4))
        self.wait(future.done, 5)
        event['new_goal_rejected'] = not future.result().accepted
        if not event['new_goal_rejected']:
            raise RuntimeError('clock-fault navigation request was admitted')

    def run_fault_scenario(self):
        active = self.navigation(1.5)
        self.wait(lambda: navigation.trajectory_stream_observed(
            self.report['streams'], time.monotonic(), self.node.get_clock().now().nanoseconds), 25)
        if active['result'].done():
            raise RuntimeError('navigation ended before the intended injection')
        scenario = self.args.scenario
        event = dict(scenario=scenario, steady=time.monotonic(), state=self.state(),
                     goal_uuid=active['entry']['goal_uuid'])
        self.report['injection'] = event
        paused = None
        try:
            if scenario == 'cancel':
                self.cancel_exact(active)
            elif scenario == 'visual-loss':
                event['camera_forwarded_before'] = self.vio['status'].camera_forwarded
                origin = self.scene_origin()
                panel = self.directory/'camera-occluder.sdf'
                panel.write_text('''<sdf version="1.6"><model name="vio_camera_occluder">
<static>true</static><pose>0.9 0 1.8 0 0 0</pose><link name="panel"><visual name="blackout">
<cast_shadows>false</cast_shadows><geometry><box><size>0.02 10 10</size></box></geometry>
<material><lighting>false</lighting><ambient>0.02 0.02 0.02 1</ambient>
<diffuse>0.02 0.02 0.02 1</diffuse><emissive>0.02 0.02 0.02 1</emissive></material>
</visual></link></model></sdf>''')
                command = ['gz', 'model', '--spawn-file='+str(panel),
                           '--model-name=vio_camera_occluder',
                           '-x', str(origin[0]+.9), '-y', str(origin[1]), '-z', '1.8']
                result = subprocess.run(command, env=self.environment, capture_output=True,
                                        text=True, timeout=10)
                event['occluder_command_returncode'] = result.returncode
                event['occluder_command'] = command
                event['occluder_sdf_sha256'] = hashlib.sha256(panel.read_bytes()).hexdigest()
                if result.returncode:
                    raise RuntimeError('physical render occluder spawn failed')
                event['occluder_native_witness'] = self.model_witness(
                    'vio_camera_occluder', [origin[0]+.9, origin[1], 1.8])
                self.wait(lambda: self.tracking is not None and
                          self.tracking['tracked_features'] < 8, 8)
                event['observed_lost_tracks'] = self.tracking
            else:
                suffix = ('/semantic_map_node' if scenario == 'map-loss'
                          else '/run_subscribe_msckf')
                candidates = [(p, d) for p, d in self.owned.items()
                              if d['executable'].endswith(suffix)]
                if len(candidates) != 1:
                    raise RuntimeError('unique owned fault target unavailable')
                pid, identity = candidates[0]
                if navigation.children(self.process.pid).get(pid) != identity:
                    raise RuntimeError('fault target process identity changed')
                event['pid'] = pid
                if scenario == 'backend-exit':
                    os.kill(pid, signal.SIGINT)
                else:
                    os.kill(pid, signal.SIGSTOP)
                    paused = pid
            self.finish(active, expected=5 if scenario == 'cancel' else 6, seconds=30)
            event['terminal_steady'] = time.monotonic()
            event['drain_wall_s'] = event['terminal_steady']-event['steady']
            self.wait(lambda: navigation.takeover_streams_quiet(
                [r for r in self.report['streams'] if r['topic'] != '/uav_command' or
                 r.get('command_mode') == navigation.UAVCommand.TRAJECTORY],
                event['terminal_steady'], time.monotonic()), 5)
            event['actual_trajectory_stream_quiet'] = True
            if scenario == 'visual-loss':
                if self.vio['status'].camera_forwarded <= event['camera_forwarded_before']+2:
                    raise RuntimeError('camera input stopped; not an isolated visual-loss test')
                event['camera_input_continued'] = True
        finally:
            if paused is not None and navigation.children(self.process.pid).get(
                    paused) == self.owned[paused]:
                os.kill(paused, signal.SIGCONT)
                event['resumed_steady'] = time.monotonic()
        if scenario == 'cancel':
            recovery = self.navigation(.6)
            self.finish(recovery, seconds=90)
            self.report['recovery_navigation_error_m'] = math.dist(
                recovery['entry']['terminal_state']['position'],
                self.report['navigation_targets'][-1])
            if self.report['recovery_navigation_error_m'] > .11:
                raise RuntimeError('recovery actual navigation arrival not confirmed')

    def close(self):
        self.report['mapping_frames'] = self.tf_buffer.all_frames_as_yaml()
        self.report['mapping_input_counts'] = {
            k: sum(r['topic'] == k for r in self.mapping_inputs)
            for k in ['/livox/lidar', '/pointcloud_filtered']}
        (self.directory/'mapping-inputs.json').write_text(
            json.dumps(self.mapping_inputs, indent=2)+'\n')
        if self.truth is not None:
            if self.truth.poll() is None:
                self.truth.send_signal(signal.SIGINT)
                try:
                    self.truth.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    self.truth.kill()
                    self.truth.wait(timeout=5)
                    self.report['forced'] = True
            self.report['truth_returncode'] = self.truth.returncode
            self.truth_log.close()
        with (self.directory/'vio.jsonl').open('x') as stream:
            for row in self.vio_samples:
                stream.write(json.dumps(row)+'\n')
        self.report['vio_discarded'] = self.discarded
        (self.directory/'process-budget.json').write_text(json.dumps(dict(
            clock_ticks_per_second=os.sysconf('SC_CLK_TCK'), page_size=os.sysconf('SC_PAGE_SIZE'),
            samples=self.cpu_samples), indent=2)+'\n')
        super().close()
        pressure_clean = True
        for process, log in zip(self.pressure_workers, self.pressure_logs):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
                    self.report['forced'] = True
            pressure_clean = pressure_clean and process.returncode == 0
            log.close()
        self.report['cpu_pressure_workers_clean'] = pressure_clean
        # OpenVINS includes a startup observer and three coupled children;
        # verify actual process identities instead of the FAST-LIO count of 24.
        self.report['passed'] = (
            self.report['error'] is None and not self.report['forced'] and
            self.report.get('launch_returncode') == self.report.get('truth_returncode') == 0 and
            len(self.report['started_processes']) >= 20 and
            set(self.report['normal_exits']) == set(self.report['started_processes']) and
            not self.report['abnormal_lines'] and self.report['transport_drained'] and
            not self.report['leftovers'] and not self.discarded and pressure_clean and
            'final_disarmed_state' in self.report)
        (self.directory/'result.json').write_text(json.dumps(self.report, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--px4-dir', type=Path, required=True)
    parser.add_argument('--truth-executable', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=221)
    parser.add_argument('--gazebo-port', type=int, default=12621)
    parser.add_argument('--scene', default='primitives_sparse_seed42_v1')
    parser.add_argument('--scenario', choices=[
        'dynamic', 'map-bootstrap', 'mission', 'cancel', 'map-loss', 'localization-loss',
        'backend-exit', 'clock-reset', 'visual-loss', 'obstacle', 'long-run'],
        default='dynamic')
    parser.add_argument('--strategy', choices=['astar', 'ego', 'combined'], default='combined')
    parser.add_argument('--hold-source-seconds', type=float, default=12.)
    parser.add_argument('--no-input-audit', action='store_true')
    parser.add_argument('--battery-drain-seconds', type=float, default=60.)
    parser.add_argument('--cpu-workers', type=int, choices=[0, 1, 2], default=0)
    parser.add_argument('--managed-profile', action='store_true')
    parser.add_argument('--heading-degrees', type=float, default=0.)
    parser.add_argument('--bootstrap-climb-speed', type=float, default=.3)
    parser.add_argument('--initialization-translation', type=float, default=0.)
    parser.add_argument('--visual-feature-board', action='store_true')
    parser.add_argument('--obstacle-lateral-offset', type=float, default=0.)
    parser.add_argument('--execute-simulation', action='store_true', required=True)
    args = parser.parse_args()
    if not 1 <= args.domain <= 232 or not 1024 <= args.gazebo_port <= 65535:
        parser.error('explicit isolated domain and local Gazebo port required')
    if not 10 <= args.hold_source_seconds <= 600:
        parser.error('hold duration must be 10..600 source seconds')
    if not 60 <= args.battery_drain_seconds <= 86400:
        parser.error('native simulation battery duration must be 60..86400 seconds')
    if not math.isfinite(args.heading_degrees) or abs(args.heading_degrees) > 90:
        parser.error('explicit measured view heading must be within 90 degrees')
    if not math.isfinite(args.bootstrap_climb_speed) or not .3 <= args.bootstrap_climb_speed <= 1.:
        parser.error('initialization flight climb speed must be .3..1 m/s')
    if not math.isfinite(args.initialization_translation) or \
            not 0 <= args.initialization_translation <= .5:
        parser.error('explicit initialization displacement must be 0..0.5m')
    if not math.isfinite(args.obstacle_lateral_offset) or abs(args.obstacle_lateral_offset) > 2:
        parser.error('explicit obstacle route lateral offset must be within 2m')
    args.output_dir.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    probe = Probe(args)
    try:
        probe.run()
    except Exception as error:
        probe.report['error'] = repr(error)
    finally:
        probe.close()
    print(json.dumps({k: probe.report[k] for k in ['passed', 'error', 'forced']}, indent=2))
    return 0 if probe.report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

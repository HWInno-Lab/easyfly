#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Explicit, owned PX4/Gazebo ORB-SLAM3 acceptance; native truth is read-only."""
import hashlib
import argparse
import importlib.util
import json
import os
import math
import signal
import subprocess
import time
from pathlib import Path

SPEC = importlib.util.spec_from_file_location(
    'orb_flight_fixture', Path(__file__).with_name('fastlivo_simulation.py'))
fixture = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(fixture)
common = fixture.common
navigation = common.navigation
from diagnostic_msgs.msg import DiagnosticArray
from easyfly_msgs.msg import LocalizationStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image


class Probe(common.Probe):
    def __init__(self, args):
        from easyfly_fastdds_compat.runtime import activate

        middleware = activate(openvins=True)
        self.original_counts = dict(camera=0, depth=0)
        self.relays = []
        super().__init__(args)
        self.environment['LD_LIBRARY_PATH'] = os.environ['LD_LIBRARY_PATH']
        self.report.update(backend='orbslam3', scene=args.scene,
                           dds_compatibility=middleware)
        self.report.pop('openvins_provenance_sha256', None)
        self.runtime = self.directory / 'orb-runtime'
        self.runtime.mkdir()
        root = Path(__file__).resolve().parents[3]
        limits = root / 'artifacts/new-algorithms/new3-orbslam3-20261021/acceptance-limits.json'
        self.report['acceptance_limits'] = json.loads(limits.read_text())
        self.report['acceptance_limits_sha256'] = hashlib.sha256(limits.read_bytes()).hexdigest()
        self.report['orb_source_hashes'] = {}
        for directory in ['src/perception/p3_tracking/easyfly_orbslam3_integration',
                          'src/third_party/algorithms/localization/orb_slam3']:
            for source in (root / directory).rglob('*'):
                if not source.is_file() or '__pycache__' in source.parts:
                    continue
                relative = source.relative_to(root)
                data = source.read_bytes()
                self.report['orb_source_hashes'][str(relative)] = hashlib.sha256(data).hexdigest()
                target = self.directory / 'source-snapshot/orb' / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(data)
        self.report['orb_probe_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        from audit_fastlivo_bag import fingerprint
        self.hashlog = (self.directory / 'input-fingerprints.jsonl').open('x')
        audit_qos = QoSProfile(depth=64, reliability=ReliabilityPolicy.RELIABLE)
        self.report['input_observer_qos'] = dict(depth=64, reliability='RELIABLE',
                                                purpose='complete original/forwarded field audit')
        for kind, topic in [('rgb', '/camera/color/image_raw'),
                            ('depth', '/camera/aligned_depth_to_color/depth/image_raw'),
                            ('forwarded_rgb', '/orbslam3/input/rgb'),
                            ('forwarded_depth', '/orbslam3/input/depth')]:
            def image(message, k=kind):
                stamp, digest = fingerprint(message)
                self.hashlog.write(json.dumps(dict(kind=k, stamp_ns=stamp, sha256=digest,
                                                   receipt=time.monotonic())) + '\n')
            self.node.create_subscription(Image, topic, image, audit_qos)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic, cls in [('status', LocalizationStatus),
                           ('alignment_status', LocalizationStatus),
                           ('navigation_status', LocalizationStatus),
                           ('pose', PoseWithCovarianceStamped), ('aligned_pose', PoseStamped)]:
            self.node.create_subscription(
                cls, '/localization/orbslam3/' + topic,
                lambda m, k=topic: self.on_vio(k, m), qos if 'status' in topic else 100)
        self.node.create_subscription(
            DiagnosticArray, '/orbslam3/raw/visual_tracking', self.on_tracking, 32)
        for topic in ['map_status', 'atlas_events']:
            self.node.create_subscription(
                DiagnosticArray, '/orbslam3/raw/' + topic,
                lambda m, k=topic: self.on_map_event(k, m), 100)
        self.node.create_subscription(
            Image, '/camera/aligned_depth_to_color/depth/image_raw',
            lambda m: self.original_counts.__setitem__('depth', self.original_counts['depth'] + 1),
            qos_profile_sensor_data)
        for key in ['EASYFLY_GUARD_TAKE_TRACE', 'EASYFLY_GUARD_CLOCK_TRACE',
                    'EASYFLY_GUARD_FRESHNESS_TRACE']:
            self.environment.pop(key, None)

    configure_simulated_battery = fixture.Probe.configure_simulated_battery
    def scene_origin(self):
        if self.args.scene != 'gcopter_3d_lane_v1':
            return super().scene_origin()
        # The lane is an existing registered, zero-yaw native scene.
        from ament_index_python.packages import get_package_share_directory
        import yaml
        path = Path(get_package_share_directory('easyfly_gazebo_worlds')) / \
            'scenes/gcopter_3d_lane_v1.yaml'
        data = yaml.safe_load(path.read_text())
        if data['spawn']['yaw'] != 0.0:
            raise RuntimeError('qualified lane requires zero-yaw spawn')
        self.report['injection_scene_manifest_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
        return data['spawn']['position']

    def send(self, name, action_type, goal):
        if name == '/takeoff':
            # Keep all startup samples, but score dynamic continuity from the
            # actual takeoff request rather than a first disarmed pose that
            # may precede native fixture insertion and FCU startup settling.
            self.first_estimating = time.monotonic()
            self.report['dynamic_observation_start'] = dict(
                steady=self.first_estimating, source_ns=self.node.get_clock().now().nanoseconds,
                reason='actual takeoff request; preflight rows retained, all flight TTL/reset checks unchanged')
        return super().send(name, action_type, goal)

    def spawn_feature_board(self):
        super().spawn_feature_board()
        self.synchronize_fixture_observer()

    def synchronize_fixture_observer(self):
        # Native model insertion/witness subprocesses do not spin this observer.
        # Drain to a genuinely source-fresh status before starting the existing
        # three-second initialization window; never clear a backend reset.
        completed = time.monotonic()
        def synchronized():
            status = self.vio.get('status')
            if status is None or self.vio_receipts.get('status', 0) < completed:
                return False
            source = status.header.stamp.sec*10**9+status.header.stamp.nanosec
            age = self.node.get_clock().now().nanoseconds-source
            return status.state in (1, 2) and -.2e9 <= age <= .5e9 and \
                time.monotonic()-self.vio_receipts['status'] <= .5
        self.wait(synchronized, 10)
        self.report['fixture_observer_synchronized'] = dict(
            source_ns=self.node.get_clock().now().nanoseconds,
            receipt=self.vio_receipts['status'], model_insertion_completed=completed,
            reason='new callback and original source freshness after blocking native fixture setup')

    def on_image(self, message):
        self.original_counts['camera'] += 1
        return super().on_image(message)

    def on_map_event(self, kind, message):
        self.vio_samples.append(dict(
            stream=kind, receipt=time.monotonic(),
            stamp_ns=message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
            values=[{v.key: v.value for v in s.values} for s in message.status]))

    def on_vio(self, key, message):
        super().on_vio(key, message)
        if 'status' in key:
            self.vio_samples[-1]['covariance_available'] = message.covariance_available

    def navigation(self, distance):
        if self.args.scenario != 'map-loss':
            return super().navigation(distance)
        self.stable_hover()
        p = self.state()['position']
        target = [p[0] + distance, p[1], p[2]]
        self.navigation_snapshot(target, 'map-loss-navigation-input.json')
        self.report.setdefault('navigation_targets', []).append(target)
        # Preserve the production map timeout; keep a real task active past it.
        return self.send('/navigate', navigation.NavigateTo, navigation.NavigateTo.Goal(
            target_x=target[0], target_y=target[1], target_z=target[2], max_speed=.2))

    def run_fault_scenario(self):
        self.fault_active = None
        try:
            return self._run_fault_scenario()
        except Exception as error:
            self.report['fault_scenario_error'] = repr(error)
            active = self.fault_active
            if active is not None and not active['result'].done():
                try:
                    self.cancel_exact(active)
                    self.finish(active, expected=5, seconds=30)
                    self.report['failed_probe_navigation_cancelled_before_landing'] = True
                except Exception as cleanup_error:
                    self.report['failed_probe_navigation_cleanup_error'] = repr(cleanup_error)
            raise

    def _run_fault_scenario(self):
        scenario = self.args.scenario
        active = self.navigation(1.5)
        self.fault_active = active
        origin = self.state()['position']
        self.wait(lambda: math.dist(origin, self.state()['position']) >= .25, 25)
        if active['result'].done():
            raise RuntimeError('task finished before moving fault injection')
        event = dict(scenario=scenario, steady=time.monotonic(), state=self.state(),
                     goal_uuid=active['entry']['goal_uuid'],
                     input_counts_before=dict(self.original_counts))
        self.report['injection'] = event
        paused = None
        try:
            if scenario == 'cancel':
                self.cancel_exact(active)
            elif scenario == 'visual-loss':
                origin = self.scene_origin()
                panel = self.directory / 'camera-occluder.sdf'
                panel.write_text('''<sdf version="1.6"><model name="orb_camera_occluder">
<static>true</static><link name="panel"><visual name="blackout"><cast_shadows>false</cast_shadows>
<geometry><box><size>0.02 10 10</size></box></geometry><material><lighting>false</lighting>
<ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse>
<emissive>0.02 0.02 0.02 1</emissive></material></visual></link></model></sdf>''')
                command = ['gz', 'model', '--spawn-file=' + str(panel),
                           '--model-name=orb_camera_occluder', '-x', str(origin[0] + .9),
                           '-y', str(origin[1]), '-z', '1.8']
                reply = subprocess.run(command, env=self.environment, capture_output=True,
                                       text=True, timeout=10)
                if reply.returncode:
                    raise RuntimeError('physical native occluder insertion failed')
                event.update(occluder_sdf_sha256=hashlib.sha256(panel.read_bytes()).hexdigest(),
                             occluder_native_witness=self.model_witness(
                                 'orb_camera_occluder', [origin[0] + .9, origin[1], 1.8]))
            elif scenario == 'localization-loss':
                kind, process, _ = self.relays[0]
                if process.poll() is not None:
                    raise RuntimeError('sensor relay stopped before injection')
                event['sensor'] = kind
                process.send_signal(signal.SIGINT)
                process.wait(timeout=5)
            else:
                # Managed launch stages mapping/planning children after source
                # startup. Capture their actual current ownership, preserving
                # every earlier PID/start-time identity for final cleanup audit.
                self.refresh_owned_descendants()
                suffix = '/semantic_map_node' if scenario == 'map-loss' else '/orbslam3_rgbd'
                targets = [(p, d) for p, d in self.owned.items()
                           if d['executable'].endswith(suffix)]
                if len(targets) != 1:
                    raise RuntimeError('unique owned fault target unavailable')
                pid, identity = targets[0]
                if navigation.children(self.process.pid).get(pid) != identity:
                    raise RuntimeError('owned fault process identity changed')
                event['pid'] = pid
                os.kill(pid, signal.SIGINT if scenario == 'backend-exit' else signal.SIGSTOP)
                if scenario != 'backend-exit':
                    paused = pid
            self.finish(active, expected=5 if scenario == 'cancel' else 6, seconds=30)
            event['terminal_steady'] = time.monotonic()
            self.wait(lambda: navigation.takeover_streams_quiet(
                [r for r in self.report['streams'] if r['topic'] != '/uav_command' or
                 r.get('command_mode') == navigation.UAVCommand.TRAJECTORY],
                event['terminal_steady'], time.monotonic()), 5)
            event['actual_trajectory_stream_quiet'] = True
            event['input_counts_after'] = dict(self.original_counts)
            if scenario in ['visual-loss', 'localization-loss'] and any(self.original_counts[k] <=
                                               event['input_counts_before'][k] + 2
                                               for k in self.original_counts):
                raise RuntimeError('original RGB/depth did not continue during visual loss')
            if scenario in ['visual-loss', 'backend-exit', 'localization-loss']:
                # Admission must remain closed after an asynchronous source failure.
                state = self.state()['position']
                before = len(self.report['actions'])
                try:
                    self.send('/navigate', navigation.NavigateTo, navigation.NavigateTo.Goal(
                        target_x=state[0] + .3, target_y=state[1], target_z=state[2], max_speed=.6))
                except RuntimeError as exc:
                    if str(exc) != '/navigate rejected':
                        raise
                if len(self.report['actions']) != before + 1 or \
                        self.report['actions'][-1].get('accepted') is not False:
                    raise RuntimeError('new task admitted after localization failure')
                event['new_navigation_rejected'] = True
        finally:
            if paused is not None and navigation.children(self.process.pid).get(paused) == self.owned[paused]:
                os.kill(paused, signal.SIGCONT)
        if scenario == 'cancel':
            recovery = self.navigation(.6)
            self.finish(recovery, seconds=90)
            error = math.dist(recovery['entry']['terminal_state']['position'],
                              self.report['navigation_targets'][-1])
            self.report['recovery_navigation_error_m'] = error
            if error > .11:
                raise RuntimeError('navigation recovery arrival error')

    def launch_command(self):
        if self.args.scenario == 'localization-loss':
            kind = self.args.fault_kind
            log = (self.directory / (kind + '-relay.log')).open('x')
            process = subprocess.Popen([
                '/usr/bin/python3', str(Path(__file__).with_name('fastlivo_sensor_relay.py')),
                kind, str(self.directory / (kind + '-relay.jsonl'))],
                env=self.environment, stdout=log, stderr=subprocess.STDOUT)
            self.relays.append((kind, process, log))
            return ['ros2', 'launch', 'easyfly_px4_sitl', 'perception_sitl.launch.py',
                    'localization_backend:=orbslam3', 'orbslam3_navigation:=true',
                    'orbslam3_runtime_directory:=' + str(self.runtime),
                    'orbslam3_save_atlas:=' + str(self.runtime / 'atlas'),
                    'orbslam3_' + ('rgb' if kind == 'camera' else 'depth') + '_topic:=/acceptance/' + kind,
                    'scene:=' + self.args.scene, 'scene_profile:=navigation_dev',
                    'navigation_strategy:=' + self.args.strategy,
                    'headless:=true', 'rviz:=false', 'detection_use_gpu:=false']
        return ['ros2', 'launch', 'easyfly_sim_launcher', 'profile.launch.py',
                'profile:=navigation_dev', 'orbslam3_navigation:=true',
                'orbslam3_runtime_directory:=' + str(self.runtime),
                'orbslam3_save_atlas:=' + str(self.runtime / 'atlas'),
                'algorithms_json:=' + json.dumps(dict(
                    slam='orbslam3', strategy=self.args.strategy, provider='cpu')),
                'scene:=' + self.args.scene, 'headless:=true', 'rviz:=false']

    def start(self):
        navigation.Probe.start(self)
        candidates = [(p, d) for p, d in self.owned.items()
                      if d['executable'].endswith('/orbslam3_rgbd')]
        if len(candidates) != 1:
            raise RuntimeError('unique owned real ORB-SLAM3 required')
        pid, entry = candidates[0]
        self.report['orb_runtime'] = dict(
            pid=pid, **entry,
            executable_sha256=hashlib.sha256(Path(entry['executable']).read_bytes()).hexdigest())
        libraries = {}
        for line in Path('/proc', str(pid), 'maps').read_text().splitlines():
            if any(s in line for s in ['libeasyfly_orbslam3', 'libpangolin']):
                path = Path(line.split()[-1])
                libraries[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        self.report['orb_runtime_libraries_sha256'] = libraries
        self.report['global_tf_publishers'] = [
            dict(name=p.node_name, namespace=p.node_namespace)
            for p in self.node.get_publishers_info_by_topic('/tf')]
        if any(p['name'] == 'orbslam3_rgbd' for p in self.report['global_tf_publishers']):
            raise RuntimeError('raw ORB-SLAM3 TF leaked into control tree')

    def refresh_owned_descendants(self):
        if self.process is None or self.process.poll() is not None:
            return
        current = navigation.children(self.process.pid)
        for pid in set(current).intersection(self.owned):
            if current[pid] != self.owned[pid]:
                raise RuntimeError('owned descendant identity changed')
        late = {pid:data for pid,data in current.items() if pid not in self.owned}
        self.owned.update(late)
        self.report.setdefault('late_owned_processes', {}).update(late)

    def close(self):
        self.refresh_owned_descendants()
        self.report['original_rgbd_receipts'] = self.original_counts
        super().close()
        self.hashlog.close()
        final = self.runtime / 'atlas-final.json'
        self.report['atlas_final'] = json.loads(final.read_text()) if final.exists() else None
        console = (self.directory / 'console.log').read_text()
        self.report['launch_escalated'] = 'escalating to' in console
        self.report['passed'] = (self.report['passed'] and not self.report['launch_escalated']
                                 and bool(self.report['atlas_final'])
                                 and self.report['atlas_final'].get('workers_joined', False))
        self.report['relay_exits'] = {}
        for kind, process, log in self.relays:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.report['forced'] = True
                    process.kill()
                    process.wait(timeout=5)
            self.report['relay_exits'][kind] = process.returncode
            log.close()
        self.report['passed'] = self.report['passed'] and not self.report['forced'] and all(
            code == 0 for code in self.report['relay_exits'].values())
        (self.directory / 'result.json').write_text(json.dumps(self.report, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--px4-dir', type=Path, required=True)
    parser.add_argument('--truth-executable', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=223)
    parser.add_argument('--gazebo-port', type=int, default=12623)
    parser.add_argument('--scene', default='primitives_sparse_seed42_v1')
    parser.add_argument('--scenario', choices=['mission', 'long-run', 'cancel', 'map-loss',
                        'backend-exit', 'localization-loss', 'visual-loss', 'clock-reset'], default='mission')
    parser.add_argument('--fault-kind', choices=['camera', 'depth'], default='camera')
    parser.add_argument('--strategy', choices=['astar', 'ego', 'combined'], default='combined')
    parser.add_argument('--hold-source-seconds', type=float, default=12.)
    parser.add_argument('--managed-profile', action='store_true')
    parser.add_argument('--visual-feature-board', action='store_true')
    parser.add_argument('--execute-simulation', action='store_true', required=True)
    args = parser.parse_args()
    if not 1 <= args.domain <= 232 or not 1024 <= args.gazebo_port <= 65535 or not 10 <= args.hold_source_seconds <= 600:
        parser.error('invalid isolation or duration')
    for key, value in dict(battery_drain_seconds=1800., cpu_workers=0, no_input_audit=True,
                           bootstrap_climb_speed=.3, initialization_translation=0.,
                           heading_degrees=0., feature_board_distance=4., obstacle_lateral_offset=0.).items():
        setattr(args, key, value)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    probe = Probe(args)
    try:
        probe.run()
    except Exception as exc:
        probe.report['error'] = repr(exc)
    finally:
        probe.close()
    print(json.dumps({k: probe.report[k] for k in ['passed', 'error', 'forced']}, indent=2))
    return 0 if probe.report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

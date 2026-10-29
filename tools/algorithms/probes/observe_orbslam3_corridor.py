#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Live native RGB-D/IMU loop observation; flight control stays on FAST-LIO.

A fixed maximum of two physical circuits revisits all views after initialization.
An actual completed loop may end acquisition at the next safe waypoint. No pose
from ORB is consumed by the flight controller and no truth enters the estimator.
"""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from audit_fastlivo_bag import fingerprint

spec = importlib.util.spec_from_file_location('orb_observed_corridor_fixture', Path(__file__).with_name('capture_orbslam3_inertial_corridor.py'))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)


class ObservedCorridor(fixture.InertialCorridor, fixture.base.base.orb.Probe):
    def __init__(self, args):
        self.observer = None
        self.observer_required = False
        self.observer_budget = []; self.observer_sample_due = 0.
        super().__init__(args)
        self.report.update(backend='orbslam3', control_backend='fast_lio', sensor_mode='rgbd_imu',
                           scope='live_observation_not_orb_control', maximum_physical_circuits=2,
                           acquisition_speed_mps=.2, capture_goal_wall_timeout_s=120)
        self.report['motion_fixture']['acquisition_speed_mps'] = .2
        data = Path(__file__).read_bytes()
        (self.directory/'observer-capture-source.py').write_bytes(data)
        self.report['observer_capture_sha256'] = hashlib.sha256(data).hexdigest()
        # Retain every native output, including the pose rejected by continuity
        # isolation. Public poses alone cannot audit the actual discontinuity.
        self.node.create_subscription(
            PoseStamped, '/orbslam3/raw/body_pose',
            lambda message: self.on_vio('raw_body_pose', message),
            QoSProfile(depth=2000, reliability=ReliabilityPolicy.RELIABLE))
        qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.RELIABLE)
        for kind, topic in [('imu', '/livox/imu'), ('forwarded_imu', '/orbslam3/input/imu')]:
            def sample(message, k=kind):
                stamp, digest = fingerprint(message)
                self.hashlog.write(json.dumps(dict(kind=k, stamp_ns=stamp, sha256=digest,
                                                   receipt=time.monotonic()))+'\n')
            self.node.create_subscription(Imu, topic, sample, qos)

    def on_image(self, message):
        self.native_rgb = message
        # The acquisition base retains a witness only. Also perform the real
        # ORB observer accounting and the shared original-image observation.
        return fixture.base.base.orb.Probe.on_image(self, message)

    def on_vio(self, key, message):
        super().on_vio(key, message)
        if key == 'pose':
            self.vio_samples[-1]['stream'] = 'local_body_pose'

    def on_map_event(self, kind, message):
        super().on_map_event(kind, message)
        row = self.vio_samples[-1]
        row['stream'] = 'atlas_event' if kind == 'atlas_events' else 'map_lifecycle'
        row['values'] = row['values'][0]

    def completed_loops(self):
        for row in reversed(self.vio_samples):
            if row['stream'] == 'atlas_event':
                return int(row['values'].get('loop_closures', 0))
        return 0

    def waypoint_speed(self, label):
        return .2

    def waypoint_timeout(self, label):
        # Wall-clock acquisition budget follows the slower actual flight.
        # Source/receipt freshness and all navigation limits remain unchanged.
        return 120

    def route_waypoints(self, origin, yaw0):
        for circuit in range(2):
            for label, target, heading in super().route_waypoints(origin, yaw0):
                if self.completed_loops() > 0:
                    self.report['acquisition_completed_on_actual_loop'] = True
                    return
                yield f'circuit-{circuit+1}-{label}', target, heading

    def start_recorder(self):
        # Parent calls this while native physics is paused and disarmed.
        super().start_recorder()
        executable = Path(get_package_prefix('easyfly_orbslam3_integration'))/'lib/easyfly_orbslam3_integration/orbslam3_rgbd'
        share = Path(get_package_share_directory('easyfly_orbslam3_integration'))
        core = Path(get_package_share_directory('easyfly_orbslam3_core'))
        command = [str(executable), '--ros-args']
        params = dict(use_sim_time='true', sensor_mode='rgbd_imu', runtime_directory=str(self.runtime),
                      settings_template=str(self.observer_settings_template(share)),
                      vocabulary=str(core/'vocabulary/ORBvoc.txt'), save_atlas=str(self.runtime/'atlas'))
        for key, value in params.items(): command += ['-p', key+':='+value]
        self.observer_log = (self.directory/'orb-observer.log').open('x')
        self.observer = subprocess.Popen(command, env=self.environment, stdout=self.observer_log,
                                         stderr=subprocess.STDOUT)
        self.observer_required = True
        proc = Path('/proc')/str(self.observer.pid)
        self.observer_identity = (proc/'stat').read_text().rsplit(')', 1)[1].split()[19]
        self.wait(lambda: self.node.count_publishers('/localization/orbslam3/status') == 1, 60)
        actual = (proc/'exe').resolve(strict=True)
        if actual != executable.resolve(strict=True): raise RuntimeError('unexpected observer executable')
        libraries = sorted(set(line.split()[-1] for line in (proc/'maps').read_text().splitlines()
                               if any(n in line for n in ['libeasyfly_orbslam3', 'libpangolin'])))
        self.report['observer_runtime'] = dict(pid=self.observer.pid, start_ticks=self.observer_identity,
            executable=str(actual), executable_sha256=hashlib.sha256(actual.read_bytes()).hexdigest(),
            libraries_sha256={p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in libraries},
            command=command, enable_navigation=False, alignment_node_started=False)
        if any(p.node_name == 'orbslam3_rgbd' for p in self.node.get_publishers_info_by_topic('/tf')):
            raise RuntimeError('observer leaked raw global TF')
        for topic in ['/localization/orbslam3/navigation_state', '/localization/orbslam3/aligned_pose']:
            if self.node.count_publishers(topic):
                raise RuntimeError('an ORB control/alignment bridge is active in observation-only capture')
        self.report['observer_control_bridge_publishers'] = 0

    def observer_settings_template(self, share):
        return share/'config/gazebo_rgbd_imu.yaml'

    def wait(self, predicate, seconds):
        def healthy():
            if self.observer_required and self.observer is not None:
                if self.observer.poll() is not None:
                    raise RuntimeError('owned ORB observer exited during acquisition')
                if time.monotonic()-self.observer_sample_due >= 1.:
                    self.observer_sample_due = time.monotonic()
                    proc = Path('/proc')/str(self.observer.pid)
                    fields = (proc/'stat').read_text().rsplit(')', 1)[1].split()
                    if fields[19] != self.observer_identity: raise RuntimeError('observer identity changed')
                    self.observer_budget.append(dict(steady=self.observer_sample_due,
                        source_ns=self.node.get_clock().now().nanoseconds,
                        cpu_seconds=(int(fields[11])+int(fields[12]))/os.sysconf('SC_CLK_TCK'),
                        rss_bytes=int(fields[21])*os.sysconf('SC_PAGE_SIZE'),threads=int(fields[17])))
            return predicate()
        return super().wait(healthy, seconds)

    def close(self):
        self.observer_required = False
        observer_error = None
        if self.observer is not None:
            if self.observer.poll() is None:
                proc = Path('/proc')/str(self.observer.pid)
                if (proc/'stat').read_text().rsplit(')', 1)[1].split()[19] != self.observer_identity:
                    raise RuntimeError('owned observer identity changed before shutdown')
                self.observer.send_signal(signal.SIGINT)
                try: self.observer.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    # Preserve failure and let normal parent teardown proceed;
                    # no success can be inferred from a stalled observer.
                    observer_error = 'observer shutdown deadline exceeded'
                    self.report['forced'] = True
                    self.observer.terminate(); self.observer.wait(timeout=10)
            self.observer_log.close()
            self.report['observer_returncode'] = self.observer.returncode
        super().close()
        self.report['passed'] = self.report['passed'] and self.observer is not None and \
            self.observer.returncode == 0 and observer_error is None
        self.report['observer_error'] = observer_error
        (self.directory/'orb-observer-resources.json').write_text(json.dumps(self.observer_budget,indent=2)+'\n')
        self.report['actual_completed_loops'] = self.completed_loops()
        (self.directory/'result.json').write_text(json.dumps(self.report, indent=2)+'\n')


if __name__ == '__main__':
    fixture.base.base.orb.fixture.Probe = ObservedCorridor
    raise SystemExit(fixture.base.base.orb.fixture.main())

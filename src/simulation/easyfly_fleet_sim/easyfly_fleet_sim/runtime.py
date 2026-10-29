# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Own one shared Gazebo world and all explicitly isolated native PX4 vehicle processes."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time
import uuid

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from easyfly_precision_landing.simulation_setup import prepare_staging
from easyfly_px4_sitl.hrt_compatibility import select_px4_directory
from easyfly_px4_sitl.imu_compatibility import require_native_imu
from easyfly_px4_sitl.telemetry import configured_heartbeat, configured_stream, configure_owned

from .configuration import load, new_directory, Reservation
from .models import derive


def executable(package, name):
    """Resolve only installed executables from the selected overlay."""
    return str(Path(get_package_prefix(package)) / 'lib' / package / name)


class Child:
    """Drain bounded native output and retain exact parent/process-group ownership."""

    def __init__(self, name, command, env, directory, cpus, cwd=None):
        self.name, self.command = name, command
        self.directory = directory
        self.tail = bytearray()
        self.lock = threading.Lock()
        self.error = ''
        self.native_failure = ''
        self.process = subprocess.Popen(['taskset', '-c', ','.join(map(str, cpus)), *command],
                                        env=env, cwd=cwd, stdin=subprocess.DEVNULL,
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        start_new_session=True)
        self.pid = self.process.pid
        self.start_ticks = self.identity()[1]
        self.thread = threading.Thread(target=self.drain, name='fleet-log-' + name)
        self.thread.start()

    def identity(self):
        """Require the same owned PID start time and process group before signaling it."""
        fields = (Path('/proc') / str(self.pid) / 'stat').read_text().rsplit(')', 1)[1].split()
        return int(fields[2]), fields[19]

    def drain(self):
        """Keep two 8 MiB log files and a 128 KiB tail without blocking a noisy native child."""
        current = self.directory / (self.name + '.log')
        stream, size = None, 0
        try:
            while True:
                data = os.read(self.process.stdout.fileno(), 4096)
                if not data:
                    break
                with self.lock:
                    self.tail.extend(data)
                    del self.tail[:-131072]
                    native_output = self.tail.decode(errors='replace')
                    failure = re.search(
                        r'process has died[^\n]*|sending signal [^\n]*(?:SIGTERM|SIGKILL)',
                        native_output)
                    if failure:
                        self.native_failure = failure.group(0)
                if self.error:
                    continue
                try:
                    if stream and size + len(data) > 8 * 1024 * 1024:
                        stream.close()
                        stream = None
                        os.replace(current, self.directory / (self.name + '.previous.log'))
                        size = 0
                    if stream is None:
                        stream = current.open('wb')
                    stream.write(data)
                    stream.flush()
                    size += len(data)
                except OSError as exc:
                    self.error = str(exc)
        finally:
            if stream:
                stream.close()
            self.process.stdout.close()

    def output(self):
        """Snapshot the bounded native tail, not an inferred readiness flag."""
        with self.lock:
            return self.tail.decode(errors='replace')

    def interrupt(self):
        """Interrupt the owner once; ROS launch forwards its own signal to descendants."""
        if self.process.poll() is None:
            if self.identity() != (self.pid, self.start_ticks):
                raise RuntimeError('owned process identity changed: ' + self.name)
            self.process.send_signal(signal.SIGINT)
        elif self.members():
            # A session/group ID remains reserved while its members live, so it cannot
            # become an unrelated session after the original leader exits.
            os.killpg(self.pid, signal.SIGINT)

    def members(self):
        """List live members of the fresh session and process group created by this owner."""
        output = []
        for path in Path('/proc').glob('[0-9]*/stat'):
            try:
                fields = path.read_text().rsplit(')', 1)[1].split()
                if (int(fields[2]) == self.pid and int(fields[3]) == self.pid and
                        fields[0] != 'Z'):
                    output.append((int(path.parent.name), fields[19]))
            except (OSError, ValueError, IndexError):
                continue
        return output

    def stopped(self):
        """Confirm the parent and every owned descendant have actually terminated."""
        if self.process.poll() is None or self.members():
            return False
        self.thread.join()
        return True


class Fleet:
    """Reserve the entire fleet before spawning; a critical child loss ends this session."""

    def __init__(self, config, output, px4_directory=None):
        self.config = config
        self.px4 = select_px4_directory(px4_directory)
        require_native_imu(self.px4)
        self.root = new_directory(output)
        self.children = []
        self.stopping = threading.Event()
        self.reservation = Reservation()
        self.reservation.acquire_fleet(config)
        self.phase = 'preparing'
        self.failure = ''
        self.started = time.monotonic()
        self.manifest = dict(version=1, simulation_only=True, session=uuid.uuid4().hex,
                             configuration=config, output_directory=str(self.root),
                             models={}, staging={})
        self.manifest_path = self.root / 'manifest.json'
        try:
            self.prepare()
        except BaseException:
            self.reservation.close()
            raise

    def prepare(self):
        """Freeze independent writable rootfs, exact model derivatives and resource bindings."""
        model_share = Path(get_package_share_directory('easyfly_px4_sitl'))
        assets = Path(get_package_share_directory('easyfly_gazebo_models'))
        base = model_share / 'models/easyfly_iris/easyfly_iris.sdf'
        lidar = assets / 'easyfly/runtime_models/mid360/mid360.sdf'
        for vehicle in self.config['vehicles']:
            name = vehicle['id']
            vehicle['session'] = uuid.uuid4().hex
            self.manifest['models'][name] = derive(base, lidar, self.root, vehicle,
                                                   self.config['camera_rate_hz'])
            self.manifest['staging'][name] = prepare_staging(self.px4, self.root / (name + '-px4'))
        binary = self.px4 / 'build/px4_sitl_default/bin/px4'
        self.manifest.update(px4_binary=str(binary),
                             px4_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
        world = Path(get_package_share_directory('easyfly_gazebo_worlds')) / 'empty.world'
        self.manifest['world'] = dict(path=str(world.resolve()),
                                      sha256=hashlib.sha256(world.read_bytes()).hexdigest())
        self.manifest_path.write_text(json.dumps(self.manifest, indent=2) + '\n')
        plugins = [str(self.px4 / 'build/px4_sitl_default/build_gazebo-classic'),
                   str(Path(get_package_prefix('easyfly_gazebo_plugins')) / 'lib'),
                   '/opt/ros/humble/lib']
        models = [str(model_share / 'models'), str(assets / 'easyfly/runtime_models'),
                  str(assets), str(self.px4 /
                                   'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models')]
        self.environment = dict(os.environ, ROS_DOMAIN_ID=str(self.config['coordinator_domain']),
                                ROS_LOCALHOST_ONLY='1', GAZEBO_MASTER_URI=(
                                    'http://127.0.0.1:' + str(self.config['gazebo_port'])),
                                GAZEBO_MODEL_PATH=':'.join(models), GAZEBO_MODEL_DATABASE_URI='',
                                GAZEBO_PLUGIN_PATH=':'.join(plugins), LP_NUM_THREADS='1',
                                OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1',
                                LD_LIBRARY_PATH=plugins[0] + ':' + os.environ.get(
                                    'LD_LIBRARY_PATH', ''), PYTHONNOUSERSITE='1')
        transport = Path(get_package_share_directory('easyfly_fleet_sim')) / (
            'config/raw_image_transport.xml')
        self.environment['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(transport)
        self.manifest['dds_transport'] = dict(
            path=str(transport.resolve()),
            sha256=hashlib.sha256(transport.read_bytes()).hexdigest())
        self.manifest_path.write_text(json.dumps(self.manifest, indent=2) + '\n')
        self.save()

    def save(self, error=''):
        """Record actual owned processes and lifecycle separately from flight admission."""
        value = dict(session=self.manifest['session'], phase=self.phase,
                     error=error or self.failure,
                     elapsed_s=time.monotonic() - self.started,
                     processes=[dict(name=c.name, pid=c.pid, start_ticks=c.start_ticks,
                                     returncode=c.process.poll(), log_error=c.error,
                                     native_failure=c.native_failure)
                                for c in self.children])
        temporary = self.root / 'status.pending'
        temporary.write_text(json.dumps(value, indent=2) + '\n')
        os.replace(temporary, self.root / 'status.json')

    def child(self, name, command, cpus, env=None, cwd=None):
        """Start a child only while this fleet owns its full reservation."""
        if self.stopping.is_set():
            raise InterruptedError('fleet startup interrupted')
        child = Child(name, command, env or self.environment, self.root, cpus, cwd)
        self.children.append(child)
        self.save()
        return child

    def check(self):
        """Treat every unexpected native exit as a fleet failure, including premature exit zero."""
        if self.stopping.is_set():
            raise InterruptedError('fleet stop requested')
        for child in self.children:
            code = child.process.poll()
            if code is not None:
                raise RuntimeError(f'fleet child {child.name} exited unexpectedly: {code}')
            if child.error:
                raise RuntimeError('fleet log write failed: ' + child.name)
            if child.native_failure:
                raise RuntimeError('native descendant failed: ' + child.name + ': ' +
                                   child.native_failure)

    def wait(self, predicate, seconds, description):
        """Bound startup waits and keep the exact child failures visible."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.check()
            if predicate():
                return
            self.stopping.wait(.05)
        raise TimeoutError(description)

    def launch(self):
        """Run one physical world with distinct native instances and observation boundaries."""
        world = Path(get_package_share_directory('easyfly_gazebo_worlds')) / 'empty.world'
        server = executable('easyfly_gazebo_plugins', 'easyfly_gzserver')
        self.phase = 'starting_gazebo'
        gazebo_command = [server, '--verbose', '-s', 'libgazebo_ros_init.so',
                          str(world), '--ros-args', '--param', 'gazebo:publish_rate:=100.0']
        gazebo = self.child('gazebo', gazebo_command, self.config['gazebo_cpus'])
        self.wait(lambda: 'Connected to gazebo master' in gazebo.output(), 20,
                  'Gazebo master startup timed out')
        self.model_pose('ground_plane')
        # Spawn commands finish normally; they are recorded separately from critical children.
        for vehicle in self.config['vehicles']:
            model = self.manifest['models'][vehicle['id']]['path']
            x, y, z = vehicle['spawn']
            command = ['gz', 'model', '--spawn-file=' + model, '--model-name=' + vehicle['id'],
                       '-x', str(x), '-y', str(y), '-z', str(z)]
            result = subprocess.run(command, env=self.environment, capture_output=True,
                                    text=True, timeout=15)
            (self.root / (vehicle['id'] + '-spawn.log')).write_text(result.stdout + result.stderr)
            if result.returncode:
                raise RuntimeError('vehicle model spawn failed: ' + vehicle['id'])
            self.model_pose(vehicle['id'])
        px4_children = []
        for vehicle in self.config['vehicles']:
            name, slot = vehicle['id'], vehicle['instance']
            stage = self.manifest['staging'][name]
            env = dict(self.environment, ROS_DOMAIN_ID=str(vehicle['domain']),
                       PX4_SIM_MODEL='gazebo-classic_iris')
            px4 = self.child(
                name + '-px4', [stage['wrapper'], '-i', str(slot), '-d'], vehicle['cpus'],
                env=env, cwd=str(Path(stage['rootfs']).parent))
            px4_children.append((vehicle, px4))
        self.phase = 'configuring_native_telemetry'
        for vehicle, px4 in px4_children:
            self.wait(lambda p=px4: 'Startup script returned successfully' in p.output(), 45,
                      vehicle['id'] + ' PX4 startup timed out')
            record = configure_owned(px4.pid, self.manifest['px4_binary'], 5,
                                     battery_rate_hz=5, instance=vehicle['instance'])
            if record['native_output_deferred']:
                def proof(p=px4, v=vehicle):
                    output = p.output()
                    return configured_heartbeat(output, output, 5, instance=v['instance']) and (
                        configured_stream(output, output, 5, 'BATTERY_STATUS',
                                          instance=v['instance']))
                self.wait(proof, 8, vehicle['id'] + ' native stream proof timed out')
                record.update(native_output_deferred=False, native_output=px4.output())
            (self.root / (vehicle['id'] + '-native-telemetry.json')).write_text(
                json.dumps(record, indent=2) + '\n')
        self.phase = 'starting_vehicle_graphs'
        for vehicle in self.config['vehicles']:
            name, slot = vehicle['id'], vehicle['instance']
            env = dict(self.environment, ROS_DOMAIN_ID=str(vehicle['domain']))
            self.child(name + '-sensor-boundary', [
                executable('easyfly_fleet_sim', 'fleet_sensor_boundary'), name,
                str(self.config['coordinator_domain']), str(vehicle['domain']),
                str(self.root / (name + '-native-sensor-statistics.json'))], vehicle['cpus'])
            self.child(name + '-boundary', [executable('easyfly_fleet_sim', 'fleet_boundary'),
                                            '--manifest', str(self.manifest_path),
                                            '--vehicle', name], vehicle['cpus'])
            mavros_command = ['ros2', 'launch', 'easyfly_mavlink_bridge',
                              'mavlink_bridge.launch.py', 'use_sim_time:=true',
                              f'fcu_url:=udp://127.0.0.1:{14540 + slot}@127.0.0.1:{14580 + slot}',
                              'target_system:=' + str(slot + 1)]
            self.child(name + '-mavros', mavros_command, vehicle['cpus'], env=env)
            for tag, package, launch in [
                    ('coordinates', 'easyfly_coord_transform', 'coord_tf.launch.py'),
                    ('calibration', 'easyfly_camera_calib', 'calib_tf.launch.py'),
                    ('flight', 'easyfly_flight_control', 'flight_control.launch.py'),
                    ('mission', 'easyfly_mission_fsm', 'mission_fsm.launch.py')]:
                self.child(name + '-' + tag, ['ros2', 'launch', package, launch,
                                              'use_sim_time:=true'], vehicle['cpus'], env=env)
            self.child(name + '-mission-services', [executable('easyfly_mission_srvs',
                       'mission_services'), '--ros-args', '-p', 'use_sim_time:=true'],
                       vehicle['cpus'], env=env)
        self.phase = 'running'
        self.save()
        while not self.stopping.wait(.2):
            self.check()
            self.save()

    def model_pose(self, name):
        """Require a real named world entity, not only the spawn utility's exit status."""
        query = executable('easyfly_gazebo_plugins', 'easyfly_model_query')
        result = subprocess.run([query, name, '--pose'], env=self.environment,
                                capture_output=True, text=True, timeout=15)
        (self.root / (name + '-initial-pose.log')).write_text(result.stdout + result.stderr)
        self.check()
        if result.returncode or len(result.stdout.split()) != 6:
            raise RuntimeError('native Gazebo entity not ready: ' + name)

    def close(self):
        """Stop flight clients before clocks/PX4/world, waiting for actual owned group exit."""
        self.phase = 'stopping'
        self.save()
        stages = [
            [c for c in reversed(self.children) if not c.name.endswith(('-boundary', '-px4'))
             and c.name != 'gazebo'],
            [c for c in self.children if c.name.endswith('-boundary')],
            [c for c in self.children if c.name.endswith('-px4')],
            [c for c in self.children if c.name == 'gazebo']]
        for stage in stages:
            for child in stage:
                child.interrupt()
            last_report = 0.
            while any(not child.stopped() for child in stage):
                if time.monotonic() - last_report > 1.:
                    self.save()
                    last_report = time.monotonic()
                time.sleep(.05)
        if any(c.process.returncode != 0 or c.native_failure for c in self.children):
            self.failure = self.failure or 'one or more native children exited abnormally'
        self.phase = 'failed' if self.failure else 'stopped'
        self.save()
        self.reservation.close()


def main():
    """Start an explicitly configured simulation fleet and keep all resource locks until exit."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--px4-dir')
    arguments = parser.parse_args()
    fleet = Fleet(load(arguments.config), arguments.output, arguments.px4_dir)
    signal.signal(signal.SIGINT, lambda *_: fleet.stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: fleet.stopping.set())
    try:
        fleet.launch()
    except InterruptedError:
        pass
    except Exception as exc:
        fleet.phase = 'failed'
        fleet.failure = str(exc)
        fleet.save(str(exc))
        raise
    finally:
        fleet.close()
    if fleet.failure:
        raise SystemExit(fleet.failure)

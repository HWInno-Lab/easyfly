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
"""Owned simulation/communication worker. JSON stdin/stdout; no flight commands."""
import argparse
import fcntl
import json
import hashlib
import math
import os
from pathlib import Path
import selectors
import select
import signal
import socket
import stat
import subprocess
import sys
import xml.etree.ElementTree as ET
import time
import traceback


MAX_INPUT = 16384
_OUTPUT_AVAILABLE = True


class Cancelled(Exception):
    """A queued normal stop arrived before the first backend process existed."""


def private(path):
    path = Path(path)
    if not path.is_absolute() or path.resolve() != path:
        raise ValueError('需要无符号链接的绝对私有目录')
    path.mkdir(mode=0o700, exist_ok=True)
    info = path.lstat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
        raise ValueError('目录必须归当前用户所有且权限为 0700')
    return path


def urls(profile):
    fcu = profile['fcu']
    if fcu['transport'] == 'serial':
        address = f"{fcu['serial_device']}:{fcu['baud']}"
    elif fcu['transport'] == 'tcp':
        address = f"tcp://{fcu['remote_host']}:{fcu['remote_port']}"
    else:
        address = (f"udp://{fcu['bind_host']}:{fcu['bind_port']}@"
                   f"{fcu['remote_host']}:{fcu['remote_port']}")
    gcs = profile['gcs']
    bind = '127.0.0.1' if gcs['host'] in ('127.0.0.1', 'localhost') else '0.0.0.0'
    forward = (f"udp://{bind}:{gcs['bind_port']}@{gcs['host']}:{gcs['port']}"
               if gcs['enabled'] else '')
    return address, forward


def preview_geometry(data):
    """Bounded direct static SDF primitives; omitted includes/meshes are never guessed."""
    if len(data) > 2 * 1024 * 1024:
        return []
    root = ET.fromstring(data)
    result = []
    for model in root.findall('./world/model')[:128]:
        if model.findtext('static') != 'true':
            continue
        pose = [float(x) for x in model.findtext('pose', '0 0 0 0 0 0').split()]
        if len(pose) != 6 or not all(math.isfinite(x) and abs(x) < 10000 for x in pose):
            continue
        if abs(pose[3]) > 0.0001 or abs(pose[4]) > 0.0001:
            continue
        for link in model.findall('link')[:16]:
            if any(float(x) != 0 for x in link.findtext('pose', '0 0 0 0 0 0').split()):
                continue
            for collision in link.findall('collision')[:16]:
                if len(result) >= 128:
                    return result
                if any(float(x) != 0 for x in collision.findtext('pose', '0 0 0 0 0 0').split()):
                    continue
                size = collision.findtext('geometry/box/size', '').split()
                if len(size) != 3:
                    continue
                width, height, depth = [float(x) for x in size]
                if not all(math.isfinite(x) and 0 < x <= 1000 for x in (width, height, depth)):
                    continue
                result.append(dict(x=pose[0], y=pose[1], width=width, height=height,
                                   yaw=pose[5], ground=pose[2] <= 0))
    return result


def inventory(px4_directory=''):
    from easyfly_px4_sitl.scene_environment import installed_catalog
    catalog, notes = installed_catalog(px4_dir=Path(px4_directory) if px4_directory else None)
    rows = []
    for key, scene in sorted(catalog.scenes.items()):
        report = dict(checked=False)
        preview = []
        world = scene['world']
        try:
            base = Path(catalog.packages[world['package']]).resolve()
            path = (base / world['path']).resolve()
            path.relative_to(base)
            if path.stat().st_size <= 2 * 1024 * 1024:
                data = path.read_bytes()
                if hashlib.sha256(data).hexdigest() == world['sha256']:
                    preview = preview_geometry(data)
        except (OSError, ValueError, KeyError, ET.ParseError):
            pass
        rows.append(dict(id=key, name=scene['name'], status=scene['status'],
                         profiles=scene['supported_profiles'], validation=report,
                         preview=preview))
    return dict(scenes=rows, notes=notes)


def check(profile):
    """Offline dependencies/resources first; never launch or prepare assets here."""
    from ament_index_python.packages import get_package_share_directory
    from easyfly_params.algorithms import validate, localization_backend
    algorithms = validate(profile.get('algorithms', {}))
    for kind in ('ellipse', 'color_line'):
        if algorithms[kind + '_enabled']:
            get_package_share_directory('easyfly_' + kind + '_detection')
    if profile['mode'] not in ('sitl', 'communication'):
        raise ValueError('后端仅支持本机通信或本机仿真')
    fcu, gcs = urls(profile)
    if profile['mode'] == 'sitl':
        backend = localization_backend(algorithms, profile['selected_profile'])
        if backend == 'fast_lio':
            get_package_share_directory('fast_lio')
            get_package_share_directory('easyfly_fast_lio_integration')
        elif backend == 'openvins':
            get_package_share_directory('ov_msckf')
            get_package_share_directory('easyfly_openvins_integration')
        if not profile['simulation_time'] or profile['fcu']['transport'] != 'udp':
            raise ValueError('仿真必须使用模拟时间和 UDP')
        from easyfly_px4_sitl.hrt_compatibility import select_px4_directory
        from easyfly_px4_sitl.imu_compatibility import require_native_imu
        from easyfly_px4_sitl.scene_environment import installed_catalog
        from easyfly_px4_sitl.scene_inputs import resolve_scene
        from easyfly_params.configuration import load_profile, ensure_launchable
        selected = select_px4_directory(profile['simulation']['px4_directory'] or None)
        if (selected / 'easyfly_staging.json').exists():
            selected = Path(json.loads((selected / 'easyfly_staging.json').read_text())['source'])
        require_native_imu(selected)
        catalog, _ = installed_catalog(px4_dir=selected)
        resolve_scene(catalog, profile['simulation']['scene'], profile['selected_profile'])
        ensure_launchable(load_profile(profile['selected_profile']))
        get_package_share_directory('easyfly_sim_launcher')
    else:
        selected = None
        if profile['fcu']['transport'] == 'serial':
            device = Path(profile['fcu']['serial_device'])
            if not device.exists() or not stat.S_ISCHR(device.stat().st_mode):
                raise ValueError('串口设备不存在或不是字符设备；请刷新设备列表')
            if not os.access(device, os.R_OK | os.W_OK):
                raise ValueError('当前用户没有串口读写权限；请由管理员配置 dialout 设备权限')
    get_package_share_directory('easyfly_mavlink_bridge')
    get_package_share_directory('mavros')
    return dict(ok=True, fcu_url=fcu, gcs_url=gcs,
                selected_px4=str(selected) if selected else '')


def command(profile):
    fcu, gcs = urls(profile)
    common = [f'fcu_url:={fcu}', f"target_system:={profile['fcu']['target_system']}"]
    # ros2 launch rejects a CLI name:= without a value. Use the declared empty default.
    if gcs:
        common.append(f'gcs_url:={gcs}')
    if profile['mode'] == 'sitl':
        from easyfly_params.algorithms import validate, localization_backend
        algorithms = validate(profile.get('algorithms', {}))
        localization_backend(algorithms, profile['selected_profile'])
        topic = (profile.get('down_topic', '/camera_down/color/image_raw')
                 if algorithms['camera'] == 'down' else
                 profile.get('front_topic', '/camera/color/image_raw'))
        return ['ros2', 'launch', 'easyfly_sim_launcher', 'profile.launch.py',
                'profile:=' + profile['selected_profile'],
                'scene:=' + profile['simulation']['scene'],
                'headless:=' + str(not profile['simulation']['gazebo_gui']).lower(),
                'algorithms_json:=' + json.dumps(algorithms, separators=(',', ':')),
                'algorithm_image_topic:=' + (topic or '/camera/color/image_raw'),
                'rviz:=false', *common]
    return ['ros2', 'launch', 'easyfly_mavlink_bridge', 'mavlink_bridge.launch.py',
            'use_sim_time:=' + str(profile['simulation_time']).lower(), *common]


def can_stop(state, now):
    """Unknown/old/armed/airborne evidence cannot permit a backend stop."""
    return (state.get('connected') is True and state.get('armed') is False and
            state.get('landed') == 1 and now - state.get('state_at', -1e30) <= 2.5 and
            now - state.get('landed_at', -1e30) <= 2.5 and
            state.get('state_sources') == 1 and state.get('landed_sources') == 1 and
            not state.get('active_actions', False))


def reserve(profile):
    directory = private(Path('/tmp') / f'easyfly-communication-{os.getuid()}')
    descriptor = os.open(directory / 'backend.lock',
                         os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600)
    try:
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
            raise ValueError('通信实例锁权限无效')
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError('已有本机通信/仿真管理实例，请返回原管理窗口处理') from exc
        from easyfly_px4_sitl.session_guard import require_free_port
        if profile['fcu']['transport'] == 'udp':
            require_free_port(profile['fcu']['bind_port'], socket.SOCK_DGRAM, 'MAVROS FCU UDP')
        if profile['gcs']['enabled']:
            require_free_port(profile['gcs']['bind_port'], socket.SOCK_DGRAM, 'MAVROS GCS UDP')
        return descriptor
    except BaseException:
        os.close(descriptor)
        raise


def emit(value):
    global _OUTPUT_AVAILABLE
    if not _OUTPUT_AVAILABLE:
        return
    try:
        sys.stdout.write(json.dumps(value, ensure_ascii=False, separators=(',', ':')) + '\n')
        sys.stdout.flush()
    except BrokenPipeError:
        _OUTPUT_AVAILABLE = False


class Backend:
    def __init__(self, profile, directory):
        self.profile = profile
        self.directory = private(directory)
        self.result = check(profile)
        self.reservation = reserve(profile)
        self.process = None
        self.state = {}
        self.ever_connected = False
        self.started = time.monotonic()
        self.phase = 'starting'
        self.message = '启动后端，等待真实飞控和 DDS 状态'
        self.stopping_at = None
        self.gcs = {}
        self.node = None
        self.context = None
        self.executor = None
        self.error = None
        self.action_states = {}
        self.owner_channel = None
        self.clock_ns = 0
        self.clock_progress_at = None

    def start(self):
        # Activate only the installed, verified loader-reply compatibility policy.
        from easyfly_fastdds_compat.runtime import activate
        dds = activate()
        self.dds = dict(library_directory=str(Path(dds['path']).parent),
                        profiles_file=dds['profile_environment']['FASTDDS_DEFAULT_PROFILES_FILE'],
                        sha256=dds['sha256'])
        environment = dict(os.environ)
        environment.pop('EASYFLY_BACKEND_OWNER_TOKEN', None)
        environment['ROS_LOG_DIR'] = str(self.directory / 'ros')
        port = self.profile['simulation']['gazebo_port']
        environment['GAZEBO_MASTER_URI'] = f"http://127.0.0.1:{port}"
        if self.profile['mode'] == 'sitl':
            from easyfly_precision_landing.simulation_setup import prepare_staging
            staged = prepare_staging(self.result['selected_px4'], self.directory / 'px4')
            environment['PX4_DIR'] = staged['staging']
            # The process uses a reviewed private rootfs, never the shared default.
            self.result['px4_staging'] = staged
        from rclpy import create_node
        from rclpy.context import Context
        from rclpy.parameter import Parameter
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from mavros_msgs.msg import State, ExtendedState
        from diagnostic_msgs.msg import DiagnosticArray
        from action_msgs.msg import GoalStatusArray
        from rclpy.qos import QoSProfile, DurabilityPolicy
        self.context = Context()
        self.context.init()
        self.node = create_node(
            'easyfly_connection_observer', context=self.context,
            parameter_overrides=[Parameter('use_sim_time', value=self.profile['simulation_time'])])
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.node.create_subscription(State, '/mavros/state', self.on_state,
                                      qos_profile_sensor_data)
        self.node.create_subscription(ExtendedState, '/mavros/extended_state', self.on_landed,
                                      qos_profile_sensor_data)
        self.node.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, 10)
        from rosgraph_msgs.msg import Clock
        self.node.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        actions = ('execute_mission', 'navigate', 'takeoff', 'land', 'search_area', 'track_target')
        for name in actions:
            self.node.create_subscription(
                GoalStatusArray, f'/{name}/_action/status',
                lambda msg, key=name: self.on_actions(key, msg), status_qos)
        # Discovery is asynchronous; allow existing endpoints to become visible.
        end = time.monotonic() + 1.2
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.05)
        if self.node.count_publishers('/uav_state') or self.node.count_publishers('/mavros/state'):
            raise RuntimeError('当前 DDS domain 已发现飞控/状态发布源；请连接已有平台或选择独立 domain')
        if select.select([sys.stdin.fileno()], [], [], 0)[0]:
            pending = os.read(sys.stdin.fileno(), MAX_INPUT + 1)
            if not pending:
                raise Cancelled()
            if len(pending) > MAX_INPUT:
                raise ValueError('启动期间请求超出预算')
            if any(json.loads(line) == {'op': 'stop'} for line in pending.splitlines()):
                raise Cancelled()
        self.log = (self.directory / 'backend.log').open('xb')
        args = command(self.profile)
        (self.directory / 'launch.json').write_text(
            json.dumps(dict(argv=args, **self.result), indent=2))
        self.process = subprocess.Popen(args, env=environment, cwd=self.directory,
                                        stdout=self.log, stderr=subprocess.STDOUT,
                                        start_new_session=True)
        self.started = time.monotonic()

    def on_clock(self, msg):
        stamp = msg.clock.sec * 10**9 + msg.clock.nanosec
        if stamp > 0 and stamp != self.clock_ns:
            self.clock_progress_at = time.monotonic()
        self.clock_ns = stamp

    def on_state(self, msg):
        self.ever_connected |= msg.connected or msg.armed
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        now = self.node.get_clock().now().nanoseconds
        if now <= 0 or stamp <= 0 or not -0.2e9 <= now - stamp <= 2.5e9:
            return
        if stamp == self.state.get('state_stamp'):
            return
        if stamp < self.state.get('state_stamp', 0):
            self.state.clear()
        self.state.update(connected=msg.connected, armed=msg.armed, mode=msg.mode,
                          state_at=time.monotonic(), state_stamp=stamp)
        self.ever_connected |= msg.connected

    def on_landed(self, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        now = self.node.get_clock().now().nanoseconds
        if now <= 0 or stamp <= 0 or not -0.2e9 <= now - stamp <= 2.5e9:
            return
        if stamp == self.state.get('landed_stamp'):
            return
        if stamp < self.state.get('landed_stamp', 0):
            self.state.clear()
        self.state.update(landed=msg.landed_state, landed_at=time.monotonic(), landed_stamp=stamp)

    def on_actions(self, key, msg):
        self.action_states[key] = any(item.status in (1, 2, 3) for item in msg.status_list)
        self.state['active_actions'] = any(self.action_states.values())

    def on_diagnostics(self, msg):
        wanted = self.result['gcs_url']
        if not wanted:
            return
        matches = [entry for entry in msg.status
                   if 'endpoint' in entry.name and wanted in entry.name]
        if len(matches) == 1:
            item = matches[0]
            level = item.level
            if isinstance(level, bytes):
                level = int.from_bytes(level, 'little')
            self.gcs = dict(at=time.monotonic(), level=level, message=item.message,
                            values={value.key: value.value for value in item.values})

    def status(self):
        now = time.monotonic()
        self.state['active_actions'] = any(self.action_states.values())
        if self.node:
            self.state['state_sources'] = self.node.count_publishers('/mavros/state')
            self.state['landed_sources'] = self.node.count_publishers('/mavros/extended_state')
        stopped = bool(self.process and self.process.poll() is not None)
        if stopped:
            normal = self.stopping_at and self.process.returncode == 0
            self.phase = 'stopped' if normal else 'failed'
            self.message = f'后端退出码 {self.process.returncode}；日志已保存'
        elif self.stopping_at:
            self.phase = 'stop_unknown' if now - self.stopping_at > 35 else 'stopping'
            self.message = '等待正常停止完成；不会强制结束' if self.phase == 'stopping' else '停止超时，进程仍由本会话持有'
        elif can_stop(self.state, now):
            self.phase = 'ready'
            self.message = '飞控已连接、未解锁、在地面'
        elif now - self.started > 90:
            self.phase = 'degraded'
            self.message = '后端仍运行，但就绪条件未满足；请查看日志与连接配置'
        safe = can_stop(self.state, now)
        fresh = now - self.state.get('state_at', -1e30) < 2.5
        code = self.process.returncode if stopped else None
        abort_startup = not self.ever_connected and not self.state['active_actions']
        clock_age = (now - self.clock_progress_at
                     if self.clock_progress_at is not None else None)
        diagnosis = 'ready' if safe else 'waiting_fcu'
        recovery = '等待真实飞控状态；可检查飞控地址、DDS domain 和串口配置'
        if stopped:
            diagnosis, recovery = 'exited', '日志已保留；确认原因后可重新启动'
        elif self.stopping_at:
            diagnosis = self.phase
            recovery = '等待正常退出；停止超时请查看日志，进程归属继续保留'
        elif self.profile['simulation_time'] and (clock_age is None or clock_age > 2.5):
            diagnosis = 'clock_missing' if clock_age is None else 'clock_stalled'
            recovery = '模拟时钟未收到或已停止；请检查 Gazebo 是否暂停，以及日志中的 IMU/PX4 错误'
        elif not fresh and self.ever_connected:
            diagnosis, recovery = 'fcu_stale', '飞控状态已过期；检查 MAVROS 和仿真日志，禁止按旧状态停机'
        elif self.state.get('state_sources', 0) > 1 or self.state.get('landed_sources', 0) > 1:
            diagnosis, recovery = 'multiple_sources', '发现重复状态源；请关闭重复实例，等待新的唯一状态'
        elif fresh and self.state.get('connected') and not safe:
            diagnosis, recovery = 'vehicle_active', '当前解锁、落地或任务条件不允许停止；等待真实的新鲜地面状态'
        elif safe:
            recovery = '可打开工作窗口；结束工作和 QGC 后可正常停止'
        if not stopped and not self.stopping_at:
            if diagnosis in ('clock_stalled', 'fcu_stale', 'multiple_sources'):
                self.phase, self.message = 'degraded', recovery
            elif diagnosis == 'vehicle_active':
                self.phase, self.message = 'running', '飞控状态已收到，当前不允许停止后端'
        return dict(phase=self.phase, message=self.message, process_running=not stopped,
                    error=getattr(self, 'error', None) or '',
                    diagnosis=diagnosis, recovery=recovery, clock_ns=self.clock_ns,
                    clock_age=clock_age,
                    dds=getattr(self, "dds", {}),
                    pid=self.process.pid if self.process else None,
                    fcu_url=self.result['fcu_url'], gcs_url=self.result['gcs_url'],
                    connected=(self.state.get('connected', False) and fresh and
                               self.state.get('state_sources') == 1),
                    armed=self.state.get('armed'), landed=self.state.get('landed'),
                    safe_to_stop=safe, can_abort_startup=abort_startup,
                    gcs=self.gcs if now - self.gcs.get('at', -1e30) < 3 else {},
                    directory=str(self.directory), exit_code=code)

    def stop(self):
        if not self.process or self.process.poll() is not None or self.stopping_at:
            return
        self.state['active_actions'] = any(self.action_states.values())
        if self.state['active_actions']:
            raise RuntimeError('已观测到活动任务，拒绝停止后端')
        if not can_stop(self.state, time.monotonic()) and self.ever_connected:
            raise RuntimeError('飞控已解锁、离地或状态未知/过期；拒绝停止通信或仿真')
        # launch owns child shutdown. Never signal a broad process name or foreign PID.
        self.process.send_signal(signal.SIGINT)
        self.stopping_at = time.monotonic()

    def run(self):
        selector = selectors.DefaultSelector()
        selector.register(sys.stdin.fileno(), selectors.EVENT_READ)
        data = bytearray()
        next_report = 0
        try:
            while self.process.poll() is None:
                try:
                    self.executor.spin_once(timeout_sec=0.05)
                except Exception as exc:
                    self.state.clear()
                    self.error = f'观测异常，保留后端: {exc}'
                    time.sleep(0.05)
                if self.owner_channel:
                    self.owner_channel.poll()
                if selector.select(0):
                    chunk = os.read(sys.stdin.fileno(), 4096)
                    if not chunk:
                        # Keep ownership on supervisor loss; do not silently stop a vehicle.
                        selector.unregister(sys.stdin.fileno())
                    data.extend(chunk)
                    if len(data) > MAX_INPUT:
                        data.clear()
                        self.error = '控制请求超出长度限制'
                    while b'\n' in data:
                        line, _, rest = data.partition(b'\n')
                        data[:] = rest
                        try:
                            request = json.loads(line)
                            if request != {'op': 'stop'}:
                                raise ValueError('未知后端操作')
                            self.stop()
                            self.error = None
                        except (ValueError, RuntimeError) as exc:
                            self.error = str(exc)
                if time.monotonic() >= next_report:
                    value = self.status()
                    if self.error:
                        value['error'] = self.error
                    try:
                        emit(value)
                    except (TypeError, ValueError) as exc:
                        self.gcs = {}
                        self.error = f'状态编码异常，保留后端: {exc}'
                        emit(dict(phase='unknown', process_running=True, error=self.error))
                    next_report = time.monotonic() + 0.5
            terminal = self.status()
            if self.owner_channel:
                self.owner_channel.finish(terminal)
            emit(terminal)
            return 0 if self.phase == 'stopped' else 1
        finally:
            selector.close()

    def close(self):
        # Called only after launch has exited, or before it was created.
        if self.process and self.process.poll() is None:
            return
        if self.executor:
            self.executor.shutdown(timeout_sec=1)
        if self.node:
            self.node.destroy_node()
        if self.context:
            self.context.try_shutdown()
        if self.owner_channel:
            self.owner_channel.close()
        os.close(self.reservation)
        if hasattr(self, 'log'):
            self.log.close()


def run_owned(worker):
    """Preserve the launch owner even if a reporting/observer callback fails."""
    while True:
        try:
            return worker.run()
        except Exception as exc:
            traceback.print_exc(file=sys.stderr)
            worker.state.clear()
            worker.error = f'观测异常，保留后端: {type(exc).__name__}: {exc}'
            if worker.process.poll() is not None:
                emit(dict(phase='failed', process_running=False, error=worker.error))
                return 1
            emit(dict(phase='unknown', process_running=True, error=worker.error))
            time.sleep(0.1)


def read_profile():
    line = bytearray()
    while len(line) <= MAX_INPUT and not line.endswith(b'\n'):
        value = os.read(sys.stdin.fileno(), 1)
        if not value:
            break
        line.extend(value)
    if not line.endswith(b'\n') or len(line) > MAX_INPUT:
        raise ValueError('缺少有界配置请求')
    value = json.loads(line)
    if not isinstance(value, dict):
        raise ValueError('配置必须是对象')
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--catalog', action='store_true')
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--px4-directory', default='')
    parser.add_argument('--directory', type=Path)
    parser.add_argument('--control-directory', type=Path)
    args = parser.parse_args()
    worker = None
    try:
        if args.catalog:
            emit(inventory(args.px4_directory))
            return 0
        profile = read_profile()
        if args.check:
            emit(check(profile))
            return 0
        emit(dict(phase='preflight', process_running=False, can_abort_startup=True,
                  message='正在校验场景、资源、兼容输入和飞控端口；尚未启动后端'))
        worker = Backend(profile, args.directory)
        if args.control_directory:
            from easyfly_connection_manager.ownership import OwnerChannel
            worker.owner_channel = OwnerChannel(
                private(args.control_directory), os.environ.get('EASYFLY_BACKEND_OWNER_TOKEN', ''),
                os.environ.get('EASYFLY_CONNECTION_SESSION', ''), worker)
        worker.start()
        emit(worker.status())
        return run_owned(worker)
    except Cancelled:
        value = dict(phase='stopped', process_running=False, exit_code=0,
                     message='已取消启动，尚未创建后端')
        if args.control_directory:
            from easyfly_connection_manager.ownership import save_terminal
            save_terminal(private(args.directory), os.environ['EASYFLY_BACKEND_OWNER_TOKEN'],
                          os.environ['EASYFLY_CONNECTION_SESSION'], value)
        emit(value)
        return 0
    except Exception as exc:
        traceback.print_exc(file=sys.stderr)
        if worker and worker.process and worker.process.poll() is None:
            worker.error = f'观测异常，后端保留: {type(exc).__name__}: {exc}'
            worker.state.clear()
            return run_owned(worker)
        value = dict(phase='failed', process_running=False, exit_code=2,
                     error=f'{type(exc).__name__}: {exc}')
        if args.control_directory:
            from easyfly_connection_manager.ownership import save_terminal
            save_terminal(private(args.directory), os.environ['EASYFLY_BACKEND_OWNER_TOKEN'],
                          os.environ['EASYFLY_CONNECTION_SESSION'], value)
        emit(value)
        return 2
    finally:
        if worker:
            worker.close()


if __name__ == '__main__':
    raise SystemExit(main())

"""Explicit FUEL action, bounded native IPC, navigation lease, source-time gating."""
import base64
import copy
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import signal
import time
import uuid

from ament_index_python.packages import get_package_prefix
from easyfly_msgs.action import Explore, NavigateTo
from easyfly_msgs.msg import UAVState, VoxelGrid
from easyfly_msgs.srv import GetActionOwnership, ReserveNavigation, SetExplorationView
from geometry_msgs.msg import Point
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.serialization import serialize_message
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.task import Future
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.clock import Clock, ClockType
from std_msgs.msg import String
from std_srvs.srv import Trigger
from unique_identifier_msgs.msg import UUID
import yaml

from .policy import Policy, geometry, identity
from .camera import CameraView


def point(p):
    return [p.x, p.y, p.z]


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec / 1e9


class Exploration(Node):
    def __init__(self):
        super().__init__('fuel_exploration')
        self.group = ReentrantCallbackGroup()
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        value = self.declare_parameter('evidence_directory', '').value
        if not value:
            raise ValueError('FUEL requires an explicit evidence_directory')
        self.root = Path(value).resolve(); self.root.mkdir(parents=True, exist_ok=True)
        self.admitted = False; self.active = None; self.fault_lock = False
        self.stopping = False
        self.state = None; self.state_received = 0.; self.pair = None; self.pair_received = 0.
        self.pending = [{}, {}]; self.policy = None; self.map_epoch = None; self.error = ''
        self.coverage = 0.; self.completed = 0; self.worker = None; self.child = None
        self.owned_missions = {}
        self.create_service(GetActionOwnership, '/exploration/ownership', self.mission_ownership, callback_group=self.group)
        self.clock_last = 0.; self.worker_log = None; self.map_log = None
        self.lease_pending = None
        self.view_pending = None; self.view_id = ''
        descriptor = ParameterDescriptor(read_only=True)
        self.extended = self.declare_parameter('extended_profile', False, descriptor).value
        self.native_speed = self.declare_parameter('native_max_speed', 1.2, descriptor).value
        if not math.isfinite(self.native_speed) or not .1 <= self.native_speed <= 2.:
            raise ValueError('FUEL native_max_speed must be between .1 and 2 m/s')
        self.camera = CameraView(self) if self.extended else None
        self.camera_timer = self.create_timer(.1, self.camera.ready, clock=self.steady) if self.extended else None
        self.view_service = self.create_client(SetExplorationView, '/planning/fuel_viewpoint',
                            callback_group=self.group) if self.extended else None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(VoxelGrid, '/planning/voxel_grid', qos)
        self.events = self.create_publisher(String, '/exploration/status', 10)
        self.create_subscription(UAVState, '/uav_state', self.on_state, 10)
        for i, topic in enumerate(['/planning/fuel_observed_grid', '/planning/fuel_raw_grid']):
            self.create_subscription(VoxelGrid, topic, lambda msg, kind=i: self.on_map(msg, kind), qos)
        self.nav = ActionClient(self, NavigateTo, '/navigate', callback_group=self.group)
        self.lease = self.create_client(ReserveNavigation, '/navigation/reserve', callback_group=self.group)
        self.ownership = self.create_client(GetActionOwnership, '/navigation/ownership', callback_group=self.group)
        self.server = ActionServer(self, Explore, '/explore', execute_callback=self.execute,
                                   goal_callback=self.admit, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=self.group)
        self.create_service(Trigger, '/exploration/management_idle', self.management_idle,
                            callback_group=self.group)

    def management_idle(self, request, response):
        """Report actual idle ownership without altering a task or issuing a flight request."""
        del request
        response.success = bool(
            not self.admitted and self.active is None and not self.fault_lock and
            self.worker is None and self.lease_pending is None and
            getattr(self, 'child_pending', None) is None and
            self.view_pending is None and not self.view_id)
        response.message = 'idle' if response.success else 'owned_or_unresolved_work'
        return response

    def mission_ownership(self, request, response):
        response.goal_id=request.goal_id
        response.state=self.owned_missions.get(request.goal_id, GetActionOwnership.Response.UNKNOWN)
        return response

    def on_state(self, msg):
        self.state = msg; self.state_received = time.monotonic()

    def fresh(self, msg, receipt):
        now = self.get_clock().now().nanoseconds / 1e9
        return stamp(msg) > 0 and -.2 <= now-stamp(msg) <= 1. and time.monotonic()-receipt <= 1.

    def ready(self):
        s = self.state
        camera_ready = (not self.extended or (self.camera.ready() and
                        self.view_service.service_is_ready() and s and
                        abs(s.roll) <= .2 and abs(s.pitch) <= .2))
        return bool(camera_ready and s and s.connected and s.armed and s.pose_valid and s.velocity_valid and
                    s.attitude_valid and s.flight_mode_name == 'OFFBOARD' and
                    s.header.frame_id == 'odom' and self.fresh(s, self.state_received) and
                    all(math.isfinite(x) for x in [s.position_x, s.position_y, s.position_z,
                        s.velocity_x, s.velocity_y, s.velocity_z, s.yaw]) and
                    .5 <= s.position_z <= 2.5 and self.pair and self.fresh(self.pair[0], self.pair_received))

    def on_map(self, msg, kind):
        if self.active and not msg.valid:
            self.error = self.error or 'SOURCE_MAP_INVALID'
        key = identity(msg)
        self.pending[kind][key] = msg
        while len(self.pending[kind]) > 8:
            del self.pending[kind][next(iter(self.pending[kind]))]
        other = self.pending[1-kind].get(key)
        if other is None:
            return
        pair = (msg, other) if kind == 0 else (other, msg)
        # A late older scan cannot replace the current source pair.
        if self.pair and key[:2] == identity(self.pair[0])[:2] and msg.revision <= self.pair[0].revision:
            return
        self.pair = pair; self.pair_received = time.monotonic()
        if not self.active or self.policy is None:
            return
        try:
            if (msg.session_id, msg.generation, geometry(msg)) != self.map_epoch:
                raise ValueError('SOURCE_MAP_EPOCH_CHANGED')
            self.publish_pair(pair)
        except Exception as exc:
            self.error = self.error or str(exc)

    def publish_pair(self, pair):
        _, b, self.coverage = self.policy.apply(*pair, reserve_unknown=self.extended)
        filtered = copy.deepcopy(pair[1]); filtered.session_id = self.mission_id
        filtered.data = b.tolist(); self.pub.publish(filtered)
        record = dict(receipt=time.monotonic(), clock=self.get_clock().now().nanoseconds,
                      identity=identity(pair[0]), inflation=pair[1].inflation_radius, coverage=self.coverage,
                      eligible_sha=self.policy.eligible_sha,
                      raw=base64.b64encode(bytes(x % 256 for x in pair[0].data)).decode(),
                      inflated=base64.b64encode(bytes(x % 256 for x in pair[1].data)).decode())
        self.map_log.write(json.dumps(record, separators=(',', ':'))+'\n')

    def admit(self, req):
        try:
            if self.stopping or self.admitted or self.fault_lock or not self.ready() or not self.nav.server_is_ready():
                if self.extended:
                    self.get_logger().warning("exploration readiness rejected: "+self.camera.reason)
                return GoalResponse.REJECT
            if not math.isfinite(req.timeout_sec) or not 5. <= req.timeout_sec <= 900. or len(req.forbidden_boxes) % 2 or len(req.forbidden_boxes) > 32:
                return GoalResponse.REJECT
            policy = Policy(self.pair[0], point(req.min_bound), point(req.max_bound),
                            [point(p) for p in req.forbidden_boxes])
            policy.apply(*self.pair)
            if not policy.contains(self.position()):
                return GoalResponse.REJECT
            self.admitted = True
            return GoalResponse.ACCEPT
        except Exception as exc:
            self.get_logger().warning('exploration rejected: '+str(exc))
            return GoalResponse.REJECT

    def position(self):
        return [self.state.position_x, self.state.position_y, self.state.position_z]

    async def pause(self, seconds=.05):
        future = Future()
        timer = self.create_timer(seconds, lambda: future.set_result(True) if not future.done() else None,
                                  callback_group=self.group, clock=self.steady)
        try:
            await future
        finally:
            self.destroy_timer(timer)

    async def bounded(self, future, seconds=3.):
        end = time.monotonic()+seconds
        while not future.done() and time.monotonic() < end:
            await self.pause()
        if not future.done():
            raise RuntimeError('RPC_TIMEOUT')
        return future.result()

    async def reserve(self, child='', active=True):
        if not self.lease.service_is_ready():
            raise RuntimeError('LEASE_SERVICE_UNAVAILABLE')
        self.lease_pending = self.lease.call_async(ReserveNavigation.Request(
            mission_id=self.mission_id, child_goal_id=child, active=active))
        response = await self.bounded(self.lease_pending)
        self.lease_pending = None
        if not response.success:
            raise RuntimeError('LEASE_DENIED:'+response.message)

    def check(self):
        if self.stopping:
            raise RuntimeError('NODE_SHUTDOWN')
        clock = self.get_clock().now().nanoseconds/1e9
        if clock < self.clock_last:
            self.error = self.error or 'CLOCK_RESET'
        self.clock_last = clock
        if self.active.is_cancel_requested:
            raise RuntimeError('CANCELED')
        if self.error:
            raise RuntimeError(self.error)
        if not self.ready():
            if self.extended:
                self.event('INPUT_UNAVAILABLE', camera=self.camera.diagnostics(),
                    state_source_age=clock-stamp(self.state) if self.state else None,
                    map_source_age=clock-stamp(self.pair[0]) if self.pair else None)
            raise RuntimeError('SOURCE_STATE_OR_MAP_STALE' +
                               (':'+self.camera.reason if self.extended else ''))
        if clock-self.start_source > self.active.request.timeout_sec or time.monotonic()-self.start_wall > 1800.:
            raise RuntimeError('MISSION_DEADLINE')
        if self.worker and self.worker.poll() is not None:
            code=self.worker.returncode
            raise RuntimeError('NATIVE_RESOURCE_EXHAUSTED' if code==3 else 'NATIVE_CPU_LIMIT' if code==-24 else 'NATIVE_WORKER_EXIT')

    def event(self, phase, **data):
        row = dict(phase=phase, receipt=time.monotonic(), source=self.get_clock().now().nanoseconds,
                   coverage=self.coverage, completed=self.completed, **data)
        self.journal.write(json.dumps(row)+'\n'); self.journal.flush()
        self.events.publish(String(data=json.dumps(row)))
        if self.active:
            self.active.publish_feedback(Explore.Feedback(phase=phase, coverage=self.coverage,
                                                          completed_viewpoints=self.completed))

    async def release_child(self, child_id):
        end = time.monotonic()+12.
        while time.monotonic() < end:
            answer = await self.bounded(self.ownership.call_async(GetActionOwnership.Request(goal_id=child_id)))
            if answer.goal_id == child_id and answer.state == GetActionOwnership.Response.RELEASED:
                self.child = None
                return
            await self.pause(.1)
        raise RuntimeError('NAVIGATION_STOP_UNCONFIRMED')

    async def clear_view(self):
        if not self.extended or not self.view_id:
            return
        if self.view_pending is not None:
            await self.bounded(self.view_pending, 12.)
            self.view_pending = None
        request = SetExplorationView.Request(mission_id=self.mission_id,
                      navigation_goal_id=self.view_id, active=False)
        self.view_pending = self.view_service.call_async(request)
        response = await self.bounded(self.view_pending)
        self.view_pending = None
        if not response.success:
            raise RuntimeError('VIEW_RELEASE_UNCONFIRMED:'+response.message)
        self.event('VIEW_BINDING_RELEASED', child_id=self.view_id)
        self.view_id = ''

    async def bind_view(self, child_id, target, yaw):
        if not self.extended:
            return
        if self.view_id or self.view_pending is not None or not math.isfinite(yaw):
            raise RuntimeError('VIEW_BINDING_INVALID')
        request = SetExplorationView.Request(mission_id=self.mission_id,
            navigation_goal_id=child_id, active=True, target_x=float(target[0]),
            target_y=float(target[1]), target_z=float(target[2]), target_yaw=float(yaw))
        request.header = copy.deepcopy(self.state.header)
        self.view_id = child_id
        self.view_pending = self.view_service.call_async(request)
        response = await self.bounded(self.view_pending)
        self.view_pending = None
        if not response.success:
            raise RuntimeError('VIEW_BINDING_REJECTED:'+response.message)
        self.event('VIEW_BINDING', child_id=child_id, target=target, target_yaw=yaw)

    async def navigate(self, target, phase, yaw=None, frontier=None):
        self.check()
        if not self.policy.contains(target):
            raise RuntimeError('VIEWPOINT_OUTSIDE_POLICY')
        child_id = uuid.uuid4().hex
        await self.reserve(child_id)
        if self.extended:
            await self.bind_view(child_id, target, self.state.yaw if yaw is None else yaw)
        self.event(phase, target=target, child_id=child_id)
        req = NavigateTo.Goal(target_x=target[0], target_y=target[1], target_z=target[2],
                             max_speed=self.native_speed if self.extended else .6)
        self.child_result = None
        future = self.nav.send_goal_async(req, goal_uuid=UUID(uuid=list(bytes.fromhex(child_id))))
        # An unresolved admission remains owned; cleanup waits for its exact response.
        self.child_pending = future; self.child_id = child_id
        self.child = await self.bounded(future)
        self.child_pending = None
        if not self.child.accepted:
            self.child = None
            raise RuntimeError('NAVIGATION_REJECTED')
        self.child_result = self.child.get_result_async()
        while not self.child_result.done():
            self.check(); await self.pause()
        reply = self.child_result.result()
        await self.release_child(child_id)
        await self.clear_view()
        if reply.status != 4 or not reply.result.success:
            raise RuntimeError('NAVIGATION_FAILED:'+reply.result.message)
        error = math.dist(self.position(), target)
        self.event('ARRIVED', target=target, error=error, child_id=child_id)
        if error > .11:
            raise RuntimeError('ARRIVAL_ERROR')
        if self.extended and phase == 'VIEWPOINT':
            arrived = self.get_clock().now().nanoseconds
            end = time.monotonic()+4.
            while time.monotonic() < end:
                self.check()
                witness = self.camera.witness(frontier, arrived)
                yaw_error = abs(math.remainder(self.state.yaw-yaw, 2*math.pi))
                if witness['after_arrival'] and witness['visible_count'] >= 4 and yaw_error <= .08:
                    frame = self.camera.latest
                    files = {}
                    for key in ('image', 'info'):
                        path = self.directory/(child_id+'-camera-'+key+'.cdr')
                        payload = serialize_message(frame[key]);path.write_bytes(payload)
                        files[key] = dict(file=path.name, sha256=hashlib.sha256(payload).hexdigest())
                    self.event('CAMERA_VIEW_VERIFIED', child_id=child_id, yaw_error=yaw_error,
                               target_yaw=yaw, witness=witness, frames=files)
                    break
                await self.pause(.05)
            else:
                raise RuntimeError('CAMERA_VIEW_NOT_REACHED')
        await self.pause(1.)
        self.check()

    async def decide(self, index):
        self.check(); a, b, _ = self.policy.apply(*self.pair, reserve_unknown=self.extended)
        request_id = uuid.uuid4().hex
        raw = self.pair[0]; s = self.state
        row = dict(request_id=request_id, size=[raw.size_x, raw.size_y, raw.size_z],
                   resolution=raw.resolution, origin=point(raw.origin),
                   min_bound=self.policy.low.tolist(), max_bound=self.policy.high.tolist(),
                   raw=a.tolist(), inflated=b.tolist(), inflation_radius=self.pair[1].inflation_radius, position=self.position(),
                   velocity=[s.velocity_x, s.velocity_y, s.velocity_z], yaw=s.yaw,
                   source_identity=identity(raw), source_stamp=stamp(s))
        source = self.directory / f'{index:03d}-{request_id}.json'
        source.write_text(json.dumps(row))
        self.worker.stdin.write(str(source)+'\n'); self.worker.stdin.flush()
        result = Path(str(source)+'.result.yaml'); start = time.monotonic()
        while not result.is_file():
            self.check()
            if time.monotonic()-start > 4.:
                raise RuntimeError('NATIVE_DECISION_DEADLINE')
            await self.pause(.02)
        data = yaml.safe_load(result.read_text())
        if data['request_id'] != request_id or data['wall_sec'] > 4. or data['cpu_total_sec'] > 120. or data['max_rss_kib'] > 768*1024:
            raise RuntimeError('NATIVE_RESPONSE_INVALID')
        self.event('NATIVE_DECISION', request=str(source.name), decision=data['reason'],
                   updates=data['updates'], frontier_count=len(data['frontiers']),
                   global_tour=data['global_tour'], rss_kib=data['max_rss_kib'], wall_sec=data['wall_sec'])
        return data

    async def cleanup(self, leased):
        failures=[]
        # A timed-out active reservation can still succeed later. Resolve its
        # exact response before releasing; otherwise retain fault ownership.
        try:
            if self.lease_pending is not None:
                response = await self.bounded(self.lease_pending, 12.)
                self.lease_pending = None
                leased = leased or response.success
        except Exception as exc:
            failures.append('LEASE_ADMISSION_UNCONFIRMED:'+str(exc))
        try:
            if self.child_pending is not None:
                self.child = await self.bounded(self.child_pending, 12.)
                self.child_pending = None
            if self.child and self.child.accepted:
                if not self.child_result or not self.child_result.done():
                    await self.bounded(self.child.cancel_goal_async())
                    if self.child_result is None:
                        self.child_result = self.child.get_result_async()
                    await self.bounded(self.child_result, 12.)
                await self.release_child(self.child_id)
        except Exception as exc:
            failures.append(str(exc))
        try:
            # A binding is cleared only after the exact navigation owner released.
            if self.child is None and self.child_pending is None:
                await self.clear_view()
        except Exception as exc:
            failures.append(str(exc))
        # Always reap our compute process, including an unresolved navigation STOP.
        try:
            if self.worker:
                if self.worker.poll() is None:
                    try:
                        self.worker.stdin.write('STOP\n'); self.worker.stdin.flush()
                    except BrokenPipeError:
                        pass
                    end=time.monotonic()+2.
                    while self.worker.poll() is None and time.monotonic()<end:
                        await self.pause()
                    if self.worker.poll() is None:
                        self.worker.terminate();end=time.monotonic()+2.
                        while self.worker.poll() is None and time.monotonic()<end:
                            await self.pause()
                        if self.worker.poll() is None:
                            self.worker.kill();await self.pause(.1)
                self.worker.wait(timeout=.5)
                self.event('WORKER_REAPED',pid=self.worker.pid,returncode=self.worker.returncode)
                self.worker.stdin.close();self.worker=None
        except Exception as exc:
            failures.append(str(exc))
        if leased and not failures:
            try:
                await self.reserve(active=False)
            except Exception as exc:
                failures.append(str(exc))
        # On unresolved STOP retain the parent lease and the last map so that
        # the coordinator can complete its stop protocol; never admit new work.
        if not failures:
            invalid=VoxelGrid();invalid.header.frame_id='odom';invalid.session_id=self.mission_id
            self.pub.publish(invalid)
        if failures:
            raise RuntimeError(';'.join(failures))

    async def execute(self, goal):
        self.active=goal;self.mission_id=bytes(goal.goal_id.uuid).hex()
        self.owned_missions[self.mission_id]=GetActionOwnership.Response.OWNED
        while len(self.owned_missions)>64:
            del self.owned_missions[next(iter(self.owned_missions))]
        self.directory=self.root/self.mission_id;self.directory.mkdir()
        self.journal=(self.directory/'events.jsonl').open('x')
        self.map_log=gzip.open(self.directory/'source-maps.jsonl.gz','xt',compresslevel=1)
        self.worker_log=(self.directory/'native-worker.log').open('x')
        self.start_wall=time.monotonic();self.start_source=self.get_clock().now().nanoseconds/1e9
        self.clock_last=self.start_source;self.error='';self.completed=0
        self.child=None;self.child_pending=None;self.child_result=None
        self.lease_pending=None
        self.view_pending=None;self.view_id=''
        self.policy=None;leased=False;success=False;reason='INITIALIZATION_FAILED'
        try:
            await self.reserve();leased=True
            req=goal.request
            self.policy=Policy(self.pair[0], point(req.min_bound), point(req.max_bound), [point(p) for p in req.forbidden_boxes])
            self.map_epoch=(self.pair[0].session_id,self.pair[0].generation,geometry(self.pair[0]))
            _,_,self.coverage=self.policy.apply(*self.pair, reserve_unknown=self.extended)
            home=self.position();initial=self.coverage
            home_yaw=self.state.yaw
            raw,inflated=self.pair
            (self.directory/'initial-source.json').write_text(json.dumps(dict(identity=identity(raw),
                inflation=inflated.inflation_radius,raw=list(raw.data),inflated=list(inflated.data),
                receipt=self.pair_received,clock=self.get_clock().now().nanoseconds)))
            (self.directory/'mission.json').write_text(json.dumps(dict(mission_id=self.mission_id,
                home=home, min_bound=point(req.min_bound),max_bound=point(req.max_bound),
                forbidden_boxes=[point(p) for p in req.forbidden_boxes],return_home=req.return_home,
                timeout_sec=req.timeout_sec,initial_coverage=initial,eligible_sha=self.policy.eligible_sha)))
            # Publish the admitted initial policy map before native computation;
            # never wait for a later scan to replace the previous invalid epoch.
            self.publish_pair(self.pair)
            executable=Path(get_package_prefix('easyfly_fuel_exploration'))/'lib/easyfly_fuel_exploration/fuel_worker'
            command=[str(executable),str(self.directory)]
            if self.extended:
                config={**self.camera.sample()['configuration'], 'speed':self.native_speed}
                file=self.directory/'camera-config.yaml';file.write_text(yaml.safe_dump(config))
                command.append(str(file))
                self.event('EXTENDED_PROFILE', configuration=config, writers=self.camera.writers)
            self.worker=subprocess.Popen(command,stdin=subprocess.PIPE,
                stdout=self.worker_log,stderr=subprocess.STDOUT,text=True,start_new_session=True,
                env={**os.environ,'OMP_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1'})
            ticks=Path(f'/proc/{self.worker.pid}/stat').read_text().rsplit(')',1)[1].split()[19]
            (self.directory/'worker-identity.json').write_text(json.dumps(dict(pid=self.worker.pid,start_ticks=ticks,
                executable=str(executable),sha256=hashlib.sha256(executable.read_bytes()).hexdigest())))
            self.event('START',initial_coverage=initial)
            visited=[]
            for index in range(32):
                self.check()
                if self.coverage >= .90:
                    success=True;reason='COVERAGE_COMPLETE';break
                data=await self.decide(index)
                if data['reason']!='NEXT_VIEWPOINT':
                    reason=data['reason'];break
                target=data['goal']
                if any(math.dist(target,p)<.15 for p in visited):
                    reason='REPEATED_VIEWPOINT_WITHOUT_COMPLETION';break
                await self.navigate(target,'VIEWPOINT',data.get('yaw'),data.get('visible_frontier'))
                visited.append(target);self.completed+=1
                self.event('VIEWPOINT_COMPLETE',target=target)
            else:
                reason='VIEWPOINT_BUDGET'
            if self.coverage >= .90:
                success=True;reason='COVERAGE_COMPLETE'
            if success and req.return_home:
                if not self.extended or math.dist(self.position(),home)>.11:
                    await self.navigate(home,'RETURN_HOME',home_yaw)
            self.check()
        except Exception as exc:
            success=False;reason=str(exc)
        try:
            await self.cleanup(leased)
        except Exception as exc:
            self.fault_lock=True;success=False;reason+=';CLEANUP_UNCONFIRMED:'+str(exc)
        if not self.fault_lock:
            self.owned_missions[self.mission_id]=GetActionOwnership.Response.RELEASED
        self.event('TERMINAL',success=success,reason=reason,fault_lock=self.fault_lock)
        result=Explore.Result(success=success,reason=reason,coverage=self.coverage,completed_viewpoints=self.completed)
        if goal.is_cancel_requested:
            goal.canceled()
        elif success:
            goal.succeed()
        else:
            goal.abort()
        self.policy=None;self.active=None;self.admitted=False
        self.map_log.close();self.journal.close();self.worker_log.close()
        return result


def main():
    # Keep the context valid until the executor has drained the owned action.
    # Native SIGINT shutdown otherwise races WaitSet construction in rclpy.
    stopping = False

    def request_stop(*_):
        nonlocal stopping
        stopping = True

    previous = {sig: signal.signal(sig, request_stop)
                for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    executor = None
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = Exploration()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        while rclpy.ok():
            if stopping:
                node.stopping = True
                if not node.admitted:
                    break
            executor.spin_once(timeout_sec=.1)
    finally:
        if executor is not None:
            if node is not None:
                executor.remove_node(node)
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)

# Copyright 2026 EasyFly Project
#
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

"""Real GCOPTER process with synthetic inputs; these are not flight results."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time

from ament_index_python.packages import get_package_prefix
from easyfly_msgs.msg import NavigationSession, PolynomialTrajectory, UAVState, VoxelGrid
from easyfly_msgs.srv import SetNavigationSession, SetExplorationView
import pytest
from quadrotor_msgs.msg import PositionCommand
import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data


class Harness:
    def __init__(self, tmp_path, allow_policy=False, dynamic=False, native_fuel=False):
        os.environ["ROS_DOMAIN_ID"] = "197"
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        rclpy.init()
        self.node = rclpy.create_node("gcopter_component_fixture")
        self.log = tempfile.TemporaryFile(mode="w+")
        executable = (
            Path(get_package_prefix("easyfly_trajectory_optimizer"))
            / "lib/easyfly_trajectory_optimizer/gcopter_navigation"
        )
        self.process = subprocess.Popen(
            [
                str(executable),
                "--ros-args",
                "-p",
                "use_sim_time:=false",
                "-p",
                "evidence_directory:=" + str(tmp_path),
                "-p",
                "allow_idle_policy_map_epoch:=" + str(allow_policy).lower(),
                "-p", "dynamic_map:=" + str(dynamic).lower(),
                "-p", "fuel_native_trajectory:=" + str(native_fuel).lower(),
            ],
            stdout=self.log,
            stderr=subprocess.STDOUT,
        )
        self.state_pub = self.node.create_publisher(UAVState, "/uav_state", 10)
        self.qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.node.create_publisher(VoxelGrid, "/planning/voxel_grid", self.qos)
        self.commands, self.plans, self.faults = [], [], []
        self.replans = []
        self.position, self.velocity = [0.0, 0.0, 1.3], [0.0, 0.0, 0.0]
        self.valid, self.publish_map, self.revision = True, True, 1
        self.generation = 1
        self.map_session = 'a'*32 if native_fuel else 'recorded-map-session'
        self.origin_x = -2.0
        self.state_delay=0.;self.measurement_bias=0.
        self.attitude = True
        self.cells = [0] * 30 * 30 * 18
        self.subscriptions = [
            self.node.create_subscription(
                PositionCommand, "/position_cmd", self.command, qos_profile_sensor_data
            ),
            self.node.create_subscription(
                PolynomialTrajectory, "/planning/gcopter_trajectory", self.plans.append, self.qos
            ),
            self.node.create_subscription(
                NavigationSession, "/planning/navigation_fault", self.faults.append, self.qos
            ),
            self.node.create_subscription(
                NavigationSession, "/planning/gcopter_replan", self.replans.append, self.qos
            ),
        ]
        self.client = self.node.create_client(SetNavigationSession, "/planning/gcopter_session")
        self.timer = self.node.create_timer(0.02, self.inputs)
        self.wait(self.client.service_is_ready, 10.0)
        self.spin(1.0)

    def command(self, message):
        self.commands.append((time.monotonic(), message))
        self.position = [message.position.x, message.position.y, message.position.z]
        self.velocity = [message.velocity.x, message.velocity.y, message.velocity.z]

    def inputs(self):
        stamp = self.node.get_clock().now().to_msg()
        state = UAVState()
        state.header.frame_id, state.header.stamp = "odom", stamp
        position,velocity=self.position,self.velocity
        if self.state_delay:
            source=self.node.get_clock().now()-Duration(seconds=self.state_delay)
            state.header.stamp=source.to_msg()
            history=[c for _,c in self.commands if
                     c.header.stamp.sec*10**9+c.header.stamp.nanosec<=source.nanoseconds]
            if history:
                c=history[-1]
                position=[c.position.x,c.position.y,c.position.z]
                velocity=[c.velocity.x,c.velocity.y,c.velocity.z]
            else:position,velocity=[0.,0.,1.3],[0.,0.,0.]
        state.connected = state.armed = state.pose_valid = state.velocity_valid = True
        state.flight_mode = 3
        state.attitude_valid = self.attitude
        state.position_x, state.position_y, state.position_z = position
        state.position_y+=self.measurement_bias
        state.velocity_x, state.velocity_y, state.velocity_z = velocity
        self.state_pub.publish(state)
        if self.publish_map:
            grid = VoxelGrid()
            grid.header.frame_id, grid.header.stamp = "odom", stamp
            grid.session_id, grid.generation, grid.revision = (
                self.map_session,
                self.generation,
                self.revision,
            )
            grid.valid, grid.resolution, grid.inflation_radius = self.valid, 0.2, 0.65
            grid.origin.x, grid.origin.y = self.origin_x, -3.0
            grid.size_x, grid.size_y, grid.size_z = 30, 30, 18
            grid.data = self.cells
            if not self.valid:
                grid.header.stamp.sec = grid.header.stamp.nanosec = 0
                grid.data = [-1] * len(self.cells)
            self.map_pub.publish(grid)

    def spin(self, seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            assert self.process.poll() is None, "planner exited"

    def wait(self, predicate, seconds=5.0):
        until = time.monotonic() + seconds
        while not predicate() and time.monotonic() < until:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            assert self.process.poll() is None, "planner exited"
        assert predicate(), "component observation deadline"

    def session(self, identity, active=True):
        request = SetNavigationSession.Request(
            session_id=identity,
            active=active,
            target_x=2.0,
            target_y=0.0,
            target_z=1.3,
            max_speed=0.6,
        )
        future = self.client.call_async(request)
        self.wait(future.done)
        return future.result()

    def close(self):
        self.node.destroy_timer(self.timer)
        self.process.send_signal(signal.SIGINT)
        try:
            assert self.process.wait(timeout=5) == 0
        finally:
            if self.process.poll() is None:
                self.process.kill()
                self.process.wait(timeout=3)
            self.log.seek(0)
            print(self.log.read(), flush=True)
            self.log.close()
            self.node.destroy_node()
            rclpy.shutdown()


@pytest.fixture
def h(tmp_path):
    value = Harness(tmp_path)
    try:
        yield value
    finally:
        value.close()


def test_real_optimizer_stream_and_exact_session_stop(h):
    reply = h.session("component-positive")
    assert reply.success, reply.message
    h.wait(lambda: len(h.commands) > 4)
    assert h.plans and not h.faults
    plan = h.plans[-1]
    assert plan.navigation_session_id == "component-positive"
    assert plan.max_speed <= 0.6 and plan.max_acceleration <= 0.15 and plan.max_jerk <= 0.3
    assert len(plan.coefficients) == 18 * len(plan.durations)
    assert (
        plan.map_session_id == "recorded-map-session"
        and plan.validated_map_revision >= plan.map_revision
    )
    assert h.session("component-positive", False).success
    h.spin(0.2)
    count = len(h.commands)
    h.spin(0.7)
    assert len(h.commands) == count


def test_cancel_while_worker_is_owned_never_activates_late_result(h, tmp_path):
    reply = h.session("cancel-during-compute")
    assert reply.success, reply.message
    assert h.session("cancel-during-compute", False).success
    h.spin(0.2)
    count = len(h.commands)
    h.spin(2.0)
    assert len(h.commands) == count
    for owner in tmp_path.glob("gcopter-*/job-*.owner"):
        assert not Path("/proc/" + owner.read_text().strip()).exists(), "worker not reaped"
    assert h.session("new-exact-session").success
    h.wait(lambda: any(p.navigation_session_id == "new-exact-session" for p in h.plans))
    assert h.session("new-exact-session", False).success


@pytest.mark.parametrize(
    "fault",
    [
        "invalid-map",
        "map-loss",
        "new-obstacle",
        "current-obstacle",
        "duplicate-output",
        "attitude-loss",
    ],
)
def test_runtime_fault_stops_output_and_requires_exact_stop(h, fault):
    assert h.session("fault-session").success
    h.wait(lambda: len(h.commands) > 3)
    duplicate = None
    if fault == "invalid-map":
        h.valid = False
    elif fault == "map-loss":
        h.publish_map = False
    elif fault == "new-obstacle":
        # Obstacle at a future point, after a polynomial has already activated.
        plan = h.plans[-1]
        t = sum(plan.durations) * 0.65
        piece = 0
        while t > plan.durations[piece]:
            t -= plan.durations[piece]
            piece += 1
        point = [
            sum(plan.coefficients[piece * 18 + a * 6 + j] * t ** (5 - j) for j in range(6))
            for a in range(3)
        ]
        import math

        cell = [math.floor((p - o) / 0.2) for p, o in zip(point, [-2.0, -3.0, 0.0])]
        h.cells[cell[0] + 30 * (cell[1] + 30 * cell[2])] = 100
        h.revision += 1
    elif fault == "attitude-loss":
        h.attitude = False
    elif fault == "current-obstacle":
        import math

        cell = [math.floor((p - o) / 0.2) for p, o in zip(h.position, [-2.0, -3.0, 0.0])]
        h.cells[cell[0] + 30 * (cell[1] + 30 * cell[2])] = 100
        h.revision += 1
    else:
        duplicate = h.node.create_publisher(PositionCommand, "/position_cmd", 1)
    events = h.replans if fault == "new-obstacle" else h.faults
    h.wait(lambda: bool(events), 3.0)
    assert events[-1].session_id == "fault-session"
    assert not (h.faults if fault == "new-obstacle" else h.replans)
    h.spin(0.2)
    count = len(h.commands)
    h.spin(0.6)
    assert len(h.commands) == count
    assert not h.session("different-session").success
    assert not h.session("different-session", False).success
    assert h.session("fault-session", False).success
    if duplicate:
        h.node.destroy_publisher(duplicate)


def test_unknown_goal_is_rejected_without_a_trajectory(h):
    h.cells[20 + 30 * (15 + 30 * 6)] = -1
    h.revision += 1
    h.spin(0.2)
    assert not h.session("unknown-goal").success
    h.spin(0.2)
    assert not h.commands and not h.plans


def test_map_generation_change_latches_restart_requirement(h):
    assert h.session("before-reset").success
    h.wait(lambda: len(h.commands) > 3)
    h.generation += 1
    h.wait(lambda: bool(h.faults))
    assert h.session("before-reset", False).success
    assert not h.session("after-reset").success
    h.generation = 1
    h.spin(0.2)
    assert not h.session("restored-old-generation").success


def test_explicit_policy_epoch_recovery_requires_idle_exact_stop(tmp_path):
    h=Harness(tmp_path,allow_policy=True)
    try:
        assert h.session('first-parent-child').success
        h.wait(lambda:len(h.commands)>3)
        # An active parent/source identity change still fails closed.
        h.map_session='second-policy-uuid'
        h.wait(lambda:bool(h.faults))
        assert not h.session('unreleased-replacement').success
        assert not h.session('wrong-child',False).success
        assert h.session('first-parent-child',False).success
        h.velocity=[0.,0.,0.];h.spin(.3)
        count=len(h.commands);h.spin(.3)
        assert len(h.commands)==count, 'new policy must not resume old navigation'
        assert h.session('explicit-second-child').success
        h.wait(lambda:len(h.commands)>count+3)
        assert h.plans[-1].map_session_id=='second-policy-uuid'
        assert h.session('explicit-second-child',False).success
        # A same-policy generation reset remains latched even after STOP.
        h.generation+=1;h.spin(.2)
        assert not h.session('same-policy-source-reset').success
    finally:h.close()


def test_map_revision_rollback_cannot_restore_an_old_free_map(h):
    h.revision = 10
    h.spin(0.2)
    assert h.session("before-revision-rollback").success
    h.wait(lambda: len(h.commands) > 3)
    h.revision = 9
    h.wait(lambda: bool(h.faults))
    assert h.session("before-revision-rollback", False).success
    h.revision = 11
    h.spin(0.2)
    assert not h.session("after-revision-rollback").success


def test_worker_wall_timeout_cannot_activate_a_late_result(h, tmp_path):
    # Pause only the actual newly owned optimizer process; no substitute worker.
    captured, errors = [], []

    def pause_worker():
        until = time.monotonic() + 5.0
        while time.monotonic() < until:
            for owner in tmp_path.glob("gcopter-*/job-*.owner"):
                value = owner.read_text().strip()
                if not value:
                    continue
                pid = int(value)
                try:
                    executable = Path("/proc") / str(pid) / "exe"
                    assert executable.resolve().name == "gcopter_worker"
                    os.kill(pid, signal.SIGSTOP)
                    status = (Path("/proc") / str(pid) / "status").read_text()
                    if "\nState:\tT" not in status:
                        errors.append("native worker was not paused before exit")
                    captured.append(pid)
                except Exception as error:
                    errors.append(repr(error))
                return
            time.sleep(0.001)
        errors.append("no owned native worker observed")

    watcher = threading.Thread(target=pause_worker)
    watcher.start()
    try:
        assert h.session("deadline-session").success
        watcher.join(timeout=6.0)
        assert not watcher.is_alive() and captured and not errors, errors
        h.wait(lambda: bool(h.faults), 5.0)
        assert not h.plans and not h.commands
        assert not h.session("late-replacement").success
    finally:
        watcher.join(timeout=6.0)
        for pid in captured:
            try:
                os.kill(pid, signal.SIGCONT)
            except ProcessLookupError:
                pass
    h.spin(0.5)
    assert h.session("deadline-session", False).success
    h.spin(0.5)
    assert not h.plans and not h.commands
    assert all(not Path("/proc/" + str(pid)).exists() for pid in captured)
    assert h.session("explicit-recovery").success
    h.wait(lambda: bool(h.plans))
    assert h.session("explicit-recovery", False).success


def test_command_header_and_polynomial_use_the_same_clock_sample(h):
    assert h.session("command-clock-consistency").success
    h.wait(lambda: len(h.commands) >= 16, 5.0)
    plan = h.plans[-1]
    for _, command in h.commands:
        source = command.header.stamp.sec * 10 ** 9 + command.header.stamp.nanosec
        start = plan.header.stamp.sec * 10 ** 9 + plan.header.stamp.nanosec
        t = max(0.0, min((source - start) * 1e-9, sum(plan.durations)))
        piece = 0
        while t > plan.durations[piece] and piece + 1 < len(plan.durations):
            t -= plan.durations[piece]
            piece += 1
        for derivative, actual in enumerate(
            [command.position, command.velocity, command.acceleration]
        ):
            for axis, observed in enumerate([actual.x, actual.y, actual.z]):
                c = list(plan.coefficients[piece * 18 + axis * 6 : piece * 18 + axis * 6 + 6])
                for _ in range(derivative):
                    c = [v * (len(c) - i - 1) for i, v in enumerate(c[:-1])]
                expected = 0.0
                for coefficient in c:
                    expected = expected * t + coefficient
                assert abs(observed - expected) < 1e-6
    assert h.session("command-clock-consistency", False).success


def test_tracking_uses_measured_source_stamp_instead_of_delivery_time(h):
    h.state_delay=.8;h.spin(.2)
    assert h.session('delayed-source-states').success
    h.wait(lambda:len(h.commands)>140,10.)
    assert not h.faults, 'fresh source-aligned poses must not be compared with a later reference'
    assert h.session('delayed-source-states',False).success


def test_source_aligned_tracking_error_still_stops_at_frozen_limit(h):
    assert h.session('actual-tracking-error').success
    h.wait(lambda:len(h.commands)>10)
    h.measurement_bias=.3
    h.wait(lambda:bool(h.faults))
    h.spin(.2);count=len(h.commands);h.spin(.3)
    assert len(h.commands)==count
    assert h.session('actual-tracking-error',False).success


@pytest.mark.parametrize('dynamic,shift,continues', [
    (True, .2, True), (True, .1, False), (False, .2, False)])
def test_rolling_requires_explicit_mode_and_preserves_lattice(tmp_path, dynamic, shift, continues):
    h = Harness(tmp_path, dynamic=dynamic)
    try:
        assert h.session('rolling-owner').success
        h.wait(lambda: len(h.commands) > 4)
        before = len(h.commands)
        h.origin_x += shift
        h.revision += 1
        if continues:
            h.wait(lambda: len(h.commands) > before + 15)
            assert not h.faults and not h.replans
        else:
            h.wait(lambda: bool(h.faults))
            h.spin(.2)
            count = len(h.commands)
            h.spin(.3)
            assert len(h.commands) == count
        assert h.session('rolling-owner', False).success
        if not continues:
            h.origin_x = -2.
            h.velocity = [0., 0., 0.]
            h.revision += 1
            h.spin(.3)
            assert not h.session('restored-old-lattice').success
    finally:
        h.close()


def test_unknown_vehicle_envelope_is_checked_by_actual_owner(tmp_path):
    h = Harness(tmp_path, dynamic=True)
    try:
        # The centre path remains free, but unknown space intersects the full
        # reserved vehicle/tracking envelope beside the start position.
        h.cells[10 + 30 * (18 + 30 * 6)] = -1
        h.revision += 1
        h.spin(.2)
        reply = h.session('unknown-side-envelope')
        assert not reply.success and reply.message == 'occupied_or_unknown_endpoint'
        assert not h.commands and not h.plans
    finally:
        h.close()


def test_native_fuel_binding_exact_owner_yaw_and_stop(tmp_path):
    import math
    value = Harness(tmp_path, allow_policy=True, dynamic=True, native_fuel=True)
    client = value.node.create_client(SetExplorationView, '/planning/fuel_viewpoint')
    parent = 'b'*32
    def bind(active=True, goal=parent, yaw=.5):
        request = SetExplorationView.Request(mission_id='a'*32, navigation_goal_id=goal,
            active=active, target_x=2., target_y=0., target_z=1.3, target_yaw=yaw)
        request.header.frame_id='odom';request.header.stamp=value.node.get_clock().now().to_msg()
        future=client.call_async(request);value.wait(future.done)
        return future.result()
    try:
        value.wait(client.service_is_ready)
        assert not value.session(parent+':1').success
        assert bind().success
        value.spin(.2);assert not value.commands and not value.plans
        assert bind().success
        assert not bind(yaw=.6).success
        assert not bind(goal='c'*32).success
        assert not value.session('c'*32+':1').success
        assert value.session(parent+':1').success
        assert not bind(active=False).success
        value.wait(lambda:len(value.commands)>45,10.)
        assert value.plans and value.plans[-1].navigation_session_id==parent+':1'
        commands=[message for _,message in value.commands]
        assert max(message.yaw for message in commands)>.1
        for first,last in zip(commands,commands[1:]):
            dt=(last.header.stamp.sec-first.header.stamp.sec)+(last.header.stamp.nanosec-first.header.stamp.nanosec)/1e9
            if dt>0:assert abs(math.remainder(last.yaw-first.yaw,2*math.pi))/dt<.301
        assert not bind(active=False,goal='c'*32).success
        assert not value.session('c'*32+':1',active=False).success
        assert value.session(parent+':1',active=False).success
        count=len(value.commands);value.spin(.3);assert len(value.commands)==count
        assert bind(active=False).success
        assert bind(active=False).success
        assert not value.session(parent+':2').success
    finally:
        value.close()

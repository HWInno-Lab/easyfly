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

"""Own one native trajectory transaction, stream measured guidance, and retire its exact UUID."""

import fcntl
import json
import os
from pathlib import Path
import re
import stat
import time
import uuid

from easyfly_msgs.action import FollowTrajectory
from easyfly_msgs.srv import GetTrajectoryState
import numpy as np
from quadrotor_msgs.msg import PositionCommand
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from unique_identifier_msgs.msg import UUID

from .configuration import read
from .geometry import advance, blocked, stamp


def durable(path, value):
    """Persist intent or an observed terminal result before reporting ownership release."""
    temporary = Path(str(path) + '.pending')
    with temporary.open('w') as stream:
        stream.write(json.dumps(value, indent=2, allow_nan=False) + '\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


class Controller:
    """Own explicit hover-to-hover tasks without automatic restoration."""

    def __init__(self, inputs, executor, output, stopping):
        self.inputs, self.executor, self.config = inputs, executor, inputs.config
        self.output, self.stopping = output, stopping
        self.session, self.goal_id = uuid.uuid4().hex, uuid.uuid4()
        self.client = ActionClient(inputs, FollowTrajectory, '/follow_trajectory')
        self.publisher = inputs.create_publisher(PositionCommand, '/position_cmd',
                                                 qos_profile_sensor_data)
        self.status = inputs.create_publisher(String, '/visual_navigation/status', 5)
        self.handle = self.admission = self.result = None
        self.cancel_sent = False
        self.phase, self.native_status = 'preparing', 'not_sent'
        self.reason = ''
        self.success = False
        self.started = time.monotonic()
        self.last_save = 0.
        self.command_count = self.blocked_count = 0
        self.minimum_clearance_reason = []
        self.controller_session = None
        self.lock_fd = None
        self.ledger_path = None
        self.position = self.velocity = None
        self.last_command_ns = 0
        self.endpoint = np.array(self.config['target'])
        self.controller_future = None
        self.last_controller_check = 0.
        self.save()

    def save(self):
        """Expose exact native ownership and measured source provenance in a bounded status."""
        self.last_save = time.monotonic()
        busy = self.native_status not in ('not_sent', 'rejected', 'terminal')
        value = dict(session=self.session, goal_id=self.goal_id.hex,
                     controller_session=self.controller_session, phase=self.phase,
                     native_status=self.native_status, busy=busy, success=self.success,
                     cancel_sent=self.cancel_sent, reason=self.reason[:2048],
                     source_ns=self.inputs.clock_ns, visual_samples=self.inputs.samples,
                     visual_source_ns=(self.inputs.visual['source_ns']
                                       if self.inputs.visual else None),
                     commands=self.command_count, obstacle_holds=self.blocked_count,
                     last_obstacle_reasons=self.minimum_clearance_reason,
                     endpoint=self.endpoint.tolist(), elapsed_s=time.monotonic() - self.started,
                     source_writers=self.inputs.writers)
        durable(self.output / 'status.json', value)
        if self.ledger_path:
            durable(self.ledger_path, value)
        self.status.publish(String(data=json.dumps(value)))

    def poll(self):
        """Preserve uncertain ownership on transport errors; release only on terminal replies."""
        if (self.admission is not None and self.native_status == 'admission_pending' and
                self.admission.done()):
            try:
                self.handle = self.admission.result()
                if self.handle is None:
                    raise RuntimeError('missing native action response')
                if not self.handle.accepted:
                    self.native_status = 'rejected'
                else:
                    self.native_status = 'active'
                    self.result = self.handle.get_result_async()
            except Exception as error:
                self.native_status = 'ownership_unknown'
                self.reason = str(error)
        if self.result is not None and self.result.done() and self.native_status == 'active':
            try:
                result = self.result.result()
                if result.status not in (4, 5, 6):
                    raise RuntimeError('native result not terminal')
                self.native_status = 'terminal'
                self.success = result.status == 4 and result.result.success
                self.reason = result.result.message
            except Exception as error:
                self.native_status = 'ownership_unknown'
                self.reason = str(error)

    def pump(self, seconds=.01):
        """Service callbacks without automatically continuing a command stream."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.005)
            self.poll()

    def ready(self):
        """Wait for current inputs and native services before sending an intent."""
        deadline = time.monotonic() + 15.
        settled_since = None
        while time.monotonic() < deadline and not self.stopping.is_set():
            self.pump()
            try:
                current = self.inputs.update()
                state, position, velocity = current[:3]
                settled = (state.armed and state.flight_mode == 3 and
                           np.linalg.norm(velocity) < .08 and
                           abs(state.roll) < .04 and abs(state.pitch) < .04 and
                           abs(position[2] - self.endpoint[2]) < .1)
                if not settled:
                    settled_since = None
                elif settled_since is None:
                    settled_since = time.monotonic()
                if (settled_since is not None and time.monotonic() - settled_since >= .8 and
                        self.inputs.trajectory_state.service_is_ready() and
                        self.client.server_is_ready()):
                    return current
            except RuntimeError as error:
                settled_since = None
                self.reason = str(error)
        raise RuntimeError('visual task prerequisites unavailable: ' + self.reason)

    def controller_state(self):
        """Read actual controller session and stream watermark, not a guessed next sequence."""
        future = self.inputs.trajectory_state.call_async(GetTrajectoryState.Request())
        deadline = time.monotonic() + 3.
        while not future.done() and time.monotonic() < deadline:
            self.pump()
            self.inputs.update()
        if not future.done():
            raise RuntimeError('controller state service deadline exceeded')
        value = future.result()
        if (not re.fullmatch('[0-9a-f]{32}', value.controller_session_id) or
                not -.1 <= (self.inputs.clock_ns - stamp(value.header.stamp)) / 1e9 <= .5):
            raise RuntimeError('invalid native controller identity')
        return value

    def reserve(self, controller):
        """Block unresolved previous ownership in a private per-user/domain ledger."""
        domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
        root = Path('/tmp') / ('easyfly-visual-owner-' + str(os.getuid()))
        root.mkdir(mode=0o700, exist_ok=True)
        metadata = root.lstat()
        if (not stat.S_ISDIR(metadata.st_mode) or metadata.st_uid != os.getuid() or
                stat.S_IMODE(metadata.st_mode) != 0o700):
            raise RuntimeError('visual ownership directory must be private and owned')
        self.lock_fd = os.open(root / (str(domain) + '.lock'),
                               os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
        fcntl.flock(self.lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        ledger = root / (str(domain) + '.json')
        if ledger.exists():
            old = read(ledger)
            if (old.get('busy') and
                    old.get('controller_session') == controller.controller_session_id):
                raise RuntimeError('previous visual ownership unresolved; restart controller')
        if not controller.idle or controller.highest_trajectory_id >= 2**32 - 1:
            raise RuntimeError('native controller busy or stream watermark exhausted')
        self.controller_session = controller.controller_session_id
        self.next_id = controller.highest_trajectory_id + 1
        self.ledger_path = ledger
        self.save()

    def submit(self):
        """Durably record exact goal identity before sending a single native trajectory request."""
        goal = FollowTrajectory.Goal(target_x=float(self.endpoint[0]),
                                     target_y=float(self.endpoint[1]),
                                     target_z=float(self.endpoint[2]),
                                     navigation_session_id=self.session,
                                     acceptance_radius=.15,
                                     timeout=float(self.config['timeout_s']))
        self.native_status = 'admission_pending'
        self.phase = 'admission'
        self.save()
        try:
            self.admission = self.client.send_goal_async(
                goal, goal_uuid=UUID(uuid=list(self.goal_id.bytes)))
        except Exception:
            self.native_status = 'ownership_unknown'
            raise
        deadline = time.monotonic() + 5.
        while self.native_status == 'admission_pending' and time.monotonic() < deadline:
            self.pump()
            self.inputs.update()
            if self.stopping.is_set():
                raise InterruptedError('visual task canceled during admission')
        if self.native_status != 'active':
            raise RuntimeError('native trajectory admission: ' + self.native_status)

    def execute(self):
        """Execute fresh measured guidance in the declared corridor with complete hazard checks."""
        current = self.ready()
        state, position, velocity, _, _, visual = current
        if (not state.armed or state.flight_mode != 3 or np.linalg.norm(velocity) > .15 or
                abs(position[2] - self.endpoint[2]) > .15):
            raise RuntimeError('visual task requires settled offboard hover at its altitude')
        if self.config['mode'] == 'ellipse':
            self.endpoint[:2] = visual['center'][:2]
        self.reserve(self.controller_state())
        self.position, self.velocity = position.copy(), np.zeros(3)
        self.yaw = state.yaw
        self.submit()
        self.phase = 'guiding'
        self.reason = 'metric_visual_guidance'
        self.last_command_ns = self.inputs.clock_ns
        self.last_controller_check = time.monotonic()
        deadline = time.monotonic() + self.config['timeout_s']
        while self.native_status == 'active':
            self.pump()
            if self.native_status != 'active':
                break
            if self.stopping.is_set():
                raise InterruptedError('visual task canceled')
            if time.monotonic() >= deadline:
                raise TimeoutError('visual task wall-clock deadline exceeded')
            state, position, _, semantic, predictions, visual = self.inputs.update()
            if not state.armed or state.flight_mode != 3:
                raise RuntimeError('native offboard control lost')
            low = np.array(self.config['corridor_min'])
            high = np.array(self.config['corridor_max'])
            if np.any(position < low) or np.any(position > high):
                raise RuntimeError('vehicle left the declared control corridor')
            if np.linalg.norm(position - self.position) > .35:
                raise RuntimeError('vehicle tracking error exceeds visual control tube')
            if self.config['mode'] == 'ellipse' and np.linalg.norm(
                    visual['center'][:2] - self.endpoint[:2]) > .15:
                raise RuntimeError('static ellipse target moved during this task')
            dt = (self.inputs.clock_ns - self.last_command_ns) / 1e9
            if dt < .025:
                continue
            if dt > .1:
                raise RuntimeError('control source-time step exceeded 100 ms')
            desired = visual['desired'].copy()
            if np.linalg.norm(self.endpoint - position) < .55:
                desired = self.endpoint.copy()
            # Inspect the full upcoming path plus braking distance, before increasing speed.
            travel = desired - position
            length = np.linalg.norm(travel)
            stopping_distance = float(self.velocity @ self.velocity) / (
                2 * self.config['max_acceleration']) + .3
            end = position + travel * (max(length, stopping_distance) / max(length, 1e-9))
            reasons = blocked(position, end, semantic, predictions, self.inputs.clock_ns)
            self.minimum_clearance_reason = reasons
            self.reason = 'holding_for_obstacle' if reasons else 'metric_visual_guidance'
            self.blocked_count += bool(reasons)
            self.position, self.velocity, acceleration = advance(
                self.position, self.velocity, desired, dt, self.config['max_speed'],
                self.config['max_acceleration'], bool(reasons))
            if np.any(self.position < low) or np.any(self.position > high):
                raise RuntimeError('reference left the declared control corridor')
            command = PositionCommand()
            command.header.stamp.sec, command.header.stamp.nanosec = divmod(
                self.inputs.clock_ns, 10**9)
            command.header.frame_id = 'odom'
            command.navigation_session_id = self.session
            command.trajectory_id = self.next_id
            command.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
            command.position.x, command.position.y, command.position.z = self.position.tolist()
            command.velocity.x, command.velocity.y, command.velocity.z = self.velocity.tolist()
            command.acceleration.x, command.acceleration.y, command.acceleration.z = (
                acceleration.tolist())
            command.yaw = self.yaw
            self.publisher.publish(command)
            self.command_count += 1
            self.last_command_ns = self.inputs.clock_ns
            if time.monotonic() - self.last_save > .2:
                self.save()
            if (self.controller_future is None and
                    time.monotonic() - self.last_controller_check > 1.):
                self.controller_future = self.inputs.trajectory_state.call_async(
                    GetTrajectoryState.Request())
                self.last_controller_check = time.monotonic()
            if self.controller_future is not None:
                if self.controller_future.done():
                    identity = self.controller_future.result().controller_session_id
                    if identity != self.controller_session:
                        raise RuntimeError('controller session changed')
                    self.controller_future = None
                elif time.monotonic() - self.last_controller_check > 2.:
                    raise RuntimeError('controller identity check expired')
        if self.native_status != 'terminal' or not self.success:
            raise RuntimeError('native visual trajectory failed: ' + self.reason)
        self.phase = 'complete'
        self.save()

    def retire(self):
        """Stop commands, cancel the owned goal once, and drain its terminal result."""
        deadline = time.monotonic() + 15.
        while self.native_status not in ('not_sent', 'rejected', 'terminal'):
            self.pump()
            if self.handle is not None and self.handle.accepted and not self.cancel_sent:
                self.cancel_sent = True
                self.save()
                try:
                    self.handle.cancel_goal_async()
                except Exception as error:
                    self.reason += '; cancel transport unknown: ' + str(error)
            if time.monotonic() >= deadline:
                self.reason += '; exact native ownership remains unresolved'
                break
        self.save()

    def close(self):
        """Release native client before executor/context teardown; retain any unresolved ledger."""
        self.client.destroy()
        if self.lock_fd is not None:
            os.close(self.lock_fd)
            self.lock_fd = None

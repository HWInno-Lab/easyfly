#!/usr/bin/env python3
"""Exercise only an installed legacy adapter against local mock ROS peers."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '231'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, PositionTarget
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand

import tempfile
TEMP = tempfile.TemporaryDirectory(prefix="easyfly-interface-regression-")
ROOT = Path(TEMP.name)
OUT = ROOT / 'legacy-bridge'
OUT.mkdir(exist_ok=False)
from ament_index_python.packages import get_package_prefix
EXE = Path(get_package_prefix('ego_to_mavros')) / 'lib/ego_to_mavros/ego_to_mavros_node'
checks = []
observations = {}

def check(name, passed, **evidence):
    checks.append(dict(name=name, passed=bool(passed), **evidence))

def stop(p):
    if p.poll() is None:
        p.send_signal(signal.SIGINT)
        try:
            p.wait(8)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGTERM)
            p.wait(5)
    return p.returncode

class Fixture:
    def __init__(self, name, yaw=False):
        self.name = name
        self.node = rclpy.create_node('mock_' + name)
        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.pose.pose.orientation.w = 1.0
        self.odom.pose.pose.position.x = 1.0
        self.odom.pose.pose.position.y = 2.0
        self.odom.pose.pose.position.z = 3.0
        self.state = State(connected=True, armed=False, mode='MANUAL')
        self.cmd = PositionCommand()
        self.cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        self.cmd.position.x, self.cmd.position.y, self.cmd.position.z = 4.0, 5.0, 6.0
        self.cmd.velocity.x, self.cmd.acceleration.y = 0.2, 0.3
        self.cmd.yaw, self.cmd.yaw_dot = 0.6, 0.4
        self.setpoints, self.relays, self.triggers = [], [], []
        topics = ['/drone_0_planning/pos_cmd', '/mavros/state', '/mavros/local_position/odom',
                  '/mavros/setpoint_raw/local', '/drone_0_visual_slam/odom', '/traj_start_trigger']
        remap = {t: '/legacy_probe/' + name + t for t in topics}
        self.pub_cmd = self.node.create_publisher(PositionCommand, remap[topics[0]], qos_profile_sensor_data)
        self.pub_state = self.node.create_publisher(State, remap[topics[1]], 10)
        self.pub_odom = self.node.create_publisher(Odometry, remap[topics[2]], 10)
        self.node.create_subscription(PositionTarget, remap[topics[3]], self.setpoints.append, 10)
        self.node.create_subscription(Odometry, remap[topics[4]], self.relays.append, 10)
        self.node.create_subscription(PoseStamped, remap[topics[5]], self.triggers.append, 10)
        command = [str(EXE), '--ros-args', '-p', 'enable_yaw_cmd:=' + str(yaw).lower()]
        for source, destination in remap.items():
            command += ['-r', source + ':=' + destination]
        self.log = (OUT / (name + '.log')).open('w')
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        self.pump(2)

    def pump(self, duration, cmd=False):
        end = time.monotonic() + duration
        while time.monotonic() < end and self.process.poll() is None:
            self.odom.header.stamp = self.node.get_clock().now().to_msg()
            self.pub_odom.publish(self.odom)
            self.pub_state.publish(self.state)
            if cmd:
                self.cmd.header.stamp = self.node.get_clock().now().to_msg()
                self.pub_cmd.publish(self.cmd)
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(0.01)

    def ready(self, seed=False):
        if seed:
            self.pump(0.3, cmd=True)
        self.state.armed, self.state.mode = True, 'OFFBOARD'
        self.pump(0.25)

    def close(self):
        code = stop(self.process)
        self.node.destroy_node()
        self.log.close()
        return code

rclpy.init()
try:
    launch_log = (OUT / 'default-launch.log').open('w')
    proc = subprocess.Popen(['ros2', 'launch', 'ego_to_mavros', 'bridge.launch.py'],
                            stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(4)
    launch_alive = proc.poll() is None
    launch_rc = stop(proc)
    launch_log.close()
    launch_text = (OUT / 'default-launch.log').read_text()
    check('default_launch_node_stays_alive', launch_alive and 'process has died' not in launch_text,
          parent_returncode=launch_rc, invalid_parameter_type='InvalidParameterTypeException' in launch_text,
          launch_log=launch_text)

    f = Fixture('first_hold')
    try:
        check('unarmed_no_setpoint', not f.setpoints)
        check('odom_enu_relay', bool(f.relays) and f.relays[-1].header.frame_id == 'world'
              and f.relays[-1].pose.pose.position.x == 1.0)
        f.ready()
        f.pump(0.5)
        check('hold_before_first_command_does_not_crash', f.process.poll() is None and bool(f.setpoints),
              returncode=f.process.poll(), setpoints=len(f.setpoints))
    finally:
        observations['first_hold_exit'] = f.close()

    for yaw in (False, True):
        f = Fixture('seeded_yaw_' + str(yaw).lower(), yaw=yaw)
        try:
            f.ready(seed=True)
            f.pump(0.5, cmd=True)
            s = f.setpoints[-1] if f.setpoints else None
            check('trajectory_enu_passthrough_yaw_' + str(yaw), s is not None and
                  [s.position.x, s.position.y, s.position.z] == [4.0, 5.0, 6.0] and
                  abs(s.velocity.x - 0.2) < 1e-6 and abs(s.acceleration_or_force.y - 0.3) < 1e-6)
            check('yaw_mask_' + str(yaw), s is not None and
                  bool(s.type_mask & PositionTarget.IGNORE_YAW) == (not yaw) and
                  bool(s.type_mask & PositionTarget.IGNORE_YAW_RATE) and
                  (not yaw or abs(s.yaw - 0.6) < 1e-6))
            if yaw:
                continue
            observations['trigger_with_auto_trigger_false'] = len(f.triggers)
            f.cmd.trajectory_flag = 0
            f.cmd.position.x = 90.0
            f.pump(0.2, cmd=True)
            check('nonready_command_ignored', f.setpoints[-1].position.x == 4.0)
            f.odom.pose.pose.position.x, f.odom.pose.pose.position.y = 7.0, 8.0
            f.pump(0.8)
            s = f.setpoints[-1]
            check('timeout_uses_current_hold_position', [s.position.x, s.position.y] == [7.0, 8.0],
                  actual=[s.position.x, s.position.y, s.position.z], current_odom=[7.0, 8.0, 3.0])
            check('timeout_disables_velocity_acceleration', bool(s.type_mask & PositionTarget.IGNORE_VX)
                  and bool(s.type_mask & PositionTarget.IGNORE_AFX))
            f.cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
            f.cmd.position.x = float('nan')
            f.setpoints.clear()
            f.pump(0.3, cmd=True)
            invalid_count = sum(not math.isfinite(s.position.x) for s in f.setpoints)
            check('nonfinite_command_not_forwarded', invalid_count == 0, nonfinite_setpoints=invalid_count)
            f.state.mode = 'POSCTL'
            f.pump(0.3)
            f.setpoints.clear()
            f.pump(0.3)
            check('external_mode_stops_setpoints', not f.setpoints)
        finally:
            observations[f.name + '_exit'] = f.close()
finally:
    rclpy.shutdown()
result = dict(hardware=False, fixture='isolated ROS mocks, no MAVROS or PX4 process',
              domain=231, installed_executable=str(EXE), checks=checks, observations=observations,
              passed=all(c['passed'] for c in checks))
(OUT / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
raise SystemExit(0 if result['passed'] else 1)

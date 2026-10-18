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

"""Actual flight/bridge processes stop MAVROS output after raw telemetry loss."""

from contextlib import ExitStack
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from easyfly_msgs.action import GoToWaypoint, Takeoff
from easyfly_msgs.msg import UAVState
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu


def test_frozen_clock_telemetry_loss_stops_full_output_chain(tmp_path):
    os.environ["ROS_DOMAIN_ID"] = "196"
    rclpy.init()
    node = rclpy.create_node("stop_stream_chain_peer")
    state = State(connected=True, armed=False, mode="AUTO.LOITER")
    clock = Clock()
    clock.clock.sec = 10
    clock_origin = time.monotonic()
    clock_frozen = False
    pose = PoseStamped()
    pose.header.frame_id = "odom"
    pose.header.stamp = clock.clock
    pose.pose.orientation.w = 1.0
    imu = Imu()
    imu.header.stamp = clock.clock
    imu.orientation.w = 1.0
    active_inputs = True
    last_state = None
    setpoints = []
    calls = []
    processes = []
    forced = False

    def armed(request, response):
        calls.append(("arm", request.value))
        state.armed = request.value
        # Fake flight dynamics only: this test is not a real takeoff accuracy
        # test. Real-node state/action/stream boundaries are under test.
        pose.pose.position.z = 1.0 if request.value else 0.0
        response.success = True
        return response

    def mode(request, response):
        calls.append(("mode", request.custom_mode))
        state.mode = request.custom_mode
        response.mode_sent = True
        return response

    def observed(message):
        nonlocal last_state
        last_state = message

    node.create_service(CommandBool, "/mavros/cmd/arming", armed)
    node.create_service(SetMode, "/mavros/set_mode", mode)
    state_pub = node.create_publisher(State, "/mavros/state", 10)
    pose_pub = node.create_publisher(
        PoseStamped, "/mavros/local_position/pose", 10
    )
    imu_pub = node.create_publisher(Imu, "/mavros/imu/data", 10)
    clock_pub = node.create_publisher(Clock, "/clock", 10)
    node.create_subscription(
        UAVState, "/uav_state", observed, qos_profile_sensor_data
    )
    node.create_subscription(
        PoseStamped,
        "/mavros/setpoint_position/local",
        lambda _: setpoints.append(time.monotonic()),
        10,
    )

    def publish():
        # Prepare with distinct samples; freeze only during telemetry loss.
        if not clock_frozen:
            source_ns = 10000000000 + int(
                (time.monotonic() - clock_origin) * 1e9
            )
            clock.clock.sec, clock.clock.nanosec = divmod(
                source_ns, 1000000000
            )
        clock_pub.publish(clock)
        if active_inputs:
            state.header.stamp = clock.clock
            pose.header.stamp = clock.clock
            imu.header.stamp = clock.clock
            state_pub.publish(state)
            pose_pub.publish(pose)
            imu_pub.publish(imu)

    node.create_timer(0.02, publish)
    takeoff = ActionClient(node, Takeoff, "/takeoff")
    waypoint = ActionClient(node, GoToWaypoint, "/goto_waypoint")

    def until(predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            assert all(
                p.poll() is None for p in processes
            ), "node exited early"
            rclpy.spin_once(node, timeout_sec=0.01)
        assert predicate(), "fixture timed out"

    def spin(seconds):
        start = time.monotonic()
        until(lambda: time.monotonic() - start >= seconds, seconds + 1.0)

    def send(client, goal):
        future = client.send_goal_async(goal)
        until(future.done)
        return future.result()

    try:
        with ExitStack() as stack:
            for package, executable, parameters in (
                (
                    "easyfly_mavlink_bridge",
                    "mavlink_bridge_node",
                    [
                        "motion_freshness_s:=0.15",
                        "slow_sensor_freshness_s:=0.2",
                        "setpoint_timeout:=0.15",
                    ],
                ),
                ("easyfly_flight_control", "flight_control_node", []),
            ):
                output = stack.enter_context(
                    (tmp_path / (package + ".log")).open("w")
                )
                command = [
                    str(
                        Path(get_package_prefix(package))
                        / "lib"
                        / package
                        / executable
                    ),
                    "--ros-args",
                    "-p",
                    "use_sim_time:=true",
                ]
                for value in parameters:
                    command += ["-p", value]
                processes.append(
                    subprocess.Popen(
                        command, stdout=output, stderr=subprocess.STDOUT
                    )
                )
            until(
                lambda: takeoff.server_is_ready()
                and waypoint.server_is_ready()
                and last_state is not None
                and last_state.connected
                and last_state.pose_valid
                and last_state.attitude_valid
            )
            # Observer readiness does not prove the flight node has received
            # its own first clock/state sample yet; let DDS discovery settle.
            spin(0.2)
            handle = send(
                takeoff, Takeoff.Goal(target_altitude=1.0, climb_speed=0.5)
            )
            assert handle.accepted
            result = handle.get_result_async()
            until(result.done)
            assert (
                result.result().status == 4 and result.result().result.success
            )
            handle = send(
                waypoint,
                GoToWaypoint.Goal(
                    target_x=10.0,
                    target_z=1.0,
                    max_speed=1.0,
                    acceptance_radius=0.05,
                ),
            )
            assert handle.accepted
            result = handle.get_result_async()
            spin(0.15)
            assert not result.done()
            assert setpoints, "fixture never observed MAVROS output"
            clock_frozen = True
            frozen_source_ns = (
                clock.clock.sec * 1000000000 + clock.clock.nanosec
            )
            active_inputs = False
            until(result.done, timeout=3.0)
            assert (
                result.result().status == 6
                and not result.result().result.success
            )
            spin(
                0.4
            )  # Allow the bridge watchdog and queued DDS samples to drain.
            count = len(setpoints)
            calls_after = list(calls)
            spin(0.3)
            assert (
                len(setpoints) == count
            ), "old setpoint survived frozen-clock telemetry loss"
            assert (
                calls == calls_after
            ), "telemetry loss initiated a compensating flight request"
            assert (
                clock.clock.sec * 1000000000 + clock.clock.nanosec
            ) == frozen_source_ns, "loss window did not keep ROS time frozen"
            clock_frozen = False
            active_inputs = True
            until(
                lambda: (
                    last_state.header.stamp.sec * 1000000000
                    + last_state.header.stamp.nanosec
                )
                > frozen_source_ns
                and last_state.connected
                and last_state.pose_valid
                and last_state.attitude_valid
            )
            spin(0.15)
            assert (
                len(setpoints) == count
            ), "restored feedback resurrected old hover/waypoint"
            assert not send(
                waypoint, GoToWaypoint.Goal(target_x=1.0, target_z=1.0)
            ).accepted
            assert not send(
                takeoff, Takeoff.Goal(target_altitude=1.0)
            ).accepted
            assert not any(
                kind == "arm" and value is False for kind, value in calls
            )
            for process in processes:
                process.send_signal(signal.SIGINT)
            for process in processes:
                assert process.wait(timeout=3.0) == 0
    finally:
        for process in processes:
            if process.poll() is None:
                forced = True
                process.kill()
                process.wait(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
    assert not forced, "forced cleanup is not a PASS"

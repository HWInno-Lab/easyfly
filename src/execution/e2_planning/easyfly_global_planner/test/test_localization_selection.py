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

#
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#

"""An observed OpenVINS pose cannot authorize navigation before frame/map integration."""
import os
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, GoalResponse
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rclpy.qos import QoSProfile, DurabilityPolicy
from easyfly_msgs.action import NavigateTo, FollowTrajectory
from easyfly_msgs.msg import LocalizationStatus
from easyfly_msgs.srv import SetNavigationSession


@pytest.fixture
def ros_runtime():
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor

    context = Context()
    rclpy.init(context=context)
    executor = SingleThreadedExecutor(context=context)
    try:
        yield context, executor
    finally:
        executor.shutdown()
        context.try_shutdown()
        context.destroy()


@pytest.mark.parametrize(
    "backend,strategy",
    [
        ("openvins", "astar"),
        ("openvins", "ego"),
        ("openvins", "combined"),
        ("disabled", "astar"),
        ("fast_livo2", "astar"),
        ("fast_livo2", "ego"),
        ("fast_livo2", "combined"),
        ("orbslam3", "astar"),
        ("orbslam3", "ego"),
        ("orbslam3", "combined"),
    ],
)
def test_unaligned_backend_rejects_navigation_even_with_healthy_looking_status(
    backend, strategy, ros_runtime
):
    context, executor = ros_runtime
    node = rclpy.create_node("localization_admission_mock", context=context)
    children = []

    def admit(request):
        children.append(request)
        return GoalResponse.REJECT

    servers = [
        ActionServer(
            node,
            NavigateTo,
            "/navigate_astar",
            execute_callback=lambda _: None,
            goal_callback=admit,
        ),
        ActionServer(
            node,
            FollowTrajectory,
            "/follow_trajectory",
            execute_callback=lambda _: None,
            goal_callback=admit,
        ),
    ]
    service = node.create_service(
        SetNavigationSession, "/planning/navigation_session", lambda _, response: response
    )
    publisher = node.create_publisher(
        LocalizationStatus,
        "/localization/openvins/status",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    process = subprocess.Popen(
        [
            "ros2",
            "launch",
            "easyfly_global_planner",
            "navigate.launch.py",
            "use_sim_time:=false",
            "localization_backend:=" + backend,
            "strategy:=" + strategy,
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )

    def wait(future):
        deadline = time.monotonic() + 6
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, executor=executor, timeout_sec=0.05)
        assert future.done()
        return future.result()

    client = None
    try:
        client = ActionClient(node, NavigateTo, "/navigate")
        assert client.wait_for_server(timeout_sec=8)
        status = LocalizationStatus(
            backend_id="openvins",
            session_id="test-session",
            state=LocalizationStatus.ESTIMATING,
            pose_valid=True,
            covariance_available=True,
            tracked_frame="livox_imu_frame",
        )
        status.header.frame_id = "openvins_odom"
        status.header.stamp = node.get_clock().now().to_msg()
        status.pose_stamp = status.header.stamp
        publisher.publish(status)
        goal = NavigateTo.Goal(target_x=1.0, target_y=0.0, target_z=1.0, max_speed=1.0)
        assert not wait(client.send_goal_async(goal)).accepted
        parameters = node.create_client(SetParameters, "/navigation_coordinator/set_parameters")
        assert parameters.wait_for_service(timeout_sec=5)
        change = Parameter("localization_backend", Parameter.Type.STRING, "fast_lio")
        result = wait(
            parameters.call_async(SetParameters.Request(parameters=[change.to_parameter_msg()]))
        )
        assert not result.results[0].successful
        assert not wait(client.send_goal_async(goal)).accepted
        assert not children
        names = {name for name, _ in node.get_node_names_and_namespaces()}
        assert "easyfly_global_planner" not in names
        assert "obstacle_supervisor" not in names
    finally:
        try:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
                pytest.fail("navigation launch did not stop on SIGINT")
        finally:
            if client is not None:
                client.destroy()
            for server in servers:
                server.destroy()
            node.destroy_service(service)
            node.destroy_node()


def test_opted_in_coordinator_rejects_identity_removed_after_binding(ros_runtime):
    """Synthetic contract fixture; actual coordinator admission, no simulated aircraft."""
    from ament_index_python.packages import get_package_prefix
    from pathlib import Path
    from nav_msgs.msg import OccupancyGrid
    from rosgraph_msgs.msg import Clock
    from easyfly_msgs.msg import UAVState

    context, executor = ros_runtime
    node = rclpy.create_node("vio_identity_contract_fixture", context=context)
    status = LocalizationStatus(
        backend_id="openvins_navigation",
        session_id="bound-session",
        tracked_frame="base_link",
        state=2,
        pose_valid=True,
        tracking_quality_available=True,
    )
    status.header.frame_id = "odom"
    publisher = node.create_publisher(
        LocalizationStatus,
        "/localization/openvins/navigation_status",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    clock = node.create_publisher(Clock, "/clock", 10)
    state_pub = node.create_publisher(UAVState, "/uav_state", 10)
    grid_pub = node.create_publisher(OccupancyGrid, "/occupancy_grid", 10)
    children = []

    def reject(request):
        children.append(request)
        return GoalResponse.REJECT

    server = ActionServer(
        node, NavigateTo, "/navigate_astar", execute_callback=lambda _: None, goal_callback=reject
    )
    executable = Path(get_package_prefix("easyfly_global_planner")) / (
        "lib/easyfly_global_planner/navigation_coordinator_node"
    )
    process = subprocess.Popen(
        [
            str(executable),
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            "localization_backend:=openvins",
            "-p",
            "openvins_navigation:=true",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, executor=executor, timeout_sec=0.01)

    def publish():
        stamp = rclpy.time.Time(nanoseconds=10_000_000_000).to_msg()
        clock.publish(Clock(clock=stamp))
        status.header.stamp = status.pose_stamp = stamp
        publisher.publish(status)
        state = UAVState(
            connected=True,
            armed=True,
            pose_valid=True,
            attitude_valid=True,
            velocity_valid=True,
            flight_mode=3,
            position_z=1.0,
        )
        state.header.stamp, state.header.frame_id = stamp, "odom"
        state_pub.publish(state)
        grid = OccupancyGrid()
        grid.header.stamp, grid.header.frame_id = stamp, "odom"
        grid.info.resolution, grid.info.width, grid.info.height = 0.5, 8, 8
        grid.info.origin.position.x = grid.info.origin.position.y = -2.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * 64
        grid_pub.publish(grid)

    timer = node.create_timer(0.05, publish)
    client = None
    try:
        client = ActionClient(node, NavigateTo, "/navigate")
        assert client.wait_for_server(timeout_sec=8)
        spin(0.4)
        status.session_id = ""
        spin(0.2)
        future = client.send_goal_async(
            NavigateTo.Goal(target_x=0.5, target_y=0.0, target_z=1.0, max_speed=0.3)
        )
        deadline = time.monotonic() + 5
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, executor=executor, timeout_sec=0.02)
        assert future.done()
        assert not future.result().accepted
        assert not children
    finally:
        try:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
                pytest.fail("owned navigation coordinator forced termination")
        finally:
            if client is not None:
                client.destroy()
            node.destroy_timer(timer)
            server.destroy()
            node.destroy_node()


def test_failed_child_wait_still_destroys_native_action_handles(monkeypatch, ros_runtime):
    """A process-cleanup error must not leak ActionClients into the next context."""
    import sys
    from rclpy.impl.implementation_singleton import rclpy_implementation

    InvalidHandle = rclpy_implementation.InvalidHandle
    real_popen, real_client = subprocess.Popen, ActionClient
    children, clients = [], []

    class InterruptedWait:
        def __init__(self, *args, **kwargs):
            self.process = real_popen(*args, **kwargs)
            children.append(self.process)

        def __getattr__(self, name):
            return getattr(self.process, name)

        def wait(self, **_kwargs):
            raise RuntimeError("injected owned-child wait failure")

    def observed_client(*args, **kwargs):
        client = real_client(*args, **kwargs)
        clients.append(client)
        return client

    monkeypatch.setattr(subprocess, "Popen", InterruptedWait)
    monkeypatch.setattr(sys.modules[__name__], "ActionClient", observed_client)
    try:
        with pytest.raises(RuntimeError, match="injected owned-child wait failure"):
            test_unaligned_backend_rejects_navigation_even_with_healthy_looking_status(
                "openvins", "astar", ros_runtime
            )
    finally:
        for process in children:
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
                pytest.fail("owned child did not stop after injected wait error")
    assert len(clients) == 1
    with pytest.raises(InvalidHandle):
        clients[0].server_is_ready()

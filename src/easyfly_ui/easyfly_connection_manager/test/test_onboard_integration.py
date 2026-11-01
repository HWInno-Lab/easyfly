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
"""Real TLS, ROS and native image algorithm; no FCU, sensor device or flight goals."""

import copy
import hashlib
import os
from pathlib import Path
import threading
import time

import pytest

from easyfly_connection_manager.onboard_client import PlatformClient
from easyfly_connection_manager.onboard_deployment import client_settings, export_client, unit_text
from easyfly_connection_manager.onboard_profiles import template
from easyfly_connection_manager.onboard_runtime import PlatformOwner
from easyfly_connection_manager.onboard_service import PlatformHandler, bootstrap_local
from easyfly_connection_manager.remote_profiles import private_json
from easyfly_connection_manager.remote_service import Server


def test_portable_ground_bundle_and_service_text(tmp_path):
    """Portable credentials preserve verified host identity without exporting the server key."""
    root = tmp_path / "platform"
    server, client_path = bootstrap_local(root, 182, host="aircraft.example.test")
    destination = tmp_path / "ground"
    exported = Path(export_client(root, destination))
    assert (destination / "operator.token").stat().st_mode & 0o777 == 0o600
    assert not (destination / "server.key").exists()
    settings = client_settings(exported)
    assert settings["client_id"] == private_json(client_path)["client_id"]
    assert settings["url"].startswith("https://aircraft.example.test:")
    assert Path(settings["ca_file"]).is_file()
    with pytest.raises(ValueError):
        export_client(root, destination)
    unit = unit_text(server, "/installed path/service")
    assert 'ExecStart="/installed path/service"' in unit
    assert "SendSIGKILL=no" in unit and "KillMode=process" in unit


def test_real_tls_native_algorithm_and_mission_configuration(tmp_path):
    """Configure/start/query/stop a real native algorithm through the deployed-style API."""
    from rclpy.context import Context
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    os.environ["ROS_DOMAIN_ID"] = "182"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    root = tmp_path / "platform"
    server_path, client_path = bootstrap_local(root, 182)
    config = private_json(server_path)
    config["local_only"] = True
    context = Context()
    context.init(domain_id=182)
    camera = Node("onboard_image_fixture", context=context)
    publisher = camera.create_publisher(Image, "/camera/color/image_raw", 10)
    stopping = threading.Event()

    def frames():
        while not stopping.wait(0.05):
            frame = Image()
            frame.header.stamp = camera.get_clock().now().to_msg()
            frame.header.frame_id = "camera_color_optical_frame"
            frame.height, frame.width, frame.encoding, frame.step = 48, 64, "bgr8", 64 * 3
            frame.data = bytes([0, 0, 255]) * (64 * 48)
            publisher.publish(frame)

    camera_thread = threading.Thread(target=frames)
    camera_thread.start()
    server_stopping = threading.Event()
    created = threading.Event()
    errors = []
    owner_ref = []

    def serve():
        owner = None
        service = None
        try:
            owner = PlatformOwner(config)
            owner_ref.append(owner)
            service = Server(config, owner, PlatformHandler)
            service.timeout = 0.05
            created.set()
            while not server_stopping.is_set():
                service.handle_request()
                owner.tick()
        except Exception as error:
            errors.append(error)
            created.set()
        finally:
            if service:
                service.server_close()
            if owner and owner.idle():
                owner.close()

    service_thread = threading.Thread(target=serve)
    service_thread.start()
    client = PlatformClient(client_settings(client_path))
    try:
        assert created.wait(5) and not errors
        snapshot = client.status()
        assert snapshot["v"] == 2 and snapshot["hardware"]
        assert snapshot["state"]["phase"] == "stopped"
        bad = copy.deepcopy(client_settings(client_path))
        token = tmp_path / "wrong.token"
        token.write_text("x" * 48)
        token.chmod(0o600)
        bad["token_file"] = str(token)
        with pytest.raises(ValueError, match="authentication"):
            PlatformClient(bad).status()
        recipe = template()
        recipe["components"]["color_line"]["enabled"] = True
        reply = client.operation(
            snapshot, "configure", dict(recipe=recipe, expected_hash=snapshot["recipe_hash"])
        )
        assert reply["accepted"], reply
        deadline = time.monotonic() + 6
        while True:
            snapshot = client.status()
            if snapshot["conditions"]["observer_alive"] and snapshot["conditions"]["isolated"]:
                break
            assert time.monotonic() < deadline
            time.sleep(0.1)
        reply = client.operation(snapshot, "start", dict(recipe_hash=snapshot["recipe_hash"]))
        assert reply["accepted"], reply
        deadline = time.monotonic() + 25
        while True:
            snapshot = client.status()
            if snapshot["state"]["phase"] == "running":
                break
            assert snapshot["state"]["phase"] not in ("failed", "failed_owned"), snapshot
            assert time.monotonic() < deadline, snapshot
            time.sleep(0.15)
        state = snapshot["state"]["components"]["color_line"]
        diagnostics = client.logs("color_line")
        assert diagnostics["accepted"] and diagnostics["component"] == "color_line"
        assert "color_line" in diagnostics["content"]
        assert state["phase"] == "ready" and state["pid"] > 0
        assert not snapshot["conditions"]["mission_stack_ready"]
        assert not snapshot["mission_execution_enabled"]
        # A second independent ground-client object sees the same owned process.
        other = PlatformClient(client_settings(client_path))
        assert other.status()["state"]["components"]["color_line"]["pid"] == state["pid"]
        content = (
            "schema_version: 1\nframe_id: odom\n"
            "units: {position: m, angle: rad, time: s}\nfailure_policy: abort\nsteps:\n"
            "  - {type: takeoff, altitude: 1.5, timeout: 60}\n"
            "  - {type: land, timeout: 90}\n"
        )
        saved = client.operation(
            snapshot, "mission_save", dict(name="native_fixture.yaml", content=content)
        )
        assert saved["accepted"], saved
        loaded = client.operation(snapshot, "mission_read", dict(name="native_fixture.yaml"))
        assert loaded["content"] == content
        assert loaded["sha256"] == hashlib.sha256(content.encode()).hexdigest()
        blocked = client.operation(
            snapshot,
            "mission_start",
            dict(name=loaded["name"], sha256=loaded["sha256"], confirmed=True),
        )
        assert not blocked["accepted"]
        stopped = client.operation(snapshot, "stop", dict(session=snapshot["state"]["session"]))
        assert stopped["accepted"], stopped
        deadline = time.monotonic() + 12
        while client.status()["state"]["phase"] != "stopped":
            assert time.monotonic() < deadline
            time.sleep(0.1)
    finally:
        # Stop only the exact owned observation fixture if an assertion failed.
        if owner_ref:
            owner = owner_ref[0]
            owner.abort.set()
            if owner.worker_alive():
                owner.worker.join(timeout=3)
            for process in list(owner.children.values()):
                if process.poll() is None:
                    import signal

                    process.send_signal(signal.SIGINT)
                    process.wait(timeout=8)
            owner.state["phase"] = "stopping"
        server_stopping.set()
        service_thread.join(timeout=5)
        stopping.set()
        camera_thread.join(timeout=3)
        camera.destroy_node()
        context.shutdown()
    assert not errors


def test_qt_recipe_editor_and_review_invalidation():
    """A native form edits algorithms/tasks and preserves drafts during status refresh."""
    from PyQt5.QtCore import Qt
    from PyQt5.QtWidgets import QApplication
    from easyfly_connection_manager.onboard_window import PlatformWindow

    app = QApplication.instance() or QApplication([])
    window = PlatformWindow()
    try:
        assert window.tabs.count() == 3
        assert not window.start.isEnabled()
        for index in range(window.components.count()):
            item = window.components.item(index)
            if item.data(Qt.UserRole) == "navigation":
                item.setCheckState(Qt.Checked)
                break
        assert window.recipe["components"]["fast_lio"]["enabled"]
        assert window.recipe["components"]["livox"]["enabled"]
        assert window.recipe["components"]["mavlink"]["enabled"]
        window.review = ("mission.yaml", "a" * 64)
        window.permit.setChecked(True)
        window.mission_text.insertPlainText("# local edit\n")
        assert window.review is None and not window.permit.isChecked()
        assert window.mission_dirty
        app.processEvents()
    finally:
        window.dirty = False
        window.mission_dirty = False
        window.close()
        app.processEvents()


def test_real_reviewed_mission_facade_uses_exact_content_and_request(tmp_path):
    """Exercise the native ROS facade against a mock action, never a flight controller."""
    from ament_index_python.packages import get_package_prefix
    from easyfly_msgs.action import ExecuteMission
    from easyfly_msgs.srv import GetActionOwnership, GetCapabilities
    from rclpy.action import ActionServer, GoalResponse
    from rclpy.context import Context
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from easyfly_connection_manager.onboard_observer import PlatformObserver

    os.environ["ROS_DOMAIN_ID"] = "182"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    context = Context()
    context.init(domain_id=182)
    node = Node("onboard_mission_fixture", context=context)
    terminal, goals = set(), []

    def execute(handle):
        goals.append(handle.request.mission_file)
        terminal.add(bytes(handle.goal_id.uuid).hex())
        handle.succeed()
        return ExecuteMission.Result(success=True, message="mock action only")

    def ownership(request, response):
        response.goal_id = request.goal_id
        response.state = 2 if request.goal_id in terminal else 1
        return response

    backend = ActionServer(
        node,
        ExecuteMission,
        "/execute_mission",
        execute,
        goal_callback=lambda _: GoalResponse.ACCEPT,
    )
    node.create_service(GetActionOwnership, "/mission/ownership", ownership)
    executor = MultiThreadedExecutor(context=context, num_threads=2)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin)
    executor_thread.start()
    mission = tmp_path / "review.yaml"
    content = (
        "schema_version: 1\nframe_id: odom\n"
        "units: {position: m, angle: rad, time: s}\nfailure_policy: abort\nsteps:\n"
        "  - {type: land, timeout: 90}\n"
    )
    mission.write_text(content)
    observer = PlatformObserver(template())
    observer.mission_clients["capabilities"] = observer.node.create_client(
        GetCapabilities, "/platform/mission/capabilities"
    )
    import subprocess
    import signal
    import uuid

    executable = (
        Path(get_package_prefix("easyfly_mission_srvs"))
        / "lib/easyfly_mission_srvs/mission_services"
    )
    log = (tmp_path / "mission-service.log").open("w")
    process = subprocess.Popen(
        [
            str(executable),
            "--ros-args",
            "-p",
            "use_sim_time:=false",
            "-p",
            "mission_directory:=" + str(tmp_path),
        ],
        stdout=log,
        stderr=log,
        start_new_session=True,
    )
    try:
        deadline = time.monotonic() + 10
        while not all(client.service_is_ready() for client in observer.mission_clients.values()):
            assert process.poll() is None
            assert time.monotonic() < deadline
            time.sleep(0.05)
        while True:
            # Client discovery may precede the per-node graph used to reject duplicate providers.
            # Retry only this read; mutating reviewed requests below are still sent exactly once.
            try:
                ready = observer.call("capabilities", {})["ready"][1]
            except RuntimeError:
                ready = False
            if ready:
                break
            assert process.poll() is None
            assert time.monotonic() < deadline
            time.sleep(0.05)
        details = observer.mission_details("review.yaml")
        assert details["content_sha256"] == hashlib.sha256(content.encode()).hexdigest()
        session = observer.call("reviewed", dict(operation="session"))
        assert session["success"]
        identity = uuid.uuid4().hex
        values = dict(
            operation="start",
            server_instance_id=session["server_instance_id"],
            client_request_id=identity,
            mission_name="review.yaml",
            expected_content_sha256=details["content_sha256"],
        )
        accepted = observer.call("reviewed", values)
        assert accepted["success"], accepted
        assert observer.call("reviewed", values)["execution_id"] == accepted["execution_id"]
        deadline = time.monotonic() + 8
        while True:
            result = observer.call(
                "reviewed",
                dict(
                    operation="lookup",
                    server_instance_id=session["server_instance_id"],
                    client_request_id=identity,
                ),
            )
            if not result["execution_pending"]:
                break
            assert time.monotonic() < deadline
            time.sleep(0.05)
        assert len(goals) == 1
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=8)
        log.close()
        observer.close()
        executor.shutdown(timeout_sec=3)
        executor_thread.join(timeout=3)
        backend.destroy()
        node.destroy_node()
        context.shutdown()

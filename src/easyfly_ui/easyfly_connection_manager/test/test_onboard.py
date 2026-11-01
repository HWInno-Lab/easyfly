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
"""Test onboard configuration and real harmless owned processes, never hardware."""

import importlib.util
import json
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from easyfly_connection_manager import onboard_profiles as profiles  # noqa: E402
from easyfly_connection_manager.onboard_runtime import PlatformOwner  # noqa: E402
from easyfly_connection_manager.onboard_service import bootstrap_local  # noqa: E402
from easyfly_connection_manager.remote_profiles import private_json  # noqa: E402

MISSION = """schema_version: 1
frame_id: odom
units: {position: m, angle: rad, time: s}
failure_policy: abort
steps:
  - {type: takeoff, altitude: 1.5, timeout: 60}
  - {type: land, timeout: 90}
"""


class Witness:
    """Explicit observation/ROS facade fixture, not a sensor acceptance."""

    def __init__(self, recipe):
        self.recipe = recipe
        self.ever_connected = False
        self.calls = []
        self.permit = dict(
            grounded=False, isolated=True, observer_alive=True, startup_abort_safe=True
        )
        self.conflict = False
        self.ready_value = True
        self.closed = False

    def snapshot(self, name):
        """Return deliberately arranged observations."""
        return dict(self.permit)

    def source_conflict(self, name):
        """Inject an external publisher in the negative case only."""
        return self.conflict

    def ready(self, name, since):
        """Return arranged readiness without claiming physical output."""
        return self.ready_value

    def close(self):
        """Record observer replacement."""
        self.closed = True

    def mission_details(self, name):
        """Return the arranged exact review digest."""
        return dict(content_sha256=self.review_hash)

    def task_backend_ready(self, step_type):
        """Arrange discovery independently of process readiness in capability tests."""
        return step_type not in getattr(self, "missing_backends", set())

    def call(self, kind, values, timeout=1.5):
        """Provide reviewed protocol replies without a real flight action."""
        self.calls.append((kind, values))
        if values["operation"] == "session":
            return dict(success=True, server_instance_id="a" * 32)
        return dict(
            success=True,
            server_instance_id="a" * 32,
            client_request_id=values["client_request_id"],
            status="running",
            execution_id="b" * 32,
            execution_pending=True,
            admission_resolved=True,
            admission_success=True,
        )


def wait(predicate, timeout=5):
    """Wait on a fixture transition instead of a fixed startup duration."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("expected fixture transition timed out")


@pytest.fixture
def owner(tmp_path, monkeypatch):
    """Own real harmless process groups, never ros2 or a physical adapter."""
    server, _ = bootstrap_local(tmp_path / "platform", 187)
    config = private_json(server)
    config["allow_mission_execution"] = True
    prefix = tmp_path / "installed"
    executable = prefix / "lib/easyfly_color_line_detection/color_line_detection_node"
    executable.parent.mkdir(parents=True)
    executable.write_text("fixture identity\n")
    executable.chmod(0o700)
    monkeypatch.setattr(profiles, "get_package_prefix", lambda _: str(prefix))
    monkeypatch.setattr(profiles, "get_package_share_directory", lambda _: str(prefix))
    spawned = []

    def popen(argv, **kwargs):
        assert argv[:4] == [
            "ros2",
            "launch",
            "easyfly_connection_manager",
            "onboard_component.launch.py",
        ]
        child = subprocess.Popen(
            [
                sys.executable,
                "-u",
                "-c",
                "import signal,time,sys;"
                "signal.signal(signal.SIGINT,lambda *_:sys.exit(0));"
                "print('owned fixture ready',flush=True);time.sleep(60)",
            ],
            **kwargs
        )
        spawned.append(child)
        assert child.stdout.readline().strip() == b"owned fixture ready"
        return child

    value = PlatformOwner(config, observer_factory=Witness, popen=popen)
    value.spawned = spawned
    yield value
    for process in spawned:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=3)
    if value.worker_alive():
        value.abort.set()
        value.worker.join(timeout=3)
    value.state["tasks"].clear()
    value.state["phase"] = "stopping"
    value.refresh()
    value.close()


def request(owner, operation, data, identity=None, client="c" * 32):
    """Construct the same bound operation as the authenticated client."""
    return dict(
        v=2,
        server_id=owner.config["server_id"],
        boot_id=owner.boot,
        id=identity or uuid.uuid4().hex,
        client_id=client,
        op=operation,
        data=data,
    )


def configure(owner):
    """Select an image observation with its camera supplied externally."""
    recipe = profiles.template()
    recipe["components"]["color_line"]["enabled"] = True
    result = owner.mutate(
        request(
            owner, "configure", dict(recipe=recipe, expected_hash=profiles.digest(owner.recipe))
        )
    )
    assert result["accepted"]
    return recipe


@pytest.mark.parametrize('strategy', ['ego', 'combined'])
def test_navigation_strategy_requires_and_orders_its_managed_local_planner(strategy):
    """The UI preview and server validation agree about EGO's runtime dependency."""
    recipe = profiles.template()
    navigation = recipe['components']['navigation']
    navigation['enabled'] = True
    navigation['parameters']['strategy'] = strategy
    with pytest.raises(ValueError, match='local_planner'):
        profiles.validate(recipe)
    prepared = profiles.enable_dependencies(recipe, ['navigation'])
    assert not recipe['components']['local_planner']['enabled']
    assert prepared['recipe']['components']['local_planner']['enabled']
    assert len(prepared['added']) == len(set(prepared['added']))
    profiles.validate(prepared['recipe'])
    ordered = profiles.order(prepared['recipe'])
    assert ordered.index('local_planner') < ordered.index('navigation')
    assert profiles.CATALOG['navigation']['role'] != profiles.CATALOG['local_planner']['role']


def test_vision_chain_shares_its_camera_and_external_inputs_remain_allowed():
    """Classification does not make cooperating vision modules mutually exclusive."""
    prepared = profiles.enable_dependencies(
        profiles.template(), ['pixel_to_3d', 'color_line', 'ellipse'])
    recipe = prepared['recipe']
    profiles.validate(recipe)
    assert prepared['added'].count('realsense') == 1
    ordered = profiles.order(recipe)
    assert ordered.index('detection') < ordered.index('tracking') < ordered.index('pixel_to_3d')
    external = profiles.template()
    external['components']['color_line']['enabled'] = True
    external['components']['ellipse']['enabled'] = True
    profiles.validate(external)
    assert set(profiles.order(external)) == {'color_line', 'ellipse'}
    assert not external['components']['realsense']['enabled']


def test_strategy_dependency_cannot_hide_a_cycle():
    recipe = profiles.template()
    recipe['components']['navigation']['enabled'] = True
    recipe['components']['navigation']['parameters']['strategy'] = 'combined'
    recipe['components']['local_planner']['enabled'] = True
    recipe['components']['local_planner']['depends_on'] = ['navigation']
    with pytest.raises(ValueError, match='cyclic'):
        profiles.validate(recipe)
    with pytest.raises(ValueError, match='cyclic'):
        profiles.enable_dependencies(recipe, 'navigation')


def test_components_with_the_same_declared_responsibility_are_rejected(monkeypatch):
    """Catalog roles enforce substitution even outside the UI selection widgets."""
    recipe = profiles.template()
    recipe['components']['color_line']['enabled'] = True
    recipe['components']['ellipse']['enabled'] = True
    replacement = dict(profiles.CATALOG['ellipse'], role=profiles.CATALOG['color_line']['role'])
    monkeypatch.setitem(profiles.CATALOG, 'ellipse', replacement)
    with pytest.raises(ValueError, match='同一职责'):
        profiles.validate(recipe)


def test_lifecycle_real_owned_children_and_replay(owner):
    """Start one owned process group; replay and querying never duplicate it."""
    configure(owner)
    operation = request(owner, "start", dict(recipe_hash=profiles.digest(owner.recipe)))
    result = owner.mutate(operation)
    assert result["accepted"]
    wait(lambda: owner.status()["state"]["phase"] == "running")
    assert len(owner.spawned) == 1
    assert owner.mutate(operation) == result
    assert not owner.mutate(request(owner, "start", operation["data"]))["accepted"]
    assert not owner.mutate(
        request(owner, "stop", dict(session=result["session"]), client="d" * 32)
    )["accepted"]
    assert owner.spawned[0].poll() is None
    assert owner.mutate(request(owner, "stop", dict(session=result["session"])))["accepted"]
    wait(lambda: owner.status()["state"]["phase"] == "stopped")
    assert owner.spawned[0].returncode == 0
    assert not owner.children


def test_identity_content_and_conflict_before_spawn(owner):
    """Reject stale service, conflicting output, changed recipe and unknown state."""
    recipe = configure(owner)
    stale = request(owner, "start", dict(recipe_hash=profiles.digest(recipe)))
    stale["boot_id"] = "f" * 32
    with pytest.raises(ValueError):
        owner.mutate(stale)
    assert not owner.mutate(
        request(owner, "configure", dict(recipe=recipe, expected_hash="0" * 64))
    )["accepted"]
    owner.observer.conflict = True
    assert not owner.mutate(request(owner, "start", dict(recipe_hash=profiles.digest(recipe))))[
        "accepted"
    ]
    owner.observer.conflict = False
    owner.observer.permit.update(isolated=False, grounded=False)
    assert not owner.mutate(request(owner, "start", dict(recipe_hash=profiles.digest(recipe))))[
        "accepted"
    ]
    assert not owner.spawned


def test_unknown_flight_state_preserves_owned_stack(owner):
    """A stale FCU after connection cannot be interpreted as disarmed/on-ground."""
    configure(owner)
    result = owner.mutate(request(owner, "start", dict(recipe_hash=profiles.digest(owner.recipe))))
    wait(lambda: owner.status()["state"]["phase"] == "running")
    owner.observer.ever_connected = True
    owner.observer.permit.update(grounded=False, isolated=False, startup_abort_safe=False)
    assert not owner.mutate(request(owner, "stop", dict(session=result["session"])))["accepted"]
    assert owner.spawned[0].poll() is None
    owner.observer.permit["grounded"] = True
    owner.mutate(request(owner, "stop", dict(session=result["session"])))
    wait(lambda: owner.status()["state"]["phase"] == "stopped")


def test_native_task_save_review_and_owned_submission(owner, monkeypatch):
    """Configure tasks while stopped; execution needs opt-in and exact reviewed bytes."""
    result = owner.mutate(
        request(owner, "mission_save", dict(name="mission.yaml", content=MISSION))
    )
    assert result["accepted"] and not owner.observer.calls and not owner.spawned
    assert not owner.mutate(
        request(owner, "mission_save", dict(name="../escape.yaml", content=MISSION))
    )["accepted"]
    assert not owner.mutate(
        request(owner, "mission_save", dict(name="bad.yaml", content="bad: data"))
    )["accepted"]
    assert not owner.mutate(
        request(owner, "mission_save", dict(name="mission.yaml", content=MISSION + "# changed"))
    )["accepted"]
    assert (
        owner.mutate(request(owner, "mission_read", dict(name="mission.yaml")))["content"]
        == MISSION
    )
    start = dict(name="mission.yaml", sha256=result["sha256"], confirmed=True)
    assert not owner.mutate(request(owner, "mission_start", start))["accepted"]
    owner.state["phase"] = "running"
    owner.observer.permit["grounded"] = True
    owner.observer.review_hash = result["sha256"]
    assert not owner.mutate(request(owner, "mission_start", start))["accepted"]
    monkeypatch.setattr(owner, "task_stack_ready", lambda: True)
    monkeypatch.setattr(owner, "task_step_support", lambda: {
        kind: dict(state="supported", reason="arranged managed fixture")
        for kind in ("takeoff", "land")})
    assert not owner.mutate(request(owner, "mission_start", dict(start, sha256="0" * 64)))[
        "accepted"
    ]
    operation = request(owner, "mission_start", start)
    reply = owner.mutate(operation)
    assert reply["accepted"] and reply["task"]["execution_pending"]
    assert owner.mutate(operation) == reply
    assert [call[1]["operation"] for call in owner.observer.calls] == ["session", "start"]
    assert not owner.mutate(request(owner, "mission_start", start))["accepted"]
    assert not owner.mutate(
        request(owner, "mission_cancel", dict(request_id=operation["id"]), client="d" * 32)
    )["accepted"]
    owner.state["tasks"].clear()


def test_native_step_types_and_review_keep_original_bytes(owner):
    """The shared native parser extracts all types, including the legacy grammar."""
    steps = [
        "{type: takeoff, altitude: 1.5}", "{type: hover, duration: 1}",
        "{type: waypoint, x: 1, y: 0, z: 1.5}",
        "{type: navigate, x: 0, y: 0, z: 1.5}",
        "{type: track, target_id: 1, duration: 1}",
        "{type: search, center_x: 0, center_y: 0, area_width: 4, area_height: 4, "
        "altitude: 1.5, pattern: boustrophedon}", "{type: land}",
        "{type: precision_land, tag_id: 1}",
        "{type: explore, min_bound: [-2,-2,0.5], max_bound: [2,2,2.5], return_home: true}",
    ]
    content = MISSION[:MISSION.index("steps:")] + "steps:\n"
    content += "".join("  - " + step + "\n" for step in steps)
    content = content.replace("\n", "\r\n")
    saved = owner.save_mission(dict(name="all.yaml", content=content))
    read = owner.read_mission(dict(name="all.yaml"))
    assert read["content"] == content and read["sha256"] == saved["sha256"]
    assert read["step_types"] == [
        "takeoff", "hover", "waypoint", "navigate", "track", "search", "land",
        "precision_land", "explore"]
    assert owner.mission_step_types(b"type = hover\nwait = 1\n") == ["hover"]
    with pytest.raises(ValueError, match="任务步骤校验失败"):
        owner.mission_step_types(b"steps: [invalid]")
    assert not owner.observer.calls and not owner.state["tasks"]


def test_task_step_support_uses_configured_and_actual_managed_readiness(owner, monkeypatch):
    """Adaptation and discovery are separate from any permission to submit flight."""
    support = owner.status()["task_step_support"]
    assert support["explore"]["state"] == "not_ready"
    assert support["track"]["state"] == "not_ready"
    assert support["precision_land"]["state"] == "not_ready"
    assert support["search"]["state"] == "not_ready"
    recipe = profiles.template()
    for item in recipe["components"].values():
        item["enabled"] = True
    recipe["components"]["navigation"]["parameters"]["strategy"] = "gcopter"
    with monkeypatch.context() as arranged:
        arranged.setattr(owner, "recipe", recipe)
        arranged.setattr(owner, "children", {key: object() for key in profiles.CATALOG})
        arranged.setattr(owner, "state", dict(owner.state, components={
            key: dict(phase="ready") for key in profiles.CATALOG}))
        assert all(value["state"] == "supported" for value in owner.task_step_support().values())
        for adapter, step in (
                ("target_tracking", "track"), ("precision_landing", "precision_land"),
                ("lidar_rays", "explore"), ("gcopter", "explore"), ("fuel", "explore")):
            owner.state["components"][adapter]["phase"] = "waiting_for_data"
            assert owner.task_step_support()[step]["state"] == "not_ready"
            owner.state["components"][adapter]["phase"] = "ready"
            owner.observer.missing_backends = {step}
            assert owner.task_step_support()[step]["state"] == "not_ready"
        owner.observer.missing_backends.clear()
        assert owner.task_step_support()["search"]["state"] == "supported"
        owner.observer.missing_backends = {"search"}
        assert owner.task_step_support()["search"]["state"] == "not_ready"
        owner.observer.missing_backends.clear()
        owner.state["components"]["flight_control"]["phase"] = "waiting_for_data"
        assert owner.task_step_support()["takeoff"]["state"] == "not_ready"


@pytest.mark.parametrize("step", [
    "{type: track, target_id: 1, duration: 1}",
    "{type: precision_land, tag_id: 1}",
    "{type: explore, min_bound: [-2,-2,0.5], max_bound: [2,2,2.5], return_home: true}",
])
def test_unconfigured_adapters_save_but_reject_before_reviewed_admission(owner, monkeypatch, step):
    """Unconfigured adapters fail before creating an execution intent or RPC."""
    content = MISSION[:MISSION.index("steps:")] + "steps:\n  - " + step + "\n"
    saved = owner.mutate(request(
        owner, "mission_save", dict(name="unsupported.yaml", content=content)))
    assert saved["accepted"]
    owner.state["phase"] = "running"
    owner.observer.permit["grounded"] = True
    owner.observer.review_hash = saved["sha256"]
    monkeypatch.setattr(owner, "task_stack_ready", lambda: True)
    reply = owner.mutate(request(owner, "mission_start", dict(
        name="unsupported.yaml", sha256=saved["sha256"], confirmed=True)))
    assert not reply["accepted"] and "任务步骤尚不能执行" in reply["reason"]
    assert not owner.observer.calls and not owner.state["tasks"]


def test_physical_link_validation_and_maintenance(owner):
    """Reject malformed links and suppress managed execution during QGC forwarding."""
    recipe = profiles.template()
    link = recipe["components"]["mavlink"]
    link["enabled"] = True
    for value in ("/dev/ttyACM0:921600", "udp://:14540@127.0.0.1:14557"):
        link["parameters"]["fcu_url"] = value
        profiles.validate(recipe)
    for value in ("/dev/ttyACM0:0", "sh -c touch /tmp/file", "udp://broken"):
        link["parameters"]["fcu_url"] = value
        with pytest.raises(ValueError):
            profiles.validate(recipe)
    link["parameters"]["fcu_url"] = "/dev/ttyACM0:921600"
    link["parameters"]["gcs_url"] = "udp://@127.0.0.1:14550"
    owner.recipe = profiles.validate(recipe)
    assert owner.maintenance()
    assert not owner.status()["mission_execution_enabled"]


@pytest.mark.parametrize(
    "failure", ["shell", "clock", "cycle", "missing_calibration", "unknown_kind"]
)
def test_recipe_guards(failure):
    """Refuse arbitrary commands, simulated time and invalid native dependencies."""
    value = profiles.template()
    if failure == "shell":
        value["components"]["color_line"]["command"] = "/bin/sh"
    elif failure == "clock":
        value["components"]["color_line"]["parameters"]["use_sim_time"] = True
    elif failure == "cycle":
        value["components"]["coord"]["enabled"] = True
        value["components"]["mavlink"]["enabled"] = True
        value["components"]["mavlink"]["depends_on"] = ["coord"]
    elif failure == "missing_calibration":
        value["components"]["fast_lio"]["enabled"] = True
    else:
        value["components"]["arbitrary_binary"] = {}
    with pytest.raises(ValueError):
        profiles.validate(value, require_files=True)


@pytest.mark.parametrize(
    "content",
    [
        "/**:\n  ros__parameters:\n    use_sim_time: true\n",
        "mapping: {extrinsic: .nan}\n",
        "mapping: &loop {child: *loop}\n",
        "invalid: [",
    ],
)
def test_bad_physical_configuration(tmp_path, content):
    """A parameter file cannot undo the physical-clock contract."""
    path = tmp_path / "hardware.yaml"
    path.write_text(content)
    with pytest.raises(ValueError):
        profiles.read_configuration(str(path), [str(tmp_path)])


def test_physical_configuration_frozen_and_contained(tmp_path, monkeypatch):
    """Editing original configuration does not alter active input bytes."""
    monkeypatch.setattr(profiles, "get_package_share_directory", lambda _: str(tmp_path))
    monkeypatch.setattr(profiles, "get_package_prefix", lambda _: str(tmp_path))
    binary = tmp_path / "lib/fast_lio/fastlio_mapping"
    binary.parent.mkdir(parents=True)
    binary.write_text("identity")
    binary.chmod(0o700)
    path = tmp_path / "physical.yaml"
    path.write_text("/**:\n  ros__parameters:\n    mapping.extrinsic_T: [0.0, 0.0, 0.0]\n")
    path.chmod(0o600)
    value = profiles.template()
    value["components"]["fast_lio"].update(enabled=True, parameter_file=str(path))
    frozen_dir = tmp_path / "run"
    frozen_dir.mkdir()
    frozen, hashes = profiles.preflight(value, [str(tmp_path)], frozen_dir)
    target = Path(frozen["components"]["fast_lio"]["parameter_file"])
    original = target.read_bytes()
    path.write_text("modified after preparation")
    assert target.read_bytes() == original and hashes["fast_lio/parameter_file"]["sha256"]
    link = tmp_path / "link.yaml"
    link.symlink_to(path)
    with pytest.raises(ValueError):
        profiles.read_configuration(str(link), [str(tmp_path)])


def test_native_launch_has_physical_time(tmp_path):
    """Expand a native component without executing launch actions or a simulator."""
    from launch import LaunchContext
    from launch_ros.utilities import evaluate_parameters

    source = PACKAGE / "launch/onboard_component.launch.py"
    spec = importlib.util.spec_from_file_location("onboard_component_launch", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    entry = profiles.template()["components"]["coord"]
    entry["enabled"] = True
    path = tmp_path / "component.json"
    path.write_text(json.dumps(dict(kind="coord", component=entry)))
    context = LaunchContext()
    context.launch_configurations["component_file"] = str(path)
    container = module.create_component(context)[0]
    descriptions = container.descriptions
    assert len(descriptions) == 1
    values = evaluate_parameters(context, descriptions[0].parameters)
    assert values[0]["use_sim_time"] is False


@pytest.mark.parametrize("physical", [False, True])
@pytest.mark.parametrize(
    "package, filename",
    [
        ("easyfly_coord_transform", "coord_tf.launch.py"),
        ("easyfly_multi_object_tracking", "tracking.launch.py"),
        ("easyfly_pixel_to_3d", "pixel_to_3d.launch.py"),
    ],
)
def test_public_launch_clock_override(package, filename, physical):
    """Keep simulation defaults while honoring a physical-clock override in public launches."""
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch_ros.utilities import evaluate_parameters

    source = Path(get_package_share_directory(package)) / "launch" / filename
    spec = importlib.util.spec_from_file_location("public_clock_launch", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    if physical:
        context.launch_configurations["use_sim_time"] = "false"
    containers = []
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
        elif hasattr(entity, "descriptions"):
            containers.append(entity)
    assert len(containers) == 1
    values = evaluate_parameters(context, containers[0].descriptions[0].parameters)
    assert values[0]["use_sim_time"] is (not physical)


@pytest.mark.parametrize("component", ["navigation", "mission_fsm", "local_planner"])
def test_multi_node_component_requires_every_unique_node(component):
    """A surviving coordinator cannot hide a missing planner, supervisor or trajectory server."""
    from types import SimpleNamespace
    from easyfly_connection_manager.onboard_observer import PlatformObserver, REQUIRED_NODES

    observer = object.__new__(PlatformObserver)
    nodes = list(REQUIRED_NODES[component])
    observer.node = SimpleNamespace(get_node_names=lambda: nodes)
    observer.profiles = {}
    assert observer.ready(component, 0.0)
    nodes.pop()
    assert not observer.ready(component, 0.0)
    assert observer.source_conflict(component)
    nodes[:] = REQUIRED_NODES[component]
    nodes.append(nodes[0])
    assert not observer.ready(component, 0.0)
    nodes.clear()
    assert not observer.source_conflict(component)


@pytest.mark.parametrize("step", ["takeoff", "search", "track", "precision_land", "explore"])
def test_task_backend_discovery_requires_unique_fresh_typed_provider(step, monkeypatch):
    """Discovery is scoped, typed and unique; it remains separate from flight admission."""
    from types import SimpleNamespace
    from easyfly_connection_manager import onboard_observer as module

    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: 100.0))
    observer = object.__new__(module.PlatformObserver)
    action, expected = module.TASK_ACTIONS[step]
    action_type = module.TASK_ACTION_TYPES[step]
    rows = [(expected, "/")]
    endpoints = [(action, [action_type])]
    observer.last_discovery = 99.9
    observer.node = SimpleNamespace(
        get_node_names_and_namespaces=lambda: rows,
        get_action_server_names_and_types_by_node=lambda *_: endpoints,
    )
    assert observer.task_backend_ready(step)
    for providers in ([], [("foreign", "/")], [(expected, "/other")],
                      [(expected, "/"), ("foreign", "/")],
                      [(expected, "/"), (expected, "/other")]):
        rows[:] = providers
        assert not observer.task_backend_ready(step)
    rows[:] = [(expected, "/")]
    endpoints[:] = [(action, ["easyfly_msgs/action/WrongType"])]
    assert not observer.task_backend_ready(step)
    endpoints[:] = [("/wrong_action", [action_type])]
    assert not observer.task_backend_ready(step)
    endpoints[:] = [(action, [action_type])]
    observer.last_discovery = 99.5
    assert not observer.task_backend_ready(step)


def test_hover_backend_discovery_stays_separate_from_flight_actions(monkeypatch):
    from types import SimpleNamespace
    from easyfly_connection_manager import onboard_observer as module

    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: 100.0))
    observer = object.__new__(module.PlatformObserver)
    observer.last_discovery = 99.9
    calls = []
    observer.ready = lambda *args: calls.append(args) or True
    assert observer.task_backend_ready("hover")
    assert calls == [("mission_fsm", 0.0)]
    observer.ready = lambda *_: False
    assert not observer.task_backend_ready("hover")
    observer.last_discovery = 99.49
    observer.ready = lambda *_: pytest.fail("stale discovery must not query readiness")
    assert not observer.task_backend_ready("hover")


@pytest.mark.parametrize('step', [
    '{type: track, target_id: 1, duration: 1}',
    '{type: precision_land, tag_id: 1}',
    '{type: explore, min_bound: [-2,-2,0.5], max_bound: [2,2,2.5], return_home: true}',
])
def test_adapted_step_uses_original_reviewed_admission(owner, monkeypatch, step):
    """Native task parsing plus arranged readiness reaches only the owned reviewed facade."""
    content = MISSION[:MISSION.index('steps:')] + 'steps:\n  - ' + step + '\n'
    saved = owner.save_mission(dict(name='adapted.yaml', content=content))
    recipe = profiles.template()
    for entry in recipe['components'].values():
        entry['enabled'] = True
    recipe['components']['navigation']['parameters']['strategy'] = 'gcopter'
    with monkeypatch.context() as arranged:
        arranged.setattr(owner, 'recipe', recipe)
        arranged.setattr(owner, 'children', {key: object() for key in profiles.CATALOG})
        arranged.setattr(owner, 'state', dict(owner.state, phase='running', components={
            key: dict(phase='ready') for key in profiles.CATALOG}))
        arranged.setattr(owner, 'refresh', lambda: dict(grounded=True))
        owner.observer.review_hash = saved['sha256']
        action = request(owner, 'mission_start', dict(
            name='adapted.yaml', sha256=saved['sha256'], confirmed=True))
        reply = owner.mutate(action)
        assert reply['accepted']
        assert [values['operation'] for _, values in owner.observer.calls] == ['session', 'start']
        submitted = owner.observer.calls[-1][1]
        assert submitted['expected_content_sha256'] == saved['sha256']
        assert submitted['client_request_id'] == action['id']
        assert owner.state['tasks'][action['id']]['client_id'] == action['client_id']
        owner.mutate(action)  # Replay resolves the original receipt without a second admission.
        assert len(owner.observer.calls) == 2


@pytest.mark.parametrize('component,publisher', [
    ('precision_landing', 'apriltag_detection'), ('gcopter', 'gcopter_voxel_map'),
    ('lidar_rays', 'lidar_rays'),
])
def test_adapter_output_requires_owned_publisher_and_valid_source_progress(
        component, publisher, monkeypatch):
    """Invalid frames, source replacements, stale scans and foreign publishers fail closed."""
    from types import SimpleNamespace as NS
    import threading
    from easyfly_connection_manager import onboard_observer as module

    monkeypatch.setattr(module, 'time', NS(monotonic=lambda: 100.0))
    observer = object.__new__(module.PlatformObserver)
    observer.recipe = profiles.template()
    observer.lock = threading.Lock()
    observer.output_stamps, observer.output_ids, observer.outputs = {}, {}, {}
    providers = [NS(node_name=publisher, node_namespace='/', endpoint_gid=b'first')]
    nodes = list(observer.required_nodes(component))
    observer.node = NS(get_node_names=lambda: nodes,
                       get_publishers_info_by_topic=lambda _: providers,
                       get_clock=lambda: NS(now=lambda: NS(nanoseconds=100_000_000_000)))
    observer.snapshot = lambda _: dict(observer_alive=True, output_publishers=len(providers))
    assert not observer.ready(component, 90.0)  # Establish a new output identity.

    def output(nanoseconds, valid=True):
        observer.output(component, NS(
            header=NS(stamp=NS(sec=nanoseconds // 10**9, nanosec=nanoseconds % 10**9)),
            valid=valid))

    output(99_000_000_000)
    output(99_100_000_000)
    assert observer.ready(component, 90.0)
    output(99_200_000_000, False)
    assert not observer.ready(component, 90.0)
    output(99_300_000_000)
    output(99_400_000_000)
    assert observer.ready(component, 90.0)
    providers[0].node_name = 'foreign'
    assert not observer.ready(component, 90.0)
    providers[0].node_name = publisher
    providers[0].node_namespace = '/foreign'
    assert not observer.ready(component, 90.0)
    providers[0].node_namespace = '/'
    providers[0].endpoint_gid = b'restarted'
    assert not observer.ready(component, 90.0)
    output(96_000_000_000)
    output(96_100_000_000)
    assert not observer.ready(component, 90.0)
    output(99_500_000_000)
    output(99_600_000_000)
    assert observer.ready(component, 90.0)
    nodes.append(nodes[0])
    assert not observer.ready(component, 90.0)


def test_idle_fuel_readiness_uses_input_and_management_not_task_only_output(monkeypatch):
    """An idle FUEL server can become ready before an exploration goal exists."""
    from types import SimpleNamespace as NS
    import threading
    from easyfly_connection_manager import onboard_observer as module

    monkeypatch.setattr(module, 'time', NS(monotonic=lambda: 100.0))
    observer = object.__new__(module.PlatformObserver)
    observer.lock = threading.Lock()
    observer.fuel_provider_ok = True
    observer.fuel_idle = (99.9, True)
    conditions = dict(map=True, action=True)
    observer.task_backend_ready = lambda step: step == 'explore' and conditions['action']
    native_ready = observer.ready
    observer.ready = lambda name, since: (conditions['map'] if name == 'gcopter'
                                          else native_ready(name, since))
    assert observer.ready('fuel', 90.0)
    observer.fuel_idle = (99.9, False)  # An owned active task remains a healthy component.
    assert observer.ready('fuel', 90.0)
    observer.fuel_idle = (99.4, True)
    assert not observer.ready('fuel', 90.0)
    observer.fuel_idle = (99.9, True)
    observer.fuel_provider_ok = False
    assert not observer.ready('fuel', 90.0)
    observer.fuel_provider_ok = True
    conditions['map'] = False
    assert not observer.ready('fuel', 90.0)
    conditions['map'], conditions['action'] = True, False
    assert not observer.ready('fuel', 90.0)


def test_fuel_file_speed_cannot_be_silently_overridden_by_recipe(tmp_path, monkeypatch):
    """Reviewed limits remain effective even when a custom parameter file is uploaded."""
    monkeypatch.setattr(profiles, 'get_package_share_directory', lambda _: str(tmp_path))
    path = tmp_path / 'fuel.yaml'
    path.write_text('fuel_exploration:\n  ros__parameters:\n'
                    '    native_max_speed: 0.2\n    extended_profile: false\n')
    path.chmod(0o600)
    recipe = profiles.template()
    recipe['components']['fuel'].update(
        enabled=True, parameter_file=str(path), parameters=dict(native_max_speed=1.2))
    with pytest.raises(ValueError, match='native_max_speed differs'):
        profiles.preflight(recipe, [str(tmp_path)])
    recipe['components']['fuel']['parameters']['native_max_speed'] = 0.2
    profiles.preflight(recipe, [str(tmp_path)])


def test_existing_recipe_exposes_new_disabled_adapters_without_resetting_choices(tmp_path):
    """An upgraded platform retains existing device settings and discovers new UI choices."""
    server, _ = bootstrap_local(tmp_path / 'upgrade-platform', 186)
    config = private_json(server)
    root = Path(config['state_directory'])
    root.mkdir(mode=0o700, exist_ok=True)
    recipe = profiles.template()
    added = {'down_camera', 'target_tracking', 'precision_landing',
             'lidar_rays', 'gcopter', 'fuel'}
    for name in added:
        recipe['components'].pop(name)
    recipe['components']['mavlink']['parameters']['fcu_url'] = '/dev/ttyUSB0:57600'
    recipe['components']['color_line']['enabled'] = True
    path = root / 'recipe.json'
    path.write_text(json.dumps(recipe))
    path.chmod(0o600)
    platform = PlatformOwner(config, observer_factory=Witness)
    try:
        assert all(not platform.recipe['components'][name]['enabled'] for name in added)
        for name, entry in recipe['components'].items():
            assert platform.recipe['components'][name] == entry
        assert private_json(path) == platform.recipe
        assert not platform.children
    finally:
        platform.close()

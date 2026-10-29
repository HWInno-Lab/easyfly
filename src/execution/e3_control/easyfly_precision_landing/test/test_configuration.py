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

"""Read-only launch inspection and isolated non-overwriting SDF regression."""

import importlib.util
import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from easyfly_precision_landing.geometry import attitude, rotation
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.utilities import (
    normalize_to_list_of_substitutions,
    perform_substitutions,
)
from launch_ros.actions import Node
import pytest
import yaml
from easyfly_msgs.msg import MissionStatus


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / "config/d5_simulation.yaml"


def test_unified_entry_defaults_to_no_flight_and_requires_explicit_dds():
    """Argument inspection alone performs no preparation, processes or goals."""
    module = load_module("d5_entry_cli", PACKAGE / "scripts/sitl_smoke.py")
    arguments = ["--directory", "/not-created", "--px4-dir", "/not-read"]
    with pytest.raises(SystemExit):
        module.parse_arguments(arguments)
    args = module.parse_arguments(arguments + ["--use-bundled-dds-profile"])
    assert not args.mission and not args.direct_action and not args.prepare_only
    assert args.startup_duration is None


@pytest.mark.parametrize("extra", [
    ["--mission", "--direct-action"],
    ["--prepare-only", "--mission"],
    ["--startup-duration", "nan"],
    ["--startup-duration", "-1"],
    ["--startup-duration", "3601"],
    ["--startup-duration", "2", "--mission"],
])
def test_unified_entry_rejects_conflicting_or_unbounded_modes(extra):
    """Ambiguous motion intent and invalid durations fail before preparation."""
    module = load_module("d5_entry_invalid", PACKAGE / "scripts/sitl_smoke.py")
    with pytest.raises(SystemExit):
        module.parse_arguments([
            "--directory", "/not-created", "--px4-dir", "/not-read",
            "--use-bundled-dds-profile", *extra,
        ])


def test_formal_mission_fixture_has_only_owned_ordered_steps():
    """Require explicit settling before precision, never append ordinary Land."""
    module = load_module("d5_mission_smoke", PACKAGE / "scripts/sitl_smoke.py")
    source = (
        PACKAGE.parents[1]
        / "e1_state_machine/easyfly_mission_fsm/missions/precision_landing_demo.yaml"
    )
    mission = module.mission_fixture(source)
    assert mission["frame_id"] == "odom"
    assert mission["failure_policy"] == "abort"
    assert [step["type"] for step in mission["steps"]] == [
        "takeoff",
        "hover",
        "precision_land",
    ]
    assert mission["steps"][1]["duration"] == 15
    assert mission["steps"][1]["timeout"] == 20
    assert mission["steps"][2]["tag_id"] == 7


def test_diagnostic_selected_tag_validity_is_not_array_validity():
    """Keep detailed pose rejection and exact calibration without authorizing flight."""
    from easyfly_precision_landing.simulation import geometry_details
    from easyfly_msgs.msg import TagObservation, TagObservationArray
    from sensor_msgs.msg import CameraInfo

    array = TagObservationArray(valid=True, reason="detected")
    array.observations = [TagObservation(tag_id=7, valid=False, reason="pose_ambiguous")]
    details = geometry_details(array, 7)
    assert details["selected_tag_count"] == 1
    assert details["selected_tags"][0]["valid"] is False
    assert details["selected_tags"][0]["reason"] == "pose_ambiguous"
    assert array.valid and not array.observations[0].valid
    camera = CameraInfo(width=640, height=480, distortion_model="plumb_bob")
    camera.k = [337.0, 0.0, 320.0, 0.0, 337.0, 240.0, 0.0, 0.0, 1.0]
    assert geometry_details(camera, 7)["camera_info"]["k"] == list(camera.k)


def test_observer_geometry_snapshot_is_bounded_and_detached():
    """Retain rejection evidence without altering input or accepting duplicate tags."""
    from easyfly_precision_landing.simulation import geometry_details
    from easyfly_msgs.msg import LandingTarget, TagObservation, TagObservationArray
    from sensor_msgs.msg import Image

    array = TagObservationArray(valid=True, reason="detected")
    array.observations = [
        TagObservation(tag_id=7, valid=False, reason="pose_ambiguous")
        for _ in range(3)
    ] + [TagObservation(tag_id=8, valid=True, reason="pose_valid")]
    details = geometry_details(array, 7)
    assert details["selected_tag_count"] == 3
    assert len(details["selected_tags"]) == 2
    assert geometry_details(array, 9) == {"selected_tag_count": 0, "selected_tags": []}
    array.observations[0].reason = "changed"
    array.observations[0].pose.position.z = 3.0
    assert details["selected_tags"][0]["reason"] == "pose_ambiguous"
    assert details["selected_tags"][0]["position"][2] == 0.0
    target = LandingTarget(valid=False, reason="tag_pose_invalid", tag_id=7)
    assert geometry_details(target, 7)["landing_pose"]["tag_id"] == 7
    assert not target.valid
    assert geometry_details(Image(), 7) == {}


@pytest.mark.parametrize('fault', [
    'none', 'not_ready', 'wrong_name', 'unimplemented', 'old_response', 'late_query',
    'zero_clock', 'wrong_frame', 'bad_stamp', 'bad_json', 'missing_state', 'zero_source',
    'clock_mismatch', 'negative_query', 'old_state_source', 'old_internal_receipt',
])
def test_takeoff_requires_current_internal_response(fault):
    """Discovery, delayed replies and malformed readiness cannot authorize a goal."""
    import json
    from easyfly_msgs.srv import GetCapabilities
    from easyfly_precision_landing.simulation import takeoff_readiness_current

    response = GetCapabilities.Response(names=['takeoff'], implemented=[True], ready=[True])
    response.header.frame_id = 'odom'
    response.header.stamp.sec = 10
    detail = {'reason': 'ready', 'ready': True, 'flight_ready': True, 'state_received': True,
              'clock_ns': 10000000000, 'state_sequence': 2, 'state_source_ns': 9900000000,
              'receipt_age_s': .01}
    ros_now, elapsed = 10000000000, .01
    if fault == 'not_ready':
        response.ready = [False]
    elif fault == 'wrong_name':
        response.names = ['precision_landing']
    elif fault == 'unimplemented':
        response.implemented = [False]
    elif fault == 'old_response':
        ros_now += 1100000000
    elif fault == 'late_query':
        elapsed = .31
    elif fault == 'zero_clock':
        ros_now = 0
    elif fault == 'wrong_frame':
        response.header.frame_id = 'world'
    elif fault == 'bad_stamp':
        response.header.stamp.nanosec = 1000000000
    elif fault == 'missing_state':
        detail['state_received'] = False
    elif fault == 'zero_source':
        detail['state_source_ns'] = 0
    elif fault == 'clock_mismatch':
        detail['clock_ns'] += 1
    elif fault == 'negative_query':
        elapsed = -.01
    elif fault == 'old_state_source':
        detail['state_source_ns'] = 8000000000
    elif fault == 'old_internal_receipt':
        detail['receipt_age_s'] = 1.
    response.details = ['bad' if fault == 'bad_json' else json.dumps(detail)]
    assert takeoff_readiness_current(response, ros_now, elapsed) is (fault == 'none')
    assert not takeoff_readiness_current(None, ros_now, elapsed)


@pytest.mark.parametrize(
    "field,value",
    [
        ("phase_name", "fault_owned_child"),
        ("mission_success", False),
        ("current_waypoint", 2),
        ("total_waypoints", 2),
        ("progress", 0.9),
    ],
)
def test_mission_terminal_requires_all_steps(field, value):
    """Public result alone is not completed-steps evidence."""
    module = load_module(
        "d5_mission_result", PACKAGE / "scripts/sitl_smoke.py"
    )
    good = MissionStatus(
        phase_name="complete",
        mission_success=True,
        current_waypoint=3,
        total_waypoints=3,
        progress=1.0,
    )
    assert module.completed_mission(good, 3)
    assert not module.completed_mission(None, 3)
    bad = copy.deepcopy(good)
    setattr(bad, field, value)
    assert not module.completed_mission(bad, 3)


def load_module(name, path):
    """Import definitions only; never execute a launch or helper CLI."""
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_explicit_dds_profile_preserves_udp_and_budgets_large_image():
    """Keep the transport opt-in and memory bounded above the raw image size."""
    namespace = {"f": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
    tree = ET.parse(PACKAGE / "config/d5_fastdds.xml")
    descriptors = tree.findall(
        "f:transport_descriptors/f:transport_descriptor", namespace
    )
    by_type = {
        item.findtext("f:type", namespaces=namespace): item
        for item in descriptors
    }
    assert set(by_type) == {"UDPv4", "SHM"}
    shm = by_type["SHM"]
    segment = int(shm.findtext("f:segment_size", namespaces=namespace))
    maximum = int(shm.findtext("f:maxMessageSize", namespaces=namespace))
    assert 640 * 480 * 3 < maximum < segment == 64 * 1024 * 1024
    assert shm.findtext("f:port_queue_capacity", namespaces=namespace) == "512"
    participant = tree.find("f:participant", namespace)
    assert participant.get("is_default_profile") == "true"
    assert (
        participant.findtext(
            "f:rtps/f:useBuiltinTransports", namespaces=namespace
        )
        == "false"
    )
    assert set(
        participant.findall(
            "f:rtps/f:userTransports/f:transport_id", namespace
        )[i].text
        for i in range(2)
    ) == {"d5_udp", "d5_shm"}


def test_smoke_release_requires_exact_uuid():
    """Reject a released response for a different goal or unknown ownership."""
    from easyfly_msgs.srv import GetActionOwnership

    module = load_module(
        "d5_ownership_smoke", PACKAGE / "scripts/sitl_smoke.py"
    )
    identity = "ab" * 16
    response = GetActionOwnership.Response(goal_id=identity, state=2)
    assert module.exact_released(response, identity)
    assert not module.exact_released(response, "cd" * 16)
    assert not module.exact_released(None, identity)
    response.state = 1
    assert not module.exact_released(response, identity)


def test_smoke_descent_and_handoff_require_one_exact_goal():
    """Mixed UUID, blank UUID and incomplete phases cannot certify a chain."""
    module = load_module("d5_chain_smoke", PACKAGE / "scripts/sitl_smoke.py")
    identity = "ab" * 16
    feedback = [
        {"goal_uuid": identity, "phase": "descend"},
        {"goal_uuid": identity, "phase": "land_handoff"},
    ]
    assert module.complete_precision_chain(feedback)
    assert not module.complete_precision_chain([])
    assert not module.complete_precision_chain(feedback[:1])
    for wrong in ("", "cd" * 16, "AB" * 16, "not_a_uuid"):
        assert not module.complete_precision_chain(
            [feedback[0], {"goal_uuid": wrong, "phase": "land_handoff"}]
        )


@pytest.fixture
def launch_module():
    """Read the source launch entry rather than possibly stale installation."""
    return load_module(
        "d5_configuration_launch",
        PACKAGE / "launch/precision_landing.launch.py",
    )


def launch_context(module, **overrides):
    """Apply declarations without executing opaque functions or nodes."""
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    return context


@pytest.mark.parametrize("rate", [1, 30])
def test_unused_camera_rate_changes_only_fixture_scheduling(tmp_path, rate):
    """Preserve physics, extrinsics and production bytes when reducing load."""
    module = load_module(
        "d5_camera_rates", PACKAGE / "scripts/prepare_down_camera.py"
    )
    source = (
        Path(get_package_share_directory("easyfly_px4_sitl"))
        / "models/easyfly_iris/easyfly_iris.sdf"
    )
    original = source.read_bytes()
    normal = tmp_path / "normal.sdf"
    modified = tmp_path / "modified.sdf"
    module.prepare(normal)
    module.prepare(modified, rate)
    expected = ET.parse(normal)
    for name in ("color", "depth"):
        expected.find(
            f"./model/link[@name='base_link']/sensor[@name='{name}']/update_rate"
        ).text = str(rate)
    assert ET.tostring(expected.getroot()) == ET.tostring(
        ET.parse(modified).getroot()
    )
    assert source.read_bytes() == original
    with pytest.raises(ValueError):
        module.prepare(tmp_path / "bad.sdf", 0)
    assert not (tmp_path / "bad.sdf").exists()


def test_consistent_configuration(launch_module):
    """Explicit bundled simulation configuration must remain consistent."""
    assert launch_module.validate_configuration(CONFIG) == yaml.safe_load(
        CONFIG.read_text(encoding="utf-8")
    )


def test_smoke_final_disarmed_requires_fresh_evidence():
    """Never sign off a cached, disconnected or armed final vehicle state."""
    from easyfly_msgs.msg import UAVState

    module = load_module(
        "d5_smoke_freshness", PACKAGE / "scripts/sitl_smoke.py"
    )
    state = UAVState(connected=True, armed=False)
    state.header.stamp.sec = 10
    check = module.fresh_disarmed_state
    assert check(state, 5.0, 10000000000, 5.1)
    assert not check(None, 5.0, 10000000000, 5.1)
    assert not check(state, 5.0, 0, 5.1)
    assert not check(state, 5.0, 11000000001, 5.1)
    assert not check(state, 5.0, 9799999999, 5.1)
    assert not check(state, 5.0, 10000000000, 5.31)
    assert not check(state, 5.2, 10000000000, 5.1)
    state.connected = False
    assert not check(state, 5.0, 10000000000, 5.1)
    state.connected = True
    state.armed = True
    assert not check(state, 5.0, 10000000000, 5.1)
    state.armed = False
    state.header.stamp.sec = 0
    assert not check(state, 5.0, 10000000000, 5.1)


@pytest.mark.parametrize(
    "section, key, value",
    [
        ("landing_detection", "selected_tag_id", 8),
        ("landing_gateway", "selected_tag_id", 8),
        ("flight_controller", "precision_tag_id", 8),
        ("apriltag_detection", "tag_size_m", 0.3),
        ("flight_controller", "precision_tag_size_m", 0.3),
        ("apriltag_detection", "optical_frame", "other_optical"),
        ("landing_detection", "optical_frame", "other_optical"),
        ("flight_controller", "precision_optical_frame", "other_optical"),
        ("flight_controller", "precision_plane_z", 0.1),
        ("landing_transform", "configured", False),
        ("flight_controller", "precision_enabled", False),
        ("flight_controller", "precision_descent_floor_height", None),
        ("flight_controller", "precision_descent_floor_height", float("nan")),
        ("flight_controller", "precision_descent_floor_height", float("inf")),
        ("flight_controller", "precision_descent_floor_height", False),
        ("flight_controller", "precision_descent_floor_height", 0.1),
        ("flight_controller", "precision_descent_floor_height", 0.6),
        ("flight_controller", "precision_descent_floor_height", 0.7),
        ("flight_controller", "precision_handoff_height", None),
    ],
)
def test_mismatch_rejected(launch_module, tmp_path, section, key, value):
    """Mismatched consumers or disabled configuration cannot materialize D5."""
    document = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    document[section]["ros__parameters"][key] = value
    modified = tmp_path / "mismatched.yaml"
    modified.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError):
        launch_module.validate_configuration(modified)


def test_default_launch_has_no_motion_or_flight_restart(launch_module):
    """Default launch has only declarations and a gated materialization hook."""
    entities = launch_module.generate_launch_description().entities
    assert all(
        isinstance(entity, (DeclareLaunchArgument, OpaqueFunction))
        for entity in entities
    )
    context = launch_context(launch_module)
    assert context.launch_configurations["config_file"] == ""
    assert context.launch_configurations["start_flight_controller"] == "false"
    with pytest.raises(ValueError, match="explicit config_file"):
        launch_module.nodes(context)


@pytest.mark.parametrize("start_flight", ["false", "true"])
def test_explicit_launch_only_materializes_known_nodes(
    launch_module, start_flight
):
    """Inspect actions without executing any process or issuing action goals."""
    context = launch_context(
        launch_module,
        config_file=str(CONFIG),
        start_flight_controller=start_flight,
    )
    entities = launch_module.nodes(context)
    assert all(isinstance(entity, Node) for entity in entities)
    executables = [
        perform_substitutions(
            context, normalize_to_list_of_substitutions(entity.node_executable)
        )
        for entity in entities
    ]
    expected = [
        "apriltag_detection_node",
        "landing_detection_node",
        "landing_transform_node",
        "landing_gateway_node",
    ]
    if start_flight == "true":
        expected.append("flight_control_node")
    assert executables == expected


def test_non_simulation_time_rejected(launch_module):
    """Hardware-like clock selection is outside this launch's acceptance scope."""
    with pytest.raises(RuntimeError, match="use_sim_time"):
        launch_context(
            launch_module, config_file=str(CONFIG), use_sim_time="false"
        )
    context = launch_context(launch_module, config_file=str(CONFIG))
    context.launch_configurations["use_sim_time"] = "false"
    with pytest.raises(ValueError, match="use_sim_time=true"):
        launch_module.nodes(context)


def test_down_camera_matches_explicit_extrinsics_without_source_mutation(
    tmp_path,
):
    """Generated optical axes agree with configured FLU extrinsic geometry."""
    module = load_module(
        "d5_down_camera_helper", PACKAGE / "scripts/prepare_down_camera.py"
    )
    source = Path(get_package_share_directory("easyfly_px4_sitl")) / (
        "models/easyfly_iris/easyfly_iris.sdf"
    )
    original = source.read_bytes()
    destination = tmp_path / "down_camera.sdf"
    module.prepare(destination)
    assert source.read_bytes() == original
    baseline = ET.fromstring(original)
    generated = ET.parse(destination).getroot()
    baseline_body = baseline.find("./model/link[@name='base_link']")
    body = generated.find("./model/link[@name='base_link']")
    camera = body.find("sensor[@name='down_color']")
    assert camera is not None
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))[
        "landing_transform"
    ]["ros__parameters"]
    pose = [float(value) for value in camera.findtext("pose").split()]
    assert pose[:3] == pytest.approx(config["camera_translation_flu"])
    assert pose[2] == pytest.approx(-0.08)
    flight = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))[
        "flight_controller"
    ]["ros__parameters"]
    optical_distance = flight["precision_handoff_height"] - abs(pose[2])
    assert optical_distance > 0
    horizontal_fov = float(camera.findtext("camera/horizontal_fov"))
    width = float(camera.findtext("camera/image/width"))
    height = float(camera.findtext("camera/image/height"))
    assert 0 < horizontal_fov < math.pi and width > 0 and height > 0
    horizontal_span = 2 * optical_distance * math.tan(horizontal_fov / 2)
    vertical_span = horizontal_span * height / width
    # Centered horizontal board: even a 45-degree in-plane rotation must fit.
    # This checks fixture geometry, not visibility at every alignment/tilt error.
    black_edge_diagonal = config["tag_size_m"] * math.sqrt(2)
    assert black_edge_diagonal < min(horizontal_span, vertical_span)
    # Include the configured alignment radius and white margin at the floor,
    # not only a perfectly centered black marker at the handoff height.
    white_half_diagonal = 1.3 * black_edge_diagonal / 2
    tilt_span = white_half_diagonal * math.sin(flight["precision_tilt_limit"])
    floor_distance = (
        flight["precision_descent_floor_height"] - abs(pose[2]) - tilt_span
    )
    shortest_half_span = (
        floor_distance * math.tan(horizontal_fov / 2) * min(1, height / width)
    )
    required_half_span = (
        white_half_diagonal + flight["precision_align_tolerance"] + 0.01
    )
    assert shortest_half_span > required_half_span
    assert pose[3:] == pytest.approx([0.0, math.pi / 2, 0.0])
    sdf_rotation = attitude(*pose[3:])
    # Gazebo camera +X is forward; optical (+x,+y,+z)=(-Y,-Z,+X).
    expected_optical_rotation = tuple(
        (-row[1], -row[2], row[0]) for row in sdf_rotation
    )
    actual_optical_rotation = rotation(config["camera_quaternion_xyzw"])
    for actual, expected in zip(
        actual_optical_rotation, expected_optical_rotation
    ):
        assert actual == pytest.approx(expected)
    assert camera.findtext("plugin/frame_name") == config["optical_frame"]
    assert camera.findtext("plugin/ros/namespace") == "/camera_down"
    body.remove(camera)
    # All production model content other than the added fixture sensor survives.
    assert ET.tostring(body) == ET.tostring(baseline_body)
    assert ET.tostring(generated) == ET.tostring(baseline)
    assert baseline_body.find("sensor[@name='down_color']") is None
    with pytest.raises(FileExistsError):
        module.prepare(destination)
    with pytest.raises(FileExistsError):
        module.prepare(source)
    assert source.read_bytes() == original


def test_existing_fixture_output_is_never_overwritten(tmp_path):
    """An existing arbitrary destination remains byte-identical after rejection."""
    module = load_module(
        "d5_down_camera_no_overwrite",
        PACKAGE / "scripts/prepare_down_camera.py",
    )
    destination = tmp_path / "existing.sdf"
    destination.write_bytes(b"caller owned fixture")
    with pytest.raises(FileExistsError):
        module.prepare(destination)
    assert destination.read_bytes() == b"caller owned fixture"


def test_static_36h11_id7_world_uses_gazebo_material_scripts():
    """Guard the ideal-lighting fixture, not physical lighting robustness."""
    source = PACKAGE.parents[2] / (
        "simulation/easyfly_gazebo_worlds/precision_landing.world"
    )
    world = ET.parse(source).getroot().find("world")
    assert world.findtext("scene/shadows") == "false"
    model = world.find("model[@name='static_tag_7']")
    assert model is not None
    assert model.findtext("static") == "true"
    assert model.find("plugin") is None
    expected_rows = [
        "00000000",
        "00010100",
        "01110000",
        "00111010",
        "00010100",
        "00110000",
        "00010000",
        "00000000",
    ]
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))[
        "landing_transform"
    ]["ros__parameters"]
    assert config["selected_tag_id"] == 7
    assert config["tag_size_m"] == pytest.approx(0.2)
    edge = config["tag_size_m"] / 8
    visuals = model.findall("link[@name='pad']/visual")
    cells = [
        visual
        for visual in visuals
        if visual.get("name", "").startswith("cell_")
    ]
    assert len(cells) == 64
    assert {cell.get("name") for cell in cells} == {
        f"cell_{row}_{column}" for row in range(8) for column in range(8)
    }
    for row, bits in enumerate(expected_rows):
        for column, bit in enumerate(bits):
            cell = model.find(f"link/visual[@name='cell_{row}_{column}']")
            size = [
                float(value)
                for value in cell.findtext("geometry/box/size").split()
            ]
            assert size[:2] == pytest.approx([edge, edge])
            assert size[2] > 0
            pose = [float(value) for value in cell.findtext("pose").split()]
            assert pose[:2] == pytest.approx(
                [(column - 3.5) * edge, (3.5 - row) * edge]
            )
            assert pose[2] > 0 and pose[3:] == pytest.approx([0, 0, 0])
            expected_material = (
                "Gazebo/White" if bit == "1" else "Gazebo/Black"
            )
            assert cell.findtext("material/script/name") == expected_material
            assert cell.findtext("material/script/uri") == (
                "file://media/materials/scripts/gazebo.material"
            )
            assert cell.find("material/lighting") is None
    margin = model.find("link/visual[@name='white_margin']")
    assert margin.findtext("material/script/name") == "Gazebo/White"
    assert margin.find("material/lighting") is None


@pytest.mark.parametrize('fault', ['none', 'identity', 'axis', 'rotation', 'missing', 'boolean'])
def test_dynamic_launch_requires_measured_calibration(tmp_path, fault):
    """Reject incoherent encoder geometry before any launch action is created."""
    from easyfly_precision_landing.dynamic_fixture import configure
    module = load_module(
        'dynamic_launch_validation', PACKAGE / 'launch/precision_landing.launch.py')
    document = yaml.safe_load(CONFIG.read_text())
    configure(document)
    mount = document['landing_camera_mount']['ros__parameters']
    if fault == 'identity':
        mount['optical_frame'] = 'unrelated_camera'
    elif fault == 'axis':
        mount['joint_axis_flu'] = [0., 0., 0.]
    elif fault == 'rotation':
        mount['camera_quaternion_xyzw_zero'] = [0., 0., 0., 0.]
    elif fault == 'missing':
        del document['landing_camera_mount']
    elif fault == 'boolean':
        document['landing_transform']['ros__parameters']['dynamic_camera'] = 'true'
    path = tmp_path / 'dynamic.yaml'
    path.write_text(yaml.safe_dump(document))
    if fault == 'none':
        assert module.validate_configuration(path) == document
    else:
        with pytest.raises(ValueError):
            module.validate_configuration(path)


def test_dynamic_parameter_file_passes_native_rcl_parser(tmp_path):
    """The actual ROS parser must accept the shared calibrated parameter file."""
    from easyfly_precision_landing.dynamic_fixture import configure
    import rclpy
    from rclpy.context import Context
    document = yaml.safe_load(CONFIG.read_text())
    configure(document)
    path = tmp_path / 'native_dynamic.yaml'
    path.write_text(yaml.safe_dump(document))
    context = Context()
    try:
        rclpy.init(args=['--ros-args', '--params-file', str(path)], context=context)
    finally:
        if context.ok():
            context.shutdown()

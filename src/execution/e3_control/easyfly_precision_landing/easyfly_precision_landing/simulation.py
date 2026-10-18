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

"""
Unified D5 simulation: default start only; flight requires an explicit mode.

This is not a G9 loop, hardware test, or statistical accuracy certification.
Requires an isolated PX4 staging rootfs, no other flight nodes, and free ports.
"""

import argparse
from collections import Counter, deque
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from easyfly_msgs.action import ExecuteMission, Land, PrecisionLand, Takeoff
from easyfly_px4_sitl.hrt_compatibility import select_px4_directory
from easyfly_msgs.msg import (
    FlightStatus,
    LandingGuidanceTarget,
    LandingTarget,
    MissionStatus,
    TagObservationArray,
    UAVCommand,
    UAVState,
)
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from easyfly_msgs.srv import GetActionOwnership, GetCapabilities
from gazebo_msgs.srv import GetEntityState
from PIL import Image as CameraImage
import yaml
from easyfly_precision_landing.simulation_setup import (
    freeze_fastdds,
    prepare_staging,
    validate_staging,
)
from easyfly_precision_landing.node import DiagnosticExecutor
from easyfly_precision_landing import node as observation_implementation
from easyfly_precision_landing.simulation_reference import (
    fixture_plane_world_z, ground_sample, height_reference,
)
from easyfly_precision_landing.simulation_collision import fixture_ground_z
from easyfly_precision_landing.simulation_scenarios import (
    SCENARIOS, PrecisionScenario, ScenarioComplete, owned_descendants, remaining_owned,
)
from easyfly_precision_landing.simulation_faults import FAULT_SCENARIOS, ModeFaultScenario
from easyfly_precision_landing.simulation_land_cancel_faults import (
    LAND_CANCEL_FAULT_SCENARIOS, CancelLandModeScenario,
)
from easyfly_precision_landing.simulation_precision_cancel_faults import (
    PRECISION_CANCEL_FAULT_SCENARIOS, CancelPrecisionModeScenario,
)
from easyfly_precision_landing.simulation_bridge_faults import (
    BRIDGE_FAULT_SCENARIOS, BridgeModeFaultScenario,
)
from easyfly_precision_landing.simulation_arm_faults import (
    BRIDGE_ARM_FAULT_SCENARIOS, BridgeArmFaultScenario,
)
from easyfly_precision_landing.simulation_transaction_faults import (
    CANCEL_TRANSACTION_SCENARIOS, UPSTREAM_ARM_SCENARIOS, TRANSACTION_SCENARIO_TYPES,
    CancelTransactionScenario,
)

OWNED_BRIDGE_FAULT_SCENARIOS = (
    *BRIDGE_FAULT_SCENARIOS, *BRIDGE_ARM_FAULT_SCENARIOS, *CANCEL_TRANSACTION_SCENARIOS)
FLIGHT_PROXY_FAULT_SCENARIOS = (
    *FAULT_SCENARIOS, *UPSTREAM_ARM_SCENARIOS, *PRECISION_CANCEL_FAULT_SCENARIOS,
    *LAND_CANCEL_FAULT_SCENARIOS)
ALL_FAULT_SCENARIOS = (*FLIGHT_PROXY_FAULT_SCENARIOS, *OWNED_BRIDGE_FAULT_SCENARIOS)


class StartupComplete(Exception):
    """Finish start-only mode through the same strict process cleanup checks."""


class SessionTermination(Exception):
    """Request exact action draining and owned process teardown on SIGTERM."""


def fresh_disarmed_state(state, receipt, ros_now, steady_now):
    """Require independent fresh connected disarmed evidence, not an old cache."""
    if state is None:
        return False
    header = state.header
    if header.stamp.sec < 0 or not 0 <= header.stamp.nanosec < 1000000000:
        return False
    source = header.stamp.sec * 1000000000 + header.stamp.nanosec
    return (
        ros_now > 0
        and source > 0
        and -0.2 <= (ros_now - source) * 1e-9 <= 1.0
        and 0 <= steady_now - receipt <= 0.3
        and state.connected
        and not state.armed
    )


def mission_fixture(source):
    """Use the formal ordered demo, with an explicit settling hover step."""
    mission = yaml.safe_load(source.read_text())
    if mission.get("schema_version") != 1 or [
        step.get("type") for step in mission.get("steps", [])
    ] != ["takeoff", "hover", "precision_land"]:
        raise ValueError("requires formal takeoff/hover/precision_land demo")
    return mission


def pose_details(pose):
    """Preserve observed metric pose for offline diagnostics, not a new decision."""
    p, q = pose.position, pose.orientation
    return {"position": [p.x, p.y, p.z], "quaternion": [q.x, q.y, q.z, q.w]}


def geometry_details(message, selected_tag_id):
    """Preserve bounded observer evidence, even without native timing diagnostics."""
    if isinstance(message, CameraInfo):
        return {"camera_info": {
            "frame_id": message.header.frame_id,
            "width": message.width, "height": message.height,
            "distortion_model": message.distortion_model,
            "k": list(message.k), "d": list(message.d),
            "r": list(message.r), "p": list(message.p),
            "binning_x": message.binning_x, "binning_y": message.binning_y,
            "roi": {name: getattr(message.roi, name) for name in
                    ("x_offset", "y_offset", "width", "height", "do_rectify")},
        }}
    if isinstance(message, TagObservationArray):
        selected = [item for item in message.observations if item.tag_id == selected_tag_id]
        return {"selected_tag_count": len(selected), "selected_tags": [
            {"tag_id": item.tag_id, "valid": item.valid, "reason": item.reason,
             "tag_size_m": item.tag_size_m,
             "reprojection_error_px": item.reprojection_error_px,
             **pose_details(item.pose)} for item in selected[:2]
        ]}
    if isinstance(message, LandingTarget):
        return {"landing_pose": {
            "tag_id": message.tag_id, "tag_size_m": message.tag_size_m,
            "optical_depth_m": message.optical_depth_m,
            "reprojection_error_px": message.reprojection_error_px,
            **pose_details(message.pose),
        }}
    return {}


def completed_mission(status, steps):
    """Map the existing terminal status index to completed_steps explicitly."""
    return (
        status is not None
        and status.phase_name == "complete"
        and status.mission_success
        and status.current_waypoint == steps
        and status.total_waypoints == steps
        and status.progress == 1.0
    )


def exact_released(response, identity):
    """Accept release only for the exact mission action identity queried."""
    return (
        response is not None
        and response.goal_id == identity
        and response.state == GetActionOwnership.Response.RELEASED
    )


def takeoff_readiness_current(response, ros_now, query_elapsed):
    """Require current read-only evidence, never a reservation or goal retry."""
    if response is None or response.names != ['takeoff'] or response.implemented != [True]:
        return False
    if response.ready != [True] or len(response.details) != 1:
        return False
    header = response.header
    source = header.stamp.sec * 1000000000 + header.stamp.nanosec
    if (header.frame_id != 'odom' or header.stamp.sec < 0
            or not 0 <= header.stamp.nanosec < 1000000000 or source <= 0 or ros_now <= 0
            or not -.2 <= (ros_now - source) * 1e-9 <= 1.
            or not 0 <= query_elapsed <= .3):
        return False
    try:
        detail = json.loads(response.details[0])
    except (ValueError, TypeError):
        return False
    return (isinstance(detail, dict) and detail.get('reason') == 'ready'
            and detail.get('ready') is True and detail.get('flight_ready') is True
            and detail.get('state_received') is True
            and detail.get('clock_ns') == source
            and type(detail.get('state_sequence')) is int
            and detail['state_sequence'] > 0
            and type(detail.get('state_source_ns')) is int
            and detail['state_source_ns'] > 0
            and -.2 <= (ros_now - detail['state_source_ns']) * 1e-9 <= 1.
            and isinstance(detail.get('receipt_age_s'), (int, float))
            and 0 <= detail['receipt_age_s'] + query_elapsed <= 1.)


def complete_precision_chain(feedback):
    """Never combine descent and handoff from different precision actions."""
    identities = {item.get("goal_uuid", "") for item in feedback}
    return (
        len(identities) == 1
        and all(
            re.fullmatch("[0-9a-f]{32}", identity) for identity in identities
        )
        and {"descend", "land_handoff"}.issubset(
            {item.get("phase") for item in feedback}
        )
    )


def parse_arguments(argv=None):
    """Require explicit inputs and flight opt-in; help never starts processes."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", required=True)
    px4 = parser.add_mutually_exclusive_group(required=True)
    px4.add_argument("--px4-staging", help="validated EasyFly staging manifest")
    px4.add_argument("--px4-dir", help="compiled real PX4; create isolated staging")
    dds = parser.add_mutually_exclusive_group(required=True)
    dds.add_argument(
        "--dds-profile",
        default=None,
        help="explicit Fast DDS XML for this isolated simulation and its children",
    )
    dds.add_argument("--use-bundled-dds-profile", action="store_true")
    parser.add_argument("--prepare-only", action="store_true")
    flight = parser.add_mutually_exclusive_group()
    flight.add_argument("--direct-action", action="store_true")
    parser.add_argument(
        "--startup-duration", type=float, default=None,
        help="start-only observation duration; default wait for Ctrl-C",
    )
    parser.add_argument(
        "--transform-diagnostics",
        action="store_true",
        help="enable readonly bounded timing diagnostics in transform and flight",
    )
    parser.add_argument(
        "--observer-rmw-diagnostics", action="store_true",
        help="retain observer native receive/take metadata without enabling producer tracing",
    )
    parser.add_argument(
        "--ingress-timing-diagnostics", action="store_true",
        help="bounded landing ingress/publish trace, written only on clean node exit",
    )
    flight.add_argument(
        "--mission",
        action="store_true",
        help="execute the formal demo through mission FSM and SDK",
    )
    parser.add_argument("--fixture-scale", type=float, default=1.0)
    parser.add_argument(
        "--scenario", choices=(*SCENARIOS, *ALL_FAULT_SCENARIOS), default="nominal",
        help="Explicit direct-action lifecycle test; default nominal only")
    parser.add_argument("--handoff-height", type=float, default=None)
    parser.add_argument("--descent-floor-height", type=float, default=None)
    parser.add_argument(
        "--unused-camera-rate", type=int, choices=(1, 30), default=30
    )
    args = parser.parse_args(argv)
    if args.scenario != "nominal" and not args.direct_action:
        parser.error("lifecycle scenarios require explicit --direct-action")
    if args.prepare_only and (args.mission or args.direct_action):
        parser.error("prepare-only cannot request motion")
    if args.startup_duration is not None and (
        args.mission or args.direct_action or args.prepare_only
        or not math.isfinite(args.startup_duration)
        or not 0 < args.startup_duration <= 3600
    ):
        parser.error("startup-duration must be finite 0..3600 and start-only")
    return args


def main():
    """Start only owned simulation processes and require actual action results."""
    args = parse_arguments()
    startup_only = not args.mission and not args.direct_action
    if args.fixture_scale not in (1.0, 2.0):
        raise ValueError("supported explicit fixture scales: 1 or 2")
    if (
        args.handoff_height is not None
        and not 0.15 <= args.handoff_height <= 1.0
    ):
        raise ValueError("explicit simulation handoff height must be .15..1 m")
    if (args.handoff_height is None) != (args.descent_floor_height is None):
        raise ValueError("configure both handoff and descent floor together")
    if args.descent_floor_height is not None and not (
        0.15 <= args.descent_floor_height < args.handoff_height
    ):
        raise ValueError("invalid explicit approach band")
    directory = Path(args.directory).resolve(strict=True)
    if any(directory.iterdir()):
        raise ValueError("requires a new empty evidence directory")
    staging = (
        prepare_staging(select_px4_directory(args.px4_dir), directory / "px4")
        if args.px4_dir else validate_staging(select_px4_directory(args.px4_staging))
    )
    os.environ.update(
        ROS_DOMAIN_ID="122",
        GAZEBO_MASTER_URI="http://127.0.0.1:11357",
        PX4_DIR=staging["staging"],
        ROS_LOG_DIR=str(directory / "ros"),
    )
    share = Path(get_package_share_directory("easyfly_precision_landing"))
    profile = args.dds_profile or share / "config/d5_fastdds.xml"
    frozen = freeze_fastdds(profile, directory / "fastdds.xml")
    dds_profile = Path(frozen["frozen"])
    dds_profile_source = Path(frozen["source"])
    os.environ.update(frozen["env"])
    prepare = (
        get_package_prefix("easyfly_precision_landing")
        + "/lib/easyfly_precision_landing/prepare_down_camera.py"
    )
    model = directory / "down_iris.sdf"
    subprocess.run(
        [
            prepare,
            str(model),
            "--unused-camera-rate",
            str(args.unused_camera_rate),
        ],
        check=True,
    )
    reference_assets = {
        "model://mid360": Path(get_package_share_directory("easyfly_gazebo_models"))
        / "mid360/mid360.sdf",
        "model://gps": Path(staging["staging"])
        / "Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/gps/gps.sdf",
    }
    expected_ground_z = fixture_ground_z(model, include_models=reference_assets)
    config = yaml.safe_load((share / "config/d5_simulation.yaml").read_text())
    if args.ingress_timing_diagnostics:
        config["landing_detection"]["ros__parameters"]["timing_trace_path"] = str(
            directory / "landing-ingress.tsv")
    if args.transform_diagnostics:
        config["landing_transform"]["ros__parameters"]["diagnostics"] = True
        config["flight_controller"]["ros__parameters"]["precision_timing_diagnostics"] = True
    for name, parameter in (
        ("landing_transform", "tag_size_m"),
        ("apriltag_detection", "tag_size_m"),
        ("flight_controller", "precision_tag_size_m"),
    ):
        config[name]["ros__parameters"][parameter] *= args.fixture_scale
    if args.handoff_height is not None:
        config["flight_controller"]["ros__parameters"][
            "precision_handoff_height"
        ] = args.handoff_height
        config["flight_controller"]["ros__parameters"][
            "precision_descent_floor_height"
        ] = args.descent_floor_height
    # Scale only a generated visual fixture, never the installed source world.
    world = ET.parse(
        Path(get_package_share_directory("easyfly_gazebo_worlds"))
        / "precision_landing.world"
    )
    board_pose = world.find("./world/model[@name='static_tag_7']/pose")
    # Generated simulation only: stamped ground truth, independent of tag pose.
    # The service also offers set-state, but this entry never calls that service.
    reference_plugin = ET.SubElement(world.getroot().find("world"), "plugin", {
        "name": "d5_ground_reference", "filename": "libgazebo_ros_state.so",
    })
    ET.SubElement(ET.SubElement(reference_plugin, "ros"), "namespace").text = "/d5_reference"
    ET.SubElement(reference_plugin, "update_rate").text = "1.0"
    board_values = [float(value) for value in board_pose.text.split()]
    # Keep the whole tilted, scaled white margin above the ground plane.
    board_values[2] = max(
        board_values[2],
        0.13 * args.fixture_scale * abs(math.sin(board_values[4])) + 0.02,
    )
    board_pose.text = " ".join(str(value) for value in board_values)
    cell_faces = [
        float(visual.findtext("pose").split()[2])
        + float(visual.findtext("geometry/box/size").split()[2]) / 2
        for visual in world.findall("./world/model[@name='static_tag_7']/link/visual")
        if visual.get("name", "").startswith("cell_")
    ]
    board_face_world_z = fixture_plane_world_z(board_values, cell_faces)
    expected_plane_z = board_face_world_z - expected_ground_z
    # Nominal preparation only. Runtime calibrates the world->odom datum before
    # starting precision nodes; the detector cannot tune its own acceptance gate.
    config["landing_transform"]["ros__parameters"][
        "landing_plane_z_m"
    ] = expected_plane_z
    config["flight_controller"]["ros__parameters"][
        "precision_plane_z"
    ] = expected_plane_z
    for visual in world.findall(
        "./world/model[@name='static_tag_7']/link/visual"
    ):
        for field in ("pose", "geometry/box/size"):
            element = visual.find(field)
            values = [float(value) for value in element.text.split()]
            values[:2] = [value * args.fixture_scale for value in values[:2]]
            element.text = " ".join(str(value) for value in values)
    world_path = directory / "precision_fixture.world"
    with world_path.open("xb") as stream:
        world.write(stream, encoding="utf-8", xml_declaration=True)
    config_path = directory / "config.yaml"
    config_path.write_text(yaml.safe_dump(config))
    spec = importlib.util.spec_from_file_location(
        "d5_launch_validation", share / "launch/precision_landing.launch.py"
    )
    validation = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(validation)
    validation.validate_configuration(config_path)
    if args.prepare_only:
        print(json.dumps({
            "mode": "prepare_only", "staging": staging, "dds": frozen,
            "config": str(config_path), "world": str(world_path),
            "model": str(model), "motion_requested": False,
        }), flush=True)
        return
    processes = {}
    logs = []
    report = {
        "phases": [],
        "actions": {},
        "exits": {},
        "flight_samples": [],
        "flight_status_samples": [],
        "precision_feedback": [],
        "mission_feedback": [],
        "mode": "mission" if args.mission else (
            "start_only" if startup_only else "direct_action"
        ),
        "staging": staging,
        "observer_process": {
            "pid": os.getpid(),
            "start_ticks": Path(f"/proc/{os.getpid()}/stat").read_text().rsplit(
                ")", 1)[1].split()[19],
        },
        "runtime": {
            "transform_source_sha256": hashlib.sha256(
                Path(observation_implementation.__file__).read_bytes()).hexdigest(),
            "landing_binary_sha256": hashlib.sha256((Path(
                get_package_prefix("easyfly_landing_detection")) /
                "lib/easyfly_landing_detection/landing_detection_node").read_bytes()).hexdigest(),
            "mission_binary_sha256": hashlib.sha256((Path(
                get_package_prefix("easyfly_mission_fsm")) /
                "lib/easyfly_mission_fsm/mission_fsm_node").read_bytes()).hexdigest(),
            "engine_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "scenario_helper_sha256": hashlib.sha256(
                Path(__file__).with_name("simulation_scenarios.py").read_bytes()).hexdigest(),
            "fault_helper_sha256": hashlib.sha256(
                Path(__file__).with_name("simulation_faults.py").read_bytes()).hexdigest(),
            "bridge_fault_helper_sha256": hashlib.sha256(
                Path(__file__).with_name("simulation_bridge_faults.py").read_bytes()).hexdigest(),
            "arm_fault_helper_sha256": hashlib.sha256(
                Path(__file__).with_name("simulation_arm_faults.py").read_bytes()).hexdigest(),
            "transaction_fault_helper_sha256": hashlib.sha256(
                Path(__file__).with_name("simulation_transaction_faults.py")
                .read_bytes()).hexdigest(),
            "precision_cancel_fault_helper_sha256": hashlib.sha256(
                Path(__file__).with_name("simulation_precision_cancel_faults.py")
                .read_bytes()).hexdigest(),
            "land_cancel_fault_helper_sha256": hashlib.sha256(
                Path(__file__).with_name("simulation_land_cancel_faults.py")
                .read_bytes()).hexdigest(),
            "px4_sha256": hashlib.sha256(Path(staging["binary"]).read_bytes()).hexdigest(),
            "bridge_binary_sha256": hashlib.sha256((Path(
                get_package_prefix("easyfly_mavlink_bridge")) /
                "lib/easyfly_mavlink_bridge/mavlink_bridge_node").read_bytes()).hexdigest(),
            "flight_binary_sha256": hashlib.sha256((Path(
                get_package_prefix("easyfly_flight_control")) /
                "lib/easyfly_flight_control/flight_control_node").read_bytes()).hexdigest(),
        },
        "motion_requested": not startup_only,
        "command_samples": [],
        "observation_samples": [],
        "observation_geometry_details": True,
        "fixture": {
            "scale": args.fixture_scale,
            "transform_diagnostics": args.transform_diagnostics,
            "observer_rmw_diagnostics": args.observer_rmw_diagnostics,
            "ingress_timing_diagnostics": args.ingress_timing_diagnostics,
            "dds_profile": str(dds_profile) if dds_profile else None,
            "dds_profile_source": str(dds_profile_source)
            if dds_profile_source
            else None,
            "dds_profile_sha256": (
                hashlib.sha256(dds_profile.read_bytes()).hexdigest()
                if dds_profile
                else None
            ),
            "unused_camera_rate_hz": args.unused_camera_rate,
            "tag_size_m": config["landing_transform"]["ros__parameters"][
                "tag_size_m"
            ],
            "plane_z_m": expected_plane_z,
            "expected_ground_world_z_m": expected_ground_z,
            "tag_face_world_z_m": board_face_world_z,
            "reference_collision_assets": {uri: {
                "path": str(path.resolve()),
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            } for uri, path in reference_assets.items()},
            "handoff_height_m": config["flight_controller"]["ros__parameters"][
                "precision_handoff_height"
            ],
            "descent_floor_height_m": config["flight_controller"][
                "ros__parameters"
            ]["precision_descent_floor_height"],
        },
    }
    counts, reasons = Counter(), Counter()
    latest = {}
    receipts = {}
    ground_states = deque(maxlen=64)
    started = time.monotonic()

    def start(name, command):
        stream = (directory / (name + ".log")).open("w")
        logs.append(stream)
        process = subprocess.Popen(
            command, stdout=stream, stderr=stream, start_new_session=True
        )
        processes[name] = process

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node(
        "d5_sitl_observer",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    subscriptions = []
    hover_observer = [None]
    closing = [False]
    observer_timing = {}
    observer_executor = None
    scenario_kind = (CancelLandModeScenario if args.scenario in LAND_CANCEL_FAULT_SCENARIOS else
                     CancelPrecisionModeScenario
                     if args.scenario in PRECISION_CANCEL_FAULT_SCENARIOS else
                     TRANSACTION_SCENARIO_TYPES[args.scenario]
                     if args.scenario in TRANSACTION_SCENARIO_TYPES else
                     BridgeArmFaultScenario if args.scenario in BRIDGE_ARM_FAULT_SCENARIOS else
                     BridgeModeFaultScenario if args.scenario in BRIDGE_FAULT_SCENARIOS else
                     ModeFaultScenario if args.scenario in FAULT_SCENARIOS else PrecisionScenario)
    scenario = (scenario_kind(args.scenario, node, report, latest, receipts, started)
                if args.scenario != "nominal" else None)
    owned = {}
    tracked_at = [0.]

    def record_transport(topic, info):
        if topic in ("/landing_target", "/landing/guidance_target", "/uav_state",
                     "/tag_observations", "/camera_down/color/image_raw",
                     "/camera_down/color/camera_info"):
            observer_timing[topic] = {
                "header_source_ns": info.get("header_source_ns", 0),
                "source_wall_ns": info.get("source_timestamp", 0),
                "received_wall_ns": info.get("received_timestamp", 0),
                "take_wall_ns": time.time_ns(),
            }

    observer_diagnostics = args.transform_diagnostics or args.observer_rmw_diagnostics
    if observer_diagnostics:
        observer_executor = DiagnosticExecutor(node, record_transport)
        observer_executor.add_node(node)

    def spin_once():
        if observer_executor is None:
            rclpy.spin_once(node, timeout_sec=0.02)
        else:
            observer_executor.spin_once(timeout_sec=0.02)

    def interrupted(number, frame):
        report["stop_signal"] = number
        if closing[0]:
            return
        if number == signal.SIGINT:
            raise KeyboardInterrupt()
        raise SessionTermination("session terminated by SIGTERM")

    previous_signals = {
        number: signal.signal(number, interrupted)
        for number in (signal.SIGINT, signal.SIGTERM)
    }

    def precision_feedback(event):
        value = event.feedback
        if args.mission:
            report.setdefault(
                "precision_started_after_stable_hover", "hover_z_m" in report
            )
        target = latest.get("/landing/guidance_target")
        if target is not None and target.valid:
            report.setdefault(
                "prelanding_target",
                {
                    "x": target.position.x,
                    "y": target.position.y,
                    "z": target.position.z,
                    "tag_id": target.tag_id,
                },
            )
        report["phases"].append(value.phase)
        report["precision_feedback"].append(
            {
                "elapsed_s": time.monotonic() - started,
                "goal_uuid": bytes(event.goal_id.uuid).hex(),
                "phase": value.phase,
                "target_valid": value.target_valid,
                "horizontal_error_m": value.horizontal_error_m,
                "height_above_target_m": value.height_above_target_m,
            }
        )

    if args.mission:
        subscriptions.append(
            node.create_subscription(
                PrecisionLand.Impl.FeedbackMessage,
                "/precision_land/_action/feedback",
                precision_feedback,
                10,
            )
        )

    def receive(topic, message):
        counts[topic] += 1
        latest[topic] = message
        receipts[topic] = time.monotonic()
        if scenario is not None:
            if topic == "/camera_down/color/image_raw":
                scenario.image(message)
            elif topic == "/uav_command":
                scenario.command(message)
            elif topic == "/uav_state":
                scenario.state(message)
            elif topic == "/flight_status":
                scenario.flight_status(message)
        if topic == "/uav_state":
            ground_states.append((message, receipts[topic]))
        if topic == "/flight_status":
            if len(report["flight_status_samples"]) >= 10000:
                raise RuntimeError("flight-status observation capacity exhausted")
            report["flight_status_samples"].append({
                "source_ns": message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec,
                "clock_ns": node.get_clock().now().nanoseconds,
                "observer_receipt_steady_s": receipts[topic],
                "receipt_age_s": message.receipt_age_s, "valid": message.valid,
                "mode": message.flight_mode_name, "armed": message.armed,
                "connected": message.connected})
        if (
            topic == "/uav_command" and counts[topic] % 5 == 0
            and len(report["command_samples"]) < 10000
        ):
            report["command_samples"].append(
                {
                    "stamp": message.header.stamp.sec
                    + message.header.stamp.nanosec * 1e-9,
                    "x": message.position_x,
                    "y": message.position_y,
                    "z": message.position_z,
                }
            )
        if (
            topic == "/uav_state" and counts[topic] % 5 == 0
            and len(report["flight_samples"]) < 10000
        ):
            report["flight_samples"].append(
                {
                    "stamp": message.header.stamp.sec
                    + message.header.stamp.nanosec * 1e-9,
                    "observer_receipt_steady_s": receipts[topic],
                    "x": message.position_x,
                    "y": message.position_y,
                    "z": message.position_z,
                    "roll": message.roll,
                    "pitch": message.pitch,
                    "yaw": message.yaw,
                    "mode": message.flight_mode_name,
                    "armed": message.armed,
                }
            )
        if topic == "/camera_down/color/image_raw" and counts[topic] % 30 == 0:
            if message.encoding in ("rgb8", "bgr8"):
                encoding = "RGB" if message.encoding == "rgb8" else "BGR"
                frame = CameraImage.frombytes(
                    "RGB",
                    (message.width, message.height),
                    bytes(message.data),
                    "raw",
                    encoding,
                    message.step,
                )
                frame.save(directory / "latest_down_camera.png")
                if counts[topic] <= 3000:
                    frame.save(directory / ("camera_%04d.png" % counts[topic]))
        if hasattr(message, "valid"):
            counts[topic + ":valid"] += message.valid
        if hasattr(message, "reason"):
            reasons[topic + ":" + message.reason] += 1
        if (
            topic
            in (
                "/tag_observations",
                "/landing_target",
                "/landing/guidance_target",
                "/camera_down/color/image_raw",
                "/camera_down/color/camera_info",
            )
            and len(report["observation_samples"]) < 10000
        ):
            report["observation_samples"].append(
                {
                    "topic": topic,
                    "elapsed_s": time.monotonic() - started,
                    "observer_receipt_steady_s": receipts[topic],
                    "clock_ns": node.get_clock().now().nanoseconds,
                    "source_ns": message.header.stamp.sec * 1000000000
                    + message.header.stamp.nanosec,
                    "valid": getattr(message, "valid", None),
                    "reason": getattr(message, "reason", "image"),
                }
            )
            if observer_diagnostics and topic in observer_timing:
                report["observation_samples"][-1]["rmw_timing"] = dict(
                    observer_timing[topic]
                )
            report["observation_samples"][-1].update(
                geometry_details(message, config["landing_detection"][
                    "ros__parameters"]["selected_tag_id"])
            )
            if topic == "/camera_down/color/image_raw":
                report["observation_samples"][-1][
                    "image_sha256"
                ] = hashlib.sha256(bytes(message.data)).hexdigest()

    for topic, kind in [
        ("/uav_state", UAVState),
        ("/flight_status", FlightStatus),
        ("/uav_command", UAVCommand),
        ("/tag_observations", TagObservationArray),
        ("/landing_target", LandingTarget),
        ("/landing/guidance_target", LandingGuidanceTarget),
        ("/camera_down/color/image_raw", Image),
        ("/camera_down/color/camera_info", CameraInfo),
        ("/mission_status", MissionStatus),
    ]:
        subscriptions.append(
            node.create_subscription(
                kind,
                topic,
                lambda message, topic=topic: receive(topic, message),
                qos_profile_sensor_data,
            )
        )

    def wait(predicate, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            spin_once()
            if time.monotonic() - tracked_at[0] >= .5:
                owned.update(owned_descendants(os.getpid()))
                tracked_at[0] = time.monotonic()
            if scenario is not None:
                scenario.tick()
                if isinstance(scenario, (CancelTransactionScenario, CancelPrecisionModeScenario,
                                         CancelLandModeScenario)):
                    scenario.record_pending(wait)
            if hover_observer[0] is not None and hover_observer[0]():
                report.setdefault("hover_z_m", latest["/uav_state"].position_z)
            if any(
                process.poll() is not None for process in processes.values()
            ):
                raise RuntimeError("a launch process exited prematurely")
            if predicate():
                return
        raise TimeoutError(
            "SITL evidence deadline: " + json.dumps(dict(reasons))
        )

    def action(name, kind, goal, budget):
        client = ActionClient(node, kind, name)
        try:
            wait(client.server_is_ready, 15)
            if name in ("/takeoff", "/execute_mission"):
                wait_takeoff_readiness()

            def feedback(event):
                if hasattr(event.feedback, "phase"):
                    precision_feedback(event)
                elif hasattr(event.feedback, "current_status"):
                    status = event.feedback.current_status
                    report["mission_feedback"].append(
                        {
                            "elapsed_s": time.monotonic() - started,
                            "goal_uuid": bytes(event.goal_id.uuid).hex(),
                            "phase": status.phase_name,
                            "step_index": status.current_waypoint,
                            "total_steps": status.total_waypoints,
                            "progress": status.progress,
                        }
                    )

            future = client.send_goal_async(goal, feedback_callback=feedback)
            try:
                wait(future.done, 5)
            except (TimeoutError, SessionTermination, KeyboardInterrupt):
                # Never resend an unknown admission. Wait for this exact response,
                # then cancel only the accepted goal and drain its real terminal.
                report["actions"][name] = {"admission": "unknown"}
                wait(future.done, 15)
                handle = future.result()
                if handle.accepted:
                    result = handle.get_result_async()
                    cancel = handle.cancel_goal_async()
                    wait(cancel.done, 5)
                    wait(result.done, 20)
                    if name == "/execute_mission":
                        wait_mission_release(handle)
                raise
            handle = future.result()
            if not handle.accepted:
                report["actions"][name] = {
                    "accepted": False,
                    "goal_uuid": bytes(handle.goal_id.uuid).hex(),
                }
                raise RuntimeError(name + ": goal rejected")
            print(json.dumps({
                "action": name, "accepted": True,
                "goal_uuid": bytes(handle.goal_id.uuid).hex(),
            }), flush=True)
            result = handle.get_result_async()
            if (scenario is not None
                    and name == getattr(scenario, "action_name", "/precision_land")):
                scenario.attach(handle)
            try:
                wait(result.done, budget)
            except (Exception, KeyboardInterrupt):
                cancel = handle.cancel_goal_async()
                try:
                    wait(cancel.done, 5)
                    wait(result.done, 20)
                    report["actions"][name] = {
                        "goal_uuid": bytes(handle.goal_id.uuid).hex(),
                        "cancel_terminal": result.result().status,
                    }
                    if name == "/execute_mission":
                        wait_mission_release(handle)
                except Exception as drain_error:
                    report["cancel_drain_error"] = str(drain_error)
                raise
            value = result.result()
            report["actions"][name] = {
                "status": value.status,
                "success": value.result.success,
                "message": value.result.message,
                "goal_uuid": bytes(handle.goal_id.uuid).hex(),
            }
            print(json.dumps(report["actions"][name]), flush=True)
            if (scenario is not None
                    and name == getattr(scenario, "action_name", "/precision_land")):
                scenario.accept_terminal(value, wait)
            if name == "/execute_mission":
                wait_mission_release(handle)
            if value.status != 4 or not value.result.success:
                raise RuntimeError(name + ": real action did not succeed")
        finally:
            client.destroy()

    def wait_takeoff_readiness():
        client = node.create_client(GetCapabilities, "/platform/flight_control/capabilities")
        samples = report.setdefault("takeoff_readiness_samples", [])
        try:
            wait(client.service_is_ready, 15)
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                began = time.monotonic()
                future = client.call_async(GetCapabilities.Request())
                wait(future.done, min(3, max(.001, deadline - time.monotonic())))
                response = future.result()
                ended = time.monotonic()
                clock = node.get_clock().now().nanoseconds
                if len(samples) >= 1000:
                    raise RuntimeError("takeoff readiness recorder capacity exhausted")
                current = takeoff_readiness_current(response, clock, ended - began)
                samples.append({
                    "elapsed_s": ended - started, "clock_ns": clock,
                    "query_elapsed_s": ended - began, "current_ready": current,
                    "names": list(response.names) if response is not None else [],
                    "ready": list(response.ready) if response is not None else [],
                    "details": list(response.details) if response is not None else [],
                })
                if current and preflight_ready():
                    report["takeoff_internal_ready"] = samples[-1]
                    return
                wait(lambda: time.monotonic() - ended >= .02, 1)
            raise TimeoutError("flight controller internal Takeoff readiness not confirmed")
        finally:
            node.destroy_client(client)

    def wait_mission_release(handle):
        client = node.create_client(GetActionOwnership, "/mission/ownership")
        try:
            wait(client.service_is_ready, 5)
            identity = bytes(handle.goal_id.uuid).hex()
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                future = client.call_async(
                    GetActionOwnership.Request(goal_id=identity)
                )
                wait(future.done, 3)
                if exact_released(future.result(), identity):
                    report["mission_ownership"] = "RELEASED"
                    report["mission_ownership_uuid"] = identity
                    return
            raise TimeoutError("mission child ownership did not drain")
        finally:
            node.destroy_client(client)

    preflight_stationary_since = None

    def preflight_ready():
        nonlocal preflight_stationary_since
        state = latest.get("/uav_state")
        if state is None:
            preflight_stationary_since = None
            return False
        source = (
            state.header.stamp.sec * 1000000000 + state.header.stamp.nanosec
        )
        age = (node.get_clock().now().nanoseconds - source) * 1e-9
        ready = (
            source > 0
            and node.get_clock().now().nanoseconds > 1000000000
            and -0.2 <= age <= 1.0
            and 0 <= time.monotonic() - receipts.get("/uav_state", 0.0) <= 0.3
            and state.connected
            and state.pose_valid
            and state.attitude_valid
            and state.velocity_valid
            and all(math.isfinite(value) for value in (
                state.position_x, state.position_y, state.position_z,
                state.roll, state.pitch, state.yaw,
                state.velocity_x, state.velocity_y, state.velocity_z,
            ))
            and math.hypot(state.velocity_x, state.velocity_y, state.velocity_z) <= 0.05
            and max(abs(state.roll), abs(state.pitch)) <= 0.1
            and not state.armed
            and state.flight_mode <= 7
        )
        # A newly valid EKF pose can still be settling. Establish the same
        # stationary prerequisites as ground_sample before its strict sampling;
        # calibration still rejects any new moving/untrusted sample unchanged.
        now = time.monotonic()
        if not ready:
            preflight_stationary_since = None
            return False
        if preflight_stationary_since is None:
            preflight_stationary_since = now
        duration = now - preflight_stationary_since
        if duration < 1.8:
            return False
        report["preflight_stationary"] = {
            "duration_s": duration, "source_ns": source,
            "velocity_mps": [state.velocity_x, state.velocity_y, state.velocity_z],
            "armed": state.armed,
        }
        return True

    def calibrate_ground_reference():
        client = node.create_client(GetEntityState, "/d5_reference/get_entity_state")
        accepted = deque(maxlen=10)
        rejected = Counter()
        report["ground_reference_rejections"] = rejected
        try:
            wait(client.service_is_ready, 15)
            deadline = time.monotonic() + 20
            last_clock = node.get_clock().now().nanoseconds
            while time.monotonic() < deadline:
                began = time.monotonic()
                future = client.call_async(GetEntityState.Request(
                    name="easyfly_iris::base_link", reference_frame="world"))
                try:
                    wait(future.done, 1)
                    response = future.result()
                finally:
                    if not future.done():
                        client.remove_pending_request(future)
                now_clock = node.get_clock().now().nanoseconds
                if now_clock < last_clock:
                    raise RuntimeError("clock rewind during ground datum calibration")
                last_clock = now_clock
                # Never fall back to an older trustworthy state after new armed,
                # disconnected, moving or nonfinite feedback.
                state, receipt = ground_states[-1] if ground_states else (None, 0.0)
                attempts = report.setdefault("ground_reference_attempts", [])
                if len(attempts) < 100:
                    attempts.append({
                        "entity": "easyfly_iris::base_link", "reference_frame": "world",
                        "request_started_steady_s": began, "response_steady_s": time.monotonic(),
                        "clock_ns": now_clock,
                        "expected_ground_world_z_m": expected_ground_z,
                        "odom_z_m": state.position_z if state is not None else None,
                        "odom_source_ns": state.header.stamp.sec * 1000000000
                        + state.header.stamp.nanosec if state is not None else None,
                        "world_source_ns": response.header.stamp.sec * 1000000000
                        + response.header.stamp.nanosec if response is not None else None,
                        "world_frame_id": response.header.frame_id
                        if response is not None else None,
                        "world_pose": pose_details(response.state.pose)
                        if response is not None else None,
                        "world_linear_velocity": [response.state.twist.linear.x,
                                                  response.state.twist.linear.y,
                                                  response.state.twist.linear.z]
                        if response is not None else None,
                    })
                try:
                    sample = ground_sample(state, receipt, response, now_clock,
                                           time.monotonic(), began, expected_ground_z)
                except ValueError as error:
                    if str(error) != "stale or unmatched ground reference":
                        raise
                    rejected[str(error)] += 1
                    sample = None
                if sample is not None and (not accepted or (
                    sample["source_ns"] > accepted[-1]["source_ns"]
                    and sample["world_source_ns"] > accepted[-1]["world_source_ns"]
                )):
                    accepted.append(sample)
                    report["ground_reference_samples"] = list(accepted)
                    try:
                        return height_reference(list(accepted), board_face_world_z)
                    except ValueError as error:
                        rejected[str(error)] += 1
                wait(lambda: time.monotonic() - began >= 0.2, 1)
            raise RuntimeError("cannot establish independent stationary ground datum")
        finally:
            node.destroy_client(client)

    error = None
    try:
        # The installed entrypoint cannot infer the repository path; use the
        # explicitly configured capture script, or run ordinary launch if absent.
        capture = os.environ.get("EASYFLY_SHUTDOWN_CAPTURE", "")
        base = ["ros2", "launch"]
        if capture:
            prefix = (
                "gdb -q -batch --return-child-result -x " + capture + " --args"
            )
            base += [
                "--launch-prefix",
                prefix,
                "--launch-prefix-filter",
                ".*gzserver$",
            ]
        start(
            "simulation",
            base
            + [
                "easyfly_px4_sitl",
                "px4_sitl.launch.py",
                "world:=" + str(world_path),
                "gzclient:=false",
                "model_sdf:=" + str(model),
            ],
        )
        # Do not start MAVROS/flight against an existing session if its guard
        # rejects us. Only our freshly created simulation log may authorize this.
        wait(
            lambda: "EASYFLY_SITL_RESERVED:"
            in (directory / "simulation.log").read_text(errors="replace"),
            10,
        )
        wait(lambda: node.get_clock().now().nanoseconds > 0, 15)
        if any(node.count_publishers(topic) for topic in (
            "/uav_state", "/uav_command", "/landing_target",
            "/landing/guidance_target",
        )):
            raise RuntimeError("existing flight/perception graph; refusing duplicate stack")
        start(
            "bridge",
            [
                "ros2",
                "launch",
                "easyfly_mavlink_bridge",
                "mavlink_bridge.launch.py",
                "use_sim_time:=true",
                "start_bridge:=" + (
                    "false" if args.scenario in OWNED_BRIDGE_FAULT_SCENARIOS else "true"),
            ],
        )
        if args.scenario in OWNED_BRIDGE_FAULT_SCENARIOS:
            start("bridge_control", [
                get_package_prefix("easyfly_mavlink_bridge")
                + "/lib/easyfly_mavlink_bridge/mavlink_bridge_node",
                "--ros-args", "--params-file", str(Path(
                    get_package_share_directory("easyfly_mavlink_bridge"))
                    / "config/mavlink_bridge.yaml"),
                "-p", "use_sim_time:=true",
                "-r", "/mavros/set_mode:=/d5_test/mavros/set_mode",
                "-r", "/mavros/cmd/arming:=/d5_test/mavros/cmd/arming",
            ])
        wait(preflight_ready, 80)
        if isinstance(scenario, ModeFaultScenario):
            wait(lambda: scenario.real_mode.service_is_ready()
                 and scenario.real_arm.service_is_ready(), 15)
            report["fault_proxy_peers_prepared_steady"] = time.monotonic()
            if getattr(scenario, "owns_bridge", False):
                report["bridge_proxy_peers_prepared_steady"] = (
                    report["fault_proxy_peers_prepared_steady"])
        reference = calibrate_ground_reference()
        report["ground_reference"] = reference
        expected_plane_z = reference["plane_z_m"]
        config["landing_transform"]["ros__parameters"]["landing_plane_z_m"] = expected_plane_z
        config["flight_controller"]["ros__parameters"]["precision_plane_z"] = expected_plane_z
        config_path.write_text(yaml.safe_dump(config))
        validation.validate_configuration(config_path)
        report["fixture"]["plane_z_m"] = expected_plane_z
        print(json.dumps({"ground_reference_plane_z_m": expected_plane_z,
                          "ground_reference_offset_z_m": reference["offset_z_m"]}), flush=True)
        precision_command = [
            "ros2", "launch", "easyfly_precision_landing", "precision_landing.launch.py",
            "config_file:=" + str(config_path),
            "start_flight_controller:=" + (
                "false" if args.scenario in ALL_FAULT_SCENARIOS else "true"),
        ]
        if args.scenario in ALL_FAULT_SCENARIOS:
            flight_command = [
                get_package_prefix("easyfly_flight_control")
                + "/lib/easyfly_flight_control/flight_control_node",
                "--ros-args", "--params-file", str(config_path),
                "-r", "__node:=flight_controller", "-p", "use_sim_time:=true",
            ]
            if args.scenario in FLIGHT_PROXY_FAULT_SCENARIOS:
                flight_command += ["-r", "/set_flight_mode:=/d5_test/set_flight_mode",
                                   "-r", "/arm_motors:=/d5_test/arm_motors"]
            start("flight", flight_command)
        if args.scenario.startswith("loss-"):
            precision_command.append("image_topic:=/d5_test/image_raw")
        start(
            "precision",
            precision_command,
        )
        wait(preflight_ready, 80)
        # Actual internal readiness is queried immediately before one goal send.
        # Observer readiness and endpoint discovery alone are not that evidence.
        state = latest["/uav_state"]
        report["preflight"] = {
            "mode": state.flight_mode_name,
            "armed": state.armed,
            "z": state.position_z,
            "stamp_sec": state.header.stamp.sec,
            "ros_now_ns": node.get_clock().now().nanoseconds,
        }
        print(json.dumps(report["preflight"]), flush=True)
        if startup_only:
            report["startup_ready"] = True
            print("D5_READY: simulation started; no flight goal sent", flush=True)
            if args.startup_duration is None:
                while True:
                    wait(lambda: True, 1)
            else:
                deadline = time.monotonic() + args.startup_duration
                wait(lambda: time.monotonic() >= deadline, args.startup_duration + 1)
            raise StartupComplete()
        # Ordinary Takeoff uses its existing arrival radius. Its terminal does
        # not prove this fixture has settled at exactly 1 m; wait on real state.
        settled_since = None
        settled_stamps = set()

        def stable_hover():
            nonlocal settled_since
            state = latest.get("/uav_state")
            stamp = (
                state.header.stamp.sec * 1000000000
                + state.header.stamp.nanosec
                if state is not None
                else 0
            )
            age = (node.get_clock().now().nanoseconds - stamp) * 1e-9
            if (
                state is None
                or not state.connected
                or stamp <= 0
                or not -0.2 <= age <= 1.0
                or time.monotonic() - receipts.get("/uav_state", 0.0) > 0.3
                or not all(
                    math.isfinite(value)
                    for value in (
                        state.position_z,
                        state.velocity_z,
                    )
                )
                or not state.armed
                or state.flight_mode != 3
                or not state.pose_valid
                or not state.velocity_valid
                or abs(state.position_z - 1.0) > 0.08
                or abs(state.velocity_z) > 0.05
            ):
                settled_since = None
                settled_stamps.clear()
                return False
            if settled_since is None:
                settled_since = time.monotonic()
            settled_stamps.add(stamp)
            return (
                time.monotonic() - settled_since >= 1.0
                and len(settled_stamps) >= 5
            )

        if args.mission:
            mission = mission_fixture(
                Path(get_package_share_directory("easyfly_mission_fsm"))
                / "missions/precision_landing_demo.yaml"
            )
            mission_path = directory / "mission.yaml"
            mission_path.write_text(yaml.safe_dump(mission))
            report["mission_file"] = str(mission_path)
            start(
                "mission",
                [
                    get_package_prefix("easyfly_mission_fsm")
                    + "/lib/easyfly_mission_fsm/mission_fsm_node",
                    "--ros-args",
                    "-p",
                    "use_sim_time:=true",
                ],
            )
            hover_observer[0] = stable_hover
            # Action discovery can precede the mission SDK's first state callback.
            # Match its real subscription and clock, then observe a stable startup
            # interval. This never retries an already submitted/unknown goal.
            mission_ready_since = None

            def mission_startup_ready():
                nonlocal mission_ready_since
                state = latest.get("/uav_state")
                matched = any(
                    item.node_name.startswith("easyfly_sdk_")
                    for item in node.get_subscriptions_info_by_topic(
                        "/uav_state"
                    )
                )
                ready = (
                    matched
                    and preflight_ready()
                    and state.battery_valid
                    and math.isfinite(state.battery_remaining)
                    and 0.15 <= state.battery_remaining <= 1.0
                    and time.monotonic() - receipts.get("/uav_state", 0.0)
                    <= 0.3
                )
                if not ready:
                    mission_ready_since = None
                    return False
                if mission_ready_since is None:
                    mission_ready_since = time.monotonic()
                return time.monotonic() - mission_ready_since >= 1.0

            wait(mission_startup_ready, 15)
            action(
                "/execute_mission",
                ExecuteMission,
                ExecuteMission.Goal(mission_file=str(mission_path)),
                220,
            )
            wait(
                lambda: completed_mission(
                    latest.get("/mission_status"), len(mission["steps"])
                ),
                5,
            )
            report["completed_steps"] = latest[
                "/mission_status"
            ].current_waypoint
            if not report.get("precision_started_after_stable_hover", False):
                raise RuntimeError(
                    "precision started before independently measured stable 1m hover"
                )
        else:
            action(
                "/takeoff",
                Takeoff,
                Takeoff.Goal(target_altitude=1.0, climb_speed=0.3),
                60,
            )
            wait(stable_hover, 30)
            report["hover_z_m"] = latest["/uav_state"].position_z
        if not args.mission:
            if args.scenario in LAND_CANCEL_FAULT_SCENARIOS:
                action('/land', Land, Land.Goal(), 15)
            wait(
                lambda: "/landing/guidance_target" in latest
                and latest["/landing/guidance_target"].valid,
                10,
            )
            target = latest["/landing/guidance_target"]
            report["prelanding_target"] = {
                "x": target.position.x,
                "y": target.position.y,
                "z": target.position.z,
                "tag_id": target.tag_id,
            }
            action(
                "/precision_land",
                PrecisionLand,
                PrecisionLand.Goal(tag_id=7, timeout=90.0),
                95,
            )
        wait(
            lambda: fresh_disarmed_state(
                latest.get("/uav_state"),
                receipts.get("/uav_state", 0.0),
                node.get_clock().now().nanoseconds,
                time.monotonic(),
            ),
            5,
        )
        if not complete_precision_chain(report["precision_feedback"]):
            raise RuntimeError(
                "success without complete descend/handoff evidence"
            )
        report["precision_goal_uuid"] = report["precision_feedback"][0][
            "goal_uuid"
        ]
        precision_owner = node.create_client(GetActionOwnership, "/precision_landing/ownership")
        try:
            wait(precision_owner.service_is_ready, 5)
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                future = precision_owner.call_async(GetActionOwnership.Request(
                    goal_id=report["precision_goal_uuid"]))
                wait(future.done, 3)
                if exact_released(future.result(), report["precision_goal_uuid"]):
                    report["precision_ownership_released_uuid"] = report["precision_goal_uuid"]
                    break
            else:
                raise TimeoutError("successful precision ownership did not drain")
        finally:
            node.destroy_client(precision_owner)
        report["final_disarmed"] = True
        final_state = latest["/uav_state"]
        report["final_state_source_ns"] = (
            final_state.header.stamp.sec * 1000000000
            + final_state.header.stamp.nanosec
        )
        report["final_state_ros_age_s"] = (
            node.get_clock().now().nanoseconds
            - report["final_state_source_ns"]
        ) * 1e-9
        report["final_state_receipt_age_s"] = (
            time.monotonic() - receipts["/uav_state"]
        )
        report["final_horizontal_error_m"] = (
            math.hypot(
                final_state.position_x - report["prelanding_target"]["x"],
                final_state.position_y - report["prelanding_target"]["y"],
            )
            if final_state.pose_valid
            and all(
                math.isfinite(value)
                for value in (final_state.position_x, final_state.position_y)
            )
            else None
        )
    except StartupComplete:
        pass
    except ScenarioComplete:
        pass
    except KeyboardInterrupt:
        if startup_only:
            report["stopped_by_operator"] = True
        else:
            error = RuntimeError("flight check interrupted; inspect exact cancel/drain evidence")
            report["error"] = str(error)
    except Exception as exception:
        error = exception
        report["error"] = str(exception)
    finally:
        closing[0] = True
        owned.update(owned_descendants(os.getpid()))
        for name in ("mission", "flight", "precision", "bridge_control", "bridge", "simulation"):
            process = processes.get(name)
            if process is None:
                continue
            if process.poll() is None:
                # Let ROS launch forward exactly once. Signalling the whole
                # group then launch forwarding causes duplicate SIGINT in cleanup.
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                report["exits"][name + ":forced_term"] = True
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
            report["exits"][name] = process.returncode
        report["counts"] = dict(counts)
        for stream in logs:
            stream.flush()
        failure_pattern = re.compile(
            r"process has died|Program received signal SIG(?:SEGV|ABRT|KILL)"
            r"|terminate called|escalating to SIG(?:TERM|KILL)"
            r"|Traceback \(most recent call last\)"
        )
        report["child_exit_failures"] = {
            name: [
                line
                for line in (directory / (name + ".log"))
                .read_text(errors="replace")
                .splitlines()
                if failure_pattern.search(line)
            ]
            for name in processes
        }
        report["reasons"] = dict(reasons)
        report["owned_processes"] = owned
        report["leftovers"] = remaining_owned(owned)
        if isinstance(scenario, ModeFaultScenario):
            flight_process = processes.get("flight")
            report["flight_exit_before_fixture_drain"] = (
                flight_process.returncode if flight_process is not None else None)
            if getattr(scenario, "owns_bridge", False):
                bridge_process = processes.get("bridge_control")
                report["bridge_exit_before_fixture_drain"] = (
                    bridge_process.returncode if bridge_process is not None else None)
            try:
                scenario.drain(spin_once)
            except Exception as teardown_error:
                report["fixture_teardown_error"] = str(teardown_error)
                error = teardown_error
        executor = observer_executor or rclpy.get_global_executor()
        report["observer_shutdown_confirmed"] = executor.shutdown(timeout_sec=3)
        if not report["observer_shutdown_confirmed"]:
            error = RuntimeError("observer callbacks did not drain before shutdown")
            report["fixture_teardown_error"] = str(error)
        if scenario is not None:
            scenario.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        report["passed"] = (error is None and not report["leftovers"]
                            and all(v == 0 for v in report["exits"].values())
                            and not any(report["child_exit_failures"].values()))
        report["phases"] = list(dict.fromkeys(report["phases"]))
        (directory / "result.json").write_text(json.dumps(report, indent=2))
        print(
            json.dumps(
                {
                    key: value
                    for key, value in report.items()
                    if key
                    not in (
                        "flight_samples",
                        "flight_status_samples",
                        "precision_feedback",
                        "mission_feedback",
                        "command_samples",
                        "observation_samples",
                    )
                }
            ),
            flush=True,
        )
        for stream in logs:
            stream.close()
        for number, handler in previous_signals.items():
            signal.signal(number, handler)
    if error is not None:
        raise error
    if any(value != 0 for value in report["exits"].values()):
        raise RuntimeError("process exit failure")
    if any(report["child_exit_failures"].values()):
        raise RuntimeError("child process exit failure; inspect launch logs")
    if report["leftovers"]:
        raise RuntimeError("owned process identity remains after shutdown")


if __name__ == "__main__":
    main()

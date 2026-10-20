#!/usr/bin/env python3
"""Launch-semantic regression tests for the composed FAST-LIO path."""

import importlib.util
import inspect
import tempfile
from pathlib import Path

import ament_index_python.packages as ament_packages
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch_ros.actions import Node
from launch_ros.actions.load_composable_nodes import get_composable_node_load_request


PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = Path(__file__).resolve().parents[5] / "src"
FASTLIO_PATH = PACKAGE_DIR / "launch" / "fastlio.launch.py"
PERCEPTION_PATH = (
    WORKSPACE_SRC / "simulation" / "easyfly_px4_sitl" / "launch" /
    "perception_sitl.launch.py"
)


def load_launch_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise AssertionError(f"unable to load launch file: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def parameter_files(node: Node, context: LaunchContext):
    """Resolve a Node's actual launch_ros parameter-file arguments."""
    node._perform_substitutions(context)
    expanded = node._Node__expanded_parameter_arguments
    return [Path(path) for path, is_file in expanded if is_file]


def make_context(**configurations: str) -> LaunchContext:
    context = LaunchContext()
    context.launch_configurations.update(configurations)
    return context


def guarded_actions(description):
    """Inspect the reservation's actions without executing/acquiring its lock."""
    for action in description.entities:
        if isinstance(action, OpaqueFunction):
            function = action._OpaqueFunction__function
            if function.__module__ == "easyfly_px4_sitl.session_guard":
                yield from inspect.getclosurevars(function).nonlocals["actions"]
        else:
            yield action


def test_standalone_legacy_config_file_is_effective() -> None:
    fastlio = load_launch_module(FASTLIO_PATH, "fastlio_launch_standalone")
    with tempfile.TemporaryDirectory(prefix="easyfly-fastlio-launch-") as tmp:
        share = Path(tmp)
        config_dir = share / "config"
        config_dir.mkdir()
        (config_dir / "custom.yaml").write_text(
            "/**:\n  ros__parameters:\n    mapping.extrinsic_T: [0.0, 0.0, 0.0]\n")
        (config_dir / "mid360.yaml").write_text(
            "/**:\n  ros__parameters:\n    mapping.extrinsic_T: [0.0, 0.0, 0.0]\n")

        original_share_lookup = fastlio.get_package_share_directory
        fastlio.get_package_share_directory = lambda package: str(share)
        try:
            context = make_context(
                use_sim_time="true",
                fastlio_config_file="mid360.yaml",
                config_file="custom.yaml",
            )
            assert fastlio._select_fastlio_config(context) == "custom.yaml"
            node = fastlio._launch_fast_lio(context)[0]
            files = parameter_files(node, context)
            dedicated_context = make_context(
                use_sim_time="true",
                fastlio_config_file="mid360.yaml",
                config_file="",
            )
            assert fastlio._select_fastlio_config(dedicated_context) == "mid360.yaml"
        finally:
            fastlio.get_package_share_directory = original_share_lookup

    assert files
    assert files[0] == config_dir / "custom.yaml"


def test_parent_group_blocks_detection_config_file() -> None:
    """Execute the real parent GroupAction up to the included child Node."""
    parent = load_launch_module(PERCEPTION_PATH, "perception_sitl_launch")
    source_dirs = {
        "easyfly_px4_sitl": WORKSPACE_SRC / "simulation" / "easyfly_px4_sitl",
        "easyfly_gazebo_models": WORKSPACE_SRC / "simulation" / "easyfly_gazebo_models",
        "easyfly_gazebo_worlds": WORKSPACE_SRC / "simulation" / "easyfly_gazebo_worlds",
        "easyfly_gazebo_plugins": WORKSPACE_SRC / "simulation" / "easyfly_gazebo_plugins",
        "easyfly_fast_lio_integration": PACKAGE_DIR,
        "fast_lio": WORKSPACE_SRC / "third_party" / "algorithms" / "localization" / "fast_lio",
    }

    def source_share(package: str) -> str:
        return str(source_dirs.get(package, WORKSPACE_SRC))

    # This fixture inspects launch actions; it never starts PX4/Gazebo. Full HRT/native-IMU
    # selection has dedicated fail-closed tests and is not a prerequisite for config scoping.
    original_resolve_px4 = parent._resolve_px4_dir
    original_native_imu = parent.require_native_imu
    parent._resolve_px4_dir = lambda: Path("/inert-test-only-px4")
    parent.require_native_imu = lambda directory: None
    original_parent_lookup = parent.get_package_share_directory
    original_ament_lookup = ament_packages.get_package_share_directory
    parent.get_package_share_directory = source_share
    ament_packages.get_package_share_directory = source_share
    try:
        description = parent.generate_launch_description()
        p2_timer = next(
            action for action in guarded_actions(description)
            if getattr(action, "period", None) == 14.0)
        fastlio_group = next(
            action for action in p2_timer.actions if isinstance(action, GroupAction))

        context = make_context(config_file="detection.yaml")
        child_description = None
        child_node = None

        # GroupAction(scoped=True, forwarding=False) is represented by reset
        # actions around the included child. Executing them exercises the actual
        # launch configuration stack rather than inspecting source text.
        for action in fastlio_group.get_sub_entities():
            if isinstance(action, IncludeLaunchDescription):
                returned = action.execute(context)
                for entity in returned:
                    if isinstance(entity, LaunchDescription):
                        child_description = entity
                    else:
                        entity.execute(context)
                break
            action.execute(context)

        assert context.launch_configurations["config_file"] == ""
        assert context.launch_configurations["fastlio_config_file"] == "mid360.yaml"
        assert context.launch_configurations["sensor_profile"] == "gazebo_mid360"
        assert child_description is not None

        for action in child_description.entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
            elif isinstance(action, OpaqueFunction):
                returned = action.execute(context)
                child_node = next(entity for entity in returned if isinstance(entity, Node))
                break

        assert child_node is not None
        files = parameter_files(child_node, context)
        assert files
        assert files[0].name == "mid360.yaml"
        assert files[1] == PACKAGE_DIR / 'config/gazebo_mid360_extrinsics.yaml'
    finally:
        parent._resolve_px4_dir = original_resolve_px4
        parent.require_native_imu = original_native_imu
        parent.get_package_share_directory = original_parent_lookup
        ament_packages.get_package_share_directory = original_ament_lookup


def test_delayed_map_clock_is_frozen_before_parent_scope_exits() -> None:
    fastlio = load_launch_module(FASTLIO_PATH, "fastlio_launch_delayed_clock")
    for value, expected in (("true", True), ("false", False)):
        context = make_context(use_sim_time=value)
        timer = fastlio._launch_map_odom_tf(context)[0]
        assert timer.period == 5.0
        container = timer.actions[0]
        description = container._ComposableNodeContainer__composable_node_descriptions[0]
        context.launch_configurations.clear()  # Outer scoped include has returned.
        request = get_composable_node_load_request(description, context)
        parameter = next(p for p in request.parameters if p.name == "use_sim_time")
        assert parameter.value.bool_value is expected


def main() -> None:
    test_standalone_legacy_config_file_is_effective()
    test_parent_group_blocks_detection_config_file()
    test_delayed_map_clock_is_frozen_before_parent_scope_exits()
    test_ui_parameters_survive_scoped_parent_and_keep_yaml_and_ros_types()
    test_gazebo_body_offset_survives_parent_scope()
    print("PASS FAST-LIO launch semantic contracts")


def test_ui_parameters_survive_scoped_parent_and_keep_yaml_and_ros_types():
    import json
    from unittest.mock import patch
    from launch_ros.utilities import evaluate_parameters
    parent = load_launch_module(PERCEPTION_PATH, 'perception_slam_ui')
    with patch.object(parent, '_resolve_px4_dir', lambda: Path('/inert-test-only-px4')), \
            patch.object(parent, 'require_native_imu', lambda directory: None):
        description = parent.generate_launch_description()
    timer = next(action for action in guarded_actions(description)
                 if getattr(action, 'period', None) == 14.0)
    group = next(action for action in timer.actions if isinstance(action, GroupAction))
    raw = json.dumps(dict(slam='fast_lio', slam_lidar_topic='/scanner/cloud',
                          slam_imu_topic='/scanner/imu', slam_max_iteration=5,
                          slam_filter_size_map=.25, slam_det_range=75))
    context = make_context(config_file='detection.yaml', fastlio_options_json=raw,
                           slam_enabled='false')
    assert not group.condition.evaluate(context)
    context.launch_configurations['slam_enabled'] = 'true'
    assert group.condition.evaluate(context)
    for action in group.get_sub_entities():
        returned = action.execute(context)
        if isinstance(action, IncludeLaunchDescription):
            for entity in returned:
                if isinstance(entity, LaunchDescription):
                    child = entity
                else:
                    entity.execute(context)
            break
    assert context.launch_configurations['config_file'] == ''
    assert context.launch_configurations['fastlio_options_json'] == raw
    for action in child.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
        elif isinstance(action, OpaqueFunction):
            node = next(item for item in action.execute(context) if isinstance(item, Node))
            break
    params = evaluate_parameters(context, node._Node__parameters)[-1]
    assert params['common.lid_topic'] == '/scanner/cloud'
    assert params['common.imu_topic'] == '/scanner/imu'
    assert params['max_iteration'] == 5 and type(params['max_iteration']) is int
    assert params['filter_size_map'] == .25
    assert params['mapping.det_range'] == 75.0 and type(params['mapping.det_range']) is float
    assert parameter_files(node, context)[0].name == 'mid360.yaml'
    assert parameter_files(node, context)[1].name == 'gazebo_mid360_extrinsics.yaml'


def test_gazebo_body_offset_survives_parent_scope():
    fastlio = load_launch_module(FASTLIO_PATH, 'fastlio_gazebo_body_offset')
    context = make_context(use_sim_time='true', sensor_profile='gazebo_mid360')
    container = fastlio._launch_map_odom_tf(context)[0].actions[0]
    description = container._ComposableNodeContainer__composable_node_descriptions[0]
    context.launch_configurations.clear()
    request = get_composable_node_load_request(description, context)
    offset = next(p for p in request.parameters if p.name == 'body_to_base_translation')
    assert list(offset.value.double_array_value) == [.011, .02329, -.12412]


if __name__ == "__main__":
    main()

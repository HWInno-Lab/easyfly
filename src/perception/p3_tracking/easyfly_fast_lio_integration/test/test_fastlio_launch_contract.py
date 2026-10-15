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
        "fast_lio": WORKSPACE_SRC / "third_party" / "slam" / "fast_lio",
    }

    def source_share(package: str) -> str:
        return str(source_dirs.get(package, WORKSPACE_SRC))

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
    finally:
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
    print("PASS FAST-LIO launch semantic contracts")


if __name__ == "__main__":
    main()

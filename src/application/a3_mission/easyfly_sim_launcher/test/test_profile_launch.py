"""Expand profile launch actions without starting PX4, Gazebo or any node."""

import importlib.util
import inspect
from pathlib import Path

import pytest
import launch.logging
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

from easyfly_params.configuration import load_profile


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[3]
CONFIG = WORKSPACE / "src/application/a1_sdk/easyfly_params/config"


@pytest.fixture(scope="module", autouse=True)
def isolated_launch_logs(tmp_path_factory):
    # LaunchContext creates logs even when no process is started.
    previous = launch.logging.launch_config._log_dir
    launch.logging.launch_config.log_dir = str(tmp_path_factory.mktemp("launch_logs"))
    yield
    launch.logging.launch_config._log_dir = previous


def load_entrypoint():
    spec = importlib.util.spec_from_file_location("profile_launch", PACKAGE / "launch/profile.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.load_profile = lambda name: load_profile(name, CONFIG)
    return module


@pytest.mark.parametrize("name,entry", [
    ("base_sim", "easyfly_sim.launch.py"),
    ("full_perception", "perception_sitl.launch.py"),
    ("navigation_dev", "perception_sitl.launch.py"),
    ("landing_dev", "perception_sitl.launch.py"),
])
def test_profile_expansion(name, entry):
    module = load_entrypoint()
    context = LaunchContext()
    context.launch_configurations.update({"profile": name, "config_file": "polluted.yaml"})
    actions = module._launch_profile(context)
    group = next(action for action in actions if isinstance(action, GroupAction))
    include = next(action for action in group.get_sub_entities() if isinstance(action, IncludeLaunchDescription))
    arguments = dict(include.launch_arguments)
    assert arguments["headless"] == "true"
    platform = [action for action in group.get_sub_entities()
                if isinstance(action, IncludeLaunchDescription)][1]
    assert dict(platform.launch_arguments) == {
        'profile': name, 'use_sim_time': 'true', 'start_mission_executor': 'false'}
    if name != "base_sim":
        assert arguments["local_planner_virtual_ceil_height"] == "3.2"
        assert arguments["navigation_strategy"] == ("combined" if name == "navigation_dev" else "astar")
    # Execute only context push/reset/set actions; never execute the include.
    for action in group.get_sub_entities():
        if action is include:
            assert "config_file" not in context.launch_configurations
            include.launch_description_source.get_launch_description(context)
            assert include.launch_description_source.location.endswith(entry)
            break
        action.execute(context)


def test_hardware_rejected_before_include():
    module = load_entrypoint()
    context = LaunchContext()
    context.launch_configurations["profile"] = "hardware_unverified"
    with pytest.raises(ValueError, match="non-launchable"):
        module._launch_profile(context)


def nested_entities(entities):
    for entity in entities:
        yield entity
        if isinstance(entity, TimerAction):
            yield from nested_entities(entity.actions)
        elif isinstance(entity, GroupAction):
            yield from nested_entities(entity.get_sub_entities())
        elif isinstance(entity, OpaqueFunction):
            # Inspect only our reservation closure. Never execute it: that would
            # acquire a live lock and return real process-starting actions.
            function = entity._OpaqueFunction__function
            if function.__module__ == "easyfly_px4_sitl.session_guard":
                actions = inspect.getclosurevars(function).nonlocals["actions"]
                yield from nested_entities(actions)


def expand_include(include, context):
    for name, value in include.launch_arguments:
        context.launch_configurations[perform_substitutions(
            context, normalize_to_list_of_substitutions(name))] = perform_substitutions(
                context, normalize_to_list_of_substitutions(value))
    description = include.launch_description_source.get_launch_description(context)
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    return description


@pytest.mark.parametrize("name", ["base_sim", "full_perception", "navigation_dev", "landing_dev"])
def test_nested_launch_effective_parameters(name):
    module = load_entrypoint()
    context = LaunchContext()
    context.launch_configurations["profile"] = name
    group = next(action for action in module._launch_profile(context) if isinstance(action, GroupAction))
    root_include = next(action for action in group.get_sub_entities() if isinstance(action, IncludeLaunchDescription))
    root_description = expand_include(root_include, context)
    wanted = "px4_sitl.launch.py" if name == "base_sim" else "local_planner.launch.py"
    for entity in nested_entities(root_description.entities):
        if not isinstance(entity, IncludeLaunchDescription):
            continue
        source = entity.launch_description_source
        # Humble exposes the resolved location only after loading; expansion is read-only.
        location = perform_substitutions(context, source._LaunchDescriptionSource__location)
        if not location.endswith(wanted):
            continue
        description = expand_include(entity, context)
        if name == "base_sim":
            gui = next(action for action in nested_entities(description.entities)
                       if isinstance(action, ExecuteProcess) and
                       perform_substitutions(context, action.cmd[0]) == "gzclient")
            assert not gui.condition.evaluate(context)
        else:
            node = next(action for action in description.entities if isinstance(action, Node)
                        and perform_substitutions(context, normalize_to_list_of_substitutions(
                            action.node_executable)) == "ego_planner_node")
            # Read the normalized arguments passed to the real Node; never execute it.
            params = evaluate_parameters(context, node._Node__parameters)[0]
            assert params["use_sim_time"] is True
            assert params["grid_map/virtual_ceil_height"] == 3.2
            assert params["grid_map/ground_height"] == 0.5
            assert params["grid_map/max_trusted_cloud_points"] == 200000
        return
    pytest.fail(f"missing nested {wanted}")


def test_graphical_connection_endpoints_are_forwarded_into_profile_scope():
    module = load_entrypoint()
    context = LaunchContext()
    context.launch_configurations.update(profile='navigation_dev',
        fcu_url='udp://127.0.0.1:14540@127.0.0.1:14555',
        gcs_url='udp://127.0.0.1:14558@127.0.0.1:14560', target_system='7',
        detection_use_gpu='false', headless='true', rviz='false')
    actions = module._launch_profile(context)
    group = next(action for action in actions if isinstance(action, GroupAction))
    include = next(action for action in group.get_sub_entities() if isinstance(action, IncludeLaunchDescription))
    arguments = dict(include.launch_arguments)
    assert arguments['gcs_url'] == 'udp://127.0.0.1:14558@127.0.0.1:14560'
    assert arguments['target_system'] == '7'
    assert arguments['detection_use_gpu'] == 'false'
    assert arguments['rviz'] == 'false'

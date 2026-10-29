#!/usr/bin/env python3
"""Regression checks for the MAVROS simulation time policy.

The test exercises the launch description without starting MAVROS.  It
evaluates the same ``ParameterFile`` that the launch action passes to
``mavros_node`` and checks both clock modes and the resulting command-line
ordering.  The latter is important because MAVROS' px4_config.yaml is passed
through ``ros_arguments`` while EasyFly's override is a launch parameter file.
"""

import importlib.util
import inspect
import sys
import unittest
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, TimerAction,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.utilities import perform_substitutions


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT.parents[2]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "mavlink_bridge.launch.py"
CONFIG_PATH = PACKAGE_ROOT / "config" / "mavros_use_sim_time.yaml"
MAVROS_SHARE = Path(get_package_share_directory("mavros"))
PX4_CONFIG_PATH = MAVROS_SHARE / "launch" / "px4_config.yaml"


def _source_share_directory(package):
    """Resolve source-tree packages used by the static launch checks."""
    source_packages = {
        "easyfly_mavlink_bridge": PACKAGE_ROOT,
        "easyfly_px4_sitl": SOURCE_ROOT / "simulation" / "easyfly_px4_sitl",
        "mavros": MAVROS_SHARE,
    }
    return str(source_packages.get(package, PACKAGE_ROOT))


def _load_launch_module():
    """Load the source launch file while keeping this test source-local."""
    spec = importlib.util.spec_from_file_location(
        "easyfly_mavlink_bridge_test_launch", LAUNCH_PATH
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load launch file: {LAUNCH_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)

    # Do not require the workspace to have been sourced or the package to have
    # been installed in order to run this static test.
    module.get_package_share_directory = _source_share_directory
    return module


def _load_entrypoint(path, module_name):
    """Load an EasyFly entrypoint against source-tree package shares."""
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load launch file: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    module.get_package_share_directory = _source_share_directory
    return module


def _iter_launch_entities(entities):
    """Inspect construction only; never execute returned simulator actions."""
    for entity in entities:
        yield entity
        if isinstance(entity, TimerAction):
            yield from _iter_launch_entities(entity.actions)
        elif isinstance(entity, GroupAction):
            yield from _iter_launch_entities(entity.get_sub_entities())
        elif isinstance(entity, OpaqueFunction):
            # Inspect the guard's captured action list; never execute preflight,
            # telemetry registration, resource checks, locks or process callbacks.
            function = entity._OpaqueFunction__function
            if function.__module__ == 'easyfly_px4_sitl.session_guard':
                guarded = inspect.getclosurevars(function).nonlocals['actions']
                yield from _iter_launch_entities(guarded)


def _source_location(include):
    """Return an IncludeLaunchDescription source path for this launch API."""
    source = include._IncludeLaunchDescription__launch_description_source
    substitutions = source._LaunchDescriptionSource__location
    return "".join(getattr(item, "text", str(item)) for item in substitutions)


def _assert_sim_time_bridge_include(test_case, launch_path, module_name):
    module = _load_entrypoint(launch_path, module_name)
    launch_description = module.generate_launch_description()
    matches = [
        entity
        for entity in _iter_launch_entities(launch_description.entities)
        if isinstance(entity, IncludeLaunchDescription)
        and _source_location(entity).endswith(
            "/easyfly_mavlink_bridge/launch/mavlink_bridge.launch.py"
        )
    ]
    test_case.assertEqual(
        len(matches),
        1,
        f"expected one mavlink bridge include in {launch_path}",
    )
    arguments = dict(matches[0]._IncludeLaunchDescription__launch_arguments)
    test_case.assertEqual(arguments.get("use_sim_time"), "true")


def _evaluate_mavros_policy(use_sim_time, **overrides):
    module = _load_launch_module()
    launch_description = module.generate_launch_description()
    context = LaunchContext()
    # Supplying this value emulates the command-line launch argument.  The
    # remaining defaults are then derived by the launch description itself.
    context.launch_configurations["use_sim_time"] = use_sim_time
    context.launch_configurations.update(overrides)
    for action in launch_description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)

    mavros_node = next(
        action
        for action in launch_description.entities
        if isinstance(action, Node) and action.node_package == "easyfly_mavros_runtime"
    )
    mavros_node._perform_substitutions(context)

    command = [
        perform_substitutions(
            context, item if isinstance(item, list) else [item]
        )
        for item in mavros_node.cmd
    ]
    parameter_files = [
        command[index + 1]
        for index, item in enumerate(command[:-1])
        if item == "--params-file"
    ]
    override_paths = [
        Path(path)
        for path in parameter_files
        if Path(path).is_file()
        and "timesync_mode:" in Path(path).read_text(encoding="utf-8")
    ]
    if len(override_paths) < 2:
        raise AssertionError(
            "expected both MAVROS px4_config.yaml and EasyFly time policy"
        )
    # px4_config.yaml is the first matching file; EasyFly's generated policy
    # must be the later one that actually wins parameter precedence.
    override_path = override_paths[-1]
    # Read generated files while their Node/ParameterFile owners are still alive.
    records = [yaml.safe_load(Path(path).read_text()) for path in parameter_files]
    return context, command, yaml.safe_load(override_path.read_text()), override_path, records


class MavlinkBridgeSimTimeContractTest(unittest.TestCase):
    def test_simulation_defaults_override_px4_config(self):
        _, command, params, override_path, _ = _evaluate_mavros_policy("true")
        time_params = params["/**/time"]["ros__parameters"]
        global_params = params["/**"]["ros__parameters"]

        self.assertIs(global_params["use_sim_time"], True)
        self.assertEqual(time_params["timesync_mode"], "NONE")
        self.assertEqual(time_params["timesync_rate"], 0.0)
        self.assertEqual(time_params["system_time_rate"], 0.0)

        px4_index = command.index(str(PX4_CONFIG_PATH))
        override_index = command.index(str(override_path))
        self.assertLess(
            px4_index,
            override_index,
            "simulation override must be applied after MAVROS px4_config.yaml",
        )

    def test_real_hardware_defaults_remain_enabled(self):
        _, _, params, _, _ = _evaluate_mavros_policy("false")
        time_params = params["/**/time"]["ros__parameters"]
        global_params = params["/**"]["ros__parameters"]

        self.assertIs(global_params["use_sim_time"], False)
        self.assertEqual(time_params["timesync_mode"], "MAVLINK")
        self.assertEqual(time_params["timesync_rate"], 10.0)
        self.assertEqual(time_params["system_time_rate"], 1.0)

    def test_config_and_sim_launcher_keep_the_contract_explicit(self):
        config_text = CONFIG_PATH.read_text(encoding="utf-8")
        self.assertIn("/**/time:", config_text)
        self.assertIn("timesync_mode: $(var timesync_mode)", config_text)
        self.assertIn("timesync_rate: $(var timesync_rate)", config_text)
        self.assertIn("system_time_rate: $(var system_time_rate)", config_text)

        sim_launch_path = (
            SOURCE_ROOT
            / "application"
            / "a3_mission"
            / "easyfly_sim_launcher"
            / "launch"
            / "easyfly_sim.launch.py"
        )
        perception_launch_path = (
            SOURCE_ROOT
            / "simulation"
            / "easyfly_px4_sitl"
            / "launch"
            / "perception_sitl.launch.py"
        )
        _assert_sim_time_bridge_include(self, sim_launch_path, "easyfly_sim_entrypoint")
        _assert_sim_time_bridge_include(
            self, perception_launch_path, "easyfly_perception_entrypoint"
        )


class GcsForwardingContract(unittest.TestCase):
    """The GUI endpoints reach real MAVROS parameters without changing clock policy."""

    def test_fcu_gcs_and_target_system_parameters(self):
        fcu = 'udp://127.0.0.1:14540@127.0.0.1:14580'
        gcs = 'udp://127.0.0.1:14558@127.0.0.1:14560'
        _, _, _, _, records = _evaluate_mavros_policy(
            'true', fcu_url=fcu, gcs_url=gcs, target_system='7')
        values = []
        for data in records:
            for node in data.values():
                parameters = node.get('ros__parameters', {})
                if 'gcs_url' in parameters:
                    values.append(parameters)
        self.assertEqual(len(values), 1)
        self.assertEqual(values[0]['fcu_url'], fcu)
        self.assertEqual(values[0]['gcs_url'], gcs)
        self.assertEqual(values[0]['tgt_system'], 7)


class BridgeOwnershipContract(unittest.TestCase):
    """Keep normal launch ownership unchanged when SITL owns a separate bridge."""

    def test_default_bridge_enabled_and_explicit_disabled_condition(self):
        for override, expected in ((None, True), ("true", True), ("false", False)):
            description = _load_launch_module().generate_launch_description()
            context = LaunchContext()
            if override is not None:
                context.launch_configurations["start_bridge"] = override
            for entity in description.entities:
                if isinstance(entity, DeclareLaunchArgument):
                    entity.execute(context)
            bridge = [entity for entity in description.entities
                      if isinstance(entity, Node) and
                      entity.node_package == "easyfly_mavlink_bridge"]
            self.assertEqual(len(bridge), 1)
            self.assertEqual(bridge[0].condition.evaluate(context), expected)


if __name__ == "__main__":
    unittest.main()

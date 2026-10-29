# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Actual launch expressions keep observation and flight admission separate."""
import importlib.util
from pathlib import Path
from launch import LaunchContext
import pytest

ROOT = Path(__file__).resolve().parents[5]
SPEC = importlib.util.spec_from_file_location(
    "fastlivo_perception_launch",
    ROOT / "src/simulation/easyfly_px4_sitl/launch/perception_sitl.launch.py",
)
launch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(launch)


@pytest.mark.parametrize(
    "backend,vio,livo,enabled,topic",
    [
        ("fast_lio", "false", "false", False, "/uav_state"),
        ("openvins", "true", "false", True, "/localization/openvins/navigation_state"),
        ("openvins", "false", "true", False, "/uav_state"),
        ("fast_livo2", "false", "true", True, "/localization/fast_livo2/navigation_state"),
        ("fast_livo2", "false", "false", False, "/uav_state"),
        ("disabled", "true", "true", False, "/uav_state"),
    ],
)
def test_explicit_backend_controls_only_its_own_navigation(backend, vio, livo, enabled, topic):
    c = LaunchContext()
    c.launch_configurations.update(
        slam_enabled="true",
        localization_backend=backend,
        openvins_navigation=vio,
        fastlivo_navigation=livo,
    )
    assert launch._vio_navigation().perform(c) == str(enabled)
    assert launch._mapping_state().perform(c) == topic


def test_slam_off_disables_fastlivo_mapping():
    c = LaunchContext()
    c.launch_configurations.update(
        slam_enabled="false",
        localization_backend="fast_livo2",
        fastlivo_navigation="true",
        openvins_navigation="false",
    )
    assert launch._vio_navigation().perform(c) == "False"
    assert launch._mapping_state().perform(c) == "/uav_state"


@pytest.mark.parametrize("enabled", ["false", "true"])
def test_atlas_output_is_explicit_typed_and_keeps_navigation_independent(enabled):
    from launch.actions import DeclareLaunchArgument
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters
    spec = importlib.util.spec_from_file_location(
        "fastlivo_session", Path(__file__).resolve().parents[1] / "launch/fastlivo.launch.py")
    session = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(session)
    description = session.generate_launch_description()
    argument = next(a for a in description.entities
                    if isinstance(a, DeclareLaunchArgument) and a.name == "enable_atlas")
    context = LaunchContext()
    assert "".join(s.perform(context) for s in argument.default_value) == "false"
    context.launch_configurations.update(enable_atlas=enabled, enable_navigation="false")
    native = next(a for a in description.entities
                  if isinstance(a, Node) and a.node_package == "fast_livo")
    values = evaluate_parameters(context, native._Node__parameters)
    assert values[-1]["publish.atlas_keyframes"] is (enabled == "true")
    navigation = next(a for a in description.entities
                      if isinstance(a, Node) and a.node_executable == "openvins_navigation")
    assert not navigation.condition.evaluate(context)

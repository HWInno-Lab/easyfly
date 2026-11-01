# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0.
"""Frozen uploaded FUEL speed limits survive launch defaults."""
import importlib.util
from pathlib import Path

from launch import LaunchContext
import yaml


SPEC = importlib.util.spec_from_file_location(
    "fuel_launch", Path(__file__).resolve().parents[1] / "launch/fuel.launch.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_file_speed_is_not_overridden_by_implicit_sim_defaults(tmp_path, monkeypatch):
    captured = []
    monkeypatch.setattr(MODULE, "Node", lambda **kwargs: captured.append(kwargs))
    parameter_file = tmp_path / "reviewed.yaml"
    parameter_file.write_text("fuel_exploration:\n  ros__parameters:\n    native_max_speed: 0.2\n")
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "parameter_file": str(parameter_file),
            "native_max_speed": "",
            "use_sim_time": "false",
            "extended_profile": "false",
            "evidence_directory": str(tmp_path),
            "uav_state_topic": "",
        }
    )
    MODULE.fuel_node(context)
    parameters = captured[-1]["parameters"]
    effective = yaml.safe_load(parameter_file.read_text())["fuel_exploration"]["ros__parameters"]
    effective.update(parameters[-1])
    assert parameters[0] == str(parameter_file)
    assert effective["native_max_speed"] == 0.2
    assert captured[-1]["remappings"] == []
    context.launch_configurations["native_max_speed"] = "0.15"
    MODULE.fuel_node(context)
    assert captured[-1]["parameters"][-1]["native_max_speed"] == 0.15
    context.launch_configurations.update({"parameter_file": "", "native_max_speed": ""})
    MODULE.fuel_node(context)
    assert captured[-1]["parameters"][-1]["native_max_speed"] == 1.2


def test_evidence_rejects_public_or_symlink_storage(tmp_path):
    import pytest

    specification = importlib.util.spec_from_file_location(
        "fuel_evidence",
        Path(__file__).resolve().parents[1] / "easyfly_fuel_exploration/evidence.py",
    )
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    private = tmp_path / "private"
    assert module.private_evidence_directory(private) == private
    assert private.stat().st_mode & 0o777 == 0o700
    public = tmp_path / "public"
    public.mkdir(mode=0o755)
    with pytest.raises(ValueError):
        module.private_evidence_directory(public)
    link = tmp_path / "link"
    link.symlink_to(private, target_is_directory=True)
    with pytest.raises(ValueError):
        module.private_evidence_directory(link)

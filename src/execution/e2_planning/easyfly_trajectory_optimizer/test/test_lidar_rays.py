# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0.
"""Physical return conversion and explicit hardware calibration contracts."""
import copy
import importlib.machinery
import importlib.util
from pathlib import Path
import struct
import sys
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from easyfly_trajectory_optimizer.hardware import (  # noqa: E402
    validate_hardware_parameters,
    validate_lidar_parameters,
)

loader = importlib.machinery.SourceFileLoader(
    "physical_lidar_rays", str(ROOT / "scripts/lidar_rays_node")
)
specification = importlib.util.spec_from_loader(loader.name, loader)
module = importlib.util.module_from_spec(specification)
loader.exec_module(module)
convert = module.hit_groups


def point(x=2.0, offset=0, tag=0):
    return NS(x=x, y=0.0, z=0.0, offset_time=offset, tag=tag)


def scan(points):
    return NS(
        header=NS(frame_id="sensor", stamp=NS(sec=1, nanosec=0)),
        timebase=1000000000,
        point_num=len(points),
        points=points,
    )


def test_only_real_hits_and_exact_acquisition_times():
    groups = convert(
        scan(
            [point(), point(3.0, 20), point(float("nan"), 30), point(0.0, 40), point(4.0, 50, 4)]
        ),
        "sensor",
        0.2,
        10.0,
    )
    assert [stamp for stamp, _ in groups] == [1000000000, 1000000020]
    assert all(
        len(data) == 24 and struct.unpack_from("<B", data, 12)[0] == 1 for _, data in groups
    )
    assert (
        convert(scan([point(0.0), point(float("inf")), point(2.0, tag=0x20)]), "sensor", 0.2, 10.0)
        == []
    )


@pytest.mark.parametrize(
    "change",
    [
        lambda msg: setattr(msg, "timebase", 1),
        lambda msg: setattr(msg, "point_num", 4),
        lambda msg: setattr(msg.header, "frame_id", "wrong"),
        lambda msg: setattr(msg.points[1], "offset_time", 200000001),
        lambda msg: setattr(msg.points[0], "offset_time", 11),
    ],
)
def test_malformed_acquisition_never_becomes_free(change):
    msg = scan([point(), point(3.0, 10)])
    change(msg)
    with pytest.raises(ValueError):
        convert(msg, "sensor", 0.2, 10.0)
    with pytest.raises(ValueError):
        convert(scan([point()]), "sensor", 0.2, 10.0, 1000000000)


def calibrated():
    return {
        "gcopter_voxel_map": {
            "ros__parameters": {
                "map_min": [-4.0, -4.0, -0.2],
                "map_max": [8.0, 4.0, 3.4],
                "resolution": 0.2,
                "inflation_radius": 0.65,
                "source_frame": "sensor",
            }
        },
        "gcopter_navigation": {"ros__parameters": {"z_min": 0.5, "z_max": 2.5}},
    }


def test_hardware_profile_has_no_implicit_calibration_or_simulation_escape():
    validate_hardware_parameters(calibrated())
    for name in calibrated():
        for key in calibrated()[name]["ros__parameters"]:
            data = calibrated()
            del data[name]["ros__parameters"][key]
            with pytest.raises(ValueError):
                validate_hardware_parameters(data)
    for name, key, value in [
        ("gcopter_voxel_map", "use_sim_time", True),
        ("gcopter_voxel_map", "physical_livox", False),
        ("gcopter_voxel_map", "dynamic_clearing", True),
        ("gcopter_navigation", "fuel_native_trajectory", True),
        ("gcopter_navigation", "z_max", float("nan")),
    ]:
        data = calibrated()
        data[name]["ros__parameters"][key] = value
        with pytest.raises(ValueError):
            validate_hardware_parameters(data)
    lidar = {
        "input_topic": "/livox/lidar",
        "source_frame": "sensor",
        "range_min": 0.2,
        "range_max": 30.0,
    }
    validate_lidar_parameters(lidar)
    for key in lidar:
        data = copy.deepcopy(lidar)
        del data[key]
        with pytest.raises(ValueError):
            validate_lidar_parameters(data)


def test_hardware_launch_only_constructs_idle_wallclock_nodes(tmp_path, monkeypatch):
    from launch import LaunchContext
    import yaml

    specification = importlib.util.spec_from_file_location(
        "hardware_gcopter_launch", ROOT / "launch/hardware_gcopter.launch.py"
    )
    launch_module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(launch_module)
    captured = []
    monkeypatch.setattr(launch_module, "Node", lambda **kwargs: captured.append(kwargs))
    parameters = tmp_path / "hardware.yaml"
    parameters.write_text(yaml.safe_dump(calibrated()))
    evidence = tmp_path / "evidence"
    evidence.mkdir(mode=0o700)
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "parameter_file": str(parameters),
            "evidence_directory": str(evidence),
            "uav_state_topic": "/uav_state",
        }
    )
    launch_module.hardware_nodes(context)
    assert [node["executable"] for node in captured] == ["gcopter_voxel_map", "gcopter_navigation"]
    assert all(node["parameters"][-1]["use_sim_time"] is False for node in captured)
    assert captured[0]["parameters"][-1]["physical_livox"] is True
    assert captured[1]["parameters"][-1]["fuel_native_trajectory"] is False
    assert captured[1]["parameters"][-1]["allow_idle_policy_map_epoch"] is True
    assert captured[0]["remappings"] == [("/planning/voxel_grid", "/planning/fuel_observed_grid")]
    evidence.chmod(0o755)
    with pytest.raises(ValueError, match="owner-private"):
        launch_module.hardware_nodes(context)

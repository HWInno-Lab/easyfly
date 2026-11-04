# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_trajectory_optimizer/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
"""Pure hardware calibration validation; no ROS graph or launch side effects."""
import math


def validate_hardware_parameters(document):
    if not isinstance(document, dict):
        raise ValueError("hardware GCOPTER requires an explicit ROS parameter file")
    required = {
        "gcopter_voxel_map": (
            "map_min",
            "map_max",
            "resolution",
            "inflation_radius",
            "source_frame",
        ),
        "gcopter_navigation": ("z_min", "z_max"),
    }
    for name, keys in required.items():
        node = document.get(name)
        values = node.get("ros__parameters", {}) if isinstance(node, dict) else {}
        if not isinstance(values, dict) or any(key not in values for key in keys):
            raise ValueError(f"{name}: explicit hardware calibration required: {keys}")
        if values.get("use_sim_time", False) is not False:
            raise ValueError("hardware GCOPTER requires wall clock")
    mapper = document["gcopter_voxel_map"]["ros__parameters"]
    planner = document["gcopter_navigation"]["ros__parameters"]
    for key in ("map_min", "map_max"):
        if (
            not isinstance(mapper[key], list)
            or len(mapper[key]) != 3
            or not all(type(v) in (float, int) and math.isfinite(v) for v in mapper[key])
        ):
            raise ValueError("hardware map bounds must be finite calibrated xyz vectors")
    for key in ("resolution", "inflation_radius"):
        if not type(mapper[key]) in (float, int) or not math.isfinite(mapper[key]):
            raise ValueError("hardware map resolution/inflation must be calibrated")
    if (
        not 0.05 <= mapper["resolution"] <= 1.0
        or not 0.35 <= mapper["inflation_radius"] <= 1.0
        or not isinstance(mapper["source_frame"], str)
        or not mapper["source_frame"]
        or mapper["source_frame"].startswith("/")
    ):
        raise ValueError("invalid calibrated resolution/inflation/source frame")
    cells = 1
    for low, high in zip(mapper["map_min"], mapper["map_max"]):
        extent = high - low
        count = math.ceil(extent / mapper["resolution"])
        if abs(low) > 10000 or abs(high) > 10000 or not 0 < extent <= 25.6 or not 0 < count <= 512:
            raise ValueError("hardware map exceeds bounded native volume")
        cells *= count
    if cells > 1000000:
        raise ValueError("hardware map cell budget")
    for key in ("z_min", "z_max"):
        if not type(planner[key]) in (float, int) or not math.isfinite(planner[key]):
            raise ValueError("calibrated flight z limits must be finite")
    if (
        planner["z_min"] < 0.3
        or planner["z_max"] > 3.5
        or planner["z_max"] - planner["z_min"] < 0.5
        or planner["z_min"] < mapper["map_min"][2]
        or planner["z_max"] > mapper["map_max"][2]
    ):
        raise ValueError("flight z envelope must fit calibrated map/native bounds")
    fixed = {
        "gcopter_voxel_map": {
            "physical_livox": True,
            "dynamic_clearing": False,
            "rolling_xy": False,
            "rays_topic": "/livox/lidar/rays",
            "uninflated_topic": "/planning/fuel_raw_grid",
        },
        "gcopter_navigation": {
            "allow_idle_policy_map_epoch": True,
            "dynamic_map": False,
            "fuel_native_trajectory": False,
        },
    }
    for name, parameters in fixed.items():
        actual = document[name]["ros__parameters"]
        for key, value in parameters.items():
            if key in actual and actual[key] != value:
                raise ValueError(f"hardware {name}.{key} must be {value}")
    return mapper, planner


def validate_lidar_parameters(values):
    if not isinstance(values, dict):
        raise ValueError("explicit Livox parameters required")
    topic, frame = values.get("input_topic"), values.get("source_frame")
    low, high = values.get("range_min"), values.get("range_max")
    if (
        not isinstance(topic, str)
        or not topic.startswith("/")
        or not isinstance(frame, str)
        or not frame
        or frame.startswith("/")
        or not type(low) in (float, int)
        or not type(high) in (float, int)
        or not math.isfinite(low)
        or not math.isfinite(high)
        or not 0.1 <= low < high <= 100.0
        or values.get("use_sim_time", False) is not False
        or values.get("rays_topic", "/livox/lidar/rays") != "/livox/lidar/rays"
        or topic == "/livox/lidar/rays"
    ):
        raise ValueError("explicit wall-clock Livox topic/frame/range calibration required")
    return values

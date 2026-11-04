# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_params/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
#

"""Resolve profiles and compare frozen contracts with source defaults offline."""

import argparse
import ast
import json
import math
from pathlib import Path
import re

import yaml


PROFILES = (
    "base_sim",
    "full_perception",
    "navigation_dev",
    "landing_dev",
    "hardware_unverified",
)
CAPABILITIES = {
    "mavlink_state",
    "perception",
    "navigation",
    "landing_detection",
    "precision_landing",
}
CONTRACT_DEFAULTS = {
    "schema_version": 1,
    "frame": "odom",
    "max_nonground_points": 200000,
    "planning_floor": 0.5,
    "planning_ceiling": 3.2,
    "trajectory_startup_grace": 8.0,
    "trajectory_stream_timeout": 0.5,
}


def read_yaml(path):
    with Path(path).open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{path}: expected a mapping")
    return value


def config_directory():
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("easyfly_params")) / "config"


def load_profile(name, directory=None):
    """Return the effective managed launch inputs without starting processes."""
    if name not in PROFILES:
        raise ValueError(
            f"unknown profile {name!r}; choose from {', '.join(PROFILES)}"
        )
    directory = (
        Path(directory) if directory is not None else config_directory()
    )
    profile = read_yaml(directory / "profiles" / f"{name}.yaml")
    required = {
        "schema_version",
        "name",
        "environment",
        "readiness",
        "entrypoint",
        "headless",
        "capabilities",
    }
    if not required <= profile.keys() or profile.keys() - required - {"note"}:
        raise ValueError("profile has missing or unknown fields")
    if (
        type(profile["schema_version"]) is not int
        or profile["schema_version"] != 1
    ):
        raise ValueError("unsupported profile schema_version")
    if profile["name"] != name or type(profile["headless"]) is not bool:
        raise ValueError("profile name/headless is invalid")
    capabilities = profile["capabilities"]
    if not isinstance(capabilities, dict) or set(capabilities) != CAPABILITIES:
        raise ValueError("profile capabilities are invalid")
    if any(type(value) is not bool for value in capabilities.values()):
        raise ValueError("capability values must be booleans")
    if capabilities["landing_detection"] or capabilities["precision_landing"]:
        raise ValueError(
            "profile cannot enable unconfigured landing or "
            "precision landing capabilities; use the explicit D5 configuration"
        )
    if name == "hardware_unverified":
        expected = ("hardware", "unverified", "disabled")
        if any(capabilities.values()):
            raise ValueError(
                "unverified hardware cannot advertise active capabilities"
            )
    else:
        expected = (
            "simulation",
            "preparation_only" if name == "landing_dev" else "implemented",
            "base" if name == "base_sim" else "perception",
        )
        expected_capabilities = {
            key: key in {"mavlink_state", "perception", "navigation"}
            and (name != "base_sim" or key == "mavlink_state")
            for key in CAPABILITIES
        }
        if capabilities != expected_capabilities:
            raise ValueError(
                "capabilities do not match the selected entrypoint"
            )
    if (
        tuple(
            profile[key] for key in ("environment", "readiness", "entrypoint")
        )
        != expected
    ):
        raise ValueError(
            "environment/readiness/entrypoint does not match this profile"
        )
    contracts = read_yaml(directory / "safety_contracts.yaml")
    if set(contracts) != set(CONTRACT_DEFAULTS):
        raise ValueError("safety contract has missing or unknown fields")
    for key, expected_value in CONTRACT_DEFAULTS.items():
        value = contracts[key]
        if type(expected_value) is int and type(value) is not int:
            raise ValueError(f"safety contract {key} must be an integer")
        if isinstance(expected_value, (int, float)):
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f"non-finite/invalid safety contract {key}")
        if value != expected_value:
            raise ValueError(
                f"safety contract {key}: expected {expected_value}, got {value}"
            )
    effective = dict(profile)
    effective["contracts"] = contracts
    effective["launch_arguments"] = {
        "headless": str(profile["headless"]).lower()
    }
    if profile["entrypoint"] == "perception":
        effective["launch_arguments"][
            "local_planner_virtual_ceil_height"
        ] = str(contracts["planning_ceiling"])
        effective["launch_arguments"]["navigation_strategy"] = (
            "combined" if profile["name"] == "navigation_dev" else "astar"
        )
    return effective


def ensure_launchable(profile):
    if (
        profile["environment"] != "simulation"
        or profile["entrypoint"] == "disabled"
    ):
        raise ValueError(
            "hardware_unverified is a non-launchable placeholder; G10/G11 paused"
        )


def _planner_defaults(path):
    tree = ast.parse(Path(path).read_text(encoding="utf-8"))
    defaults = {}
    ceiling = None
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == "ego_params"
            for target in node.targets
        ):
            if not isinstance(node.value, ast.Dict):
                raise ValueError(
                    "ego_params must remain an auditable dictionary"
                )
            for key, value in zip(node.value.keys, node.value.values):
                if isinstance(key, ast.Constant) and isinstance(
                    value, ast.Constant
                ):
                    if key.value in defaults:
                        raise ValueError(
                            f"duplicate planner parameter: {key.value}"
                        )
                    defaults[key.value] = value.value
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
            if node.func.id == "DeclareLaunchArgument" and node.args:
                if (
                    isinstance(node.args[0], ast.Constant)
                    and node.args[0].value == "virtual_ceil_height"
                ):
                    for keyword in node.keywords:
                        if keyword.arg == "default_value":
                            ceiling = float(ast.literal_eval(keyword.value))
    defaults["grid_map/virtual_ceil_height"] = ceiling
    return defaults


def validate_workspace(workspace, directory=None):
    """Validate source defaults, not arbitrary overrides or live node parameters."""
    root = Path(workspace).resolve() / "src"
    contracts = load_profile("navigation_dev", directory)["contracts"]
    semantic = read_yaml(
        root
        / "perception/p4_fusion/easyfly_semantic_map/config/semantic_map.yaml"
    )["semantic_map"]
    filter_config = read_yaml(
        root
        / "perception/p1_preprocess/easyfly_pointcloud_filter/config/pointcloud_filter.yaml"
    )["pointcloud_filter"]
    pixel = read_yaml(
        root
        / "perception/p4_fusion/easyfly_pixel_to_3d/config/pixel_to_3d.yaml"
    )["pixel_to_3d"]
    flight_root = root / "execution/e3_control/easyfly_flight_control"
    flight = read_yaml(flight_root / "config/flight_control.yaml")[
        "flight_controller"
    ]["ros__parameters"]
    planner = _planner_defaults(
        root
        / "execution/e2_planning/easyfly_local_planner/launch/local_planner.launch.py"
    )
    header = (
        flight_root
        / "include/easyfly_flight_control/trajectory_startup_timeout.hpp"
    ).read_text()
    match = re.search(r"kTrajectoryStreamTimeout\s*=\s*([0-9.]+)\s*;", header)
    checks = {
        "semantic output frame": (semantic["world_frame"], contracts["frame"]),
        "filter output frame": (
            filter_config["output_frame"],
            contracts["frame"],
        ),
        "target output frame": (pixel["target_frame"], contracts["frame"]),
        "EGO frame": (planner["grid_map/frame_id"], contracts["frame"]),
        "semantic cloud budget": (
            semantic["max_nonground_points"],
            contracts["max_nonground_points"],
        ),
        "EGO cloud budget": (
            planner["grid_map/max_trusted_cloud_points"],
            contracts["max_nonground_points"],
        ),
        "EGO floor": (
            planner["grid_map/ground_height"],
            contracts["planning_floor"],
        ),
        "EGO ceiling": (
            planner["grid_map/virtual_ceil_height"],
            contracts["planning_ceiling"],
        ),
        "trajectory startup grace": (
            flight["trajectory_startup_grace"],
            contracts["trajectory_startup_grace"],
        ),
        "trajectory stream watchdog": (
            float(match.group(1)) if match else None,
            contracts["trajectory_stream_timeout"],
        ),
    }
    errors = [
        f"{name}: expected {expected}, got {actual}"
        for name, (actual, expected) in checks.items()
        if type(actual) is bool or actual != expected
    ]
    if errors:
        raise ValueError("safety contract mismatch: " + "; ".join(errors))
    return list(checks)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile", choices=PROFILES, default="navigation_dev"
    )
    parser.add_argument("--config-dir", type=Path)
    parser.add_argument(
        "--workspace",
        type=Path,
        help="compare source defaults in this workspace",
    )
    args = parser.parse_args(argv)
    try:
        result = load_profile(args.profile, args.config_dir)
        if args.workspace:
            result["source_contract_checks"] = validate_workspace(
                args.workspace, args.config_dir
            )
        print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    except (
        ValueError,
        OSError,
        KeyError,
        SyntaxError,
        yaml.YAMLError,
    ) as error:
        parser.exit(2, f"configuration rejected: {error}\n")


if __name__ == "__main__":
    main()

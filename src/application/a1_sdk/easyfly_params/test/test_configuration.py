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

"""Behavior tests for invalid profiles and cross-package contract drift."""

from copy import deepcopy
from pathlib import Path
import shutil

import pytest
import yaml

from easyfly_params.configuration import (
    ensure_launchable,
    load_profile,
    validate_workspace,
)


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[3]
CONFIG = PACKAGE / "config"
SOURCE_FILES = (
    "perception/p4_fusion/easyfly_semantic_map/config/semantic_map.yaml",
    "perception/p1_preprocess/easyfly_pointcloud_filter/config/pointcloud_filter.yaml",
    "perception/p4_fusion/easyfly_pixel_to_3d/config/pixel_to_3d.yaml",
    "execution/e3_control/easyfly_flight_control/config/flight_control.yaml",
    "execution/e3_control/easyfly_flight_control/include/"
    "easyfly_flight_control/trajectory_startup_timeout.hpp",
    "execution/e2_planning/easyfly_local_planner/launch/local_planner.launch.py",
)


@pytest.mark.parametrize(
    "name", ["base_sim", "full_perception", "navigation_dev", "landing_dev"]
)
def test_simulation_profiles(name):
    profile = load_profile(name, CONFIG)
    ensure_launchable(profile)
    assert profile["launch_arguments"]["headless"] == "true"
    assert not profile["capabilities"]["precision_landing"]
    if name == "navigation_dev":
        assert profile["launch_arguments"]["navigation_strategy"] == "combined"
    if name == "landing_dev":
        assert profile["readiness"] == "preparation_only"


def test_hardware_can_be_inspected_but_never_launched():
    profile = load_profile("hardware_unverified", CONFIG)
    with pytest.raises(ValueError, match="non-launchable"):
        ensure_launchable(profile)


@pytest.mark.parametrize("name", ["../base_sim", "unknown", ""])
def test_unknown_profiles_rejected(name):
    with pytest.raises(ValueError, match="unknown profile"):
        load_profile(name, CONFIG)


@pytest.mark.parametrize(
    "field,value",
    [
        ("headless", "true"),
        ("schema_version", True),
        ("entrypoint", "disabled"),
        ("environment", "hardware"),
        ("readiness", "unverified"),
        ("extra", 1),
    ],
)
def test_invalid_profile_fields(tmp_path, field, value):
    shutil.copytree(CONFIG, tmp_path / "config")
    path = tmp_path / "config/profiles/base_sim.yaml"
    profile = deepcopy(yaml.safe_load(path.read_text()))
    profile[field] = value
    path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError):
        load_profile("base_sim", tmp_path / "config")


def test_unconfigured_landing_capability_rejected(tmp_path):
    shutil.copytree(CONFIG, tmp_path / "config")
    path = tmp_path / "config/profiles/landing_dev.yaml"
    profile = yaml.safe_load(path.read_text())
    profile["capabilities"]["precision_landing"] = True
    path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match="unconfigured"):
        load_profile("landing_dev", tmp_path / "config")


def test_source_defaults_match_frozen_contracts():
    assert len(validate_workspace(WORKSPACE, CONFIG)) == 10


@pytest.mark.parametrize(
    "index,old,new",
    [
        (0, "max_nonground_points: 200000", "max_nonground_points: 400000"),
        (1, "output_frame: odom", "output_frame: map"),
        (2, "target_frame: odom", "target_frame: map"),
        (3, "trajectory_startup_grace: 8.0", "trajectory_startup_grace: 12.0"),
        (
            4,
            "kTrajectoryStreamTimeout = 0.5",
            "kTrajectoryStreamTimeout = 2.0",
        ),
        (5, "'grid_map/ground_height': 0.5", "'grid_map/ground_height': 0.0"),
        (5, "default_value='3.2'", "default_value='5.0'"),
    ],
)
def test_cross_package_drift_rejected(tmp_path, index, old, new):
    for filename in SOURCE_FILES:
        target = tmp_path / "src" / filename
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(WORKSPACE / "src" / filename, target)
    path = tmp_path / "src" / SOURCE_FILES[index]
    contents = path.read_text()
    assert old in contents
    path.write_text(contents.replace(old, new))
    with pytest.raises(ValueError, match="contract mismatch"):
        validate_workspace(tmp_path, CONFIG)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), True, -1])
def test_invalid_safety_values_rejected(tmp_path, value):
    shutil.copytree(CONFIG, tmp_path / "config")
    path = tmp_path / "config/safety_contracts.yaml"
    contracts = yaml.safe_load(path.read_text())
    contracts["trajectory_stream_timeout"] = value
    path.write_text(yaml.safe_dump(contracts))
    with pytest.raises(ValueError):
        load_profile("base_sim", tmp_path / "config")


@pytest.mark.parametrize(
    "field,value",
    [
        ("schema_version", 1.0),
        ("max_nonground_points", 200000.0),
    ],
)
def test_integer_contract_fields_reject_equal_floats(tmp_path, field, value):
    shutil.copytree(CONFIG, tmp_path / "config")
    path = tmp_path / "config/safety_contracts.yaml"
    contracts = yaml.safe_load(path.read_text())
    contracts[field] = value
    path.write_text(yaml.safe_dump(contracts))
    with pytest.raises(ValueError, match="must be an integer"):
        load_profile("base_sim", tmp_path / "config")


def test_algorithm_schema_rejects_invalid_ranges_and_preserves_ros_parameter_types():
    from easyfly_params.algorithms import validate, detector_parameters
    options = validate(dict(ellipse_enabled=True, hue_min=170, hue_max=10))
    assert options['ellipse_enabled'] is True
    assert type(detector_parameters(options, 'ellipse')['min_axis_px']) is float
    assert type(detector_parameters(options, 'color_line')['hue_min']) is int
    for change in (dict(morphology_kernel=2), dict(min_axis_px=3),
                   dict(max_radial_error=.3), dict(canny_low=200, canny_high=100),
                   dict(opencv_threads=True), dict(provider='unknown'), dict(shell='bad')):
        with pytest.raises(ValueError):
            validate(change)


def test_slam_selection_inputs_and_typed_parameters_are_bounded():
    from easyfly_params.algorithms import validate, slam_enabled, slam_parameters
    options = validate(dict(slam='fast_lio', slam_lidar_topic='/scanner/lidar',
                            slam_imu_topic='/scanner/imu', slam_point_filter_num=4,
                            slam_filter_size_map=.25))
    params = slam_parameters(options)
    assert params['common.lid_topic'] == '/scanner/lidar'
    assert params['common.imu_topic'] == '/scanner/imu'
    assert type(params['point_filter_num']) is int
    assert type(params['mapping.det_range']) is float
    assert params['filter_size_map'] == .25
    assert slam_enabled(options, 'navigation_dev')
    with pytest.raises(ValueError, match='FAST-LIO'):
        slam_enabled(options, 'base_sim')
    assert not slam_enabled(validate({}), 'base_sim')
    assert slam_enabled(validate({}), 'full_perception')
    assert not slam_enabled(validate(dict(slam='disabled')), 'navigation_dev')
    for change in (dict(slam='orb_slam'), dict(slam_lidar_topic='relative/topic'),
                   dict(slam_imu_topic='/imu;command'), dict(slam_lidar_topic=''),
                   dict(slam_max_iteration=1.5), dict(slam_point_filter_num=True),
                   dict(slam_filter_size_map=0), dict(slam_blind=5, slam_det_range=5)):
        with pytest.raises(ValueError):
            validate(change)


def test_openvins_selection_is_fixed_profile_and_not_a_fastlio_alias():
    from easyfly_params.algorithms import validate, localization_backend
    assert localization_backend(validate(dict(slam='openvins')), 'full_perception') == 'openvins'
    assert localization_backend(validate({}), 'navigation_dev') == 'fast_lio'
    with pytest.raises(ValueError, match='OpenVINS'):
        localization_backend(validate(dict(slam='openvins')), 'base_sim')
    for change in (dict(slam_imu_topic='/other/imu'), dict(slam_filter_size_map=.25)):
        with pytest.raises(ValueError, match='FAST-LIO 参数'):
            localization_backend(validate(dict(slam='openvins', **change)), 'navigation_dev')

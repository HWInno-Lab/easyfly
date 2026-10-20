# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0.
# You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
# Distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

"""Pin limited upstream changes and the explicit non-flight probe verdict."""

import hashlib
import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
CAMERA = Path(__file__).resolve().parents[1]
NOTE = ('// EasyFly modification (2026-10-07): distinct plugin/class name and reject a null\n'
        '// ROS node before QoS access. Upstream gazebo_ros_pkgs tag 3.9.0, Apache-2.0.\n')
GUARD = ('  if (!impl_->ros_node_) {\n'
         '    RCLCPP_ERROR(\n'
         '      rclcpp::get_logger("easyfly_gazebo_camera"),\n'
         '      "Camera plugin rejected: ROS node creation failed '
         '(duplicate name or namespace)");\n'
         '    return;\n'
         '  }\n')
spec = importlib.util.spec_from_file_location(
    'shutdown_probe', ROOT / 'scripts/gazebo_shutdown_probe.py')
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


def test_source_is_only_named_upstream_plus_guard_and_qos_log():
    source = (CAMERA / 'src/safe_camera.cpp').read_text()
    assert source.startswith(NOTE)
    assert source.count(GUARD) == 1
    restored = source[len(NOTE):].replace(GUARD, '').replace(
        'EasyFlyGazeboRosCamera', 'GazeboRosCamera').replace(
        '#include "easyfly_gazebo_plugins/safe_camera.hpp"',
        '#include <gazebo_plugins/gazebo_ros_camera.hpp>')
    restored = restored.replace(
        '"Publishing camera info to [%s]; history depth=%zu",\n'
        '      impl_->camera_info_pub_.back()->get_topic_name(),\n'
        '      impl_->camera_info_pub_.back()->get_actual_qos().depth()',
        '"Publishing camera info to [%s]",\n'
        '      impl_->camera_info_pub_.back()->get_topic_name()')
    assert hashlib.sha256(restored.encode()).hexdigest() == (
        '0385ceec25a7b55f57aea38c060267bc593c4959a0f097ae4cb89561cf34a416')


def test_header_preserves_upstream_contract():
    header = (CAMERA / 'include/easyfly_gazebo_plugins/safe_camera.hpp').read_text()
    assert header.startswith(NOTE)
    restored = header[len(NOTE):].replace('EasyFlyGazeboRosCamera', 'GazeboRosCamera').replace(
        'EASYFLY_GAZEBO_PLUGINS__SAFE_CAMERA_HPP_',
        'GAZEBO_PLUGINS__GAZEBO_ROS_CAMERA_HPP_').replace(
        'libeasyfly_gazebo_ros_camera.so', 'libgazebo_ros_camera.so')
    assert hashlib.sha256(restored.encode()).hexdigest() == (
        '9d2f1b9808a21648bc852e5358e99ff3d96c9d47fe9544b117f1b675190486ec')


def test_duplicate_fixture_does_not_hide_the_name_conflict():
    iris = ROOT / 'src/simulation/easyfly_px4_sitl/models/easyfly_iris/easyfly_iris.sdf'
    lidar = ROOT / 'src/simulation/easyfly_gazebo_models/mid360/mid360.sdf'
    tree = probe.fixture(iris, lidar, Path('/chosen-camera.so'), True)
    cameras = tree.findall('.//sensor')
    assert len(cameras) == 2
    assert cameras[0].get('name') != cameras[1].get('name')
    assert cameras[0].find('plugin').attrib == cameras[1].find('plugin').attrib
    assert (cameras[0].findtext('plugin/ros/namespace') ==
            cameras[1].findtext('plugin/ros/namespace'))
    assert [c.findtext('update_rate') for c in cameras] == ['30', '30']
    assert len(tree.findall('.//plugin')) == 2  # no PX4, motor, or flight control


@pytest.mark.parametrize('changes,log,expected', [
    ({}, 'Camera plugin rejected:', True),
    ({'duplicate_reported': False}, 'Camera plugin rejected:', False),
    ({}, '', False),
    ({'forced': True}, 'Camera plugin rejected:', False),
    ({'returncode': -2}, 'Camera plugin rejected:', False),
    ({'ready': False}, 'Camera plugin rejected:', False),
])
def test_safe_duplicate_requires_rejection_and_true_clean_exit(changes, log, expected):
    result = dict(forced=False, error=None, expected_crash=False, crash_captured=False,
                  duplicate_reported=True, duplicate=True, ready=True, returncode=0)
    result.update(changes)
    assert probe.probe_passed(result, log) is expected

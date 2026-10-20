# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Inspect launch semantics and cross-file calibration/topic consistency."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch_ros.actions import Node
import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def test_installed_profile_uses_guarded_inputs_and_coupled_lifetime(monkeypatch):
    spec = importlib.util.spec_from_file_location(
        'openvins_launch', PACKAGE / 'launch/openvins.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(PACKAGE))
    context = LaunchContext()
    context.launch_configurations.update(sensor_profile='gazebo_mono',
                                         config_file='unrelated.yaml', use_sim_time='false')
    actions = module.nodes(context)
    nodes = [action for action in actions if isinstance(action, Node)]
    assert len(nodes) == 2 and len(actions) == 4
    assert nodes[0].node_package == 'easyfly_openvins_integration'
    assert nodes[1].node_package == 'ov_msckf'
    from launch_ros.utilities import evaluate_parameters
    params = evaluate_parameters(context, nodes[1]._Node__parameters)
    assert params[0]['use_sim_time'] is True
    assert params[0]['publish_global_to_imu_tf'] is False
    assert params[0]['publish_calibration_tf'] is False
    base = PACKAGE / 'config/gazebo_mono'
    config = yaml.safe_load((base / 'estimator_config.yaml').read_text().split('\n', 1)[1])
    assert config['max_cameras'] == 1 and config['use_stereo'] is False
    assert config['multi_threading_subs'] is False and config['multi_threading_pubs'] is False
    cam = yaml.safe_load((base / 'kalibr_imucam_chain.yaml').read_text().split('\n', 1)[1])
    imu = yaml.safe_load((base / 'kalibr_imu_chain.yaml').read_text().split('\n', 1)[1])
    assert cam['cam0']['rostopic'] == '/openvins/input/camera'
    assert imu['imu0']['rostopic'] == '/openvins/input/imu'
    guard = yaml.safe_load((PACKAGE / 'config/guard.yaml').read_text())['/**']['ros__parameters']
    assert cam['cam0']['intrinsics'] == guard['intrinsics']
    assert cam['cam0']['resolution'] == [guard['width'], guard['height']]
    context.launch_configurations['sensor_profile'] = 'd435i'
    with pytest.raises(ValueError, match='only the calibrated'):
        module.nodes(context)


def test_coupled_exit_emits_shutdown_once_and_respects_parent_shutdown():
    spec = importlib.util.spec_from_file_location(
        'openvins_exit', PACKAGE / 'launch/openvins.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    exited = module.session_exit_handler()
    assert len(exited(None, context)) == 1
    assert exited(None, context) == []
    context._set_is_shutdown(True)
    assert module.session_exit_handler()(None, context) == []


def test_startup_launches_one_session_only_after_success(monkeypatch):
    from types import SimpleNamespace
    spec = importlib.util.spec_from_file_location(
        'startup_launch', PACKAGE / 'launch/openvins.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(PACKAGE))
    context = LaunchContext()
    context.launch_configurations.update(sensor_profile='gazebo_mono', wait_for_inputs='true')
    actions = module.nodes(context)
    nodes = [action for action in actions if isinstance(action, Node)]
    assert len(nodes) == 1 and nodes[0].node_executable == 'openvins_startup'
    children = module.session_nodes(context)
    exited = module.startup_exit_handler(children)
    # Included launch arguments are no longer present when this event runs.
    context.launch_configurations.clear()
    assert exited(SimpleNamespace(returncode=0), context) is children
    assert exited(SimpleNamespace(returncode=0), context) == []
    for code in [2, 130]:
        result = module.startup_exit_handler(children)(SimpleNamespace(returncode=code), context)
        assert len(result) == 1 and not isinstance(result[0], Node)
    context._set_is_shutdown(True)
    assert module.startup_exit_handler(children)(SimpleNamespace(returncode=0), context) == []


def test_managed_alignment_is_in_same_lifetime_and_matches_calibrated_imu_origin(monkeypatch):
    import xml.etree.ElementTree as ET
    import numpy as np
    spec = importlib.util.spec_from_file_location(
        'openvins_alignment_launch', PACKAGE / 'launch/openvins.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(PACKAGE))
    context = LaunchContext()
    context.launch_configurations.update(sensor_profile='gazebo_mono', start_alignment='true')
    actions = module.nodes(context)
    nodes = [action for action in actions if isinstance(action, Node)]
    assert len(nodes) == 3 and len(actions) == 6
    assert nodes[-1].node_executable == 'openvins_alignment'
    parameters = yaml.safe_load((PACKAGE / 'config/alignment_gazebo.yaml').read_text())
    translation = parameters['/**']['ros__parameters']['body_to_base_translation']
    urdf = PACKAGE.parents[2] / 'simulation/easyfly_px4_sitl/models/easyfly_iris/easyfly_iris.urdf'
    joints = {j.find('child').get('link'): j for j in ET.parse(urdf).findall('joint')}
    child = 'livox_imu_frame'
    xyz = np.zeros(3)
    while child != 'base_link':
        joint = joints[child]
        assert [float(v) for v in joint.find('origin').get('rpy').split()] == [0., 0., 0.]
        xyz += [float(x) for x in joint.find('origin').get('xyz').split()]
        child = joint.find('parent').get('link')
    np.testing.assert_allclose(xyz+translation, np.zeros(3), atol=1e-12)

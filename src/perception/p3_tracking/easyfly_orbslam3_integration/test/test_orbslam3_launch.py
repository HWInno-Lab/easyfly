# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Actual launch expressions: each explicit flag can select only its own source."""
import importlib.util
from pathlib import Path
import pytest
from launch import LaunchContext

ROOT=Path(__file__).resolve().parents[5]
spec=importlib.util.spec_from_file_location('orb_perception_launch',ROOT/'src/simulation/easyfly_px4_sitl/launch/perception_sitl.launch.py')
launch=importlib.util.module_from_spec(spec);spec.loader.exec_module(launch)


@pytest.mark.parametrize('backend', ['openvins','fast_livo2','orbslam3','fast_lio','disabled'])
@pytest.mark.parametrize('flag', ['openvins_navigation','fastlivo_navigation','orbslam3_navigation'])
def test_flags_do_not_enable_another_selected_backend(backend,flag):
    c=LaunchContext();c.launch_configurations.update(slam_enabled='true',localization_backend=backend,**{flag:'true'})
    match={'openvins':'openvins_navigation','fast_livo2':'fastlivo_navigation','orbslam3':'orbslam3_navigation'}
    expected=match.get(backend)==flag
    assert launch._vio_navigation().perform(c)==str(expected)
    assert launch._mapping_state().perform(c)==('/localization/'+backend+'/navigation_state' if expected else '/uav_state')


def test_orbslam_observation_cannot_emit_navigation_state():
    c=LaunchContext();c.launch_configurations.update(slam_enabled='true',localization_backend='orbslam3')
    assert launch._vio_navigation().perform(c)=='False'
    assert launch._mapping_state().perform(c)=='/uav_state'
    c.launch_configurations['orbslam3_navigation']='true'
    c.launch_configurations['slam_enabled']='false'
    assert launch._vio_navigation().perform(c)=='False'


@pytest.mark.parametrize('mode', ['rgbd', 'rgbd_imu'])
@pytest.mark.parametrize('navigation', ['true', 'false'])
def test_native_mode_selects_calibration_and_covariance_contract(mode, navigation, tmp_path):
    from launch.actions import DeclareLaunchArgument
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters
    file = Path(__file__).resolve().parents[1]/'launch/orbslam3.launch.py'
    spec = importlib.util.spec_from_file_location('orb_native_mode', file)
    entry = importlib.util.module_from_spec(spec);spec.loader.exec_module(entry)
    c = LaunchContext()
    c.launch_configurations.update(runtime_directory=str(tmp_path), sensor_mode=mode, enable_navigation=navigation)
    nodes = []
    for action in entry.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(c)
        elif isinstance(action, Node):
            nodes.append(evaluate_parameters(c, action._Node__parameters)[0])
    assert nodes[0]['sensor_mode'] == mode
    assert nodes[0]['settings_template'].endswith('/gazebo_'+mode+'.yaml')
    assert nodes[1]['require_source_covariance'] == (mode == 'rgbd_imu')
    assert nodes[0]['require_navigation_admission'] == (mode == 'rgbd_imu' and navigation == 'true')


def test_managed_inertial_selection_is_bound_to_orb_backend(tmp_path):
    import json
    from launch.actions import GroupAction, IncludeLaunchDescription
    file = ROOT/'src/application/a3_mission/easyfly_sim_launcher/launch/profile.launch.py'
    spec = importlib.util.spec_from_file_location('orb_managed_selection', file)
    entry = importlib.util.module_from_spec(spec);spec.loader.exec_module(entry)
    c = LaunchContext()
    c.launch_configurations.update(profile='navigation_dev', orbslam3_sensor_mode='rgbd_imu',
                                  orbslam3_runtime_directory=str(tmp_path),
                                  algorithms_json=json.dumps({'slam': 'orbslam3'}))
    group = next(a for a in entry._launch_profile(c) if isinstance(a, GroupAction))
    include = next(a for a in group.get_sub_entities() if isinstance(a, IncludeLaunchDescription))
    assert dict(include.launch_arguments)['orbslam3_sensor_mode'] == 'rgbd_imu'
    c.launch_configurations['algorithms_json'] = json.dumps({'slam': 'fast_lio'})
    with pytest.raises(ValueError, match='matching selected backend'):
        entry._launch_profile(c)

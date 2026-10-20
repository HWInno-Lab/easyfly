#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Stage explicit diagnostic sensor ablations; never edit installed resources."""

import copy
from dataclasses import asdict
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

from easyfly_params.configuration import load_profile
from easyfly_px4_sitl.hrt_compatibility import select_px4_directory
from easyfly_px4_sitl.scene_environment import installed_catalog
from easyfly_px4_sitl.scene_inputs import resolve_scene


def stage(out, variant):
    catalog, _ = installed_catalog(px4_dir=select_px4_directory(None))
    scene = resolve_scene(catalog, 'primitives_sparse_seed42_v1', 'navigation_dev')
    iris = Path(scene.model_sdf)
    roots = [Path(p) for p in scene.model_path.split(':')]
    mid, = [p / 'mid360/mid360.sdf' for p in roots if (p / 'mid360/mid360.sdf').is_file()]
    directory = out / 'sensor-fixture'
    directory.mkdir()
    staged_roots = []
    for index, root in enumerate(roots):
        if root != mid.parent.parent:
            staged_roots.append(root)
            continue
        target = directory / f'root-{index}'
        target.mkdir()
        for entry in root.iterdir():
            if entry.name != 'mid360':
                (target / entry.name).symlink_to(entry)
        (target / 'mid360').mkdir()
        for entry in mid.parent.iterdir():
            if entry.name != mid.name:
                (target / 'mid360' / entry.name).symlink_to(entry)
        staged_roots.append(target)
    original_iris, original_mid = ET.parse(iris), ET.parse(mid)
    staged_iris, staged_mid = copy.deepcopy(original_iris), copy.deepcopy(original_mid)
    disabled = []
    if variant in ('no-depth', 'rgb-imu'):
        disabled.append((staged_iris, 'depth'))
    if variant in ('no-lidar', 'rgb-imu'):
        disabled.append((staged_mid, 'laser_livox'))
    for tree, name in disabled:
        matches = [(parent, sensor) for parent in tree.iter()
                   for sensor in parent.findall('sensor') if sensor.get('name') == name]
        assert len(matches) == 1, (name, matches)
        matches[0][0].remove(matches[0][1])
    staged_iris_path = directory / iris.name
    staged_mid_path = next(p / 'mid360/mid360.sdf' for p in staged_roots
                           if p.parent == directory)
    staged_iris.write(staged_iris_path, encoding='utf-8', xml_declaration=True)
    staged_mid.write(staged_mid_path, encoding='utf-8', xml_declaration=True)
    # Removing exactly these nodes must account for every semantic XML change.

    def structure(element):
        return (element.tag, dict(element.attrib), (element.text or '').strip(),
                [structure(child) for child in element])
    for original, staged, path in [(original_iris, staged_iris, staged_iris_path),
                                   (original_mid, staged_mid, staged_mid_path)]:
        expected = copy.deepcopy(original)
        for parent in expected.iter():
            for sensor in parent.findall('sensor'):
                if sensor.get('name') in [name for tree, name in disabled if tree is staged]:
                    parent.remove(sensor)
        assert structure(expected.getroot()) == structure(ET.parse(path).getroot())
    arguments = dict(load_profile('navigation_dev')['launch_arguments'])
    arguments.update(
        headless='true', rviz='false', localization_backend='openvins', slam_enabled='true',
        detection_use_gpu='false', world=scene.world, model_sdf=str(staged_iris_path),
        debug_model_path=':'.join(map(str, staged_roots)),
        debug_spawn_pose=' '.join(map(str, (*scene.position, scene.yaw))),
        debug_heartbeat_rate_hz=str(scene.heartbeat_rate_hz),
        debug_clock_rate_hz=str(scene.clock_rate_hz),
        ground_model_frame=scene.ground_model_frame, grid_footprint_mode=scene.grid_footprint_mode,
        grid_height_limit_m=str(scene.grid_height_limit_m))
    if scene.battery_rate_hz:
        arguments['debug_battery_rate_hz'] = str(scene.battery_rate_hz)
    launch = directory / 'sensor_load.launch.py'
    launch.write_text('''from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    def include(package, filename, arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(
            get_package_share_directory(package) + '/launch/' + filename),
            launch_arguments=arguments.items())
    return LaunchDescription([
        include('easyfly_px4_sitl', 'perception_sitl.launch.py', ''' + repr(arguments) + '''),
        include('easyfly_mission_srvs', 'platform.launch.py',
                {'profile': 'navigation_dev', 'use_sim_time': 'true',
                 'start_mission_executor': 'false'}),
    ])
''')
    return dict(variant=variant, original_scene=asdict(scene), launch=str(launch),
                launch_arguments=arguments, disabled_sensors=[name for _, name in disabled],
                expected_publishers={'/livox/lidar': int(variant not in ('no-lidar', 'rgb-imu')),
                                     '/camera/aligned_depth_to_color/image_raw': int(
                                         variant not in ('no-depth', 'rgb-imu')),
                                     '/camera/color/image_raw': 1, '/livox/imu': 1},
                only_requested_sensor_nodes_changed=True,
                source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in [iris, mid]},
                staged_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in [staged_iris_path, staged_mid_path, launch]})

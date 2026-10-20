#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Create reviewed, explicit launch copies for a stationary stack-layer experiment."""

import ast
import copy
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from sensor_load_fixture import stage as stage_sensors


LAYERS = ('tiny', 'world', 'sitl', 'communications', 'calibration',
          'bridge', 'perception', 'full', 'mavros', 'project_bridge')
TIMERS = {'gui_group', 'bridge_delayed', 'exec_nodes', 'p1_nodes', 'p4_nodes',
          'p5_nodes', 'm40_nodes', 'm41_nodes', 'm42_nodes', 'm43_nodes', 'px4_delayed'}
ENABLED = {
    'tiny': set(), 'world': set(), 'sitl': {'px4_delayed'},
    'communications': {'px4_delayed', 'bridge_delayed'},
    'mavros': {'px4_delayed', 'bridge_delayed'},
    'project_bridge': {'px4_delayed', 'bridge_delayed'},
    'calibration': {'px4_delayed', 'p1_nodes'},
    'bridge': {'px4_delayed', 'bridge_delayed', 'p1_nodes'},
    'perception': {'px4_delayed', 'bridge_delayed', 'p1_nodes',
                   'p4_nodes', 'p5_nodes', 'm40_nodes'},
    'full': TIMERS,
}


def stage(out, layer, sensor_variant="rgb-imu"):
    assert layer in LAYERS
    assert sensor_variant == 'rgb-imu' or layer == 'full'
    fixture = stage_sensors(out, sensor_variant)
    directory = out / 'stack-layer'
    directory.mkdir()
    arguments = fixture['launch_arguments']
    arguments['openvins_wait_for_inputs'] = 'false'
    original_world = Path(arguments['world'])
    original_iris = Path(fixture['original_scene']['model_sdf'])
    source = Path(get_package_share_directory('easyfly_px4_sitl')) / \
        'launch/perception_sitl.launch.py'
    tree = ast.parse(source.read_text())
    communication_source = Path(get_package_share_directory('easyfly_mavlink_bridge')) / \
        'launch/mavlink_bridge.launch.py'
    communication = None
    changed = []
    if layer in ('mavros', 'project_bridge'):
        comm_tree = ast.parse(communication_source.read_text())
        returns = [n for n in ast.walk(comm_tree) if isinstance(n, ast.Return) and
                   isinstance(n.value, ast.Call) and isinstance(n.value.func, ast.Name) and
                   n.value.func.id == 'LaunchDescription']
        returned, = returns
        actions = returned.value.args[0]
        assert isinstance(actions, ast.List)
        removed = {'bridge_node', 'rc_node'} if layer == 'mavros' else {'mavros_node'}
        assert removed <= {n.id for n in actions.elts if isinstance(n, ast.Name)}
        actions.elts = [n for n in actions.elts
                        if not isinstance(n, ast.Name) or n.id not in removed]
        communication = directory / 'communication.launch.py'
        communication.write_text(ast.unparse(ast.fix_missing_locations(comm_tree)) + '\n')
        changed.append('mavlink_bridge.source_path')
    original_actions = None
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign) or len(node.targets) != 1 or \
                not isinstance(node.targets[0], ast.Name):
            continue
        name = node.targets[0].id
        if name == 'mavlink_bridge' and communication:
            launch_source = node.value.args[0]
            assert isinstance(launch_source, ast.Call) and len(launch_source.args) == 1
            launch_source.args[0] = ast.Constant(str(communication))
        if name == 'actions':
            assert isinstance(node.value, ast.List)
            original_actions = copy.deepcopy(node.value)
            kept = []
            for action in node.value.elts:
                if isinstance(action, ast.Name) and action.id in TIMERS and \
                        action.id not in ENABLED[layer]:
                    changed.append(action.id)
                    continue
                if layer in ('tiny', 'world') and isinstance(action, ast.Call) and \
                        isinstance(action.func, ast.Name) and \
                        action.func.id == 'heartbeat_configuration_action':
                    changed.append('heartbeat_configuration_action')
                    continue
                kept.append(action)
            node.value.elts = kept
        elif name == 'p2_nodes' and layer not in ('perception', 'full'):
            actions, = [kw for kw in node.value.keywords if kw.arg == 'actions']
            assert [a.id for a in actions.value.elts] == ['detection', 'fastlio', 'openvins']
            actions.value.elts = [ast.Name(id='openvins', ctx=ast.Load())]
            changed.append('p2_nodes.actions')
    assert original_actions is not None
    assert {a.id for a in original_actions.elts if isinstance(a, ast.Name)} >= TIMERS
    original_tree = ast.parse(source.read_text())
    # Split groups also redirect the include path, independently checked by the analyzer.

    sites = ('actions', 'p2_nodes', 'mavlink_bridge') if communication else ('actions', 'p2_nodes')

    def erase_sites(value):
        for node in ast.walk(value):
            if isinstance(node, ast.Assign) and len(node.targets) == 1 and \
                    isinstance(node.targets[0], ast.Name) and \
                    node.targets[0].id in sites:
                node.value = ast.Constant(None)
        return ast.dump(value, include_attributes=False)
    assert erase_sites(copy.deepcopy(tree)) == erase_sites(original_tree)
    layer_launch = directory / 'layer.launch.py'
    layer_launch.write_text(ast.unparse(ast.fix_missing_locations(tree)) + '\n')
    additional_files = [communication] if communication else []
    if layer in ('tiny', 'world'):
        # Same RGB and nested MID-360 IMU configuration and mount as the airframe.
        iris = ET.parse(original_iris)
        sdf = ET.Element('sdf', version='1.6')
        model = ET.SubElement(sdf, 'model', name='easyfly_iris')
        ET.SubElement(model, 'static').text = 'true'
        link = ET.SubElement(model, 'link', name='base_link')
        color, = iris.findall(".//sensor[@name='color']")
        link.append(copy.deepcopy(color))
        include, = [v for v in iris.findall('.//include')
                    if v.findtext('uri') == 'model://mid360']
        joint, = iris.findall(".//joint[@name='mid360_joint']")
        model.extend([copy.deepcopy(include), copy.deepcopy(joint)])
        static_model = directory / 'static_rgb_imu.sdf'
        ET.ElementTree(sdf).write(static_model, encoding='utf-8', xml_declaration=True)
        arguments['model_sdf'] = str(static_model)
        additional_files.append(static_model)
    if layer == 'tiny':
        # Keep scene lighting, gravity and physics; change only environmental models.
        world_tree = ET.parse(original_world)
        world = world_tree.getroot().find('world')
        for element in list(world):
            if element.tag in ('model', 'include', 'actor'):
                world.remove(element)
        for index, (pose, size, color) in enumerate([
                ('4 0 1 0 0 0', '0.2 4 2', '0.8 0.8 0.8 1'),
                ('3 -1 0.5 0 0 0', '0.4 0.4 1', '0.8 0.1 0.1 1'),
                ('3 1 0.8 0 0 0', '0.4 0.4 1.6', '0.1 0.1 0.8 1')]):
            prop = ET.SubElement(world, 'model', name=f'box_{index}')
            ET.SubElement(prop, 'static').text = 'true'
            ET.SubElement(prop, 'pose').text = pose
            visual = ET.SubElement(ET.SubElement(prop, 'link', name='link'), 'visual', name='box')
            box = ET.SubElement(ET.SubElement(visual, 'geometry'), 'box')
            ET.SubElement(box, 'size').text = size
            material = ET.SubElement(visual, 'material')
            ET.SubElement(material, 'diffuse').text = color
            ET.SubElement(material, 'ambient').text = color
        world_path = directory / 'tiny.world'
        world_tree.write(world_path, encoding='utf-8', xml_declaration=True)
        arguments['world'] = str(world_path)
        additional_files.append(world_path)
    wrapper = directory / 'stack_layer.launch.py'
    wrapper.write_text('''from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    actions = [IncludeLaunchDescription(PythonLaunchDescriptionSource(''' + repr(
        str(layer_launch)) + '''), launch_arguments=''' + repr(arguments) + '''.items())]
''' + ('''    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
        get_package_share_directory('easyfly_mission_srvs') + '/launch/platform.launch.py'),
        launch_arguments={'profile': 'navigation_dev', 'use_sim_time': 'true',
                          'start_mission_executor': 'false'}.items()))
''' if layer == 'full' else '') + '''    return LaunchDescription(actions)
''')
    fixture['launch'] = str(wrapper)
    fixture['staged_sha256'].update({str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                                    for p in [layer_launch, wrapper, *additional_files]})
    fixture['source_sha256'].update({str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                                    for p in [source, original_world, communication_source]})
    fixture['stack_layer'] = dict(
        name=layer, changed_launch_assignments=changed, kept_timers=sorted(ENABLED[layer]),
        platform_services=layer == 'full', source_launch=str(source),
        generated_launch=str(layer_launch), static_sensor_fixture=layer in ('tiny', 'world'),
        modified_world_geometry=layer == 'tiny',
        ast_changes_limited_to_actions_and_p2=communication is None,
        communication_source=str(communication_source),
        communication_generated=str(communication) if communication else None)

    return fixture

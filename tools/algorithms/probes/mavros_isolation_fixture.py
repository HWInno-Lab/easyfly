#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Private communication launch, alignment switch and step-observer world copies."""

import ast
import copy
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from stack_layer_fixture import stage as stage_layer


MODES = ('all_on', 'all_off', 'reference_on', 'reference_off')
REFERENCE_PLUGINS = ['sys_status', 'sys_time', 'imu', 'local_position']


def stage(out, mode, step_plugin):
    assert mode in MODES
    fixture = stage_layer(out, 'communications')
    directory = out/'stack-layer'
    meta = fixture['stack_layer']
    comm_source = Path(meta['communication_source'])
    tree = ast.parse(comm_source.read_text())
    if mode.startswith('reference_'):
        mavros, = [n for n in ast.walk(tree) if isinstance(n, ast.Assign) and
                   isinstance(n.targets[0], ast.Name) and n.targets[0].id == 'mavros_node']
        parameters, = [k for k in mavros.value.keywords if k.arg == 'parameters']
        parameters.value.elts.append(ast.parse(repr(dict(
            plugin_allowlist=REFERENCE_PLUGINS))).body[0].value)
    communication = directory/'isolated_communication.launch.py'
    communication.write_text(ast.unparse(ast.fix_missing_locations(tree))+'\n')
    layer_path = Path(meta['generated_launch'])
    layer = ast.parse(layer_path.read_text())
    for node in ast.walk(layer):
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name) and \
                node.targets[0].id == 'mavlink_bridge':
            node.value.args[0].args[0] = ast.Constant(str(communication))
        if isinstance(node, ast.FunctionDef) and node.name == '_openvins':
            dictionary, = [v for v in ast.walk(node) if isinstance(v, ast.Dict) and
                           any(isinstance(k, ast.Constant) and k.value == 'start_alignment'
                               for k in v.keys)]
            index = next(i for i, k in enumerate(dictionary.keys) if k.value == 'start_alignment')
            assert dictionary.values[index].value == 'true'
            dictionary.values[index] = ast.Constant('false' if mode.endswith('_off') else 'true')
    layer_path.write_text(ast.unparse(ast.fix_missing_locations(layer))+'\n')
    original_world = Path(fixture['launch_arguments']['world'])
    world = ET.parse(original_world)
    root = world.getroot().find('world')
    assert not root.findall("plugin[@name='easyfly_step_trace']")
    ET.SubElement(root, 'plugin', name='easyfly_step_trace', filename=str(step_plugin))
    world_path = directory/'step_trace.world'
    world.write(world_path, encoding='utf-8', xml_declaration=True)
    # The wrapper embeds its arguments, so replace exactly the original world literal.
    wrapper = Path(fixture['launch'])
    tree = ast.parse(wrapper.read_text())
    world_constants = [n for n in ast.walk(tree) if isinstance(n, ast.Constant) and
                       n.value == str(original_world)]
    constant, = world_constants
    constant.value = str(world_path)
    wrapper.write_text(ast.unparse(ast.fix_missing_locations(tree))+'\n')
    fixture['launch_arguments']['world'] = str(world_path)
    fixture['mavros_isolation'] = dict(
        mode=mode, alignment=mode.endswith('_on'),
        plugin_allowlist=REFERENCE_PLUGINS if mode.startswith('reference_') else None,
        communication=str(communication), original_world=str(original_world),
        world=str(world_path), step_plugin=str(step_plugin))
    for p in [communication, layer_path, wrapper, world_path, step_plugin]:
        fixture['staged_sha256'][str(p)] = hashlib.sha256(p.read_bytes()).hexdigest()
    manifest = Path(get_package_share_directory('mavros'))/'mavros_plugins.xml'
    fixture['source_sha256'][str(manifest)] = hashlib.sha256(manifest.read_bytes()).hexdigest()
    return fixture


def audit_trees(fixture, original, staged):
    """Validate/normalize only the two isolation sites before the existing layer audit."""
    meta = fixture['mavros_isolation']

    def bridge(tree):
        return next(n for n in ast.walk(tree) if isinstance(n, ast.Assign) and
                    isinstance(n.targets[0], ast.Name) and n.targets[0].id == 'mavlink_bridge')
    before, after = bridge(original), bridge(staged)
    assert after.value.args[0].args[0].value == meta['communication']
    after.value.args[0].args[0] = copy.deepcopy(before.value.args[0].args[0])
    assert ast.dump(before) == ast.dump(after)

    def alignment(tree):
        function, = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and
                     n.name == '_openvins']
        pairs = [(v, i) for v in ast.walk(function) if isinstance(v, ast.Dict)
                 for i, k in enumerate(v.keys) if isinstance(k, ast.Constant) and
                 k.value == 'start_alignment']
        return pairs[0]
    expected, i = alignment(original)
    observed, j = alignment(staged)
    assert observed.values[j].value == ('true' if meta['alignment'] else 'false')
    observed.values[j] = copy.deepcopy(expected.values[i])
    source = ast.parse(Path(fixture['stack_layer']['communication_source']).read_text())
    generated = ast.parse(Path(meta['communication']).read_text())
    if meta['plugin_allowlist'] is not None:
        assignment, = [n for n in ast.walk(generated) if isinstance(n, ast.Assign) and
                       isinstance(n.targets[0], ast.Name) and n.targets[0].id == 'mavros_node']
        parameters, = [k for k in assignment.value.keywords if k.arg == 'parameters']
        override = ast.literal_eval(parameters.value.elts.pop())
        assert override == {'plugin_allowlist': REFERENCE_PLUGINS}
    assert ast.dump(source) == ast.dump(generated)
    from analyze_sensor_load import structure
    world = ET.parse(meta['world'])
    root = world.getroot().find('world')
    plugin, = root.findall("plugin[@name='easyfly_step_trace']")
    assert plugin.attrib == dict(name='easyfly_step_trace', filename=meta['step_plugin'])
    root.remove(plugin)
    assert structure(world.getroot()) == structure(ET.parse(meta['original_world']).getroot())
    return True

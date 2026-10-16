# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Offline, deterministic primitive worlds. Generation never certifies flight."""

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import jsonschema
import yaml

VERSION = 'primitive_v1'
PACKAGE = 'easyfly_gazebo_worlds'


def world_share():
    return Path(get_package_share_directory(PACKAGE))


def canonical(value):
    return (json.dumps(value, sort_keys=True, indent=2, ensure_ascii=False,
                       allow_nan=False) + '\n').encode('utf-8')


def sha(data):
    return hashlib.sha256(data).hexdigest()


def _pairs(items):
    result = {}
    for key, value in items:
        if key in result:
            raise ValueError('duplicate JSON key: ' + key)
        result[key] = value
    return result


def _constant(value):
    raise ValueError('non-finite JSON number: ' + value)


def load_config(path):
    path = Path(path)
    if path.stat().st_size > 65536:
        raise ValueError('configuration exceeds 64 KiB')
    data = json.loads(path.read_text(encoding='utf-8'), object_pairs_hook=_pairs,
                      parse_constant=_constant)
    validate_config(data)
    return data


def validate_config(data):
    schema = json.loads((world_share() / 'schemas/generator-v1.schema.json').read_text())
    jsonschema.Draft7Validator(schema).validate(data)
    # Python's bool is an int, but the JSON seed/count fields must be integers.
    for value in [data['seed'], data['obstacles']['count']]:
        if type(value) is not int:
            raise ValueError('seed/count must be integers')


class Random64:
    """Specified SplitMix64 stream; independent of Python random/NumPy versions."""

    def __init__(self, seed):
        self.state = seed

    def next(self):
        mask = (1 << 64) - 1
        self.state = (self.state + 0x9e3779b97f4a7c15) & mask
        value = self.state
        value = ((value ^ (value >> 30)) * 0xbf58476d1ce4e5b9) & mask
        value = ((value ^ (value >> 27)) * 0x94d049bb133111eb) & mask
        return value ^ (value >> 31)

    def between(self, low, high):
        # Closed integer-millimetre interval, with a completely specified stream.
        a, b = math.ceil(low * 1000), math.floor(high * 1000)
        if a > b:
            raise ValueError('empty millimetre interval')
        return (a + self.next() % (b - a + 1)) / 1000


def box(name, x, y, sx, sy, height):
    return dict(name=name, shape='box', x=x, y=y, size=[sx, sy], height=height)


def footprint(obstacle):
    sx, sy = obstacle['size']
    return [obstacle['x'] - sx / 2, obstacle['y'] - sy / 2,
            obstacle['x'] + sx / 2, obstacle['y'] + sy / 2]


def separation(a, b):
    """Distance between conservative axis-aligned footprints."""
    return math.hypot(max(a[0] - b[2], b[0] - a[2], 0),
                      max(a[1] - b[3], b[1] - a[3], 0))


def route_clearance(obstacle, x, y, length):
    if obstacle['shape'] == 'cylinder':
        return math.hypot(max(x - obstacle['x'], obstacle['x'] - (x + length), 0),
                          obstacle['y'] - y) - obstacle['size'][0] / 2
    return separation(footprint(obstacle), [x, y, x + length, y])


def layout(config):
    lo, hi = config['bounds']['min'], config['bounds']['max']
    x, y, z = config['spawn']
    length, width = config['route']['length_m'], config['route']['width_m']
    spec = config['obstacles']
    if any(b - a < 8 or b - a > 40 for a, b in zip(lo, hi)):
        raise ValueError('each arena dimension must be 8..40m')
    if spec['min_size_m'] > spec['max_size_m'] or spec['min_height_m'] > spec['max_height_m']:
        raise ValueError('inverted obstacle ranges')
    # The complete route capsule includes the takeoff cylinder and both ends.
    clearance = min(x - lo[0], hi[0] - x - length, y - lo[1], hi[1] - y)
    if clearance < width / 2:
        raise ValueError('route/takeoff clearance leaves arena bounds')
    if z != 0.1:
        raise ValueError('v1 spawn must be 0.1m above the flat z=0 floor')
    cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
    sx, sy = hi[0] - lo[0], hi[1] - lo[1]
    obstacles = [box('boundary_west', lo[0] - .1, cy, .2, sy + .4, 4),
                 box('boundary_east', hi[0] + .1, cy, .2, sy + .4, 4),
                 box('boundary_south', cx, lo[1] - .1, sx, .2, 4),
                 box('boundary_north', cx, hi[1] + .1, sx, .2, 4)]
    if config['layout'] == 'corridor':
        for direction, name in [(-1, 'corridor_south'), (1, 'corridor_north')]:
            obstacles.append(box(name, x + length / 2, y + direction * (width / 2 + .1),
                                 length + 4, .2, 4))
    if config['layout'] == 'blocked':
        obstacles.append(box('rejected_barrier', x + length / 2, cy, .3, sy, 4))
    for obstacle in obstacles:
        if route_clearance(obstacle, x, y, length) + 1e-9 < width / 2:
            raise ValueError('reference corridor blocked by ' + obstacle['name'])
    rng = Random64(config['seed'])
    placed = 0
    for _ in range(spec['count'] * 500):
        if placed == spec['count']:
            break
        size = rng.between(spec['min_size_m'], spec['max_size_m'])
        height = rng.between(spec['min_height_m'], spec['max_height_m'])
        px = rng.between(lo[0] + size / 2 + .2, hi[0] - size / 2 - .2)
        py = rng.between(lo[1] + size / 2 + .2, hi[1] - size / 2 - .2)
        shape = 'cylinder' if rng.next() & 1 else 'box'
        candidate = dict(name=f'obstacle_{placed:03}', shape=shape, x=px, y=py,
                         size=[size, size], height=height)
        if route_clearance(candidate, x, y, length) < width / 2:
            continue
        if any(separation(footprint(candidate), footprint(o)) < spec['spacing_m']
               for o in obstacles):
            continue
        obstacles.append(candidate)
        placed += 1
    if placed != spec['count']:
        raise ValueError(f'placement budget exhausted: {placed}/{spec["count"]}; no partial scene')
    return obstacles


def _element(parent, tag, text=None, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    if text is not None:
        child.text = str(text)
    return child


def _numbers(values):
    return ' '.join(format(v, '.6f') for v in values)


def render_world(config, obstacles):
    root = ET.Element('sdf', version='1.6')
    world = _element(root, 'world', name='default')
    _element(world, 'gravity', '0 0 -9.8')
    _element(world, 'magnetic_field', '6e-06 2.3e-05 -4.2e-05')
    physics = _element(world, 'physics', name='default_physics', type='ode')
    for key, value in [('max_step_size', '.001'), ('real_time_factor', '1'),
                       ('real_time_update_rate', '1000')]:
        _element(physics, key, value)
    scene = _element(world, 'scene')
    for key, value in [('ambient', '.5 .5 .5 1'), ('background', '.7 .7 .7 1'),
                       ('shadows', 'false')]:
        _element(scene, key, value)
    light = _element(world, 'light', name='sun', type='directional')
    _element(light, 'pose', '0 0 10 0 0 0')
    _element(light, 'diffuse', '.8 .8 .8 1')
    _element(light, 'direction', '-.2 .1 -1')
    lo, hi = config['bounds']['min'], config['bounds']['max']
    floor = box('arena_floor', (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2,
                hi[0] - lo[0] + .4, hi[1] - lo[1] + .4, .1)
    for item in [floor] + obstacles:
        model = _element(world, 'model', name=item['name'])
        _element(model, 'static', 'true')
        _element(model, 'pose', _numbers([item['x'], item['y'],
                                         -.05 if item is floor else item['height'] / 2, 0, 0, 0]))
        link = _element(model, 'link', name='body')
        geometry = ET.Element('geometry')
        primitive = _element(geometry, item['shape'])
        if item['shape'] == 'box':
            _element(primitive, 'size', _numbers(item['size'] + [item['height']]))
        else:
            _element(primitive, 'radius', format(item['size'][0] / 2, '.6f'))
            _element(primitive, 'length', format(item['height'], '.6f'))
        for kind in ['collision', 'visual']:
            child = _element(link, kind, name=kind)
            child.append(copy.deepcopy(geometry))
            if kind == 'visual':
                material = _element(child, 'material')
                color = '.65 .65 .65 1' if item is floor else '.25 .45 .65 1'
                _element(material, 'ambient', color)
                _element(material, 'diffuse', color)
    ET.indent(root, space='  ')
    return b'<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(root, encoding='utf-8') + b'\n'


def bundle(config):
    validate_config(config)
    obstacles = layout(config)
    identifier = config['scene_id']
    world_path = f'worlds/benchmark/navigation/{identifier}.world'
    evidence_path = f'evidence/{identifier}/generation.json'
    world = render_world(config, obstacles)
    scene = dict(schema_version=1, id=identifier, name=identifier, status='draft',
                 category='benchmark/navigation', tags=['procedural', 'static', config['layout']],
                 simulator='gazebo_classic', world=dict(package=PACKAGE, path=world_path, sha256=sha(world)),
                 provenance=dict(status='recorded', source_url=None, revision=VERSION,
                                 declared_license='Apache-2.0',
                                 evidence=[dict(package=PACKAGE, path='licenses/PROJECT-LICENSE'),
                                           dict(package=PACKAGE, path=evidence_path)],
                                 note='Project-authored primitive geometry; pinned input, generator and layout fingerprints.'),
                 asset_sets=[], required_plugins=[], supported_profiles=[], legacy_profiles=[],
                 spawn=dict(frame='world', position=config['spawn'], yaw=0.,
                            basis='Analytic flat floor and reserved route capsule; native geometry and calibration pending.'),
                 navigation=dict(frame='odom', world_to_navigation=None, recommended_region=None,
                                 ground_model_frame='map', grid_footprint_mode='observed_cells'),
                 runtime=dict(entrypoint=dict(package='easyfly_px4_sitl', path='models/easyfly_iris/easyfly_iris.sdf'),
                              asset_sets=['easyfly_airframe_v1', 'easyfly_runtime_v1', 'gazebo_builtin_v1'],
                              required_plugins=['libgazebo_ros_init.so'],
                              additional_files=[dict(package='easyfly_gazebo_plugins',
                                                     path='livox_laser_plugin/scan_mode/mid360-real-centr.csv')],
                              notes=['Same managed vehicle, sensors and admission limits as qualified indoor scenes.'],
                              heartbeat_rate_hz=5), recommended_missions=[], map_products=[],
                 verification=dict(resources_complete=True, world_loaded=False, sensors_checked=False,
                                   navigation_tested=False,
                                   evidence_refs=[dict(package=PACKAGE, path=evidence_path)]))
    scene_bytes = yaml.safe_dump(scene, sort_keys=False, allow_unicode=True).encode('utf-8')
    # Preserve qualified metadata separately; this record always describes the
    # original draft and immutable layout, never a claim of runtime readiness.
    record = dict(generator=VERSION, generator_sha256=sha(Path(__file__).read_bytes()),
                  config_sha256=sha(canonical(config)), layout_sha256=sha(canonical(obstacles)),
                  world_sha256=sha(world), initial_scene_sha256=sha(scene_bytes), seed=config['seed'],
                  bounds=config['bounds'], route=config['route'], obstacles=obstacles,
                  scope='Analytic placement only; no native collision, sensor or flight verification.')
    return {world_path: world, f'scenes/{identifier}.yaml': scene_bytes,
            f'generators/{identifier}.json': canonical(config), evidence_path: canonical(record),
            'licenses/PROJECT-LICENSE': (world_share() / 'licenses/PROJECT-LICENSE').read_bytes()}


def generate(config, destination):
    files = bundle(config)  # Fail every format/geometry check before any write.
    destination = Path(destination)
    destination.mkdir()  # Exclusive: never replace an existing version or bundle.
    try:
        for relative, content in files.items():
            path = destination / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open('xb') as stream:
                stream.write(content)
    except BaseException:
        shutil.rmtree(destination)
        raise
    return files


def check_layout(config, destination):
    """Check immutable output; qualification may update only measured metadata."""
    destination = Path(destination)
    for relative, content in bundle(config).items():
        current = (destination / relative).read_bytes()
        if relative.startswith('scenes/'):
            before, after = yaml.safe_load(content), yaml.safe_load(current)
            for key in ['schema_version', 'id', 'name', 'category', 'tags', 'simulator',
                        'world', 'provenance', 'asset_sets', 'required_plugins', 'runtime']:
                if before[key] != after[key]:
                    raise ValueError('immutable scene field differs: ' + key)
            for key in ['frame', 'position', 'yaw']:
                if before['spawn'][key] != after['spawn'][key]:
                    raise ValueError('immutable spawn field differs: ' + key)
        elif current != content:
            raise ValueError('generated bytes differ: ' + relative)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True,
                        help='New bundle directory; merge reviewed files into the world package explicitly')
    parser.add_argument('--check-layout', action='store_true',
                        help='Read-only fingerprint check; does not certify runtime readiness')
    args = parser.parse_args()
    try:
        config = load_config(args.config)
        if args.check_layout:
            check_layout(config, args.output_dir)
        else:
            generate(config, args.output_dir)
        print(json.dumps(dict(passed=True, scene=config['scene_id'], generator=VERSION,
                              operation='check_layout' if args.check_layout else 'generate',
                              runtime_verified=False)))
    except (ValueError, OSError, jsonschema.ValidationError) as error:
        print(json.dumps(dict(passed=False, error=str(error)), ensure_ascii=False), file=sys.stderr)
        raise SystemExit(1)


if __name__ == '__main__':
    main()

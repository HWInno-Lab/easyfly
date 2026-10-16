#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Prepare one reviewed BSD forest layout with unchanged CC-BY pine assets.

Reads only fixed local source bytes. Neither upstream code nor ROS 1 plugins run.
Model payload is prepared separately by scene_catalog fetch, outside installation.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path, PurePosixPath
import tarfile
import xml.etree.ElementTree as ET

import yaml


MODEL_REVISION = '8163eb4b5e7e21985c6591d1c0bfb56468c0093f'
LAYOUT_REVISION = '4ca3e45cd4fe8f5d8ac9fd13daafec67c0d0417d'
ARCHIVE_SHA = 'e4aafc9a6e37f0e496c1ee1712ec26fb01576c5ea2d28f1edcc59dad6e536b7a'
LAYOUT_SHA = '4ede6670f286172a0d9b2ede3b44375d73ae90e85a7a19f2748a972ff09498de'
LAYOUT_LICENSE_SHA = '865d2c04727123ef4dbf4c2189604cf00755f3634ebae46ce09a82eb53c9b7f7'
MODEL_LICENSE_SHA = '776eee6c1a315c1acca92898edb797a9549c71da0e41dcb0eb6864df6301678e'
ASSET = 'osrf_pine_tree_v1'
SCENE = 'forest_pine_v1'


class Dumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def sha(data):
    return hashlib.sha256(data).hexdigest()


def encoded(value):
    return (json.dumps(value, indent=2) + '\n').encode()


def reviewed_outputs(archive_path, layout_root):
    if sha(archive_path.read_bytes()) != ARCHIVE_SHA:
        raise ValueError('reviewed official gazebo_models snapshot required')
    source_world = (layout_root / 'worlds/forest0.world').read_bytes()
    layout_license = (layout_root / 'LICENSE').read_bytes()
    if sha(source_world) != LAYOUT_SHA or sha(layout_license) != LAYOUT_LICENSE_SHA:
        raise ValueError('reviewed first forest layout and original BSD license required')
    prefix = 'gazebo_models-' + MODEL_REVISION
    selected, names = {}, set()
    with tarfile.open(archive_path) as archive:
        entries = archive.getmembers()
        for entry in entries:
            path = PurePosixPath(entry.name)
            if (path.is_absolute() or '..' in path.parts or path.parts[0] != prefix
                    or any(c in entry.name for c in ':\\%') or entry.name in names
                    or not (entry.isfile() or entry.isdir())):
                raise ValueError('unsafe archive member: ' + entry.name)
            names.add(entry.name)
            relative = path.relative_to(prefix).as_posix()
            if entry.isfile() and (relative == 'LICENSE' or relative.startswith('pine_tree/')):
                selected[relative] = archive.extractfile(entry).read()
    expected = {'LICENSE', 'pine_tree/model.config', 'pine_tree/model.sdf',
                'pine_tree/meshes/pine_tree.dae', 'pine_tree/materials/scripts/pine_tree.material',
                'pine_tree/materials/textures/bark_diffuse.png',
                'pine_tree/materials/textures/branch_2_diffuse.png'}
    if set(selected) != expected or sha(selected['LICENSE']) != MODEL_LICENSE_SHA:
        raise ValueError('reviewed six-file model closure/license required')
    provenance = dict(status='recorded', source_url='https://github.com/osrf/gazebo_models',
                      revision=MODEL_REVISION, declared_license='CC-BY-3.0', evidence=[
                          dict(package='easyfly_gazebo_models', path='licenses/osrf_pine/' + p)
                          for p in ['LICENSE', 'ATTRIBUTION.md', 'origins.json']],
                      note='Only the pine_tree six-file closure is selected. Original file bytes '
                           'and model author credits retained; no claim about other models in the archive.')
    inventory = [dict(path=n, sha256=sha(data)) for n, data in sorted(selected.items()) if n != 'LICENSE']
    asset = dict(schema_version=1, asset_set_id=ASSET,
                 model_root=dict(external='asset_cache', path='models'), provenance=provenance,
                 models=[dict(name='pine_tree', sdf='model.sdf', provenance=provenance)],
                 files=inventory, storage='cached', local_changes=[],
                 inventory_note='All six pine_tree files, unchanged. Unrelated repository models '
                                'remain in the original archive and are not extracted into model roots.',
                 archive=dict(format='tar.gz', urls=[
                     'https://codeload.github.com/osrf/gazebo_models/tar.gz/' + MODEL_REVISION],
                     sha256=ARCHIVE_SHA, size_bytes=archive_path.stat().st_size, prefix=prefix,
                     max_members=len(entries), max_unpacked_bytes=sum(e.size for e in entries),
                     license=dict(source='LICENSE', sha256=MODEL_LICENSE_SHA),
                     members=[dict(source=f['path'], target=f['path'], source_sha256=f['sha256'],
                                   replacements=[]) for f in inventory]))
    original = ET.fromstring(source_world).find('world')
    trees = original.findall('model/link')
    if len(trees) != 20 or len(original.findall('model')) != 1:
        raise ValueError('reviewed first twenty-tree layout required')
    tree_source = ET.fromstring(selected['pine_tree/model.sdf']).find('model/link')
    if tree_source is None or list(tree_source.iter('plugin')):
        raise ValueError('unexpected source tree structure')
    sdf = ET.Element('sdf', version='1.6')
    world = ET.SubElement(sdf, 'world', name='default')
    for model in ['ground_plane', 'sun']:
        ET.SubElement(ET.SubElement(world, 'include'), 'uri').text = 'model://' + model
    world.append(copy.deepcopy(original.find('scene')))
    ET.SubElement(world, 'gravity').text = '0 0 -9.8'
    physics = ET.SubElement(world, 'physics', name='default_physics', type='ode')
    for name, value in [('max_step_size', '0.001'), ('real_time_factor', '1'), ('real_time_update_rate', '1000')]:
        ET.SubElement(physics, name).text = value
    forest = ET.SubElement(world, 'model', name='forest_pine')
    ET.SubElement(forest, 'static').text = 'true'
    placements = []
    for source in trees:
        scale = source.findtext('visual/geometry/mesh/scale')
        if scale != source.findtext('collision/geometry/mesh/scale'):
            raise ValueError('source visual/collision scale differs')
        tree = copy.deepcopy(tree_source)
        tree.set('name', source.get('name'))
        ET.SubElement(tree, 'pose').text = source.findtext('pose')
        for mesh in tree.iter('mesh'):
            ET.SubElement(mesh, 'scale').text = scale
        forest.append(tree)
        placements.append(dict(name=source.get('name'), pose=source.findtext('pose'), scale=scale))
    ET.indent(sdf, space='  ')
    world_bytes = (b'<?xml version="1.0"?>\n'
                   b'<!-- BSD-2-Clause layout derived from ETHZ forest_gen; tree assets separately CC-BY-3.0. -->\n'
                   + ET.tostring(sdf, encoding='utf-8') + b'\n')
    layout_record = dict(scene=SCENE, layout_source='https://github.com/ethz-asl/forest_gen',
                         layout_revision=LAYOUT_REVISION, source_world='worlds/forest0.world',
                         source_world_sha256=LAYOUT_SHA, world_sha256=sha(world_bytes),
                         layout_license='BSD-2-Clause', model_asset_set=ASSET,
                         placements=placements, changes=[
                             'Preserve all twenty original link poses, yaw angles and uniform mesh scales.',
                             'Replace all original third-party tree meshes with the reviewed OSRF pine_tree '
                             'collision mesh and original Branch/Bark visuals/materials.',
                             'Omit the two ROS 1 RotorS plugins and their octomap-generation settings.',
                             'Use a cold-start SDF 1.6 world, static links, builtin ground/sun, original '
                             'scene lighting settings and explicit 1ms native ODE physics.',
                         ], limitations='Derived scene, not the original forest benchmark geometry or its '
                                        'published results. Original tree meshes and octomaps are not distributed.')
    attribution = f'''# Pine Tree attribution

Title: Pine Tree. Model author: Louise Poubel (model.config).
Source project: Open Robotics / osrf/gazebo_models.
Original repository notice: Copyright 2012 Nathan Koenig.
Source: https://github.com/osrf/gazebo_models/tree/{MODEL_REVISION}/pine_tree
License: Creative Commons Attribution 3.0 Unported (CC-BY-3.0).
License URL: https://creativecommons.org/licenses/by/3.0/

The six upstream model files are distributed unchanged. Their original author
metadata and repository LICENSE notice are retained. The forest scene scales
and places these models using separately licensed ETHZ layout data; attribution
does not imply endorsement of EasyFly by the original authors.
'''
    origin = dict(source_url=provenance['source_url'], revision=MODEL_REVISION,
                  declared_license='CC-BY-3.0', archive_sha256=ARCHIVE_SHA,
                  archive_bytes=archive_path.stat().st_size, selected_model='pine_tree',
                  selected_model_license_declarations=[], license_sha256=MODEL_LICENSE_SHA,
                  files=[dict(source=f['path'], target='cache:models/' + f['path'],
                              source_sha256=f['sha256'], sha256=f['sha256'], changes=[]) for f in inventory])
    layout_note = f'''# ETHZ forest layout attribution

Source: https://github.com/ethz-asl/forest_gen/tree/{LAYOUT_REVISION}
World: worlds/forest0.world, SHA256 {LAYOUT_SHA}.
Authors: Zachary Taylor; maintainers Zachary Taylor and Helen Oleynkova, ETHZ ASL.
License: BSD-2-Clause; the original notice is preserved in LICENSE.

Only the twenty tree poses/scales and scene settings are reused. Original
TurboSquid/Miedevalworlds tree meshes, precomputed octomaps, generation scripts
and ROS 1 plugins are not included. The derived scene uses separate CC-BY-3.0
Open Robotics Pine Tree assets, so its geometry differs from the benchmark.

Dataset citation requested by upstream:
Oleynikova, Burri, Taylor, Nieto, Siegwart, Galceran (2016),
Continuous-Time Trajectory Optimization for Online UAV Replanning, IROS.
'''
    models = 'src/simulation/easyfly_gazebo_models/'
    worlds = 'src/simulation/easyfly_gazebo_worlds/'
    return {
        models + 'manifests/' + ASSET + '.yaml': yaml.dump(asset, Dumper=Dumper, sort_keys=False).encode(),
        models + 'licenses/osrf_pine/LICENSE': selected['LICENSE'],
        models + 'licenses/osrf_pine/ATTRIBUTION.md': attribution.encode(),
        models + 'licenses/osrf_pine/origins.json': encoded(origin),
        worlds + 'licenses/forest_gen/LICENSE': layout_license,
        worlds + 'licenses/forest_gen/ATTRIBUTION.md': layout_note.encode(),
        worlds + 'worlds/outdoor/forest/' + SCENE + '.world': world_bytes,
        worlds + 'evidence/' + SCENE + '/layout-source.json': encoded(layout_record),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('layout_root', type=Path)
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    outputs = reviewed_outputs(args.archive, args.layout_root)
    for name, data in outputs.items():
        target = args.workspace / name
        if args.check:
            if target.read_bytes() != data:
                raise ValueError('recorded source import differs: ' + name)
        elif target.exists():
            raise ValueError('refusing to overwrite: ' + name)
    if not args.check:
        for name, data in outputs.items():
            target = args.workspace / name
            target.parent.mkdir(parents=True, exist_ok=True)
            with target.open('xb') as stream:
                stream.write(data)
    print(json.dumps(dict(scene=SCENE, tree_placements=20, locked_model_files=6, check=args.check)))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Prepare a fixed, unchanged OSRF office asset and a project-authored world.

Only reads a local, hash-locked official archive. No upstream code is executed.
Payload is installed separately into the external cache by scene_catalog fetch.
"""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import tarfile
import xml.etree.ElementTree as ET
import yaml

REVISION = '8163eb4b5e7e21985c6591d1c0bfb56468c0093f'
ARCHIVE_SHA = 'e4aafc9a6e37f0e496c1ee1712ec26fb01576c5ea2d28f1edcc59dad6e536b7a'
LICENSE_SHA = '776eee6c1a315c1acca92898edb797a9549c71da0e41dcb0eb6864df6301678e'
ASSET = 'osrf_willowgarage_v1'
SCENE = 'office_willowgarage_v1'


class Dumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def sha(data):
    return hashlib.sha256(data).hexdigest()


def encoded(value):
    return (json.dumps(value, indent=2) + '\n').encode()


def reviewed_outputs(archive_path):
    if sha(archive_path.read_bytes()) != ARCHIVE_SHA:
        raise ValueError('reviewed official gazebo_models snapshot required')
    prefix = 'gazebo_models-' + REVISION
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
            if entry.isfile() and (relative == 'LICENSE' or relative.startswith('willowgarage/')):
                selected[relative] = archive.extractfile(entry).read()
    if len(selected) != 15 or sha(selected['LICENSE']) != LICENSE_SHA:
        raise ValueError('reviewed fourteen-file model closure/license required')
    root = ET.fromstring(selected['willowgarage/model.sdf'])
    if root.findtext('model/pose') != '-20 -20 0 0 0 0' or list(root.iter('plugin')):
        raise ValueError('reviewed static model pose without plugins required')
    provenance = dict(status='recorded', source_url='https://github.com/osrf/gazebo_models',
                      revision=REVISION, declared_license='CC-BY-3.0', evidence=[
                          dict(package='easyfly_gazebo_models', path='licenses/osrf_willowgarage/' + p)
                          for p in ['LICENSE', 'ATTRIBUTION.md', 'origins.json']],
                      note='Only the fourteen-file Willow Garage closure is selected, including all '
                           'model.config SDF versions, two meshes and seven textures. Original bytes '
                           'and model author credits retained; no claim about other upstream models.')
    inventory = [dict(path=n, sha256=sha(data)) for n, data in sorted(selected.items()) if n != 'LICENSE']
    asset = dict(schema_version=1, asset_set_id=ASSET,
                 model_root=dict(external='asset_cache', path='models'), provenance=provenance,
                 models=[dict(name='willowgarage', sdf='model.sdf', provenance=provenance)],
                 files=inventory, storage='cached', local_changes=[],
                 inventory_note='All fourteen original model files unchanged; unrelated archive '
                                'models are not extracted into model search roots.',
                 archive=dict(format='tar.gz', urls=[
                     'https://codeload.github.com/osrf/gazebo_models/tar.gz/' + REVISION],
                     sha256=ARCHIVE_SHA, size_bytes=archive_path.stat().st_size, prefix=prefix,
                     max_members=len(entries), max_unpacked_bytes=sum(e.size for e in entries),
                     license=dict(source='LICENSE', sha256=LICENSE_SHA),
                     members=[dict(source=f['path'], target=f['path'], source_sha256=f['sha256'],
                                   replacements=[]) for f in inventory]))
    world_bytes = b'''<?xml version="1.0"?>
<!-- Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
     Project-authored wrapper; unchanged Willow Garage model separately CC-BY-3.0. -->
<sdf version="1.6">
  <world name="default">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://willowgarage</uri></include>
    <gravity>0 0 -9.8</gravity>
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene><ambient>0.7 0.7 0.7 1</ambient><shadows>false</shadows></scene>
  </world>
</sdf>
'''
    attribution = f'''# Willow Garage attribution

Title: Willow Garage (the offices of Willow Garage).
Model author: Nate Koenig (model.config); original notice: Copyright 2012 Nathan Koenig.
Source project: Open Robotics / osrf/gazebo_models.
Source: https://github.com/osrf/gazebo_models/tree/{REVISION}/willowgarage
License: Creative Commons Attribution 3.0 Unported (CC-BY-3.0).
License URL: https://creativecommons.org/licenses/by/3.0/

All fourteen upstream files are distributed unchanged, including two original
visual/collision meshes, seven textures and all four model.config SDF versions.
Original author metadata, model pose, mesh units/transforms and repository
LICENSE notice are retained. EasyFly supplies its own world wrapper and measured
spawn/route; attribution does not imply endorsement by the original authors.
'''
    origin = dict(source_url=provenance['source_url'], revision=REVISION,
                  declared_license='CC-BY-3.0', archive_sha256=ARCHIVE_SHA,
                  archive_bytes=archive_path.stat().st_size, selected_model='willowgarage',
                  selected_model_license_declarations=[], license_sha256=LICENSE_SHA,
                  files=[dict(source=f['path'], target='cache:models/' + f['path'],
                              source_sha256=f['sha256'], sha256=f['sha256'], changes=[]) for f in inventory])
    record = dict(scene=SCENE, world_sha256=sha(world_bytes), wrapper_license='Apache-2.0',
                  asset_set=ASSET, source_revision=REVISION, model_files=14,
                  selected_payload_bytes=sum(len(v) for k, v in selected.items() if k != 'LICENSE'),
                  source_model_pose='-20 -20 0 0 0 0', source_model_files_changed=False,
                  collision_mesh_sha256=sha(selected['willowgarage/meshes/willowgarage_collision.dae']),
                  visual_mesh_sha256=sha(selected['willowgarage/meshes/willowgarage_visual.dae']),
                  changes=['New project-authored SDF 1.6 wrapper, builtin sun/ground, native 1ms ODE.',
                           'No upstream ROS package, robots, launch code, plugins, saved clock or map products imported.'],
                  limitations='Geometry and native sensor/navigation qualification required independently; '
                              'no model, prior forest or third-party office benchmark PASS inherited.')
    models = 'src/simulation/easyfly_gazebo_models/'
    worlds = 'src/simulation/easyfly_gazebo_worlds/'
    return {
        models + 'manifests/' + ASSET + '.yaml': yaml.dump(asset, Dumper=Dumper, sort_keys=False).encode(),
        models + 'licenses/osrf_willowgarage/LICENSE': selected['LICENSE'],
        models + 'licenses/osrf_willowgarage/ATTRIBUTION.md': attribution.encode(),
        models + 'licenses/osrf_willowgarage/origins.json': encoded(origin),
        worlds + 'worlds/indoor/office/' + SCENE + '.world': world_bytes,
        worlds + 'evidence/' + SCENE + '/import-source.json': encoded(record),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    outputs = reviewed_outputs(args.archive)
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
    print(json.dumps(dict(scene=SCENE, locked_model_files=14, check=args.check)))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Register three unchanged, pinned OSRF buildings as an optional cache asset.

Only reads an existing official archive. No download, extraction into ROS package
payloads, upstream code execution, world generation or qualification is implied.
"""
import argparse
import hashlib
import json
from pathlib import Path
import tarfile
import xml.etree.ElementTree as ET

import yaml

from audit_osrf_urban import ARCHIVE_SHA, LICENSE_SHA, MODELS, REVISION, archive_files

ASSET = 'osrf_urban_buildings_v1'
SCENE = 'urban_block_osrf_v1'


class Dumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def sha(data):
    return hashlib.sha256(data).hexdigest()


def encoded(value):
    return (json.dumps(value, indent=2) + '\n').encode()


def reviewed_outputs(path):
    selected = archive_files(path)
    with tarfile.open(path) as archive:
        entries = archive.getmembers()
    inventory = [dict(path=name, sha256=sha(data)) for name, data in sorted(selected.items())
                 if name != 'LICENSE']
    provenance = dict(status='recorded', source_url='https://github.com/osrf/gazebo_models',
                      revision=REVISION, declared_license='CC-BY-3.0', evidence=[
                          dict(package='easyfly_gazebo_models', path='licenses/osrf_urban_buildings/' + p)
                          for p in ('LICENSE', 'ATTRIBUTION.md', 'origins.json')],
                      note='Only apartment, office_building and parking_garage; all fifteen '
                           'upstream files unchanged, including every config-listed SDF version. '
                           'Mesh units/transforms, collision, texture pixels and author metadata retained.')
    asset = dict(schema_version=1, asset_set_id=ASSET,
                 model_root=dict(external='asset_cache', path='models'), provenance=provenance,
                 models=[dict(name=name, sdf='model.sdf', provenance=provenance) for name in MODELS],
                 files=inventory, storage='cached', local_changes=[],
                 inventory_note='All fifteen unchanged building files; unrelated archive models '
                                'are not exposed, packaged or installed as scene dependencies.',
                 archive=dict(format='tar.gz', urls=[
                     'https://codeload.github.com/osrf/gazebo_models/tar.gz/' + REVISION],
                     sha256=ARCHIVE_SHA, size_bytes=path.stat().st_size,
                     prefix='gazebo_models-' + REVISION,
                     max_members=len(entries), max_unpacked_bytes=sum(e.size for e in entries),
                     license=dict(source='LICENSE', sha256=LICENSE_SHA),
                     members=[dict(source=f['path'], target=f['path'], source_sha256=f['sha256'],
                                   replacements=[]) for f in inventory]))
    authors = {}
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    for name in MODELS:
        config = ET.fromstring(selected[name + '/model.config'])
        mesh = ET.fromstring(selected[name + '/meshes/' + name + '.dae'])
        authors[name] = dict(config_authors=[e.text for e in config.findall('author/name')],
                             original_mesh_author_metadata=[e.text for e in mesh.findall(
                                 'c:asset/c:contributor/c:author', ns)])
    attribution = f'''# OSRF urban buildings attribution

Source project: Open Robotics / osrf/gazebo_models.
Pinned source: https://github.com/osrf/gazebo_models/tree/{REVISION}
License: Creative Commons Attribution 3.0 Unported (CC-BY-3.0).
License URL: https://creativecommons.org/licenses/by/3.0/
Original repository notice: Copyright 2012 Nathan Koenig (see LICENSE).

Selected models and original credits:
- Apartment: Nate Koenig (model.config); Cole (original DAE author field).
- Office Building: Cole Biesemeyer and Nate Koenig (model.config);
  Blender User (original DAE author field, retained verbatim).
- Parking garage: Nate Koenig (model.config); Cole (original DAE author field).

All fifteen upstream files are distributed unchanged: three model.config files,
five SDF versions, three original visual/collision DAE meshes and four JPEG textures.
Original units, scene transforms, author fields and model definitions are retained.
No other source models or CitySim/CMU/Clearpath world are included in this asset.
EasyFly authors its own street layout/world and measures placement, floor and
clearance separately. New layout/world uses project Apache-2.0; original building
meshes and textures remain CC-BY-3.0. Attribution does not imply endorsement.
'''
    origins = dict(source_url=provenance['source_url'], revision=REVISION,
                   declared_license='CC-BY-3.0', archive_sha256=ARCHIVE_SHA,
                   archive_bytes=path.stat().st_size, license_sha256=LICENSE_SHA,
                   selected_models=list(MODELS), original_author_metadata=authors,
                   selected_model_license_exceptions_found=[],
                   files=[dict(source=f['path'], target='cache:models/' + f['path'],
                               source_sha256=f['sha256'], sha256=f['sha256'], changes=[])
                          for f in inventory])
    record = dict(scene=SCENE, asset_set=ASSET, source_revision=REVISION,
                  archive_sha256=ARCHIVE_SHA, selected_models=list(MODELS), model_files=15,
                  selected_payload_bytes=sum(len(v) for k, v in selected.items() if k != 'LICENSE'),
                  model_license='CC-BY-3.0', source_model_files_changed=False,
                  source_mesh_unit=dict(name='inch', meter='0.0254'),
                  changes=['New cache manifest, file inventory and explicit attribution only.',
                           'World/layout and native geometry evidence maintained independently.'],
                  limitations='Asset provenance and resource closure do not qualify a UAV runtime or route.')
    models = 'src/simulation/easyfly_gazebo_models/'
    worlds = 'src/simulation/easyfly_gazebo_worlds/'
    return {
        models + 'manifests/' + ASSET + '.yaml': yaml.dump(asset, Dumper=Dumper, sort_keys=False).encode(),
        models + 'licenses/osrf_urban_buildings/LICENSE': selected['LICENSE'],
        models + 'licenses/osrf_urban_buildings/ATTRIBUTION.md': attribution.encode(),
        models + 'licenses/osrf_urban_buildings/origins.json': encoded(origins),
        worlds + 'evidence/' + SCENE + '/asset-source.json': encoded(record),
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
    print(json.dumps(dict(scene=SCENE, asset_set=ASSET, locked_model_files=15, check=args.check)))


if __name__ == '__main__':
    main()

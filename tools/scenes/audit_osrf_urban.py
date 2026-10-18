#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Audit three pinned OSRF buildings without registering or launching a scene.

Reads an existing official archive; never downloads or executes upstream code.
The temporary catalog is a resource-only fixture, not a UAV runtime or layout.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import sys
import tarfile
import tempfile
import xml.etree.ElementTree as ET

import yaml

REVISION = '8163eb4b5e7e21985c6591d1c0bfb56468c0093f'
ARCHIVE_SHA = 'e4aafc9a6e37f0e496c1ee1712ec26fb01576c5ea2d28f1edcc59dad6e536b7a'
LICENSE_SHA = '776eee6c1a315c1acca92898edb797a9549c71da0e41dcb0eb6864df6301678e'
MODELS = ('apartment', 'office_building', 'parking_garage')
WORKSPACE = Path(__file__).resolve().parents[2]


def digest(data):
    return hashlib.sha256(data).hexdigest()


def archive_files(path):
    fingerprint = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            fingerprint.update(block)
    if fingerprint.hexdigest() != ARCHIVE_SHA:
        raise ValueError('fixed official gazebo_models archive fingerprint required')
    selected, seen = {}, set()
    prefix = 'gazebo_models-' + REVISION
    with tarfile.open(path) as archive:
        for member in archive.getmembers():
            p = PurePosixPath(member.name)
            if (p.is_absolute() or '..' in p.parts or p.parts[0] != prefix
                    or member.name in seen or not (member.isfile() or member.isdir())):
                raise ValueError('unsafe or duplicate archive member: ' + member.name)
            seen.add(member.name)
            relative = p.relative_to(prefix).as_posix()
            if member.isfile() and (relative == 'LICENSE' or relative.split('/')[0] in MODELS):
                selected[relative] = archive.extractfile(member).read()
    if len(selected) != 16 or digest(selected['LICENSE']) != LICENSE_SHA:
        raise ValueError('reviewed fifteen-file building inventory and license required')
    return selected


def audit(path):
    selected = archive_files(path)
    inventory = [dict(path=name, size_bytes=len(data), sha256=digest(data))
                 for name, data in sorted(selected.items()) if name != 'LICENSE']
    model_records = []
    for name in MODELS:
        config = ET.fromstring(selected[name + '/model.config'])
        versions = []
        for version in config.findall('sdf'):
            source = name + '/' + version.text.strip()
            tree = ET.fromstring(selected[source])
            model = tree.find('model')
            if (model is None or model.findtext('static') not in ('true', '1')
                    or list(tree.iter('plugin')) or list(tree.iter('include'))):
                raise ValueError('reviewed standalone static model without plugins required')
            versions.append(dict(sdf_version=version.get('version'), path=source,
                                 mesh_uris=[e.text for e in tree.findall('.//mesh/uri')]))
        meshes = []
        ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
        for source in sorted(p for p in selected if p.startswith(name + '/') and p.endswith('.dae')):
            tree = ET.fromstring(selected[source])
            unit = tree.find('c:asset/c:unit', ns)
            meshes.append(dict(path=source,
                               author_metadata=[e.text for e in tree.findall('c:asset/c:contributor/c:author', ns)],
                               unit=unit.attrib if unit is not None else None,
                               up_axis=tree.findtext('c:asset/c:up_axis', namespaces=ns),
                               image_references=[e.text for e in tree.findall('c:library_images/c:image/c:init_from', ns)]))
        model_records.append(dict(name=name,
                                  config_authors=[e.text for e in config.findall('author/name')],
                                  declared_license='CC-BY-3.0',
                                  license_basis='Pinned repository LICENSE; model.config and DAE author metadata retained; no per-model license exception found in selected files.',
                                  sdf_versions=versions, meshes=meshes))

    sys.path.insert(0, str(WORKSPACE / 'src/simulation/easyfly_px4_sitl'))
    from easyfly_px4_sitl.scene_catalog import SceneCatalog

    with tempfile.TemporaryDirectory(prefix='easyfly-urban-resource-audit-') as directory:
        root = Path(directory)
        worlds, models, assets = root / 'worlds', root / 'models', root / 'assets'
        for p in (worlds / 'scenes', models / 'manifests', models / 'licenses', assets / 'models'):
            p.mkdir(parents=True)
        source_worlds = WORKSPACE / 'src/simulation/easyfly_gazebo_worlds'
        shutil.copytree(source_worlds / 'schemas', worlds / 'schemas')
        for source, data in selected.items():
            target = models / 'licenses/UPSTREAM-LICENSE' if source == 'LICENSE' else assets / 'models' / source
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
        fixture = ('<?xml version="1.0"?>\n'
                   '<!-- Project-authored resource-only fixture; no flight layout. -->\n'
                   '<sdf version="1.6"><world name="resource_audit">' +
                   ''.join('<include><uri>model://' + name + '</uri></include>' for name in MODELS) +
                   '</world></sdf>\n').encode()
        (worlds / 'fixture.world').write_bytes(fixture)
        shutil.copyfile(source_worlds / 'licenses/PROJECT-LICENSE', worlds / 'PROJECT-LICENSE')
        provenance = dict(status='recorded', source_url='https://github.com/osrf/gazebo_models',
                          revision=REVISION, declared_license='CC-BY-3.0', evidence=[
                              dict(package='easyfly_gazebo_models', path='licenses/UPSTREAM-LICENSE')],
                          note='Only the selected unchanged model files; original author metadata retained.')
        manifest = dict(schema_version=1, asset_set_id='urban_buildings_audit',
                        model_root=dict(external='audit_assets', path='models'),
                        provenance=provenance, storage='existing', local_changes=[],
                        inventory_note='All fifteen unchanged files, including every model.config SDF version.',
                        models=[dict(name=name, sdf='model.sdf', provenance=copy.deepcopy(provenance))
                                for name in MODELS],
                        files=[dict(path=f['path'], sha256=f['sha256']) for f in inventory])
        (models / 'manifests/urban.yaml').write_text(yaml.safe_dump(manifest, sort_keys=False))
        scene = yaml.safe_load((source_worlds / 'scenes/empty.yaml').read_text())
        scene.update(id='urban_resource_audit', name='Resource-only building fixture',
                     status='draft', category='outdoor/urban', supported_profiles=[], legacy_profiles=[],
                     asset_sets=['urban_buildings_audit'], required_plugins=[],
                     world=dict(package='easyfly_gazebo_worlds', path='fixture.world', sha256=digest(fixture)))
        scene['provenance'] = dict(status='recorded', source_url=None, revision='resource_fixture_v1',
                                   declared_license='Apache-2.0', evidence=[
                                       dict(package='easyfly_gazebo_worlds', path='PROJECT-LICENSE')],
                                   note='Project-authored resource-only fixture; no flight layout or runtime.')
        scene['spawn'] = dict(frame='world', position=None, yaw=None, basis='Not measured; resource fixture only.')
        scene['navigation'].update(world_to_navigation=None, recommended_region=None)
        scene['runtime'] = dict(entrypoint=dict(package='easyfly_gazebo_worlds', path='fixture.world'),
                                asset_sets=[], required_plugins=[],
                                additional_files=[], notes=['Resource fixture only; no UAV runtime checked.'])
        scene['recommended_missions'], scene['map_products'] = [], []
        scene['verification'] = dict(resources_complete=False, world_loaded=False, sensors_checked=False,
                                      navigation_tested=False, evidence_refs=[])
        (worlds / 'scenes/urban.yaml').write_text(yaml.safe_dump(scene, sort_keys=False))
        catalog = SceneCatalog(dict(easyfly_gazebo_worlds=worlds, easyfly_gazebo_models=models),
                               external_roots=dict(audit_assets=assets))
        report = catalog.validate('urban_resource_audit').as_dict()
        if (not report['scene_resources_complete'] or not report['licenses_complete']
                or report['registration_complete'] or report['runtime_verified']):
            raise ValueError('unexpected resource fixture result: ' + json.dumps(report))
        # Demonstrate that the positive result depends on the actual texture and locked bytes.
        texture = assets / 'models/apartment/materials/textures/apartment_diffuse.jpg'
        original = texture.read_bytes()
        texture.unlink()
        missing_rejected = not catalog.validate('urban_resource_audit').as_dict()['scene_resources_complete']
        texture.write_bytes(original + b'changed')
        changed_rejected = not catalog.validate('urban_resource_audit').as_dict()['scene_resources_complete']
        texture.write_bytes(original)
        if not (missing_rejected and changed_rejected):
            raise ValueError('missing or modified texture was not rejected')
        native_checks = []
        for record in model_records:
            for version in record['sdf_versions']:
                source = assets / 'models' / version['path']
                result = subprocess.run(['gz', 'sdf', '-k', str(source)], capture_output=True,
                                        text=True, timeout=30, env={**os.environ, 'GAZEBO_MODEL_DATABASE_URI': ''})
                if result.returncode != 0 or 'Check complete' not in result.stdout:
                    raise ValueError('native SDF syntax check failed: ' + version['path'] + result.stderr)
                native_checks.append(dict(path=version['path'], exit_code=result.returncode,
                                          stdout=result.stdout.strip(), stderr=result.stderr.strip()))
        checked_files = sorted(str(p.relative_to(root)) for p in map(Path, report['checked_files']))
        return dict(schema_version=1, audit='s6-urban-building-resources',
                    source_url='https://github.com/osrf/gazebo_models', revision=REVISION,
                    archive_sha256=ARCHIVE_SHA, archive_size_bytes=path.stat().st_size,
                    license_sha256=LICENSE_SHA, declared_license='CC-BY-3.0',
                    catalog_sha256=digest((WORKSPACE / 'src/simulation/easyfly_px4_sitl/easyfly_px4_sitl/scene_catalog.py').read_bytes()),
                    schema_sha256={name: digest((source_worlds / 'schemas' / name).read_bytes())
                                   for name in ('scene-v1.schema.json', 'asset-set-v1.schema.json')},
                    selected_models=model_records, files=inventory,
                    selected_payload_bytes=sum(f['size_bytes'] for f in inventory),
                    resource_closure_complete=True, checked_files=checked_files,
                    registration_complete=False, runtime_verified=False,
                    uav_runtime_checked=False, native_world_loaded=False,
                    geometry_checked=False, sensors_checked=False, navigation_tested=False,
                    production_catalog_modified=False,
                    native_sdf_syntax_checks=native_checks,
                    negative_checks=dict(missing_texture_rejected=missing_rejected,
                                         changed_texture_rejected=changed_rejected),
                    registration_issues=report['issues'],
                    limitation='Resource-only temporary catalog; overlapping default includes are not a flight layout. Preserve original mesh unit/transforms; measure native dimensions, collision and floor before choosing placement or spawn.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = audit(args.archive)
    report['audit_tool_sha256'] = digest(Path(__file__).read_bytes())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(dict(output=str(args.output), models=len(MODELS), files=len(report['files']),
                          payload_bytes=report['selected_payload_bytes'],
                          resource_closure_complete=True, native_sdf_checks=len(report['native_sdf_syntax_checks']),
                          production_catalog_modified=False)))


if __name__ == '__main__':
    main()

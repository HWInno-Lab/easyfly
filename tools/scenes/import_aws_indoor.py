#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Import two reviewed indoor snapshots from local archives, without executing upstream code."""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import re
import shutil
import tarfile
import tempfile
import xml.etree.ElementTree as ET

import yaml


class InventoryDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


SOURCES = {
    'house': dict(repository='aws-robomaker-small-house-world',
                  revision='ff9631ca6d1db9c1ba656498151464b5ab74aafe',
                  archive_sha256='e459bd9d7bdabdfc40f8afc6770ceb1d774316e5da94a1048f036baa7388b2d9',
                  world='small_house.world', category='residential'),
    'bookstore': dict(repository='aws-robomaker-bookstore-world',
                      revision='f5a2aaacd5418f9ecfa30f79e788856417f17c13',
                      archive_sha256='ae7f0649410c05974f125d7ad64ce5d1b2a43a815724fb7632bb1241277a9f54',
                      world='bookstore.world', category='commercial'),
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def model_references(path):
    names = set()
    for node in ET.parse(path).iter('uri'):
        uri = (node.text or '').strip().replace('file://models/', 'model://')
        if uri.startswith('model://'):
            name = uri[8:].split('/')[0]
            if not re.fullmatch(r'[A-Za-z0-9_]+', name):
                raise ValueError('invalid model name: ' + name)
            names.add(name)
    return names


def import_snapshot(kind, archive_path, workspace):
    spec = SOURCES[kind]
    if digest(archive_path) != spec['archive_sha256']:
        raise ValueError('archive fingerprint mismatch; reviewed immutable snapshot required')
    models = workspace / 'src/simulation/easyfly_gazebo_models'
    worlds = workspace / 'src/simulation/easyfly_gazebo_worlds'
    target = models / f'third_party/aws/{kind}'
    world_target = worlds / f'worlds/indoor/{spec["category"]}/{kind}_aws_v1.world'
    manifest_path = models / f'manifests/aws_{kind}_v1.yaml'
    license_dir = models / f'licenses/aws_{kind}'
    world_license = worlds / f'licenses/aws_{kind}'
    if any(p.exists() for p in [target, world_target, manifest_path, license_dir, world_license]):
        raise ValueError('refusing to overwrite an existing import')
    with tempfile.TemporaryDirectory(prefix=f'easyfly-{kind}-import-') as temp:
        root = Path(temp)
        with tarfile.open(archive_path) as archive:
            for entry in archive.getmembers():
                if (not (root / entry.name).resolve().is_relative_to(root)
                        or not (entry.isdir() or entry.isfile())):
                    raise ValueError('unsafe archive entry: ' + entry.name)
            archive.extractall(root)
        source = root / (spec['repository'] + '-' + spec['revision'])
        original_world = source / 'worlds' / spec['world']
        pending = model_references(original_world)
        used = set()
        while pending:
            name = pending.pop()
            if name in used:
                continue
            directory = source / 'models' / name
            if not (directory / 'model.config').is_file() or not (directory / 'model.sdf').is_file():
                raise ValueError('missing model entry: ' + name)
            used.add(name)
            pending.update(model_references(directory / 'model.sdf') - used)
        # License checks precede all writes to the workspace.
        declarations = [p.relative_to(source).as_posix() for p in source.rglob('*')
                        if p.is_file() and any(k in p.name.lower()
                            for k in ('license', 'notice', 'copying', 'copyright'))]
        if declarations != ['LICENSE']:
            raise ValueError('additional license declaration needs review: ' + repr(declarations))
        world_bytes = original_world.read_bytes()
        world_changes = []
        removed = []
        if kind == 'bookstore':
            pattern = rb'[ \t]*<model name="BookE_01_009">.*?</model>[ \t]*\r?\n'
            matches = re.findall(pattern, world_bytes, flags=re.S)
            if len(matches) != 1 or b'343582.718718' not in matches[0]:
                raise ValueError('unexpected upstream outlier instance')
            removed = [dict(name='BookE_01_009',
                upstream_pose='1.170199 1.196251 343582.718718 0.002241 -0.001100 -0.623328',
                reason='Outlying static book instance at 343582m; removed without inferring a replacement pose.')]
            world_bytes = re.sub(pattern, b'', world_bytes, flags=re.S)
            world_changes = ['Remove only BookE_01_009, the 343582m static outlier; all other world bytes retained.']
        target.mkdir(parents=True)
        records = []
        excluded = []
        for name in sorted(used):
            original_dir = source / 'models' / name
            shutil.copytree(original_dir, target / name, ignore=shutil.ignore_patterns('.DS_Store'))
            for original in sorted(original_dir.rglob('*')):
                if not original.is_file():
                    continue
                if original.name == '.DS_Store':
                    excluded.append(original.relative_to(source).as_posix())
                    continue
                copied = target / name / original.relative_to(original_dir)
                changes = []
                if copied.suffix == '.sdf':
                    before = copied.read_bytes()
                    after = before.replace(b'file://models/', b'model://')
                    if kind == 'bookstore' and name == 'aws_robomaker_retail_Spotlight_01':
                        after = after.replace(b'/aws_Spotlight_01_', b'/aws_spotlight_01_')
                        changes.append('Correct Spotlight mesh filename case to the existing upstream lowercase spotlight files')
                    if before != after:
                        copied.write_bytes(after)
                        if b'file://models/' in before:
                            changes.append('file://models/ -> model:// for relocatable model references; original line endings retained')
                if kind == 'house' and copied.suffix.lower() == '.dae':
                    before = copied.read_bytes()
                    pattern = rb'(?:\.\./)+photos/([A-Za-z0-9_]+\.jpg)'
                    for photo_name in sorted(set(re.findall(pattern, before))):
                        photo = source / 'photos' / photo_name.decode()
                        if not photo.is_file():
                            raise ValueError('missing referenced upstream photo: ' + str(photo))
                        local = target / name / 'materials/textures' / photo.name
                        if local.exists():
                            raise ValueError('photo relocation would overwrite a model texture')
                        local.parent.mkdir(parents=True, exist_ok=True)
                        shutil.copyfile(photo, local)
                        records.append(dict(path=local.relative_to(models).as_posix(),
                            origin=photo.relative_to(source).as_posix(),
                            upstream_sha256=digest(photo), sha256=digest(local),
                            changes=['Relocate referenced root photo inside its owning model texture directory']))
                    after = re.sub(pattern, rb'../materials/textures/\1', before)
                    if before != after:
                        copied.write_bytes(after)
                        changes.append('Relocate external photos/ image references to model-local textures; vertex/collision arrays unchanged')
                records.append(dict(path=copied.relative_to(models).as_posix(),
                    origin=original.relative_to(source).as_posix(),
                    upstream_sha256=digest(original), sha256=digest(copied), changes=changes))
        world_target.parent.mkdir(parents=True, exist_ok=True)
        world_target.write_bytes(world_bytes)
        for directory in [license_dir, world_license]:
            directory.mkdir(parents=True)
            shutil.copyfile(source / 'LICENSE', directory / 'LICENSE')
        records.append(dict(path=world_target.relative_to(worlds).as_posix(),
            origin=original_world.relative_to(source).as_posix(),
            upstream_sha256=digest(original_world), sha256=digest(world_target), changes=world_changes))
        shutil.copyfile(original_world, license_dir / ('upstream-' + spec['world']))
        if world_changes:
            patch = ''.join(difflib.unified_diff(original_world.read_text().splitlines(keepends=True),
                world_target.read_text().splitlines(keepends=True), fromfile='upstream/' + spec['world'],
                tofile=world_target.name))
            (license_dir / 'world-changes.diff').write_text(patch)
        origin = dict(source_url='https://github.com/aws-robotics/' + spec['repository'],
            revision=spec['revision'], archive_sha256=spec['archive_sha256'], declared_license='MIT-0',
            included_models=sorted(used),
            excluded_models=sorted(p.name for p in (source / 'models').iterdir() if p.is_dir() and p.name not in used),
            excluded_files=excluded, removed_instances=removed,
            license_declarations=declarations, files=records)
        (license_dir / 'origins.json').write_text(json.dumps(origin, indent=2) + '\n')
        provenance = dict(status='recorded', source_url=origin['source_url'], revision=spec['revision'],
            declared_license='MIT-0', evidence=[dict(package='easyfly_gazebo_models', path=f'licenses/aws_{kind}/{name}')
                for name in ['LICENSE', 'origins.json']],
            note='Pinned official snapshot dependency closure; original license and per-file source hashes retained.')
        manifest = dict(schema_version=1, asset_set_id=f'aws_{kind}_v1',
            model_root=dict(package='easyfly_gazebo_models', path=f'third_party/aws/{kind}'),
            provenance=provenance,
            models=[dict(name=name, sdf='model.sdf', provenance=provenance.copy()) for name in sorted(used)],
            local_changes=(['Relocate referenced house portrait photos inside owning models; preserve image and mesh geometry bytes.']
                           if kind == 'house' else
                           ['Normalize SDF file://models/ URIs and Spotlight mesh filename case; preserve all mesh/texture bytes.']),
            inventory_note='S3 pinned dependency snapshot; see origins.json for world-instance changes.',
            files=[dict(path=r['path'].removeprefix(f'third_party/aws/{kind}/'), sha256=r['sha256'])
                   for r in records if r['path'].startswith('third_party/')], storage='vendored')
        manifest_path.write_text(yaml.dump(manifest, Dumper=InventoryDumper, sort_keys=False))
        print(f'{kind}: {len(used)} shared models, {len(records)} locked world/model files, '
              f'{sum(p.stat().st_size for p in target.rglob("*") if p.is_file())} model bytes')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('kind', choices=sorted(SOURCES))
    parser.add_argument('archive', type=Path)
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[2])
    args = parser.parse_args()
    import_snapshot(args.kind, args.archive, args.workspace.resolve(strict=True))


if __name__ == '__main__':
    main()

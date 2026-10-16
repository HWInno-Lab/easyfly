#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Record the reviewed daytime racetrack snapshot, without vendoring model payloads.

Only reads a local, SHA-locked upstream tarball. No upstream scripts are run.
Runtime preparation uses the installed scene_catalog fetch command separately.
"""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import tarfile
import xml.etree.ElementTree as ET

import yaml


REVISION = 'ea90fdd9bac48344198f7cb3031dcb418a741064'
ARCHIVE_SHA256 = 'd788931a99d83fd0064559a7a97a505cca0694e0c8ab9b728c1ab5e4f3fb09ca'
REPOSITORY = 'https://github.com/aws-robotics/aws-robomaker-racetrack-world'
PREFIX = 'aws-robomaker-racetrack-world-' + REVISION
SCENE = 'racetrack_aws_day_v1'
ASSET = 'aws_racetrack_day_v1'


class Dumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def sha(data):
    return hashlib.sha256(data).hexdigest()


def reviewed_files(archive_path):
    if sha(archive_path.read_bytes()) != ARCHIVE_SHA256:
        raise ValueError('reviewed racetrack archive SHA256 required')
    with tarfile.open(archive_path) as archive:
        entries = archive.getmembers()
        files, names = {}, set()
        for entry in entries:
            path = PurePosixPath(entry.name)
            if (path.is_absolute() or '..' in path.parts or '\\' in entry.name
                    or path.parts[0] != PREFIX or entry.name in names
                    or not (entry.isdir() or entry.isfile())):
                raise ValueError('unsafe or duplicate archive member: ' + entry.name)
            names.add(entry.name)
            if entry.isfile():
                files[path.relative_to(PREFIX).as_posix()] = entry
        declarations = sorted(n for n in files if any(
            word in PurePosixPath(n).name.lower() for word in ('license', 'copying', 'notice')))
        if declarations != ['LICENSE']:
            raise ValueError('license declaration inventory changed')
        license_bytes = archive.extractfile(files['LICENSE']).read()
        world = archive.extractfile(files['worlds/racetrack_day.world']).read()
        document = ET.fromstring(world)
        if list(document.iter('plugin')):
            raise ValueError('unreviewed world plugin')
        uris = {node.text for node in document.iter('uri')}
        if not uris or any(not uri.startswith('model://') for uri in uris):
            raise ValueError('unreviewed world resource')
        models = sorted(uri.removeprefix('model://') for uri in uris)
        if len(models) != 14 or any('/' in name for name in models):
            raise ValueError('reviewed fourteen-model closure required')
        inventory, mapping, origins = [], [], []
        for source, entry in sorted(files.items()):
            parts = PurePosixPath(source).parts
            if len(parts) < 3 or parts[0] != 'models' or parts[1] not in models:
                continue
            before = archive.extractfile(entry).read()
            after, replacements = before, []
            if source.endswith('.sdf'):
                old, new = b'file://models/', b'model://'
                if old in after:
                    replacements.append(dict(old=old.decode(), new=new.decode(), count=after.count(old)))
                    after = after.replace(old, new)
                root = ET.fromstring(after)
                if list(root.iter('plugin')):
                    raise ValueError('unreviewed model plugin')
                for uri in root.iter('uri'):
                    if not uri.text.startswith('model://') or uri.text[8:].split('/')[0] not in models:
                        raise ValueError('resource outside reviewed model closure')
            target = source.removeprefix('models/')
            inventory.append(dict(path=target, sha256=sha(after)))
            mapping.append(dict(source=source, target=target, source_sha256=sha(before),
                                replacements=replacements))
            origins.append(dict(source=source, target='cache:models/' + target,
                                source_sha256=sha(before), sha256=sha(after), replacements=replacements))
        origins.append(dict(source='worlds/racetrack_day.world',
                            target='worlds/outdoor/urban/' + SCENE + '.world',
                            source_sha256=sha(world), sha256=sha(world), replacements=[]))
        spec = dict(format='tar.gz', urls=[
            'https://codeload.github.com/aws-robotics/aws-robomaker-racetrack-world/tar.gz/' + REVISION],
            sha256=ARCHIVE_SHA256, size_bytes=archive_path.stat().st_size, prefix=PREFIX,
            max_members=len(entries), max_unpacked_bytes=sum(e.size for e in entries),
            license=dict(source='LICENSE', sha256=sha(license_bytes)), members=mapping)
    provenance = dict(status='recorded', source_url=REPOSITORY, revision=REVISION,
                      declared_license='MIT-0', evidence=[
                          dict(package='easyfly_gazebo_models', path='licenses/aws_racetrack/' + name)
                          for name in ['LICENSE', 'origins.json']],
                      note='Pinned official fourteen-model daytime closure. Only SDF resource URI '
                           'prefixes relocated; mesh, collision, texture and world bytes retained.')
    asset = dict(schema_version=1, asset_set_id=ASSET,
                 model_root=dict(external='asset_cache', path='models'), provenance=provenance,
                 models=[dict(name=name, sdf='model.sdf', provenance=provenance) for name in models],
                 files=inventory, storage='cached', archive=spec,
                 local_changes=['SDF file://models/ prefixes changed to model:// for explicit '
                                'cache-root resolution; all meshes, textures and world bytes retained.'],
                 inventory_note='Complete fourteen-model daytime dependency closure, prepared only by '
                                'explicit fetch into a versioned external cache; no model payload installed.')
    record = dict(source_url=REPOSITORY, revision=REVISION, declared_license='MIT-0',
                  archive_sha256=ARCHIVE_SHA256, archive_bytes=spec['size_bytes'],
                  license_sha256=sha(license_bytes), license_declarations=declarations,
                  selected_world='worlds/racetrack_day.world', selected_models=models,
                  omitted_models=sorted({PurePosixPath(n).parts[1] for n in files
                                         if n.startswith('models/')} - set(models)),
                  scope='Daytime world only. Night and empty variants are not registered. '
                        'No upstream code or dynamic download scripts are executed.', files=origins)
    return {
        'src/simulation/easyfly_gazebo_models/manifests/' + ASSET + '.yaml':
            yaml.dump(asset, Dumper=Dumper, sort_keys=False).encode(),
        'src/simulation/easyfly_gazebo_models/licenses/aws_racetrack/LICENSE': license_bytes,
        'src/simulation/easyfly_gazebo_models/licenses/aws_racetrack/origins.json':
            (json.dumps(record, indent=2) + '\n').encode(),
        'src/simulation/easyfly_gazebo_worlds/worlds/outdoor/urban/' + SCENE + '.world': world,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    outputs = reviewed_files(args.archive)
    for name, data in outputs.items():
        target = args.workspace / name
        if args.check:
            if target.read_bytes() != data:
                raise ValueError('recorded import differs: ' + name)
        elif target.exists():
            raise ValueError('refusing to overwrite: ' + name)
    if not args.check:
        for name, data in outputs.items():
            target = args.workspace / name
            target.parent.mkdir(parents=True, exist_ok=True)
            with target.open('xb') as stream:
                stream.write(data)
    print(json.dumps(dict(scene=SCENE, files=len(outputs), check=args.check,
                          archive_sha256=ARCHIVE_SHA256)))


if __name__ == '__main__':
    main()

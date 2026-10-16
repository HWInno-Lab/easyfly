#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Lock a declarative cache recipe against the already reviewed S3 house snapshot."""
import argparse
import hashlib
from pathlib import Path
import re
import tarfile
import json
import yaml

REVISION = 'ff9631ca6d1db9c1ba656498151464b5ab74aafe'
ARCHIVE_SHA256 = 'e459bd9d7bdabdfc40f8afc6770ceb1d774316e5da94a1048f036baa7388b2d9'


class Dumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    if sha(args.archive.read_bytes()) != ARCHIVE_SHA256:
        raise ValueError('reviewed S3 source archive required')
    models = args.workspace / 'src/simulation/easyfly_gazebo_models'
    path = models / 'manifests/aws_house_v1.yaml'
    asset = yaml.safe_load(path.read_text())
    origins = json.loads((models / 'licenses/aws_house/origins.json').read_text())
    if asset['provenance']['revision'] != REVISION or origins['archive_sha256'] != ARCHIVE_SHA256:
        raise ValueError('reviewed S3 house manifest/origins required')
    prefix = 'aws-robomaker-small-house-world-' + REVISION
    records = {r['path'].removeprefix('third_party/aws/house/'): r
               for r in origins['files'] if r['path'].startswith('third_party/aws/house/')}
    if set(records) != {f['path'] for f in asset['files']}:
        raise ValueError('origin and locked model inventories differ')
    mapping = []
    with tarfile.open(args.archive) as archive:
        entries = archive.getmembers()
        for item in asset['files']:
            record = records[item['path']]
            before = archive.extractfile(prefix + '/' + record['origin']).read()
            if sha(before) != record['upstream_sha256']:
                raise ValueError('origin member hash differs')
            after = before
            changes = []
            if item['path'].endswith('.sdf') and b'file://models/' in after:
                changes.append(dict(old='file://models/', new='model://', count=after.count(b'file://models/')))
                after = after.replace(b'file://models/', b'model://')
            if item['path'].lower().endswith('.dae'):
                for old in sorted(set(re.findall(rb'(?:\.\./)+photos/[A-Za-z0-9_]+\.jpg', after))):
                    new = b'../materials/textures/' + old.rsplit(b'/', 1)[1]
                    changes.append(dict(old=old.decode(), new=new.decode(), count=after.count(old)))
                    after = after.replace(old, new)
            actual = models / 'third_party/aws/house' / item['path']
            if sha(after) != item['sha256'] or sha(actual.read_bytes()) != item['sha256']:
                raise ValueError('declarative relocation differs from the reviewed model bytes')
            mapping.append(dict(source=record['origin'], target=item['path'],
                                source_sha256=sha(before), replacements=changes))
        license_bytes = archive.extractfile(prefix + '/LICENSE').read()
        if license_bytes != (models / 'licenses/aws_house/LICENSE').read_bytes():
            raise ValueError('original license differs')
        spec = dict(format='tar.gz', urls=[
            'https://codeload.github.com/aws-robotics/aws-robomaker-small-house-world/tar.gz/' + REVISION],
            sha256=ARCHIVE_SHA256, size_bytes=args.archive.stat().st_size, prefix=prefix,
            max_members=len(entries), max_unpacked_bytes=sum(e.size for e in entries),
            license=dict(source='LICENSE', sha256=sha(license_bytes)), members=mapping)
    if args.check:
        if asset.get('archive') != spec:
            raise ValueError('recorded archive recipe differs')
    else:
        if 'archive' in asset:
            raise ValueError('refusing to overwrite an existing archive recipe')
        asset['archive'] = spec
        path.write_text(yaml.dump(asset, Dumper=Dumper, sort_keys=False))
    print(json.dumps(dict(locked_members=len(mapping), archive_bytes=spec['size_bytes'],
                          max_unpacked_bytes=spec['max_unpacked_bytes'], check=args.check)))


if __name__ == '__main__':
    main()

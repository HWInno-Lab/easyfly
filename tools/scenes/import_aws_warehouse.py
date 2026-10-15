#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Import the reviewed AWS snapshot from a local archive; never download or execute it."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import tempfile
import xml.etree.ElementTree as ET

REVISION = 'ee0af733315e78432408c3cd98d378ecee5f767c'
ARCHIVE_SHA256 = 'c536d4a027f42bdac1a15d751ad8fb37296e14871e5af909f01207a854002211'
SOURCE_URL = 'https://github.com/aws-robotics/aws-robomaker-small-warehouse-world'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[2])
    args = parser.parse_args()
    if digest(args.archive) != ARCHIVE_SHA256:
        raise ValueError('archive fingerprint mismatch; reviewed immutable snapshot required')
    workspace = args.workspace.resolve(strict=True)
    models = workspace/'src/simulation/easyfly_gazebo_models'
    worlds = workspace/'src/simulation/easyfly_gazebo_worlds'
    target = models/'third_party/aws/warehouse'
    world_target = worlds/'worlds/indoor/warehouse/warehouse_aws_v1.world'
    open_world_target = worlds/'worlds/indoor/warehouse/warehouse_aws_no_roof_v1.world'
    if target.exists() or world_target.exists() or open_world_target.exists():
        raise ValueError('refusing to overwrite an existing import')
    with tempfile.TemporaryDirectory(prefix='easyfly-warehouse-import-') as temp:
        root = Path(temp)
        with tarfile.open(args.archive) as archive:
            for entry in archive.getmembers():
                if not (root/entry.name).resolve().is_relative_to(root) or not (entry.isdir() or entry.isfile()):
                    raise ValueError(f'unsafe archive entry: {entry.name}')
            archive.extractall(root)
        source = root/f'aws-robomaker-small-warehouse-world-{REVISION}'
        original_world = source/'worlds/small_warehouse/small_warehouse.world'
        tree = ET.parse(original_world)
        used = sorted({node.text.removeprefix('model://') for node in tree.findall('.//include/uri')})
        records = []
        excluded_files = []
        target.mkdir(parents=True)
        for name in used:
            if '/' in name or not (source/'models'/name/'model.config').is_file():
                raise ValueError(f'invalid model dependency: {name}')
            shutil.copytree(source/'models'/name, target/name,
                            ignore=shutil.ignore_patterns('.DS_Store'))
            for original in sorted((source/'models'/name).rglob('*')):
                if not original.is_file():
                    continue
                if original.name == '.DS_Store':
                    excluded_files.append(original.relative_to(source).as_posix())
                    continue
                copied = target/name/original.relative_to(source/'models'/name)
                changed = []
                if copied.suffix == '.sdf':
                    content = copied.read_text()
                    modified = content.replace('file://models/', 'model://')
                    if content != modified:
                        copied.write_text(modified)
                        changed.append('file://models/ -> model:// for relocatable model references')
                records.append(dict(path=copied.relative_to(models).as_posix(),
                    origin=original.relative_to(source).as_posix(), upstream_sha256=digest(original),
                    sha256=digest(copied),changes=changed))
        # Preserve all collision geometry and layout. Add only explicit ROS clock
        # through the launcher; no upstream robot/control launch code is imported.
        world_target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(original_world, world_target)
        for package in [models,worlds]:
            license_dir=package/'licenses/aws_warehouse'
            license_dir.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source/'LICENSE',license_dir/'LICENSE')
        records.append(dict(path=world_target.relative_to(worlds).as_posix(),
            origin=original_world.relative_to(source).as_posix(),upstream_sha256=digest(original_world),
            sha256=digest(world_target),changes=[]))
        open_world = source/'worlds/no_roof_small_warehouse/no_roof_small_warehouse.world'
        shutil.copyfile(open_world, open_world_target)
        records.append(dict(path=open_world_target.relative_to(worlds).as_posix(),
            origin=open_world.relative_to(source).as_posix(),upstream_sha256=digest(open_world),
            sha256=digest(open_world_target),changes=[]))
        evidence=dict(source_url=SOURCE_URL,revision=REVISION,archive_sha256=ARCHIVE_SHA256,
            declared_license='MIT-0',included_models=used,
            excluded_models=['aws_robomaker_warehouse_DeskC_01 (not referenced by selected world)'],
            excluded_files=excluded_files,files=records)
        (models/'licenses/aws_warehouse/origins.json').write_text(json.dumps(evidence,indent=2)+'\n')
        print(f'imported {len(used)} models, {len(records)} world/model files; revision={REVISION}')


if __name__ == '__main__':
    main()

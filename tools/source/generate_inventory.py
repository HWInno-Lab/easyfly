#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Inventory actual packages and delivered files without inferring license grants."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


def sha(path):
    """Stream large binaries rather than reading an entire payload into memory."""
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def files(root):
    """Record links without following them outside the declared delivery root."""
    result = {}
    for directory, names, leaves in os.walk(root, followlinks=False):
        for name in sorted(names + leaves):
            path = Path(directory) / name
            relative = str(path.relative_to(root))
            if path.is_symlink():
                result[relative] = {'link': os.readlink(path)}
            elif path.is_file():
                result[relative] = {
                    'sha256': sha(path), 'size': path.stat().st_size,
                    'mode': oct(path.stat().st_mode & 0o7777),
                }
    return result


def main():
    """Generate a local inventory; it is not a legal or SPDX certification."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path,
                        default=Path(__file__).resolve().parents[2])
    parser.add_argument('--packages', type=Path, required=True,
                        help='colcon list --base-paths src TSV from this source root')
    parser.add_argument('--install', type=Path, required=True)
    parser.add_argument('--inputs', type=Path)
    parser.add_argument('--system-packages', action='store_true',
                        help='Record dpkg packages of the current execution environment')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source = args.source.resolve(strict=True)
    packages = []
    for line in args.packages.read_text().splitlines():
        name, relative, build_type = line.split('\t')
        directory = (source / relative).resolve(strict=True)
        if not directory.is_relative_to(source):
            raise RuntimeError('package path outside source root')
        manifest = directory / 'package.xml'
        xml = ET.parse(manifest).getroot() if manifest.is_file() else None
        licenses = [entry.text for entry in xml.findall('license')] if xml is not None else []
        metadata = manifest if xml is not None else directory / 'CMakeLists.txt'
        packages.append({
            'name': name, 'version': xml.findtext('version') if xml is not None else None,
            'build_type': build_type, 'path': str(directory.relative_to(source)),
            'metadata_file': metadata.name, 'manifest_sha256': sha(metadata),
            'declared_licenses': licenses,
            'license_metadata_unresolved': not licenses or any(
                not value or value.strip().upper() in ('TODO', 'UNKNOWN') for value in licenses),
            'dependencies': [{'kind': entry.tag, 'name': entry.text}
                             for entry in xml if entry.tag.endswith('depend')]
                            if xml is not None else [],
        })
    provenance = {}
    for path in sorted((source / 'src/third_party').rglob('*.provenance.json')):
        provenance[str(path.relative_to(source))] = {
            'sha256': sha(path), 'record': json.loads(path.read_text()),
        }
    result = {
        'schema_version': 1,
        'scope': 'Actual local source/install/input inventory; declared licenses are not new grants.',
        'packages': packages, 'provenance': provenance,
        'installed_files': files(args.install.resolve(strict=True)),
        'input_files': files(args.inputs.resolve(strict=True)) if args.inputs else {},
        'system_packages': [],
    }
    if args.system_packages:
        data = subprocess.check_output(
            ['dpkg-query', '-W', '-f=${Package}\t${Version}\t${Architecture}\n'], text=True)
        for line in data.splitlines():
            name, version, architecture = line.split('\t')
            notice = Path('/usr/share/doc') / name / 'copyright'
            result['system_packages'].append({
                'name': name, 'version': version, 'architecture': architecture,
                'copyright_path': str(notice) if notice.is_file() else None,
                'copyright_sha256': sha(notice) if notice.is_file() else None,
            })
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    print(json.dumps({'packages': len(packages),
                      'installed_files': len(result['installed_files']),
                      'system_packages': len(result['system_packages'])}))


if __name__ == '__main__':
    main()

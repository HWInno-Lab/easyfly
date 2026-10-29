#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Verify delivered original files, the exact patch, and complete reconstruction."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[2]
COMPONENTS = {
    'ego_planner': 'algorithms/planning',
    'fast_lio': 'algorithms/localization',
    'livox_sdk2': 'sensor_sdks',
}


def digest(path):
    """Hash actual bytes, including both reconstructed and current sources."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    """Require a new output directory and reject any mismatched input."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('component', choices=COMPONENTS)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    base = ROOT / 'src/third_party' / COMPONENTS[args.component]
    inputs = base / (args.component + '.sources')
    meta = json.loads((inputs / 'manifest.json').read_text())
    for field in ('original_archive', 'patch'):
        if digest(inputs / meta[field]['path']) != meta[field]['sha256']:
            raise RuntimeError('input hash mismatch: ' + field)
    output = args.output.resolve()
    output.mkdir(parents=True)
    reconstructed = output / 'reconstructed'
    reconstructed.mkdir()
    with tarfile.open(inputs / meta['original_archive']['path']) as archive:
        seen = set()
        for member in archive:
            relative = Path(member.name)
            if (not member.isfile() or relative.is_absolute() or
                    '..' in relative.parts or member.name in seen):
                raise RuntimeError('unsupported archive entry')
            seen.add(member.name)
            path = reconstructed / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(archive.extractfile(member).read())
            if digest(path) != meta['original_archive']['files'][member.name]:
                raise RuntimeError('original source mismatch: ' + member.name)
        if seen != set(meta['original_archive']['files']):
            raise RuntimeError('original source inventory mismatch')
    patch = inputs / meta['patch']['path']
    if patch.stat().st_size:
        result = subprocess.run(
            ['patch', '--batch', '--fuzz=0', '-p1', '-i', str(patch)],
            cwd=reconstructed, capture_output=True, text=True, check=True)
        (output / 'patch.log').write_text(result.stdout + result.stderr)
    actual = {str(p.relative_to(reconstructed)): digest(p)
              for p in reconstructed.rglob('*') if p.is_file()}
    if actual != meta['files']:
        raise RuntimeError('reconstructed source inventory mismatch')
    for relative, expected in meta['files'].items():
        if digest(ROOT / meta['source_root'] / relative) != expected:
            raise RuntimeError('workspace source mismatch: ' + relative)
    result = {'passed': True, 'component': args.component,
              'files': len(actual), 'zero_fuzz': True,
              'license_limitations': meta['limitations']}
    (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()

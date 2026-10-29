#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Restore the exact RealSense gitlink from the shipped, self-contained Git bundle."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / 'src/third_party/drivers/realsense-ros.provenance.json'


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def run(*arguments):
    # Only the local bundle is used; external transports and hooks are disabled.
    environment = {k: v for k, v in os.environ.items() if not k.startswith('GIT_')}
    environment['GIT_ALLOW_PROTOCOL'] = 'file'
    return subprocess.check_output(
        ['git', '-c', 'core.hooksPath=/dev/null', '-c',
         'advice.detachedHead=false', *map(str, arguments)],
        env=environment, text=True).strip()


def verify(destination, manifest):
    if run('-C', destination, 'rev-parse', '--show-toplevel') != str(destination):
        raise RuntimeError('destination is not its own repository; left unchanged')
    if run('-C', destination, 'rev-parse', 'HEAD') != manifest['gitlink_revision']:
        raise RuntimeError('existing revision differs; left unchanged')
    if run('-C', destination, 'status', '--porcelain', '--untracked-files=all'):
        raise RuntimeError('existing working tree has changes; left unchanged')
    if run('-C', destination, 'rev-parse', 'HEAD^{tree}') != manifest['tree']:
        raise RuntimeError('unexpected source tree')


def restore(destination, manifest):
    bundle = ROOT / manifest['bundle']['path']
    if sha256(bundle) != manifest['bundle']['sha256']:
        raise RuntimeError('bundle SHA256 mismatch; destination was not changed')
    if destination.is_symlink():
        raise RuntimeError('destination symlink is not supported; left unchanged')
    if destination.exists() and any(destination.iterdir()):
        verify(destination, manifest)
        return False
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.realsense-restore-',
                                     dir=destination.parent) as temporary:
        staged = Path(temporary) / 'source'
        run('clone', '--no-hardlinks', bundle, staged)
        verify(staged, manifest)
        run('-C', staged, 'fsck', '--full')
        run('-C', staged, 'remote', 'set-url', 'origin', manifest['upstream_url'])
        # os.rename replaces only an empty destination directory, never its files.
        staged.rename(destination)
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--destination', type=Path,
                        default=ROOT / 'src/communication/c1_drivers/realsense_ros')
    args = parser.parse_args()
    destination = args.destination.absolute()
    manifest = json.loads(MANIFEST.read_text())
    created = restore(destination, manifest)
    print(json.dumps({'passed': True, 'created': created,
                      'destination': str(destination),
                      'revision': manifest['gitlink_revision'], 'network': False}))


if __name__ == '__main__':
    main()

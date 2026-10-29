#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Package a prepared Ubuntu userspace without build trees or mutable runtime state."""
import argparse
import json
import os
from pathlib import Path
import tarfile

from manage_bundle import digest, identity


EXCLUDED = (
    'dev', 'proc', 'sys', 'run', 'tmp', 'root', 'home',
    'var/cache', 'var/log', 'var/tmp', 'var/lib/apt/lists', 'var/lib/easyfly',
    'opt/easyfly/build', 'opt/easyfly/build-pangolin',
    'opt/easyfly/pangolin-staging',
    'opt/easyfly/source/docs', 'opt/easyfly/source/artifacts',
    'opt/easyfly/log-sdk', 'opt/easyfly/log-workspace',
)


def excluded(relative, directory=False):
    """Keep exclusions explicit and anchored to the prepared container root."""
    return (any(relative == name or relative.startswith(name + '/') for name in EXCLUDED)
            or relative.startswith('opt/easyfly/log-')
            or (relative.startswith('opt/easyfly/build-') and
                (directory or '/' in relative[len('opt/easyfly/'):]))
            or (relative.startswith('opt/easyfly/source/') and
                any(part in ('__pycache__', '.pytest_cache') for part in Path(relative).parts)))


def main():
    """Produce an archive hash and platform contract; do not claim source authenticity."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rootfs', required=True, type=Path)
    parser.add_argument('--release-id', required=True)
    parser.add_argument('--archive', required=True, type=Path)
    parser.add_argument('--manifest', required=True, type=Path)
    args = parser.parse_args()
    release = identity(args.release_id)
    root = args.rootfs.resolve(strict=True)
    if args.archive.exists() or args.manifest.exists():
        parser.error('output files must be new')
    if args.archive.absolute().is_relative_to(root):
        parser.error('archive must be outside the input root')
    entry = root / 'opt/easyfly/bin/environment'
    if entry.is_symlink() or not entry.is_file() or not os.access(entry, os.X_OK):
        parser.error('prepared rootfs lacks an executable environment entry point')
    supervisor = entry.with_name('runtime_session.py')
    if supervisor.is_symlink() or not supervisor.is_file():
        parser.error('prepared rootfs lacks the runtime supervisor')
    count = 0
    with tarfile.open(args.archive, 'x:gz', compresslevel=1, dereference=False) as archive:
        # Read-only runtime roots still need destinations for the writable mounts.
        for name in ('dev', 'proc', 'sys', 'run', 'tmp', 'root', 'home',
                     'home/easyfly', 'var/tmp', 'var/lib/easyfly'):
            entry = tarfile.TarInfo('rootfs/' + name)
            entry.type = tarfile.DIRTYPE
            entry.mode = 0o755
            archive.addfile(entry)
            count += 1
        for directory, names, leaves in os.walk(root, followlinks=False):
            names.sort()
            leaves.sort()
            names[:] = [name for name in names
                        if not excluded(str((Path(directory) / name).relative_to(root)), True)]
            for name in names + leaves:
                path = Path(directory) / name
                relative = str(path.relative_to(root))
                if excluded(relative, name in names):
                    continue
                if not (path.is_symlink() or path.is_file() or path.is_dir()):
                    raise ValueError('special file in runtime payload: ' + relative)
                # Installed hard links are serialized independently: extraction never follows them.
                archive.inodes.clear()
                archive.add(path, arcname='rootfs/' + relative, recursive=False)
                count += 1
    result = {
        'format': 1, 'release_id': release, 'archive_sha256': digest(args.archive),
        'archive_size': args.archive.stat().st_size, 'archive_entries': count,
        'platform': 'Ubuntu 22.04 amd64 host; Python 3, bubblewrap, nsenter, user namespaces, X11 for GUI',
        'runtime': 'Bundled userspace, isolated offline network, Mesa CPU rendering',
        'state': 'prefix/state is retained across upgrades and uninstall; no schema rewrite',
        'exclusions': list(EXCLUDED),
        'integrity_scope': 'SHA-256 detects payload changes; it is not a publisher signature.',
    }
    with args.manifest.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()

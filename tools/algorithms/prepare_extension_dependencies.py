#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Acquire the already pinned NLopt packages into a NEW private prefix.

Downloads exact Ubuntu package versions and verifies the existing source-recorded
SHA256 values before extraction. Never installs or upgrades a system package.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    archives = out/'archives'
    archives.mkdir()
    record = json.loads((ROOT/'src/third_party/algorithms/planning/fuel.notices/NLopt-build-dependency.json').read_text())
    requested = []
    for package in record['packages']:
        control = dict(line.split(': ', 1) for line in package['control'].splitlines())
        requested.append(control['Package']+'='+control['Version'])
    commands = []
    for label, command in [('uris', ['apt-get', '--print-uris', 'download', *requested]),
                           ('download', ['apt-get', 'download', *requested])]:
        with (out/(label+'.log')).open('w') as log:
            code = subprocess.call(command, cwd=archives, stdout=log, stderr=subprocess.STDOUT)
        commands.append({'command': command, 'returncode': code})
        (out/'commands.json').write_text(json.dumps(commands, indent=2)+'\n')
        if code:
            raise SystemExit(code)
    for package in record['packages']:
        archive = archives/package['file']
        actual = hashlib.sha256(archive.read_bytes()).hexdigest()
        if actual != package['sha256']:
            raise ValueError('archive hash mismatch: '+package['file'])
    prefix = out/'prefix'
    for package in record['packages']:
        subprocess.run(['dpkg-deb', '-x', str(archives/package['file']), str(prefix)], check=True)
    (out/'summary.json').write_text(json.dumps({'passed': True, 'record': record,
        'prefix': str(prefix/'usr'), 'runtime_library_directory': str(prefix/'usr/lib/x86_64-linux-gnu'),
        'system_packages_changed': False, 'commands': commands}, indent=2)+'\n')
    print(prefix/'usr')


if __name__ == '__main__':
    main()

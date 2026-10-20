#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Reconstruct and verify the selected OpenVINS source against a pristine pinned Git checkout."""

import argparse
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile
import tempfile


ROOT = Path(__file__).resolve().parents[2]


def hashes(directory):
    return {p.relative_to(directory).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in directory.rglob('*') if p.is_file()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upstream', required=True, type=Path)
    args = parser.parse_args()
    metadata = ROOT / 'src/third_party/algorithms/localization/open_vins.provenance.json'
    record = json.loads(metadata.read_text())
    upstream = args.upstream.resolve(strict=True)
    revision = subprocess.check_output(['git', '-C', str(upstream), 'rev-parse', 'HEAD'],
                                       text=True).strip()
    assert revision == record['exact_upstream_revision'], 'unexpected upstream revision'
    assert not subprocess.check_output(['git', '-C', str(upstream), 'status', '--porcelain'],
                                       text=True).strip(), 'upstream checkout is modified'
    archive = subprocess.check_output(['git', '-C', str(upstream), 'archive', '--format=tar',
                                       revision, *record['selected_roots']])
    assert hashlib.sha256(archive).hexdigest() == record['acquisition']['sha256']
    with tempfile.TemporaryDirectory(prefix='easyfly-openvins-source-') as temporary:
        directory = Path(temporary)
        with tarfile.open(fileobj=io.BytesIO(archive)) as tar:
            for member in tar.getmembers():
                assert not member.issym() and not member.islnk()
                assert not member.name.startswith('/') and '..' not in Path(member.name).parts
            tar.extractall(directory)
        before = hashes(directory)
        assert before == {p: v['upstream_sha256'] for p, v in record['files'].items()}
        for item in record['patches']:
            patch = ROOT / item['path']
            assert hashlib.sha256(patch.read_bytes()).hexdigest() == item['sha256']
            subprocess.run(['git', 'apply', str(patch)], cwd=directory,
                           check=True)
        after = hashes(directory)
        assert after == {p: v['current_sha256'] for p, v in record['files'].items()}
        assert after == hashes(ROOT / record['current_path']), 'imported files drifted'
    print('PASS pinned OpenVINS runtime source, patch order and all file identities')


if __name__ == '__main__':
    main()

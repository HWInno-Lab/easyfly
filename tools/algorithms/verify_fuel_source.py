#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Reconstruct all native port bytes from pinned selected originals and zero-fuzz patch."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile

ROOT=Path(__file__).resolve().parents[2]


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();out=args.output.resolve();out.mkdir()
    provenance=ROOT/'src/third_party/algorithms/planning/fuel.provenance.json'
    meta=json.loads(provenance.read_text());base=provenance.parent
    for name in ['original_archive','port_patch']:
        path=base/meta[name]['path'];assert digest(path)==meta[name]['sha256']
    with tarfile.open(base/meta['original_archive']['path']) as archive:
        for entry in archive.getmembers():
            assert not Path(entry.name).is_absolute() and '..' not in Path(entry.name).parts
        archive.extractall(out/'reconstructed',filter='data')
    run=subprocess.run(['patch','--batch','--fuzz=0','-p1','-i',str(base/meta['port_patch']['path'])],
        cwd=out/'reconstructed',capture_output=True,text=True)
    (out/'patch.log').write_text(run.stdout+run.stderr);assert run.returncode==0
    for relative,sha in meta['files'].items():
        assert digest(out/'reconstructed'/relative)==sha
        assert digest(base/'fuel'/relative)==sha
    actual={str(f.relative_to(out/'reconstructed')) for f in (out/'reconstructed').rglob('*') if f.is_file()}
    assert actual==set(meta['files'])
    (out/'result.json').write_text(json.dumps(dict(passed=True,files=len(actual),commit=meta['commit'],zero_fuzz=True),indent=2)+'\n')


if __name__=='__main__':main()

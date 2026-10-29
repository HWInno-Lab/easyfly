#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Record the reviewed high-level port; use a fresh evidence directory per revision."""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile

ROOT=Path(__file__).resolve().parents[2]
COMMIT='662dd23c7b52b258d3c4a0155ff6632118e8984f'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upstream',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();out=args.output.resolve();out.mkdir()
    upstream=args.upstream.resolve();native=ROOT/'src/third_party/algorithms/planning/fuel'
    assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=upstream,text=True).strip()==COMMIT
    assert not subprocess.check_output(['git','status','--porcelain'],cwd=upstream,text=True).strip()
    base=native.parent;files={};changed=[];selected=[];diff=[]
    archive=out/'fuel.original-selected.tar.gz';patch=out/'fuel.ros2-highlevel.patch'
    with tarfile.open(archive,'w:gz') as tar:
        for path in sorted(native.rglob('*')):
            if not path.is_file():continue
            relative=str(path.relative_to(native));source=upstream/relative
            new=path.read_bytes();old=source.read_bytes() if source.is_file() else b''
            files[relative]=hashlib.sha256(new).hexdigest()
            if source.is_file():
                tar.add(source,arcname=relative);selected.append(relative)
            if old!=new:
                changed.append(relative);diff.extend(difflib.unified_diff(old.decode().splitlines(keepends=True),
                    new.decode().splitlines(keepends=True),fromfile='a/'+relative if source.is_file() else '/dev/null',tofile='b/'+relative))
    patch.write_text(''.join(diff))
    meta=dict(repository='https://github.com/HKUST-Aerial-Robotics/FUEL',commit=COMMIT,
        original_archive=dict(path=archive.name,sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),selected_files=len(selected)),
        port_patch=dict(path=patch.name,sha256=hashlib.sha256(patch.read_bytes()).hexdigest(),changed_files=changed),files=files,
        scope='Native incremental FIS/global bundled LKH/local viewpoints; existing GCOPTER motion, static bounded Gazebo only.',
        transport='YAML configuration and real steady clock; actual ROS2 worker/action outside native library, no emulated ROS1 graph.',
        licenses=dict(fuel='retained root GPLv3',lkh='upstream package.xml TODO, retained component notices; see fuel.notices/README.md'))
    metadata=out/'fuel.provenance.json';metadata.write_text(json.dumps(meta,indent=2)+'\n')
    for source in [archive,patch,metadata]:shutil.copy2(source,base/source.name)


if __name__=='__main__':main()

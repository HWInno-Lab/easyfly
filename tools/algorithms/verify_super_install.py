#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Consume the installed full SUPER kernel and inspect actual loaded libraries.

ROS, PCL, Eigen and the declared installed underlays remain dependencies.
The consumer uses no workspace source include and creates no ROS node.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
LIBRARIES = ['libsuper_validation.so', 'libsuper_execution_core.so',
             'libeasyfly_super_planner.so', 'libeasyfly_rog_map.so']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--request', type=Path, required=True)
    parser.add_argument('--mode', choices=['copy','symlink','relocated'], required=True)
    parser.add_argument('--build-base', type=Path)
    parser.add_argument('--relocated-from', type=Path)
    args = parser.parse_args()
    prefix = args.prefix.resolve(strict=True)
    worker=prefix/'easyfly_super_planner/lib/easyfly_super_planner/super_worker'
    if args.mode=='symlink':
        assert args.build_base is not None
        for package in ['rog_map','super_planner','easyfly_super_planner']:
            cache=(args.build_base/package/'CMakeCache.txt').read_text()
            assert 'AMENT_CMAKE_SYMLINK_INSTALL:BOOL=1' in cache, package
        assert worker.is_symlink(), 'actual wrapper executable must be symlink-installed'
    else:
        assert not worker.is_symlink()
    if args.mode=='relocated':
        assert args.relocated_from is not None and not args.relocated_from.exists()
        assert args.relocated_from.absolute()!=prefix
    args.output.mkdir(parents=True, exist_ok=False)
    out = args.output.resolve()
    shutil.copytree(ROOT/'tools/algorithms/super_consumer',out/'consumer-source')
    selected = [str(prefix/p) for p in ['easyfly_super_planner','super_planner','rog_map']]
    env = dict(os.environ)
    for key in ['AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH']:
        env[key] = ':'.join(selected+env.get(key,'').split(':'))
    env['LD_LIBRARY_PATH'] = ':'.join(str(Path(p)/'lib') for p in selected)+':'+env.get('LD_LIBRARY_PATH','')
    for index, command in enumerate([
        ['cmake','-S',str(out/'consumer-source'),'-B',str(out/'build'),
         '-DCMAKE_PREFIX_PATH='+env['CMAKE_PREFIX_PATH'].replace(':',';')],
        ['cmake','--build',str(out/'build'),'--parallel','2']]):
        with (out/f'{index}.log').open('x') as log:
            subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    commands = json.loads((out/'build/compile_commands.json').read_text())
    assert all('/src/third_party/' not in c['command'] and
               '/src/execution/' not in c['command'] for c in commands)
    shutil.copy2(args.request,out/'original.request')
    with (out/'runtime.log').open('x') as log:
        subprocess.run([str(out/'build/consumer'),str(out/'original.request'),str(out/'result.septic')],
                       cwd=out,env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=5)
    runtime = (out/'runtime.log').read_text()
    assert 'CONSUMER_PASS' in runtime
    libraries = {}
    for library in LIBRARIES:
        paths = {line.split()[-1] for line in runtime.splitlines()
                 if line.startswith('LOADED ') and line.endswith('/'+library)}
        assert len(paths)==1, (library,paths)
        path=Path(paths.pop())
        # /proc reports resolved files; ament symlink installs may load their
        # exact build products. Record that distinction instead of claiming
        # the symlink path is a separate copy.
        matches=[p for p in prefix.glob('*/lib/'+library)]
        assert len(matches)==1 and matches[0].resolve()==path, (library,path)
        libraries[library]=dict(installed_path=str(matches[0]),loaded_realpath=str(path),
            installed_is_symlink=matches[0].is_symlink(),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    report=dict(passed=True,mode=args.mode,prefix=str(prefix),libraries=libraries,
        consumer='actual original request through full native PlanFromRest and all wrapper hard guards',
        source_includes=False,ros_node_created=False,hardware=False,
        dependencies='declared installed ROS/PCL/Eigen/yaml/GCOPTER Grid header underlays',
        wrapper_executable_is_symlink=worker.is_symlink(),
        relocated_from=str(args.relocated_from.absolute()) if args.relocated_from else None,
        verifier_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        request_sha256=hashlib.sha256(args.request.read_bytes()).hexdigest(),compile_commands=commands)
    (out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='compile_commands'},indent=2))


if __name__=='__main__':
    main()

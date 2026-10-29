#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Read actual ELF mappings of planner descendants of one exact active probe."""
import argparse
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[3]


def identity(proc):
    fields=(proc/'stat').read_text().rsplit(')',1)[1].split()
    return dict(ppid=int(fields[1]),start_ticks=int(fields[19]))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',type=Path,required=True)
    parser.add_argument('--strategy',choices=['ego','super'],default='ego')
    args=parser.parse_args()
    case=args.case.resolve(strict=True)
    entries={};roots=[]
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            record=identity(proc);entries[int(proc.name)]=record
            argv=(proc/'cmdline').read_bytes().decode().strip('\0').split('\0')
            if len(argv)<2 or '--output-dir' not in argv:
                continue
            cwd=(proc/'cwd').resolve(strict=True)
            if (cwd/argv[1]).resolve()!=Path(__file__).resolve().with_name('super_simulation.py'):
                continue
            if (cwd/argv[argv.index('--output-dir')+1]).resolve()==case:
                roots.append(int(proc.name))
        except (OSError,ValueError,UnicodeDecodeError):
            continue
    assert len(roots)==1, roots
    root=roots[0];owned={root}
    while True:
        extra={pid for pid,r in entries.items() if r['ppid'] in owned}-owned
        if not extra:break
        owned.update(extra)
    selected=[]
    for pid in sorted(owned):
        proc=Path('/proc')/str(pid)
        try:
            executable=(proc/'exe').resolve(strict=True)
            if executable.name not in ['ego_planner_node','traj_server','super_navigation','super_worker']:
                continue
            before=identity(proc)
            libraries={}
            for line in (proc/'maps').read_text().splitlines():
                path=Path(line.split()[-1])
                if path.is_absolute() and str(path).startswith(str(ROOT)+'/') and '.so' in path.name and path.is_file():
                    libraries[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
            assert before==identity(proc)==entries[pid]
            selected.append(dict(pid=pid,**before,executable=str(executable),
                executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),libraries=libraries))
        except FileNotFoundError:
            continue
    assert selected, 'no actual owned planner observed'
    expected='ego_planner_node' if args.strategy=='ego' else 'super_navigation'
    assert any(Path(p['executable']).name==expected for p in selected), expected+' not ready'
    report=dict(passed=True,probe_pid=root,probe_start_ticks=entries[root]['start_ticks'],
        case=str(case),processes=selected,read_only=True,
        method='Exact running probe argv/output directory; /proc parent tree, PID/start ticks and actual mapped ELF paths',
        capture_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    output=case/'native-loaded-libraries.json'
    assert not output.exists()
    output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(passed=True,processes=[dict(pid=p['pid'],name=Path(p['executable']).name,libraries=list(p['libraries'])) for p in selected]),indent=2))


if __name__=='__main__':
    main()

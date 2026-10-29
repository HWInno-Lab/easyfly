#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Serial real missions, fresh directories and ports; retain and stop on failure."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[3]
R=ROOT/'artifacts/new-algorithms/new5-fuel-20261023'
CASES=[
    ('room','complete','fuel_room_v1','copy',True),
    ('warehouse','complete','fuel_warehouse_v1','symlink',True),
    ('cancel','cancel','fuel_room_v1','moved',True),
    ('map-loss','map-loss','fuel_room_v1','copy',True),
    ('worker-exit','worker-exit','fuel_room_v1','copy',True),
    ('worker-timeout','worker-timeout','fuel_room_v1','copy',True),
    ('timeout','timeout','fuel_room_v1','copy',True),
    ('no-fly','no-fly','fuel_room_v1','copy',True),
    ('land-takeover','land-takeover','fuel_room_v1','copy',True),
    ('unreachable','unreachable','fuel_sealed_v1','copy',False)]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--cases',nargs='+',default=[c[0] for c in CASES]);p.add_argument('--port-base',type=int,default=12640)
    p.add_argument('--execute-simulation',action='store_true',required=True);args=p.parse_args()
    out=args.output.resolve();out.mkdir();status=[]
    for i,name in enumerate(args.cases):
        case=next(c for c in CASES if c[0]==name);_,scenario,scene,mode,managed=case
        env=R/('environment.bash' if mode=='copy' else f'environment-{mode}.bash')
        cmd=[sys.executable,str(Path(__file__).with_name('fuel_simulation.py')),
            '--output-dir',str(out/name),'--px4-dir',str(ROOT/'artifacts/scene-s4-20261017/px4-mag-lpf-v2/px4-input'),
            '--truth-executable',str(ROOT/'artifacts/new-algorithms/openvins-sim-acceptance-20261020/native-long-build/gazebo_pose_receipts'),
            '--scenario',scenario,'--scene',scene,'--gazebo-port',str(args.port_base+i),'--execute-simulation']
        if managed:cmd.append('--managed-profile')
        entry=dict(name=name,scenario=scenario,scene=scene,installation=mode,managed=managed,running=True)
        status.append(entry);(out/'status.json').write_text(json.dumps(status,indent=2)+'\n')
        with (out/f'{name}.runner.log').open('x') as log:
            run=subprocess.run(['bash','-c','source '+shlex.quote(str(env))+' && exec '+shlex.join(cmd)],
                stdout=log,stderr=subprocess.STDOUT,cwd=ROOT)
        entry.update(running=False,simulation_returncode=run.returncode)
        if run.returncode==0:
            with (out/f'{name}.evaluation.log').open('x') as log:
                audit=subprocess.run([sys.executable,str(Path(__file__).with_name('evaluate_fuel.py')),str(out/name)],
                    stdout=log,stderr=subprocess.STDOUT,cwd=ROOT)
            entry['evaluation_returncode']=audit.returncode
        entry['passed']=run.returncode==0 and entry.get('evaluation_returncode')==0
        (out/'status.json').write_text(json.dumps(status,indent=2)+'\n')
        print(json.dumps(entry),flush=True)
        if not entry['passed']:return 1
        if (out/'STOP_AFTER_CURRENT').exists():
            (out/'stopped.json').write_text(json.dumps(dict(
                reason='explicit scheduler stop after normal case completion',completed=name),indent=2)+'\n')
            return 2
    return 0


if __name__=='__main__':raise SystemExit(main())

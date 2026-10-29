#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Black-box native numerical contracts. Synthetic grids are never flight evidence."""
import argparse
import copy
import json
import math
from pathlib import Path
import subprocess
import time
import yaml
from ament_index_python.packages import get_package_prefix


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();root=args.output.resolve();root.mkdir()
    worker=Path(get_package_prefix('easyfly_fuel_exploration'))/'lib/easyfly_fuel_exploration/fuel_worker'
    original=[];size=[40,30,15];origin=[-2.,-3.,-.2]
    for z in range(size[2]):
        for y in range(size[1]):
            for x in range(size[0]):
                p=[origin[0]+(x+.5)*.2,origin[1]+(y+.5)*.2,origin[2]+(z+.5)*.2]
                original.append(0 if math.dist(p,[0,0,1.5])<2.6 else -1)
    req=dict(size=size,origin=origin,resolution=.2,min_bound=[-1.8,-2.8,.6],max_bound=[5.8,2.8,2.4],
             raw=original,inflated=original,position=[0.,0.,1.5],velocity=[0.,0.,0.],yaw=0.)
    log=(root/'worker.log').open('x');process=subprocess.Popen([str(worker),str(root)],stdin=subprocess.PIPE,
        stdout=log,stderr=subprocess.STDOUT,text=True,env={**__import__('os').environ,'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'})
    def decide(index,value):
        value={**value,'request_id':str(index)};source=root/f'{index}.json';source.write_text(json.dumps(value))
        process.stdin.write(str(source)+'\n');process.stdin.flush();output=Path(str(source)+'.result.yaml')
        end=time.monotonic()+4.
        while not output.exists() and process.poll() is None and time.monotonic()<end:time.sleep(.01)
        assert output.exists(), 'bounded real native computation failed'
        return yaml.safe_load(output.read_text())
    try:
        a=decide(1,req);assert a['reason']=='NEXT_VIEWPOINT' and a['updates']==1
        assert len(a['frontiers'])>=2 and len(a['global_tour'])>=3 and len(a['refined_tour'])>=2
        assert (root/'1-native/single.tsp').is_file() and (root/'1-native/single.txt').is_file()
        req['position']=a['goal']
        # Full observation removes the genuinely previous frontier clusters in the SAME session.
        req['raw']=[0]*len(original);req['inflated']=[0]*len(original)
        b=decide(2,req);assert b['updates']==2 and b['reason']=='NO_FRONTIER'
        assert b['frontiers']==[], 'native incremental FIS retained removed frontiers'
        process.stdin.write('STOP\n');process.stdin.flush();assert process.wait(timeout=2)==0
        assert '805306368' in (root/'limits.txt').read_text()
        assert 'libfuel_native.so' in (root/'loaded-maps.txt').read_text()
    finally:
        if process.poll() is None:process.kill();process.wait(timeout=2)
        process.stdin.close();log.close()
    bad=root/'malformed';bad.mkdir();bad_req=copy.deepcopy(req);bad_req['raw'][-1]=42
    p=bad/'input.json';p.write_text(json.dumps(dict(bad_req,request_id='bad')))
    run=subprocess.run([str(worker),str(bad)],input=str(p)+'\nSTOP\n',capture_output=True,text=True,timeout=4.)
    (bad/'worker.log').write_text(run.stdout+run.stderr)
    assert run.returncode==2 and not Path(str(p)+'.result.yaml').exists()
    (root/'result.json').write_text(json.dumps(dict(passed=True,synthetic_numerical_only=True,
        first_frontiers=len(a['frontiers']),native_updates=[a['updates'],b['updates']],
        real_lkh=True,real_incremental_removal=True,malformed_rejected=True),indent=2)+'\n')


if __name__=='__main__':main()

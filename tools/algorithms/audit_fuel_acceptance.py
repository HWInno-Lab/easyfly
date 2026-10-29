#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Collect completed NEW-5 evidence; missing/failed evidence never qualifies."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]
R=ROOT/'artifacts/new-algorithms/new5-fuel-20261023'
CASES=['room','warehouse','cancel','map-loss','worker-exit','worker-timeout',
       'timeout','no-fly','land-takeover','unreachable']


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--matrix',type=Path,required=True)
    p.add_argument('--default',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();checks=[];ledger={};summaries={}
    def check(name,value,detail=None):
        checks.append(dict(name=name,passed=bool(value),detail=detail))
    def read(path):
        ledger[str(path.relative_to(ROOT))]=sha(path)
        return json.loads(path.read_text())
    frozen=read(R/'acceptance-limits.json')
    current=read(R/'current-source-v4.json')
    check('frozen production source and tests unchanged',all((ROOT/name).is_file() and sha(ROOT/name)==digest
          for name,digest in current.items()))
    status=read(args.matrix.resolve()/'status.json')
    check('ten serial cases complete',len(status)==10 and
          {c['name'] for c in status}==set(CASES) and all(c.get('passed') and not c['running'] for c in status))
    selected={c['name']:(ROOT/c['evidence_directory']).resolve() if 'evidence_directory' in c
              else args.matrix.resolve()/c['name'] for c in status}
    selection=read(args.matrix.resolve()/'selection.json')
    check('explicit final case selection and production manifest',
          selection['production_sha256']==sha(R/'current-source-v4.json') and
          selection['case_sources']=={name:str(path.relative_to(ROOT)) for name,path in selected.items()})
    for case in CASES:
        d=selected.get(case,args.matrix.resolve()/case)
        try:
            result=read(d/'result.json');audit=read(d/'independent-evaluation.json')
            check(case+': actual mission and independent audit',result['passed'] and audit['passed'],audit['failures'])
            check(case+': frozen limits unchanged',result['acceptance_limits']==frozen)
            check(case+': disarmed normal transport shutdown',
                  not result['final_disarmed_state']['armed'] and not result['forced'] and
                  not result['leftovers'] and result['transport_drained'] and
                  result['launch_returncode']==0 and result['shutdown_seconds']<=12.)
            check(case+': owned native processes reaped',result['fuel_workers_reaped'] and result['optimizer_workers_reaped'])
            check(case+': physical sensor attachment',audit['sensor_attachment']['passed'])
            check(case+': exact parent ownership released',all(x['requested']==x['returned'] and x['state']==2
                  for x in result['ownership'] if x['service']=='/exploration/ownership') and
                  len([x for x in result['ownership'] if x['service']=='/exploration/ownership'])==len(audit['missions']))
            check(case+': archived source and harness integrity',all(
                  (d/'source-snapshot'/name).is_file() and sha(d/'source-snapshot'/name)==digest
                  for name,digest in result['source_hashes'].items()))
            changed=[name for name,digest in result['source_hashes'].items() if name.startswith('src/')
                     and (not (ROOT/name).is_file() or sha(ROOT/name)!=digest)]
            check(case+': final production source matches flight',not changed,changed)
            changed=[name for field in ['runtime_executables_sha256','runtime_libraries_sha256']
                     for name,digest in result[field].items() if not Path(name).is_file() or sha(Path(name))!=digest]
            check(case+': final runtime ELF matches flight',not changed,changed)
            check(case+': actual selected native libraries',bool(result['fuel_native_libraries']) and
                  all(x['matches'] for x in result['fuel_native_libraries']))
            summaries[case]=dict(missions=audit['missions'],tracking_max_m=audit['source_time_tracking_max_m'],
                clearance_min_m=audit['independent_clearance_min_m'],shutdown_s=result['shutdown_seconds'],
                sensor_error_max_m=audit['sensor_attachment']['max_physical_origin_error_m'],
                resources=audit['resources'])
        except (OSError,KeyError,ValueError) as exc:check(case+': complete readable evidence',False,str(exc))
    try:
        d=args.default.resolve();result=read(d/'result.json')
        check('default combined actual flight',result['passed'] and result['strategy']=='combined')
        check('default exploration idle', '[fuel_exploration-' not in (d/'console.log').read_text() and
              result['polynomial_count']==0)
        check('default normal disarmed shutdown',not result['forced'] and not result['leftovers'] and
              not result['final_disarmed_state']['armed'] and result['launch_returncode']==0)
    except (OSError,KeyError,ValueError) as exc:check('default regression evidence',False,str(exc))
    for name in ['source-reconstruction-3/result.json','native-contract-3/result.json',
                 'native-memory-contract-2/result.json','native-resource-parent-contract-1/result.json']:
        try:check(name,read(R/name)['passed'])
        except (OSError,KeyError) as exc:check(name,False,str(exc))
    provenance=read(ROOT/'src/third_party/algorithms/planning/fuel.provenance.json')
    native=ROOT/'src/third_party/algorithms/planning'
    check('current native provenance and original inputs',
          all(sha(native/provenance[key]['path'])==provenance[key]['sha256']
              for key in ['original_archive','port_patch']) and
          all(sha(native/'fuel'/name)==digest and sha(R/'source-reconstruction-3/reconstructed'/name)==digest
              for name,digest in provenance['files'].items()))
    try:
        install=read(R/'installation-consumer-audit-v4.json')
        check('three actual installation consumers',set(install)=={'copy','symlink','moved'} and all(v['passed'] for v in install.values()))
        check('consumer loaded ELF still matches',all(sha(Path(path))==digest
              for value in install.values() for path,digest in value['loaded_libraries'].items()))
        parent=ROOT/'src/execution/e2_planning/easyfly_fuel_exploration/easyfly_fuel_exploration/node.py'
        check('three installed parent modules match current source',all(
              sha(R/prefix/'easyfly_fuel_exploration/local/lib/python3.10/dist-packages/easyfly_fuel_exploration/node.py')==sha(parent)
              for prefix in ['install','symlink/install','relocation/moved-install-v4']))
        check('three installed provenance archives match current source',all(
              sha(R/prefix/'easyfly_fuel_native/share/easyfly_fuel_native/provenance'/name)==sha(native/name)
              for prefix in ['install','symlink/install','relocation/moved-install-v4']
              for name in ['fuel.provenance.json','fuel.original-selected.tar.gz','fuel.ros2-highlevel.patch']))
        scenes=read(R/'scene-validation-v2.json')
        check('registered measured scene resources',all(not s['issues'] for s in scenes['scenes'].values()))
    except (OSError,KeyError,ValueError) as exc:check('installation/scene evidence',False,str(exc))
    for package,names in [('easyfly_fuel_exploration',['test_fuel_policy.xunit.xml','test_fuel_clock.xunit.xml']),
                          ('easyfly_trajectory_optimizer',['test_gcopter_navigation_ros.xunit.xml',
                                                          'test_gcopter_voxel_ros.xunit.xml','test_gcopter_core.gtest.xml']),
                          ('easyfly_global_planner',['test_local_coordinator.xunit.xml','test_waypoint_lifecycle.gtest.xml','test_goal_validation.gtest.xml'])]:
        for name in names:
            path=R/'build'/package/'test_results'/package/name
            try:
                root=ET.parse(path).getroot();suites=[root] if root.tag=='testsuite' else list(root.findall('testsuite'))
                tests=sum(int(s.attrib.get('tests',0)) for s in suites)
                failed=sum(int(s.attrib.get(k,0)) for s in suites for k in ['failures','errors','skipped'])
                check(name,tests>0 and failed==0,dict(tests=tests,failed=failed));ledger[str(path.relative_to(ROOT))]=sha(path)
            except (OSError,ET.ParseError) as exc:check(name,False,str(exc))
    for name,needle in [('platform-final-tests.log','77 passed'),('coordinator-final-tests.log','100% tests passed')]:
        path=R/name
        check(name,path.is_file() and needle in path.read_text())
        if path.is_file():ledger[str(path.relative_to(ROOT))]=sha(path)
    base=read(R/'baseline.json');missing=[];changed=[];bad=[]
    with tarfile.open(R/'pre-fuel-worktree.tar.gz') as tar:
        for name,digest in base['files'].items():
            if not (ROOT/name).is_file():missing.append(name)
            elif sha(ROOT/name)!=digest:changed.append(name)
            if hashlib.sha256(tar.extractfile(name).read()).hexdigest()!=digest:bad.append(name)
    check('all 1195 original bytes retained',len(base['files'])==1195 and not missing and not bad)
    allowed=set(read(R/'baseline-preservation-pre-matrix.json')['changed'])|{'plan-v6-new-algorithms.md',
        'src/execution/e2_planning/easyfly_trajectory_optimizer/src/navigation_node.cpp',
        'src/execution/e2_planning/easyfly_trajectory_optimizer/test/test_navigation_ros.py'}
    check('no unrelated baseline edits',not (set(changed)-allowed),sorted(set(changed)-allowed))
    check('HEAD unchanged',subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==base['head'])
    exclusions=read(R/'evidence-exclusions.json');history=[]
    for path in sorted([*R.glob('*/result.json'),*R.glob('matrix-*/*/result.json')]):
        result=json.loads(path.read_text())
        if 'scenario' not in result:continue
        name=str(path.parent.relative_to(R));ledger[str(path.relative_to(ROOT))]=sha(path)
        history.append(dict(run=name,scenario=result['scenario'],passed_as_recorded=result['passed'],
            error=result.get('error'),excluded=name in exclusions['excluded'],
            current_final_suite=path.parent in set(selected.values()) or path.parent==args.default.resolve(),
            exploration_reasons=[a.get('message') for a in result.get('actions',[]) if a['name']=='/explore']))
    delivery={name:sha(ROOT/name) for name in [
        'docs/development/fuel-simulation-acceptance.md','plan-v6-new-algorithms.md',
        'artifacts/new-algorithms/new5-fuel-20261023/benchmark-summary.json']}
    output=dict(passed=all(c['passed'] for c in checks),checks=checks,check_count=len(checks),
        scope=frozen['scope'],matrix=str(args.matrix.resolve().relative_to(ROOT)),cases=summaries,
        baseline=dict(original_files=len(base['files']),changed=changed,missing=missing),evidence_sha256=ledger,
        delivery_sha256=delivery,
        exclusions=exclusions,historical_runs=history,host=read(R/'host-platform.json'),
        auditor_sha256=sha(Path(__file__)))
    assert not args.output.exists(),'final evidence output must be fresh'
    args.output.write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(dict(passed=output['passed'],checks=len(checks),failures=[c for c in checks if not c['passed']]),indent=2))
    return 0 if output['passed'] else 1


if __name__=='__main__':raise SystemExit(main())

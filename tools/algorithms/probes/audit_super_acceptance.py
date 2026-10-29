#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Audit the frozen NEW-4 Gazebo evidence; reference failures stay failures."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import tarfile
import xml.etree.ElementTree as ET
import yaml

ROOT=Path(__file__).resolve().parents[3]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    args=parser.parse_args();root=args.root.resolve(strict=True)
    checks={};details={}
    def check(name,condition):
        checks[name]=bool(condition)
    def read(path):
        return json.loads((root/path).read_text())

    limits=read('acceptance-limits.json')
    frozen=dict(arrival_error_m=.11,planned_speed_mps=.6,max_acceleration_mps2=2.,
        max_jerk_mps3=8.,tracking_error_m=.2,inflation_radius_m=.65,
        independent_model_center_to_obstacle_aabb_m=.35,flight_z_min_m=.5,
        flight_z_max_m=2.5,source_freshness_s=1.,future_tolerance_s=.2,
        worker_wall_deadline_s=3.,worker_cpu_limit_s=3,worker_address_space_mib=1024,
        normal_shutdown_s=12.,unknown_is_blocked=True,default_backend_changed=False)
    check('original_numeric_and_policy_limits',all(limits[k]==v for k,v in frozen.items()))
    static=['static-v9/'+n for n in ['over','under','channel','narrow','no-solution']]
    faults=['fault-v9/'+n for n in ['cancel','map-loss','backend-exit','map-change']]
    final=['final-smoke-relocated','final-smoke-symlink','cold-narrow-relocated']
    selected=static+faults+final
    check('static_matrix',len(read('static-v9/remaining-matrix-status.json'))==5 and
          all(e['passed'] for e in read('static-v9/remaining-matrix-status.json')))
    check('fault_matrix',len(read('fault-v9/remaining-matrix-status.json'))==4 and
          all(e['passed'] for e in read('fault-v9/remaining-matrix-status.json')))
    cases={name:read(name+'/result.json') for name in selected+
           ['ego-smoke-v9','ego-narrow-v9','default-combined-v9']}
    worker_count=0;cpus=[];rss=[];track=[]
    for name,result in cases.items():
        check(name+'/same_frozen_limits',result['acceptance_limits']==limits)
        lifecycle_ok=(result.get('sessions_closed') is True if name!='fault-v9/backend-exit' else
            result.get('injection',{}).get('fault_lock_retains_exact_ownership') is True and
            result.get('injection',{}).get('new_navigation_rejected') is True)
        check(name+'/normal_land_shutdown',(not result['forced'] and not result['leftovers'] and
            not result['abnormal_lines'] and not result['discarded'] and lifecycle_ok and
            result['launch_returncode']==result['truth_returncode']==0 and
            set(result['normal_exits'])==set(result['started_processes']) and
            not result['final_disarmed_state']['armed'] and result['transport_drained'] and
            result['optimizer_workers_reaped'] and result['shutdown_seconds']<=limits['normal_shutdown_s']))
        owned_ok=True;live=[]
        for sample in read(name+'/process-budget.json')['samples']:
            for process in sample['processes']:
                own=result['owned_processes'].get(str(process['pid']))
                owned_ok &= own is not None and str(process['start_ticks'])==str(own['start_ticks'])
        for pid,own in result['owned_processes'].items():
            proc=Path('/proc')/pid
            try:
                start=(proc/'stat').read_text().rsplit(')',1)[1].split()[19]
                if start==str(own['start_ticks']) and str((proc/'exe').resolve())==own['executable']:
                    live.append(pid)
            except FileNotFoundError:
                pass
        check(name+'/owned_resource_identities_and_no_live_processes',owned_ok and not live)
        if name not in selected:
            continue
        evaluation=read(name+'/independent-evaluation.json')
        check(name+'/actual_super_and_independent_evaluation',result['passed'] and result['strategy']=='super' and
              evaluation['passed'] and evaluation['case_result_sha256']==sha(root/name/'result.json'))
        check(name+'/same_production_source',all(sha(ROOT/path)==value for path,value in result['source_hashes'].items()))
        if evaluation['source_time_tracking_max_m'] is not None:
            track.append(evaluation['source_time_tracking_max_m'])
        accepted=0
        for log in (root/name).glob('super-*/job-*.log'):
            text=log.read_text()
            if 'SUPER accepted:' not in text:
                continue
            accepted+=1;worker_count+=1
            usage=re.search(r'worker_resource cpu_seconds=([\d.e+-]+) peak_rss_kib=(\d+)',text)
            check(name+'/'+log.parent.name+'/'+log.stem+'/native_kernel_and_resources',
                usage is not None and 'native SUPER PlanFromRest: code=2' in text)
            if usage:
                cpus.append(float(usage[1]));rss.append(int(usage[2])/1024)
            job=log.parent/(log.stem+'.request.native')
            cfg=yaml.safe_load((job/'super-settings.yaml').read_text())
            native=read(str((job/'native-trajectories.json').relative_to(root)))
            expected=math.sqrt(3)*.5*float(cfg['rog_map']['resolution'])+1e-6
            check(name+'/'+log.parent.name+'/'+log.stem+'/exact_native_configuration',
                cfg['super_planner']['backup_traj_en'] is True and
                cfg['super_planner']['frontend_in_known_free'] is True and
                abs(float(cfg['super_planner']['robot_r'])-expected)<1e-10 and
                cfg['rog_map']['inflation_step']==0 and
                cfg['traj_opt']['exp_traj']['pos_constraint_type']==2 and
                float(cfg['traj_opt']['exp_traj']['penna_vel'])==50000000 and
                float(cfg['traj_opt']['backup_traj']['penna_vel'])==10000000 and
                float(cfg['traj_opt']['backup_traj']['penna_jerk'])==100000 and
                bool(native['committed']) and bool(native['exploration']))
        check(name+'/actual_native_outputs',accepted>=result['polynomial_count'] and
              (accepted>0 or name.endswith('/no-solution')))
    check('kernel_worker_cpu_and_peak_rss',bool(cpus) and max(cpus)<=3 and max(rss)<1024)
    check('no_solution_no_executable',cases['static-v9/no-solution']['no_solution_fail_closed'] and
          cases['static-v9/no-solution']['polynomial_count']==cases['static-v9/no-solution']['command_samples']==0)
    check('cancel_exact_uuid_and_recovery',cases['fault-v9/cancel']['explicit_recovery_completed'] and
          len(cases['fault-v9/cancel']['injection']['cancel_ack_uuids'])==1)
    check('map_loss_explicit_recovery',cases['fault-v9/map-loss']['explicit_recovery_completed'])
    fault=cases['fault-v9/backend-exit']['injection']
    check('backend_exit_ownership_lock',fault['fault_lock_retains_exact_ownership'] and fault['new_navigation_rejected'])
    check('physical_panel_bounded_replan',bool(cases['fault-v9/map-change']['map_replan_notices']) and
          bool(cases['fault-v9/map-change']['physical_map_change_injection']))

    installed=['installed-copy-v9-final','installed-symlink-v9','installed-relocated-v9']
    for name in installed:
        result=read(name+'/result.json')
        check(name,result['passed'] and len(result['libraries'])==4 and not result['source_includes'] and
              not result['ros_node_created'] and all(sha(Path(p['loaded_realpath']))==p['sha256']
              for p in result['libraries'].values()))
    check('actual_symlink_configuration',read('installed-symlink-v9/result.json')['wrapper_executable_is_symlink'])
    relocation=read('relocation-v9.json')
    check('actual_physical_relocation',relocation['physical_directory_rename'] and
          not Path(relocation['original']).exists() and Path(relocation['moved']).is_dir())
    for case,prefix in [('final-smoke-relocated','relocated-v9-install'),
                        ('cold-narrow-relocated','relocated-v9-install'),
                        ('final-smoke-symlink','symlink-v9-install')]:
        result=cases[case];base=root/prefix
        exe=base/'easyfly_super_planner/lib/easyfly_super_planner/super_navigation'
        expected={str(exe.resolve()):sha(exe)}
        actual={p:s for p,s in result['runtime_executables_sha256'].items() if p.endswith('/super_navigation')}
        lib_ok=True
        for package,library in [('rog_map','libeasyfly_rog_map.so'),('super_planner','libeasyfly_super_planner.so'),
                                ('easyfly_super_planner','libsuper_validation.so')]:
            path=base/package/'lib'/library
            lib_ok &= result['runtime_libraries_sha256'].get(str(path.resolve()))==sha(path)
        check(case+'/actual_selected_runtime_prefix',actual==expected and lib_ok and
              result['worker_sha256']==sha(base/'easyfly_super_planner/lib/easyfly_super_planner/super_worker'))
    check('same_binary_narrow_cold_repeat',cases['static-v9/narrow']['worker_sha256']==cases['cold-narrow-relocated']['worker_sha256'] and
          sorted(v for k,v in cases['static-v9/narrow']['runtime_libraries_sha256'].items() if any(n in k for n in ['libsuper_','libeasyfly_super_planner','libeasyfly_rog_map']))==
          sorted(v for k,v in cases['cold-narrow-relocated']['runtime_libraries_sha256'].items() if any(n in k for n in ['libsuper_','libeasyfly_super_planner','libeasyfly_rog_map'])))
    payload=read('runtime-final-v9/selected-payload-sha256.json')
    check('all_selected_payloads_unchanged',all(sha(root/path)==value for path,value in payload.items()))
    comparison=read('paired-comparison.json')
    check('actual_paired_comparison_complete',comparison['passed'] and all(comparison['checks'].values()))
    check('reference_outcomes_preserved',cases['ego-smoke-v9']['passed'] and
          not cases['ego-narrow-v9']['passed'] and not read('ego-narrow-v9/common-constraint-evaluation.json')['passed'] and
          'NO_TRUSTED_CLOUD' in (root/'ego-narrow-v9/console.log').read_text())
    default=cases['default-combined-v9']
    check('unchanged_default_regression',default['passed'] and default['strategy']=='combined' and
          not default['super_roles_started'] and default['polynomial_count']==0)
    expected_tests={'easyfly_super_planner/test_super_septic.gtest.xml':3,
        'easyfly_super_planner/test_super_navigation_ros.xunit.xml':13,
        'easyfly_global_planner/test_local_coordinator.xunit.xml':17}
    test_count=0
    for filename,count in expected_tests.items():
        package,file=filename.split('/')
        tree=ET.parse(root/'build'/package/'test_results'/package/file).getroot()
        actual=len(tree.findall('.//testcase'));test_count+=actual
        check('tests/'+file,actual==count and not any(tree.findall('.//'+tag) for tag in ['failure','error','skipped']))
    for name,count in [('platform-final-tests.log',36),('bspline-final-metric-tests-v2.log',4)]:
        check('tests/'+name,re.search(r'\b'+str(count)+r' passed\b',(root/name).read_text()) is not None)
        test_count+=count
    source=read('source-reconstruction-after-install.json')
    check('pinned_vendor_reconstruction',source==dict(passed=True,files=246,components=2,fuzz=0))
    baseline=read('baseline.json');missing=[];modified=[];archive_ok=True
    with tarfile.open(root/'pre-super-source.tar.gz') as archive:
        for path,value in baseline['files'].items():
            archive_ok &= hashlib.sha256(archive.extractfile(path).read()).hexdigest()==value
            if not (ROOT/path).exists():missing.append(path)
            elif sha(ROOT/path)!=value:modified.append(path)
    check('baseline_original_bytes_and_no_deletions',archive_ok and len(baseline['files'])==921 and not missing)
    check('only_recorded_existing_source_changes',set(modified)==set(read('baseline-current-delta.json')['changed_existing_files']))
    check('head_unchanged',subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==baseline['head'])
    details.update(super_flight_cases=len(selected),reference_and_default_cases=3,
        automated_test_cases=test_count,successful_native_workers=worker_count,
        super_tracking_max_m=max(track),native_worker_cpu_max_s=max(cpus),native_worker_peak_rss_max_mib=max(rss),
        baseline_modified_source_files=modified,comparison_reference_case_successes='EGO 1/2, SUPER 2/2 finite comparison routes',
        scope='Known-free full-goal SUPER Gazebo/PX4 mode; native backup output is computed/retained as applicable but does not switch into flight control; no hardware, high-speed or unknown-space qualification')
    result=dict(passed=all(checks.values()),checks=checks,details=details,
                auditor_sha256=sha(Path(__file__)))
    (root/'acceptance-audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'],check_count=len(checks),failed=[k for k,v in checks.items() if not v],details=details),indent=2))
    raise SystemExit(0 if result['passed'] else 1)


if __name__=='__main__':
    main()

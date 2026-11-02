#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Classify EXT-07 evidence without executing algorithms or changing results."""
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def cases(path):
    result = []
    for case in ET.parse(path).getroot().iter('testcase'):
        status = 'failed' if case.find('failure') is not None or case.find('error') is not None else (
            'skipped' if case.find('skipped') is not None or case.get('status') == 'notrun'
            or case.get('result') in ['skipped', 'suppressed'] else 'passed')
        record = {'name': case.get('name'), 'class': case.get('classname'),
                  'status': status, 'evidence': str(path)}
        if status == 'failed':
            failure = case.find('failure')
            if failure is None:
                failure = case.find('error')
            record['diagnostic'] = (failure.get('message', '') + '\n' + (failure.text or ''))
        result.append(record)
    return result


def counts(records):
    return {status: sum(c['status'] == status for c in records)
            for status in ['passed', 'failed', 'skipped']}


def report(out, root):
    config = read(out/'manifest.json')
    baseline = root/config['baseline']
    expected_known = {(e['package'], e['test']): e['reason'] for e in config['known_failures']}
    issues = []
    wrappers = {}
    for folder in ['regression', 'known-baseline-checks']:
        for path in sorted((out/folder/'ctest').glob('*/ctest.xml')):
            for case in cases(path):
                key = path.parent.name, case['name']
                wrappers[key] = dict(case, package=key[0], classification=(
                    'known_baseline_check' if key in expected_known else 'software_regression'))
    normal = [v for k, v in wrappers.items() if k not in expected_known]
    known = [dict(v, reason=expected_known[k]) for k, v in wrappers.items() if k in expected_known]
    if not normal:
        issues.append('no regression CTest evidence')
    if any(c['status'] != 'passed' for c in normal):
        issues.append('new regression failure or skip')
    for key in expected_known:
        if key not in wrappers:
            issues.append('known check not executed: '+':'.join(key))
    for check in config.get('resolved_checks', []):
        key = check['package'], check['test']
        if key not in wrappers or wrappers[key]['status'] != 'passed':
            issues.append('resolved regression check must remain passing: '+':'.join(key))

    baseline_entries = {}
    def add_baseline(package, path):
        for case in cases(path):
            baseline_entries[package, case['name']] = dict(case, package=package)
    for path in sorted(baseline.glob('*.xml')):
        add_baseline(path.stem, path)
    add_baseline('easyfly_ui_control', baseline/'supplement/easyfly_ui_control.xml')
    add_baseline('easyfly_flight_control', baseline/'replay-easyfly_flight_control/results.xml')
    add_baseline('easyfly_flight_control', baseline/'replay-easyfly_flight_control-test_stop_stream_chain/results.xml')
    declared_count = read(baseline/'baseline-status.json')['effective_software_ctest_entries']
    if len(baseline_entries) != declared_count:
        issues.append('baseline entry count does not match signed baseline')
    comparison = []
    for key, old in baseline_entries.items():
        current = wrappers.get(key)
        comparison.append({'package': key[0], 'test': key[1], 'baseline': old['status'],
                           'current': current['status'] if current else 'not_run',
                           'evidence': current['evidence'] if current else None})
    if any(c['current'] != 'passed' for c in comparison):
        issues.append('EXT-00 baseline entry not preserved')
    new_entries = [v for k, v in wrappers.items() if k not in baseline_entries and k not in expected_known]

    underlying = []
    for path in sorted((out/'build').glob('*/test_results/*/*.xml')):
        package = path.parent.name
        test = re.sub(r'\.(gtest|xunit)\.xml$', '', path.name)
        for case in cases(path):
            underlying.append(dict(case, package=package, wrapper=test,
                classification='known_baseline_check' if (package, test) in expected_known else 'software_regression'))
    underlying_normal = [c for c in underlying if c['classification'] == 'software_regression']
    baseline_native = read(baseline/'native-junit-final.json')
    original_cases, old_xml_changes = {}, []
    for item in baseline_native['files']:
        path = baseline/item['file']
        if sha(path) != item['sha256']:
            old_xml_changes.append(item['file'])
            continue
        for case in cases(path):
            original_cases[path.parent.name, case['class'], case['name']] = case
    current_cases = {(c['package'], c['class'], c['name']): c for c in underlying_normal}
    missing_cases = [list(key) for key in original_cases if key not in current_cases]
    nonpassing_cases = [list(key) for key in original_cases if key in current_cases
                       and current_cases[key]['status'] != 'passed']
    if old_xml_changes or missing_cases or nonpassing_cases:
        issues.append('EXT-00 underlying case compatibility not preserved')
    signatures = read(root/'tools/algorithms/extension_known_failures.json')['cases']
    for check in known:
        child = [c for c in underlying if c['package'] == check['package'] and c['wrapper'] == check['name']]
        failing = [c for c in child if c['status'] == 'failed']
        if check['status'] == 'failed' and not failing:
            issues.append('known wrapper failed without original underlying failure evidence: '+check['name'])
        for case in child:
            if case['status'] == 'skipped':
                issues.append('known wrapper contains unreviewed skip: '+case['name'])
            if case['status'] != 'failed':
                continue
            matching = any(s['package'] == case['package'] and s['wrapper'] == case['wrapper']
                           and s['case'] == case['name'] and ('class' not in s or s['class'] == case['class'])
                           and s['contains'] in case['diagnostic'] for s in signatures)
            case['matches_baseline_failure'] = matching
            if not matching:
                issues.append('new failure inside known wrapper: '+case['name'])
    native_path = out/'regression/native.xml'
    native = cases(native_path) if native_path.is_file() else []
    if not native or any(c['status'] != 'passed' for c in native):
        issues.append('native/configuration fixtures missing, failed or skipped')
    # CTest wrappers and their child XML are intentionally counted separately.
    skip_rules = read(root/'tools/algorithms/extension_scope_skips.json')['cases']
    not_applicable, uncovered, unreviewed_skips = [], [], []
    cache = (out/'build/easyfly_desktop/CMakeCache.txt').read_text()
    for case in underlying_normal:
        if case['status'] != 'skipped':
            continue
        rule = next((rule for rule in skip_rules if all(rule[key] == case[key]
                    for key in ['package', 'wrapper', 'name', 'class'])), None)
        if rule and 'requires_off_flag' in rule:
            flag = re.search(r'^'+rule['requires_off_flag']+r':BOOL=(.*)$', cache, re.MULTILINE)
            if not flag or flag[1].upper() not in ['OFF', 'FALSE', '0', 'NO']:
                rule = None
        if not rule:
            unreviewed_skips.append(case)
        elif rule['classification'] == 'not_applicable_input_mode':
            not_applicable.append(dict(case, scope_rule=rule))
        else:
            uncovered.append(dict(case, scope_rule=rule))
    if any(c['status'] == 'failed' for c in underlying_normal) or unreviewed_skips:
        issues.append('underlying non-baseline failure or unreviewed skip')
    numerical_names = ['real_sort_projection', 'exact_source_tf', 'native_detector_matches',
                       'single_native_service_real_csrt', 'reid_real_appearance',
                       'tracking_cli_backend_parameters_reach_real_sort', 'live_sort_reports']
    native_kinds = []
    for case in native:
        kind = ('real_computation_with_synthetic_input' if any(s in case['name'] for s in numerical_names)
                else 'configuration_status_or_protocol_fixture')
        native_kinds.append(dict(case, classification=kind))

    coverage_requirements = {
        'normal_native_trajectory': ['test_real_optimizer_stream_and_exact_session_stop'],
        'cancel_owned_worker': ['test_cancel_while_worker_is_owned_never_activates_late_result'],
        'invalid_input_stops_output': ['test_runtime_fault_stops_output_and_requires_exact_stop'],
        'late_result_isolation': ['test_worker_wall_timeout_cannot_activate_a_late_result'],
        'external_takeover': ['test_external_rtl', 'test_external_land'],
        'exact_stop_and_ownership_release': ['test_owned_stop', 'test_gcopter_stops_both_owners_before_new_session'],
        'frozen_clock_stop_chain': ['test_frozen_clock_telemetry_loss_stops_full_output_chain'],
    }
    coverage = {}
    for topic, names in coverage_requirements.items():
        evidence = [c for c in underlying_normal if any(c['name'].split('[')[0] == name for name in names)
                    and c['status'] == 'passed']
        matched = {c['name'].split('[')[0] for c in evidence}
        covered = all(name in matched for name in names)
        coverage[topic] = {'covered': covered, 'required_cases': names, 'evidence': evidence,
                           'scope': 'ROS/software fixtures; delayed workers and service peers may be injected'}
        if not covered:
            issues.append('execution coverage missing: '+topic)

    old_sources = read(baseline/'source-sha256.json')
    wire = {p: value for p, value in old_sources.items()
            if p.startswith('src/easyfly_common/easyfly_msgs/') and Path(p).suffix in ['.msg', '.srv', '.action']}
    changed_wire = [p for p, value in wire.items() if not (root/p).is_file() or sha(root/p) != value]
    added_wire = [str(p.relative_to(root)) for p in (root/'src/easyfly_common/easyfly_msgs').rglob('*')
                  if p.suffix in ['.msg', '.srv', '.action'] and str(p.relative_to(root)) not in wire]
    if changed_wire:
        issues.append('legacy wire changed')
    source_manifest = read(out/'source-files.json')
    source_changes = [p for p, value in source_manifest.items() if not (root/p).is_file() or sha(root/p) != value]
    if source_changes:
        issues.append('source changed during acceptance')
    asset_changes = [p for p, value in read(out/'asset-inputs.json').items()
                     if not Path(p).is_file() or sha(Path(p)) != value]
    if asset_changes:
        issues.append('runtime/model input changed during acceptance')
    tooling_changes = [p for p, value in read(out/'tooling-inputs.json').items()
                       if not (root/p).is_file() or sha(root/p) != value]
    assertion_changes = [p for p in tooling_changes if Path(p).name.startswith('test_')]
    if assertion_changes:
        issues.append('fixture assertions changed during acceptance')

    isolation = root/config['installation_evidence']
    isolation_record = read(isolation/'summary.json')
    isolation_changes = [p for p, value in isolation_record['source_files'].items()
                         if not (root/p).is_file() or sha(root/p) != value]
    if not isolation_record.get('passed') or isolation_changes:
        issues.append('EXT-06 install evidence no longer matches sources')
    sdk = root/config['ort_sdk']
    sdk_changes = [p for p, value in isolation_record['sdk_files'].items()
                   if not (sdk/p).is_file() or sha(sdk/p) != value['sha256']]
    if sdk_changes:
        issues.append('ORT SDK changed since installation acceptance')
    for evidence in ['installed-factories/summary.json', 'native-fuel-numerical/result.json', 'performance/summary.json']:
        p = out/evidence
        if not p.is_file() or not read(p).get('passed'):
            issues.append('missing successful evidence: '+evidence)
    build = read(out/'build-summary.json')
    if not build.get('passed'):
        issues.append('build incomplete')

    package_sources = {name: root/directory for name, directory, kind in
                       (line.split('\t') for line in (out/'packages.tsv').read_text().splitlines())}
    registry = read(root/'src/easyfly_common/easyfly_backend_registry/easyfly_backend_registry/data/backends.json')
    linkage = []
    cpp_kinds = {'sort', 'astar', 'onnx', 'csrt', 'kcf'}
    for row in registry['backends']:
        entry = row['entrypoint']
        source = package_sources[entry['package']]
        if entry['kind'] == 'cpp_creator':
            symbol = entry['symbol'].split('::')[-1]
            found = any(symbol+'(' in re.sub(r'\s+', '', p.read_text()) for p in (source/'include').rglob('*.hpp'))
        else:
            found = (source/'launch'/entry['file']).is_file()
        if not found:
            issues.append('registered source entry absent: '+row['id'])
        installed = (out/'install'/entry['package']).is_dir()
        linkage.append({'id': row['id'], 'capability': row['capability'], 'entrypoint': entry,
                        'source_verified': found, 'fresh_package_installed': installed,
                        'evidence_kind': ('real_factory_consumer' if row['id'] in cpp_kinds else
                            'real_native_process_and_protocol_fixtures' if row['id'] in ['gcopter', 'super', 'fuel'] else
                            'source_registration_guarded_launch_or_existing_engine_evidence'),
                        'assets': row['runtime']['assets'], 'runtime_ready': None})
    flags = [p for p in (out/'build').rglob('*') if p.name in ['flags.make', 'link.txt']]
    forbidden = [str(root/'install')+'/', str(root/'install-v3')+'/',
                 str(root/'artifacts/extensibility/ext00-20261101-isolated/install'),
                 str(root/'artifacts/extensibility/ext05-20261101/install'),
                 str(root/'artifacts/extensibility/ext06-20261101/')]
    leaks = {str(p): [prefix for prefix in forbidden if prefix in p.read_text()]
             for p in flags if any(prefix in p.read_text() for prefix in forbidden)}
    if leaks:
        issues.append('undeclared old overlay in compiler/linker inputs')
    # Missing baseline deployment resources are still open; software success
    # never silently turns those checks into passes or signs off full flight.
    open_known = [c for c in known if c['status'] != 'passed']
    software_passed = not issues
    result = {'schema_version': 1, 'stage': 'EXT-07', 'software_passed': software_passed,
              'full_stage_signed_off': software_passed and not open_known and not uncovered,
              'status': ('partial_with_known_baseline_gaps' if software_passed and open_known else
                         'partial_with_uncovered_optional_features' if software_passed and uncovered else
                         'software_scope_passed' if software_passed else 'requires_resolution'),
              'issues': issues, 'built_packages': len(build['packages']),
              'ctest': {'counts': counts(normal), 'entries': normal},
              'underlying': {'counts': counts(underlying_normal), 'cases': underlying_normal},
              'applicable_underlying_counts': counts([c for c in underlying_normal if c['status'] != 'skipped']),
              'not_applicable_cases': not_applicable, 'uncovered_optional_or_manual_cases': uncovered,
              'unreviewed_skips': unreviewed_skips,
              'performance': read(out/'performance/summary.json') if (out/'performance/summary.json').is_file() else None,
              'native': {'counts': counts(native), 'cases': native_kinds},
              'baseline_comparison': {'baseline_entries': len(baseline_entries), 'entries': comparison,
                                      'added_ctest_entries': new_entries},
              'baseline_underlying_comparison': {'original_unique_cases': len(original_cases),
                  'preserved_passed': sum(key in current_cases and current_cases[key]['status'] == 'passed'
                                          for key in original_cases),
                  'missing_cases': missing_cases, 'nonpassing_cases': nonpassing_cases,
                  'original_xml_hash_changes': old_xml_changes},
              'expected_interface_changes': {'legacy_wire_count': len(wire), 'legacy_wire_changed': changed_wire,
                  'added_wire': added_wire, 'notes': ['V2 full source/identity envelopes and isolated V1 fallback',
                    'Generic factories and normalized parameter maps; consumers rebuilt with matching ABI',
                    'Legacy concrete headers require explicit implementation targets']},
              'known_baseline_checks': known,
              'known_underlying': [c for c in underlying if c['classification'] == 'known_baseline_check'],
              'execution_coverage': coverage,
              'registry_implementation_linkage': linkage,
              'source_changes_during_acceptance': source_changes,
              'asset_changes_during_acceptance': asset_changes,
              'verification_tool_changes': tooling_changes,
              'install_isolation_evidence': {'path': str(isolation), 'source_changes': isolation_changes,
                                           'sdk_changes': sdk_changes},
              'compile_link_audit': {'files': len(flags), 'leaks': leaks},
              'scope': ['Real calculations/processes with synthetic or recorded inputs',
                        'Protocol fixtures may inject service peers, trajectory workers or delayed results',
                        'No PX4/Gazebo flight or hardware acceptance; FUEL full-task baseline gap retained',
                        'No claim that all registered engines were rerun against real sensor streams']}
    (out/'acceptance.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k: result[k] for k in ['status', 'software_passed', 'full_stage_signed_off', 'issues']}, ensure_ascii=False))
    return 0 if software_passed else 1

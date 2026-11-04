# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
"""Exercise material, historical-rights, foreign-scope and installation boundaries."""
import json
from pathlib import Path
import shutil

import pytest

import verify_license_scope as audit


@pytest.fixture
def dossier(tmp_path):
    source = Path(__file__).resolve().parents[2]
    record = audit.read_json(source / audit.DEFAULT)
    package = next(x for x in record['packages'] if x['name'] == 'easyfly_utils')
    record['packages'] = [package]
    record['files'] = [x for x in record['files'] if x['package'] == package['name']]
    record['retained_files'] = []
    record['supporting_tool_files'] = []
    record['additional_retained_notice_deliverables'] = []
    record.pop('existing_code_decision', None)
    record.pop('d04_current_review', None)
    record.pop('exceptions_registry', None)
    record.pop('followup_decisions', None)
    record.pop('contribution_policy', None)
    record.pop('source_review_followup', None)
    shutil.copytree(source / package['path'], tmp_path / package['path'])
    for key in ['root_entry', 'license_text', 'apache_text', 'owner_authority_record', 'baseline_package_inventory', 'prepublication_record']:
        name = record[key]['path']
        (tmp_path / name).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source / name, tmp_path / name)
    baseline = audit.read_json(source / record['baseline_package_inventory']['path'])
    baseline['packages'] = [x for x in baseline['packages'] if x['name'] == package['name']]
    original = audit.read_json(source / record['source_of_baseline_inventory'])
    original['files'] = [x for x in original['files'] if x['package'] == package['name']]
    original['dependency_followup'] = []
    preserved = {'src/retained/NOTICE': {'kind': 'file', 'sha256': ''}}
    notice = tmp_path / 'src/retained/NOTICE'
    notice.parent.mkdir(parents=True)
    notice.write_text('Attribution retained under its original terms.\n')
    preserved['src/retained/NOTICE']['sha256'] = audit.sha(notice)

    def write(name, value):
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value, ensure_ascii=False))

    def refresh():
        for key, data in [('baseline_package_inventory', baseline), ('retained_tracked_inputs', preserved)]:
            write(record[key]['path'], data)
            record[key]['sha256'] = audit.sha(tmp_path / record[key]['path'])
        write(record['source_of_baseline_inventory'], original)
        record['source_of_baseline_inventory_sha256'] = audit.sha(tmp_path / record['source_of_baseline_inventory'])
        write(audit.DEFAULT, record)

    refresh()
    return tmp_path, record, package, baseline, original, refresh


def test_consistent_scoped_grant_does_not_clear_release_or_ownership(dossier):
    root, *_ = dossier
    result = audit.verify(root)
    assert result['scoped_materials_consistent']
    assert result['migrated_packages'] == 1
    assert result['covered_file_entries'] == 8
    assert not result['independent_rights_clearance_inferred']
    assert not result['ready_to_publish_entire_workspace']


def test_covered_source_drift_is_rejected(dossier):
    root, *_ = dossier
    (root / 'src/easyfly_common/easyfly_utils/include/easyfly_utils/math.hpp').write_text('// changed\n')
    with pytest.raises(ValueError, match='covered file changed'):
        audit.verify(root)


def test_new_unregistered_source_is_rejected(dossier):
    root, _, package, *_ = dossier
    (root / package['path'] / 'new.cpp').write_text('// New unregistered file.\n')
    with pytest.raises(ValueError, match='source inventory changed'):
        audit.verify(root)


def test_missing_installed_license_text_is_rejected(dossier):
    root, _, package, *_ = dossier
    (root / package['path'] / 'licenses/EasyFly-Restricted-1.0.txt').unlink()
    with pytest.raises(FileNotFoundError):
        audit.verify(root)


def test_retained_copyright_notice_cannot_be_deleted(dossier):
    root, *_ = dossier
    (root / 'src/retained/NOTICE').unlink()
    with pytest.raises(ValueError, match='retained source/notice changed'):
        audit.verify(root)


def test_unknown_identifier_is_not_an_alternative_license(dossier):
    root, record, _, _, _, refresh = dossier
    record['license_identifier'] = 'Apache-2.0-NonCommercial'
    refresh()
    with pytest.raises(ValueError, match='unsupported implemented license scope'):
        audit.verify(root)


@pytest.mark.parametrize('change', [
    {'historical_grants_preserved': False}, {'third_party_relicensing_claimed': True},
    {'independent_ownership_clearance_claimed': True}, {'publication_performed': True},
])
def test_technical_consistency_cannot_expand_rights_or_claim_publication(dossier, change):
    root, record, _, _, _, refresh = dossier
    record.update(change)
    refresh()
    with pytest.raises(ValueError, match='expands authority'):
        audit.verify(root)


def test_third_party_source_cannot_enter_restricted_scope(dossier):
    root, record, _, _, _, refresh = dossier
    record['files'][0]['path'] = 'src/third_party/upstream/source.hpp'
    refresh()
    with pytest.raises(ValueError, match='third-party file'):
        audit.verify(root)


def test_pending_gpl_combination_cannot_be_bulk_approved_by_rehashing(dossier):
    root, _, package, _, original, refresh = dossier
    original['dependency_followup'].append({'package': package['name']})
    refresh()
    with pytest.raises(ValueError, match='pending/external/asset scope'):
        audit.verify(root)


def test_foreign_notice_requires_review_even_if_hashes_are_updated(dossier):
    root, _, _, _, original, refresh = dossier
    original['files'][0].setdefault('flags', []).append('other_copyright_notice_requires_review')
    refresh()
    with pytest.raises(ValueError, match='unreviewed foreign notice'):
        audit.verify(root)


def test_local_delivered_scope_cannot_expand_beyond_root_scope(dossier):
    root, _, package, _, _, refresh = dossier
    path = root / package['scope_path']
    local = audit.read_json(path)
    local['files'].append({'path': 'not-approved.cpp'})
    path.write_text(json.dumps(local))
    package['scope_sha256'] = audit.sha(path)
    package['bundle_sha256']['scope.json'] = audit.sha(path)
    refresh()
    with pytest.raises(ValueError, match='root and installed package scope disagree'):
        audit.verify(root)


def test_owner_statement_cannot_expand_to_all_third_party_rights(dossier):
    root, record, _, _, _, refresh = dossier
    entry = record['owner_authority_record']
    path = root / entry['path']
    owner = audit.read_json(path)
    owner['scope'] = 'All repository and third-party rights.'
    path.write_text(json.dumps(owner))
    entry['sha256'] = audit.sha(path)
    refresh()
    with pytest.raises(ValueError, match='owner authority or licensor scope expanded'):
        audit.verify(root)


def test_duplicate_fields_in_scope_are_rejected(dossier):
    root, *_ = dossier
    (root / audit.DEFAULT).write_text('{"schema_version": 1, "schema_version": 1}')
    with pytest.raises(ValueError, match='duplicate JSON field'):
        audit.verify(root)


def test_scope_paths_cannot_escape_workspace(dossier):
    root, record, _, _, _, refresh = dossier
    record['files'][0]['path'] = '../outside'
    refresh()
    with pytest.raises(ValueError, match='uncontained or noncanonical path'):
        audit.verify(root)


def test_historical_apache_text_cannot_be_replaced_even_after_rehash(dossier):
    root, record, _, _, _, refresh = dossier
    entry = record['apache_text']
    path = root / entry['path']
    path.write_text('A newly invented license.\n')
    entry['sha256'] = audit.sha(path)
    refresh()
    with pytest.raises(ValueError, match='historical Apache text not preserved'):
        audit.verify(root)


def test_current_own_code_cannot_gain_an_apache_parallel_license(dossier):
    root, record, package, _, _, refresh = dossier
    path = root / package['path'] / 'package.xml'
    text = path.read_text().replace('</package>', '<license>Apache-2.0</license></package>')
    path.write_text(text)
    next(x for x in record['files'] if x['path'].endswith('/package.xml'))['sha256'] = audit.sha(path)
    refresh()
    with pytest.raises(ValueError, match='package metadata offers unexpected'):
        audit.verify(root)


def test_stopping_new_apache_grants_is_not_optional_after_rehash(dossier):
    root, record, _, _, _, refresh = dossier
    record['new_apache_grants_for_covered_own_code'] = True
    refresh()
    with pytest.raises(ValueError, match='expands authority'):
        audit.verify(root)


def test_owner_publication_account_is_not_an_independent_audit(dossier):
    root, record, _, _, _, refresh = dossier
    entry = record['prepublication_record']
    path = root / entry['path']
    declaration = audit.read_json(path)
    declaration['independently_verified'] = True
    path.write_text(json.dumps(declaration))
    entry['sha256'] = audit.sha(path)
    refresh()
    with pytest.raises(ValueError, match='prepublication declaration missing or overstated'):
        audit.verify(root)


def test_old_own_apache_preamble_cannot_reappear_after_rehash(dossier):
    root, record, _, _, _, refresh = dossier
    row = next(x for x in record['files'] if x['path'].endswith('math.hpp'))
    path = root / row['path']
    path.write_text('// Licensed under the Apache License, Version 2.0.\n' + path.read_text())
    row['sha256'] = audit.sha(path)
    refresh()
    with pytest.raises(ValueError, match='own code still carries an Apache grant'):
        audit.verify(root)


def attach_current_d04(dossier):
    root, record, _, _, _, refresh = dossier
    source = Path(__file__).resolve().parents[2]
    real_scope = audit.read_json(source / audit.DEFAULT)
    entry = dict(real_scope['d04_current_review'])
    overlay = audit.read_json(source / 'licensing/history/d04-20261103-pre-gpl-exception.json')
    record['open_D04_items'] = list(overlay['current_open_items'])
    for item in [overlay['historical_dossier'], overlay['prior_owner_port_record']]:
        path = root / item['path']
        path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source / item['path'], path)
    path = root / entry['path']
    path.write_text(json.dumps(overlay))
    entry['sha256'] = audit.sha(path)
    record['d04_current_review'] = entry
    refresh()
    return overlay, path, entry


def test_withdrawn_owner_arrangement_reopens_ego_without_rewriting_old_dossier(dossier):
    root, *_ = dossier
    attach_current_d04(dossier)
    result = audit.verify(root)
    assert len(result['open_D04_items']) == 4
    assert 'D04-EGO-PORT' in result['open_D04_items']


def test_historical_three_open_result_cannot_hide_current_ego_reopening(dossier):
    root, _, _, _, _, refresh = dossier
    overlay, path, entry = attach_current_d04(dossier)
    overlay['current_open_items'].remove('D04-EGO-PORT')
    path.write_text(json.dumps(overlay))
    entry['sha256'] = audit.sha(path)
    refresh()
    with pytest.raises(ValueError, match='current D04 policy review inconsistent'):
        audit.verify(root)


def test_reopened_ego_does_not_automatically_receive_a_restricted_grant(dossier):
    root, _, _, _, _, refresh = dossier
    overlay, path, entry = attach_current_d04(dossier)
    overlay['reopened_items'][0]['new_restricted_grant_for_EGO_made_here'] = True
    path.write_text(json.dumps(overlay))
    entry['sha256'] = audit.sha(path)
    refresh()
    with pytest.raises(ValueError, match='current D04 policy review inconsistent'):
        audit.verify(root)


def attach_gpl_exception(dossier):
    root, record, _, _, _, refresh = dossier
    source = Path(__file__).resolve().parents[2]
    real = audit.read_json(source / audit.DEFAULT)
    registry = audit.read_json(source / real['exceptions_registry']['path'])
    grant = audit.read_json(source / registry['exceptions'][0]['grant']['path'])
    overlay = audit.read_json(source / real['d04_current_review']['path'])
    inputs = [real['exceptions_registry'], real['followup_decisions'], real['d04_current_review'],
              registry['exceptions'][0]['grant'], grant['source_manifest'], grant['source_archive'],
              grant['source_patch'], grant['license_text'], overlay['historical_dossier'],
              overlay['prior_owner_port_record'], overlay['prior_reopening_record']]
    names = {x['path'] for x in inputs} | set(grant['context_files_sha256'])
    for name in names:
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source / name, path)
    for key in ['exceptions_registry', 'followup_decisions', 'd04_current_review', 'open_D04_items']:
        record[key] = real[key]

    def sync():
        grant_entry = registry['exceptions'][0]['grant']
        path = root / grant_entry['path']
        path.write_text(json.dumps(grant))
        grant_entry['sha256'] = audit.sha(path)
        path = root / record['exceptions_registry']['path']
        path.write_text(json.dumps(registry))
        record['exceptions_registry']['sha256'] = audit.sha(path)
        overlay['resolved_items'][0]['grant'] = dict(grant_entry)
        path = root / record['d04_current_review']['path']
        path.write_text(json.dumps(overlay))
        record['d04_current_review']['sha256'] = audit.sha(path)
        refresh()

    sync()
    return grant, registry, overlay, sync


def test_explicit_gpl_owner_exception_closes_only_ego_owner_grant(dossier):
    root, *_ = dossier
    attach_gpl_exception(dossier)
    result = audit.verify(root)
    assert result['explicit_owner_gpl_exceptions'] == ['EGO-OWNER-ROS2-PORT-GPL-3.0-only']
    assert set(result['open_D04_items']) == {'D04-IKD', 'D04-LIVO-VIKIT', 'D04-LKH'}
    assert not result['combination_compatibility_inferred']
    assert not result['ready_to_publish_entire_workspace']


@pytest.mark.parametrize('change', [
    {'scope': 'all_EGO_and_inherited_code'},
    {'whole_files_or_repository_claimed_original': True},
    {'main_restricted_license_use_conditions_applied': True},
    {'license': 'Apache-2.0'},
    {'delivery_materials_synchronized': True},
])
def test_gpl_exception_cannot_expand_scope_or_keep_use_restrictions(dossier, change):
    root, *_ = dossier
    grant, _, _, sync = attach_gpl_exception(dossier)
    grant.update(change)
    sync()
    with pytest.raises(ValueError, match='unsupported or overbroad owner GPL exception'):
        audit.verify(root)


def test_gpl_exception_context_cannot_silently_add_an_unreviewed_file(dossier):
    root, *_ = dossier
    grant, _, _, sync = attach_gpl_exception(dossier)
    grant['context_files_sha256']['src/third_party/extra.cpp'] = '0' * 64
    grant['context_file_count'] += 1
    sync()
    with pytest.raises(ValueError, match='GPL exception source context mismatch'):
        audit.verify(root)


def test_owner_exception_cannot_close_an_unrelated_upstream_grant(dossier):
    root, record, _, _, _, _ = dossier
    _, _, overlay, sync = attach_gpl_exception(dossier)
    overlay['current_open_items'].remove('D04-LKH')
    record['open_D04_items'] = list(overlay['current_open_items'])
    sync()
    with pytest.raises(ValueError, match='current D04 GPL resolution has no valid scoped owner grant'):
        audit.verify(root)

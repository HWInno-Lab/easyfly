#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
"""Check an implemented file scope; consistency is not rights or release clearance.

Exit 0: scoped materials consistent; 1: drift/invalid materials;
2 with --require-ready: preserved/pending work still prevents a full release verdict.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from verify_license_transition import APACHE_SHA256, IDENTIFIER, external, inventory, read_json, relative_name, sha

ROOT = Path(__file__).resolve().parents[2]
DEFAULT = 'licensing/scope.json'


def local(root, name, follow=True):
    path = root / relative_name(name)
    checked = path.resolve(strict=True) if follow else path.parent.resolve(strict=True)
    if not checked.is_relative_to(root):
        raise ValueError('uncontained scope path: ' + name)
    return path


def pinned(root, entry):
    path = local(root, entry['path'])
    if not path.is_file() or path.is_symlink() or sha(path) != entry['sha256']:
        raise ValueError('scope material changed: ' + entry['path'])
    return path


def unchanged(root, name, row):
    path = local(root, name, follow=False)
    kind = row.get('baseline_kind', row.get('kind'))
    if kind == 'symlink':
        target = row.get('link_target', row.get('target'))
        if not path.is_symlink() or os.readlink(path) != target:
            raise ValueError('retained source link changed: ' + name)
    elif kind == 'file':
        if path.is_symlink() or not path.is_file() or sha(path) != row['sha256']:
            raise ValueError('retained source/notice changed: ' + name)
    else:
        raise ValueError('unknown baseline entry kind: ' + name)


def verify(root=ROOT, manifest=DEFAULT):
    root = Path(root).resolve()
    record = read_json(local(root, manifest))
    if (type(record.get('schema_version')) is not int or record['schema_version'] != 1
            or record.get('status') != 'scoped_worktree_license_revision_not_published'
            or record.get('license_identifier') != IDENTIFIER
            or record.get('start_policy') != 'review_existing_owned_contributions_as_well_as_future_contributions'
            or not record.get('edition')):
        raise ValueError('unsupported implemented license scope')
    if (record.get('historical_grants_preserved') is not True
            or record.get('current_own_code_license') != IDENTIFIER
            or record.get('new_apache_grants_for_covered_own_code') is not False
            or record.get('prior_own_apache_declarations') != 'withdrawn_private_unpublished_proposal_not_current_license'
            or record.get('publication_history_basis') != 'owner_declared_no_prior_external_distribution_not_independently_verified'
            or record.get('third_party_relicensing_claimed') is not False
            or record.get('independent_ownership_clearance_claimed') is not False
            or record.get('publication_performed') is not False):
        raise ValueError('scope expands authority or misrepresents publication/clearance')
    for key in ['root_entry', 'license_text', 'apache_text', 'owner_authority_record']:
        pinned(root, record[key])
    for key in ['contribution_policy', 'source_review_followup']:
        if key in record:
            pinned(root, record[key])
    declaration = read_json(pinned(root, record['prepublication_record']))
    if (declaration.get('status') != 'owner_declared_no_prior_external_distribution'
            or declaration.get('answer') != '从未提供给他人，仅本人本地保存'
            or declaration.get('independently_verified') is not False
            or declaration.get('actually_effective_prior_rights_revoked') is not False):
        raise ValueError('prepublication declaration missing or overstated')
    exceptions = {}
    if 'exceptions_registry' in record:
        registry = read_json(pinned(root, record['exceptions_registry']))
        decisions = read_json(pinned(root, record['followup_decisions']))
        if (registry.get('schema_version') != 1
                or registry.get('status') != 'explicit_scoped_owner_exceptions_not_blanket_relicensing'
                or registry.get('owner_decision') != record['followup_decisions']
                or registry.get('unlisted_materials_automatically_excepted') is not False
                or registry.get('third_party_rights_expanded') is not False
                or decisions['raw_answers'].get('4') != '可作为例外'
                or decisions['raw_answers'].get('6') != '暂不接收'
                or record.get('external_contributions_accepted') is not False
                or record.get('first_delivery_plan') != 'deferred_by_owner_until_easyfly_acceptance'):
            raise ValueError('exception registry exceeds recorded owner decisions')
        for entry in registry['exceptions']:
            grant = read_json(pinned(root, entry['grant']))
            if (entry.get('id') != 'EGO-OWNER-ROS2-PORT-GPL-3.0-only'
                    or entry['id'] in exceptions or entry.get('license') != 'GPL-3.0-only'
                    or grant.get('id') != entry['id'] or grant.get('license') != 'GPL-3.0-only'
                    or grant.get('status') != 'scoped_owner_gpl_grant_not_a_distribution_clearance'
                    or grant.get('scope') != 'owner_original_ros2_port_contributions_only'
                    or grant.get('preserve_inherited_licenses') is not True
                    or grant.get('whole_files_or_repository_claimed_original') is not False
                    or grant.get('main_restricted_license_use_conditions_applied') is not False
                    or grant.get('context_files_are_not_whole_file_ownership_claims') is not True
                    or grant.get('new_apache_grant') is not False
                    or grant.get('delivery_materials_synchronized') is not False
                    or grant.get('publication_performed') is not False
                    or grant.get('owner_decision') != record['followup_decisions']
                    or grant.get('author') != record['author']
                    or grant.get('authorized_licensor') != record['authorized_licensor']):
                raise ValueError('unsupported or overbroad owner GPL exception')
            source = read_json(pinned(root, grant['source_manifest']))
            for key in ['source_archive', 'source_patch', 'license_text']:
                pinned(root, grant[key])
            if (grant['source_archive']['sha256'] != source['original_archive']['sha256']
                    or grant['source_patch']['sha256'] != source['patch']['sha256']
                    or grant['context_file_count'] != len(grant['context_files_sha256'])):
                raise ValueError('GPL exception source context mismatch')
            expected_context = {source['source_root'] + '/' + k: v for k, v in source['files'].items()}
            if grant['context_files_sha256'] != expected_context:
                raise ValueError('GPL exception source context mismatch')
            for name, digest in expected_context.items():
                pinned(root, {'path': name, 'sha256': digest})
            upstream_text = 'src/third_party/algorithms/planning/ego_planner/licenses/UPSTREAM-EGO-GPL-3.0.txt'
            if sha(local(root, upstream_text)) != grant['license_text']['sha256']:
                raise ValueError('GPL exception ships a different GPL text')
            exceptions[entry['id']] = entry
    if 'd04_current_review' in record:
        current_d04 = read_json(pinned(root, record['d04_current_review']))
        prior_d04 = read_json(pinned(root, current_d04['historical_dossier']))
        prior_port = read_json(pinned(root, current_d04['prior_owner_port_record']))
        historical_open = {x['id'] for x in prior_d04['items']
                           if x['status'] == 'open_rightsholder_evidence'}
        reopened = current_d04['reopened_items']
        if (current_d04['historical_dossier_edited'] is not False
                or current_d04['prior_owner_port_record_edited'] is not False
                or current_d04['rights_clearance_claimed'] is not False
                or current_d04['third_party_licenses_changed'] is not False
                or current_d04['owner_prepublication_record'] != record['prepublication_record']
                or prior_port['license'] != 'Apache-2.0'
                or len(reopened) != 1 or reopened[0]['id'] != 'D04-EGO-PORT'
                or reopened[0]['status'] != 'reopened_owner_private_apache_arrangement_withdrawn'
                or reopened[0]['new_restricted_grant_for_EGO_made_here'] is not False
                or set(current_d04['historical_open_items']) != historical_open
                or set(record['open_D04_items']) != set(current_d04['current_open_items'])):
            raise ValueError('current D04 policy review inconsistent with withdrawn owner arrangement')
        if current_d04['status'] == 'current_D04_policy_overlay_not_a_license_grant':
            if set(current_d04['current_open_items']) != historical_open | {'D04-EGO-PORT'}:
                raise ValueError('current D04 policy review inconsistent with withdrawn owner arrangement')
        elif current_d04['status'] == 'current_D04_policy_overlay_with_scoped_owner_gpl_exception':
            previous = read_json(pinned(root, current_d04['prior_reopening_record']))
            resolved = current_d04['resolved_items']
            entry = exceptions.get('EGO-OWNER-ROS2-PORT-GPL-3.0-only')
            if (previous['status'] != 'current_D04_policy_overlay_not_a_license_grant'
                    or previous['reopened_items'] != reopened
                    or not entry or len(resolved) != 1 or resolved[0]['id'] != 'D04-EGO-PORT'
                    or resolved[0]['status'] != 'resolved_scoped_owner_gpl_grant'
                    or resolved[0]['grant'] != entry['grant']
                    or resolved[0]['delivery_materials_synchronized'] is not False
                    or set(current_d04['current_open_items']) != historical_open):
                raise ValueError('current D04 GPL resolution has no valid scoped owner grant')
        else:
            raise ValueError('unsupported current D04 policy status')
    if 'existing_code_decision' in record:
        decision = read_json(pinned(root, record['existing_code_decision']))
        if (decision.get('status') != 'owner_selected_existing_own_code_migration'
                or decision.get('third_party_rights_included') is not False
                or decision.get('historical_grants_revoked') is not False):
            raise ValueError('existing-code decision expands authority')
    owner = read_json(local(root, record['owner_authority_record']['path']))
    if (owner.get('scope') != 'Rights actually held by the owner in original EasyFly contributions; excludes inherited and third-party rights.'
            or owner.get('ownership_transfer_claimed') is not False
            or owner.get('existing_grants_revoked') is not False
            or owner.get('company_owns_all_repository_rights_claimed') is not False
            or record.get('author') != owner.get('author')
            or record.get('authorized_licensor') != owner.get('authorized_licensor')
            or record.get('contact_email') != owner.get('contact_email')):
        raise ValueError('owner authority or licensor scope expanded')
    if record['apache_text']['sha256'] != APACHE_SHA256:
        raise ValueError('historical Apache text not preserved')
    baseline = read_json(pinned(root, record['baseline_package_inventory']))
    original = read_json(pinned(root, {
        'path': record['source_of_baseline_inventory'],
        'sha256': record['source_of_baseline_inventory_sha256'],
    }))
    before_files = {x['path']: x for x in original['files']}
    if len(before_files) != len(original['files']):
        raise ValueError('duplicate baseline source entry')
    packages = {x['name']: x for x in baseline['packages']}
    gpl_followup = {x['package'] for x in original['dependency_followup']}
    covered, migrated, added = {}, {}, set()
    for row in record['files']:
        name = relative_name(row['path'])
        if name in covered or external(name):
            raise ValueError('duplicate or third-party file in restricted scope: ' + name)
        prior = before_files[name]
        if (row['package'] != prior['package'] or row['before_sha256'] != prior['sha256']
                or row.get('historical_license') != 'Apache-2.0'
                or row.get('historical_grants_preserved') is not True
                or row.get('license') != IDENTIFIER
                or row.get('new_apache_grant_in_this_revision') is not False
                or row.get('authority_basis') != 'owner_statement_and_existing_code_migration_instruction'):
            raise ValueError('file authority or historical scope expanded: ' + name)
        if set(prior.get('flags', [])) & {'other_copyright_notice_requires_review',
                                        'embedded_external_path', 'other_license_or_reference_requires_review'}:
            raise ValueError('unreviewed foreign notice in converted file: ' + name)
        package = packages[row['package']]
        if (package['classification'] != 'project_maintained_requires_rights_and_combination_review'
                or row['package'] in gpl_followup
                or not name.startswith(package['path'] + '/')
                or any(x in {'models', 'assets', 'vendor', 'third_party'}
                       for x in Path(name).relative_to(package['path']).parts)):
            raise ValueError('pending/external/asset scope cannot be bulk converted: ' + name)
        path = local(root, name)
        if path.is_symlink() or sha(path) != row['sha256']:
            raise ValueError('covered file changed: ' + name)
        if row['edit_kind'] != 'scope_only_data_or_template':
            preamble = '\n'.join(path.read_text().splitlines()[:20])
            if f'SPDX-License-Identifier: {IDENTIFIER}' not in preamble:
                raise ValueError('covered file lacks its active license header: ' + name)
            if 'Licensed under the Apache License, Version 2.0' in preamble:
                raise ValueError('covered own code still carries an Apache grant: ' + name)
        elif path.suffix not in {'.json', '.in'}:
            raise ValueError('unsupported header exemption: ' + name)
        covered[name] = row

    retained = {}
    for row in record['retained_files']:
        name = relative_name(row['path'])
        if name in retained or name in covered or row['package'] != before_files[name]['package']:
            raise ValueError('duplicate/overlapping retained scope: ' + name)
        old = before_files[name]
        if any(row.get(key) != old.get(key) for key in ['baseline_kind', 'sha256', 'link_target']):
            raise ValueError('retained baseline silently rewritten: ' + name)
        unchanged(root, name, row)
        retained[name] = row
    if set(covered) | set(retained) != set(before_files):
        raise ValueError('baseline scope has missing or extra entries')

    for package in record['packages']:
        name, base = package['name'], relative_name(package['path'])
        if name in migrated or packages[name]['path'] != base or external(base):
            raise ValueError('duplicate or external migrated package: ' + name)
        migrated[name] = package
        expected = {k: v for k, v in covered.items() if v['package'] == name}
        if not expected or package['covered_files'] != len(expected):
            raise ValueError('package scope count mismatch: ' + name)
        xml = ET.parse(local(root, base + '/package.xml')).getroot()
        notices = package.get('retained_license_notices', [])
        expected_ids = {IDENTIFIER}
        for notice in notices:
            retained_path = base + '/' + relative_name(notice['path'])
            if notice['license'] != 'MIT' or retained_path not in retained:
                raise ValueError('unreviewed retained license metadata: ' + name)
            if 'MIT License' not in local(root, retained_path).read_text():
                raise ValueError('retained MIT notice missing: ' + name)
            expected_ids.add('MIT')
        if (set(package['declared_license_ids']) != expected_ids
                or {x.text for x in xml.findall('license')} != expected_ids):
            raise ValueError('package metadata offers unexpected or missing license: ' + name)
        cmake = local(root, base + '/CMakeLists.txt').read_text()
        if ('install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/licenses/"' not in cmake
                or 'DESTINATION "share/${PROJECT_NAME}/licenses")' not in cmake):
            raise ValueError('package lacks license bundle install: ' + name)
        if package['scope_path'] != base + '/licenses/scope.json':
            raise ValueError('package license scope path mismatch: ' + name)
        local_scope = read_json(pinned(root, {'path': package['scope_path'], 'sha256': package['scope_sha256']}))
        expected_local = [{k: v for k, v in row.items() if k != 'path'} | {
            'path': Path(path).relative_to(base).as_posix()} for path, row in expected.items()]
        expected_retained = [{'path': Path(path).relative_to(base).as_posix(), 'reason': row['reason']}
                             for path, row in retained.items() if row['package'] == name]
        if (local_scope['files'] != expected_local or local_scope['retained_files'] != expected_retained
                or local_scope['edition'] != record['edition'] or local_scope['package'] != name
                or local_scope['historical_grants_preserved'] is not True
                or local_scope.get('current_own_code_license') != IDENTIFIER
                or local_scope.get('new_apache_grants_for_covered_own_code') is not False
                or local_scope.get('declared_license_ids') != package['declared_license_ids']
                or local_scope.get('retained_license_notices') != notices
                or local_scope.get('apache_text_role') != 'third_party_and_reference_only_not_a_new_grant_for_own_code'
                or local_scope['not_a_third_party_license_grant'] is not True):
            raise ValueError('root and installed package scope disagree: ' + name)
        if set(package['bundle_sha256']) != {'scope.json', 'EasyFly-Restricted-1.0.txt', 'Apache-2.0.txt'}:
            raise ValueError('incomplete license bundle: ' + name)
        for filename, digest in package['bundle_sha256'].items():
            path = base + '/licenses/' + filename
            pinned(root, {'path': path, 'sha256': digest})
            added.add(path)
        if (package['bundle_sha256']['EasyFly-Restricted-1.0.txt'] != record['license_text']['sha256']
                or package['bundle_sha256']['Apache-2.0.txt'] != APACHE_SHA256):
            raise ValueError('package ships different license text: ' + name)
    if set(migrated) != {x['package'] for x in covered.values()}:
        raise ValueError('covered file has no installed package bundle')

    actual = set()
    for package in baseline['packages']:
        if package['classification'] != 'retained_external_license':
            actual.update(inventory(root, package['path']))
    if actual != set(before_files) | added:
        raise ValueError('source inventory changed; added=' + str(sorted(actual - set(before_files) - added)[:5])
                         + '; missing=' + str(sorted((set(before_files) | added) - actual)[:5]))
    preserved = read_json(pinned(root, record['retained_tracked_inputs']))
    for name, row in preserved.items():
        if name in covered:
            raise ValueError('protected and converted scopes overlap: ' + name)
        unchanged(root, name, row)
    supporting = set()
    for entry in record.get('supporting_tool_files', []):
        path = pinned(root, entry)
        if entry['path'] in supporting or entry.get('license') != IDENTIFIER:
            raise ValueError('duplicate or unsupported supporting-tool scope')
        if f'SPDX-License-Identifier: {IDENTIFIER}' not in '\n'.join(path.read_text().splitlines()[:10]):
            raise ValueError('supporting tool lacks license header: ' + entry['path'])
        supporting.add(entry['path'])
    additional_notices = set()
    for entry in record.get('additional_retained_notice_deliverables', []):
        if (entry['path'] in additional_notices or entry['path'] in covered
                or not entry['path'].startswith('src/third_party/')
                or entry.get('relicensed') is not False
                or entry.get('before_sha256') != entry['sha256']):
            raise ValueError('retained third-party evidence reclassified or changed')
        pinned(root, entry)
        additional_notices.add(entry['path'])
    # Checks prove material consistency, never the truth of an ownership declaration.
    return {'scoped_materials_consistent': True, 'edition': record['edition'],
            'migrated_packages': len(migrated), 'covered_file_entries': len(covered),
            'retained_non_external_file_entries': len(retained),
            'retained_tracked_entries_verified': len(preserved),
            'supporting_tool_files': len(supporting),
            'explicit_owner_gpl_exceptions': list(exceptions),
            'additional_retained_notice_deliverables': len(additional_notices),
            'historical_grants_preserved': True, 'independent_rights_clearance_inferred': False,
            'new_apache_grants_for_covered_own_code': False,
            'prior_external_distribution_independently_verified': False,
            'combination_compatibility_inferred': False, 'publication_performed': False,
            'ready_to_publish_entire_workspace': False,
            'pending_packages': record['retained_or_pending_packages'],
            'open_D04_items': record['open_D04_items'],
            'limits': 'Only this recorded source scope and tracked preservation baseline; '
                      'not a full dependency SBOM, legal opinion, ignored-payload backup or publication approval.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--manifest', default=DEFAULT)
    parser.add_argument('--require-ready', action='store_true')
    args = parser.parse_args()
    try:
        result = verify(args.root, args.manifest)
    except (ValueError, KeyError, TypeError, OSError, RuntimeError, ET.ParseError) as error:
        print(json.dumps({'scoped_materials_consistent': False, 'error': str(error)}, ensure_ascii=False))
        return 1
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 2 if args.require_ready else 0


if __name__ == '__main__':
    sys.exit(main())

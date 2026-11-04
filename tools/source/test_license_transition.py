# Copyright 2026 贾国源. SPDX-License-Identifier: Apache-2.0
"""A consistent review dossier must not be mistaken for an effective grant."""
import copy
import json
from pathlib import Path
import subprocess
import sys

import pytest

import verify_license_transition as audit


@pytest.fixture
def dossier(tmp_path):
    source_root = Path(__file__).resolve().parents[2]
    # Synthetic earlier draft state; private prior source/notice snapshots are
    # deliberately not required in public source distributions.
    names = ['LICENSE', 'LICENSES/Apache-2.0.txt', 'licensing/scope.candidate.json',
             'licensing/file-scope.review.json', 'licensing/owner-authority.statement.json',
             'licensing/decisions-20261103.json', 'licensing/legal-review-notes-20261103.json',
             'licensing/use-cases.review.json', 'licensing/EasyFly-Restricted-1.0.draft.txt',
             'licensing/ROOT-LICENSE.draft.txt', 'licensing/AUTHORIZATION.draft.txt',
             'licensing/CONTRIBUTION-POLICY.draft.txt']
    owner = audit.read_json(source_root / 'licensing/owner-authority.statement.json')
    record = {'schema_version': 1, 'status': 'draft_migration_review_not_effective',
              'license_identifier': audit.IDENTIFIER, 'effective_release': None,
              'approved_scope': [], 'root_license_replacement_authorized': False,
              'owner_grant_scope': owner['scope'],
              'materials_sha256': dict.fromkeys(names),
              'pending': ['Synthetic draft scope has not been approved.']}
    for name in record['materials_sha256']:
        destination = tmp_path / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        source_name = 'LICENSES/Apache-2.0.txt' if name == 'LICENSE' else name
        destination.write_bytes((source_root / source_name).read_bytes())
    package = tmp_path / 'src/project'
    package.mkdir(parents=True)
    (package / 'package.xml').write_text('<package><name>project</name></package>\n')
    (package / 'source.cpp').write_text('// Own candidate; not approved.\n')
    candidate = audit.read_json(tmp_path / 'licensing/scope.candidate.json')
    candidate['packages'] = [{
        'name': 'project', 'path': 'src/project',
        'classification': 'project_maintained_requires_rights_and_combination_review',
        'new_license_approved': False,
        'metadata_sha256': audit.sha(package / 'package.xml'),
    }]
    files = [{
        'path': f'src/project/{name}', 'package': 'project', 'baseline_kind': 'file',
        'new_license_approved': False, 'sha256': audit.sha(package / name),
    } for name in ('source.cpp', 'package.xml')]
    review = {'files': files}

    def write(name, value):
        (tmp_path / name).write_text(json.dumps(value, ensure_ascii=False))

    def refresh():
        write('licensing/file-scope.review.json', review)
        candidate['file_review_sha256'] = audit.sha(tmp_path / 'licensing/file-scope.review.json')
        write('licensing/scope.candidate.json', candidate)
        for name in record['materials_sha256']:
            record['materials_sha256'][name] = audit.sha(tmp_path / name)
        write(audit.DEFAULT, record)

    refresh()
    return tmp_path, record, candidate, review, refresh


def test_consistent_review_does_not_approve_a_license(dossier):
    root, *_ = dossier
    result = audit.verify(root=root)
    assert result['draft_materials_consistent']
    assert result['checked_non_external_file_entries'] == 2
    assert not result['ready_to_switch_root_license']
    assert not result['rights_clearance_inferred']
    assert not result['combination_compatibility_inferred']


def test_plain_cmake_external_package_uses_cmake_metadata(dossier):
    root, _, candidate, _, refresh = dossier
    package = root / 'src/third_party/plain_sdk'
    package.mkdir(parents=True)
    metadata = package / 'CMakeLists.txt'
    metadata.write_text('cmake_minimum_required(VERSION 3.16)\nproject(plain_sdk)\n')
    candidate['packages'].append({
        'name': 'plain_sdk', 'path': 'src/third_party/plain_sdk', 'build_type': '(cmake)',
        'classification': 'retained_external_license', 'new_license_approved': False,
        'metadata_sha256': audit.sha(metadata),
    })
    refresh()
    result = audit.verify(root=root)
    assert result['inventoried_packages'] == 2
    assert result['checked_non_external_file_entries'] == 2


def test_strict_cli_distinguishes_readiness_from_material_consistency(dossier):
    root, *_ = dossier
    script = Path(audit.__file__)
    args = [sys.executable, str(script), '--root', str(root)]
    regular = subprocess.run(args, capture_output=True, text=True)
    strict = subprocess.run(args + ['--require-ready'], capture_output=True, text=True)
    assert regular.returncode == 0
    assert strict.returncode == 2
    assert json.loads(strict.stdout)['draft_materials_consistent']


def test_missing_full_license_is_reported(dossier):
    root, *_ = dossier
    (root / 'licensing/EasyFly-Restricted-1.0.draft.txt').unlink()
    with pytest.raises(FileNotFoundError):
        audit.verify(root=root)


def test_unknown_license_identifier_even_after_rehash(dossier):
    root, _, candidate, _, refresh = dossier
    candidate['proposed_license_identifier'] = 'Apache-2.0-with-noncommercial-clause'
    refresh()
    with pytest.raises(ValueError, match='candidate license scope'):
        audit.verify(root=root)


@pytest.mark.parametrize('name', ['/etc/passwd', '../outside', 'src/../outside', 'src//project'])
def test_paths_cannot_escape_or_use_ambiguous_spelling(dossier, name):
    root, record, _, _, refresh = dossier
    # Add after refresh: no reading of the supplied outside path is allowed.
    refresh()
    record['materials_sha256'][name] = '0' * 64
    (root / audit.DEFAULT).write_text(json.dumps(record))
    with pytest.raises(ValueError, match='uncontained or noncanonical'):
        audit.verify(root=root)


def test_duplicate_json_fields_rejected(dossier):
    root, *_ = dossier
    (root / audit.DEFAULT).write_text('{"schema_version": 1, "schema_version": 1}')
    with pytest.raises(ValueError, match='duplicate JSON field'):
        audit.verify(root=root)


def test_changed_source_requires_review(dossier):
    root, *_ = dossier
    (root / 'src/project/source.cpp').write_text('// Changed without review.\n')
    with pytest.raises(ValueError, match='source/notice changed'):
        audit.verify(root=root)


def test_new_file_cannot_escape_package_inventory(dossier):
    root, *_ = dossier
    (root / 'src/project/new.cpp').write_text('// Uninventoried.\n')
    with pytest.raises(ValueError, match='source inventory changed'):
        audit.verify(root=root)


def test_historic_notice_cannot_be_removed_from_source(dossier):
    root, _, _, review, refresh = dossier
    notice = root / 'src/project/NOTICE'
    notice.write_text('Upstream attribution must survive.\n')
    row = copy.deepcopy(review['files'][0])
    row.update(path='src/project/NOTICE', sha256=audit.sha(notice))
    review['files'].append(row)
    refresh()
    notice.unlink()
    with pytest.raises(FileNotFoundError):
        audit.verify(root=root)


def test_external_package_cannot_be_reclassified_even_after_rehash(dossier):
    root, _, candidate, _, refresh = dossier
    candidate['packages'][0]['path'] = 'src/third_party/copied'
    refresh()
    with pytest.raises(ValueError, match='third-party package reclassified'):
        audit.verify(root=root)


def test_owner_assertion_cannot_expand_to_entire_repository(dossier):
    root, _, _, _, refresh = dossier
    path = root / 'licensing/owner-authority.statement.json'
    owner = audit.read_json(path)
    owner['scope'] = 'All repository and third-party code'
    path.write_text(json.dumps(owner))
    refresh()
    with pytest.raises(ValueError, match='owner statement scope expanded'):
        audit.verify(root=root)


@pytest.mark.parametrize('change', [
    {'effective_release': 'v1.0'}, {'approved_scope': ['src/project/source.cpp']},
    {'root_license_replacement_authorized': True},
])
def test_review_record_cannot_become_a_release_approval(dossier, change):
    root, record, _, _, refresh = dossier
    record.update(change)
    refresh()
    with pytest.raises(ValueError, match='cannot activate'):
        audit.verify(root=root)


def test_file_approval_cannot_be_sneaked_into_draft(dossier):
    root, _, _, review, refresh = dossier
    review['files'][0]['new_license_approved'] = True
    refresh()
    with pytest.raises(ValueError, match='cannot relicense a file'):
        audit.verify(root=root)


def test_external_symlink_cannot_enter_candidate_inventory(dossier, tmp_path):
    root, _, _, review, refresh = dossier
    outside = tmp_path.parent / (tmp_path.name + '-outside')
    outside.write_text('External payload.\n')
    (root / 'src/project/source.cpp').unlink()
    (root / 'src/project/source.cpp').symlink_to(outside)
    review['files'][0].update(baseline_kind='symlink', link_target=str(outside))
    refresh()
    with pytest.raises(ValueError, match='uncontained path'):
        audit.verify(root=root)


def test_original_apache_text_cannot_be_rewritten_by_rehashing(dossier):
    root, _, candidate, _, refresh = dossier
    for name in ('LICENSE', 'LICENSES/Apache-2.0.txt'):
        (root / name).write_text('Not the Apache text.\n')
    candidate['existing_root_license_sha256'] = audit.sha(root / 'LICENSE')
    refresh()
    with pytest.raises(ValueError, match='historical Apache text not preserved'):
        audit.verify(root=root)

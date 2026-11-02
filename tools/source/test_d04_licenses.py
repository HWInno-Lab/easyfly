# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""A verified inventory must not hide a changed source or an unresolved grant."""
import json
from pathlib import Path
import subprocess
import sys

import pytest

import verify_d04_licenses as audit


@pytest.fixture
def dossier(tmp_path):
    grant = {'license': 'Apache-2.0', 'copyright_notice': 'Copyright 2026 EasyFly Project',
             'scope': 'owner_original_ros2_port_contributions_only',
             'preserve_inherited_licenses': True}
    (tmp_path / 'grant.json').write_text(json.dumps(grant))
    rows = []
    for key in sorted(audit.ITEMS):
        directory = tmp_path / key
        directory.mkdir()
        source = directory / 'original.c'
        source.write_text('/* Original, unchanged upstream source. */\n')
        row = {'id': key, 'status': 'open_rightsholder_evidence',
               'files_sha256': {str(source.relative_to(tmp_path)): audit.sha(source)},
               'exact_inventory_roots': [key], 'evidence_paths': ['grant.json'],
               'required_evidence': 'Exact-revision rightsholder grant required.'}
        if key == 'D04-EGO-PORT':
            row.update(status='resolved_scoped_owner_grant', owner_grant='grant.json')
        rows.append(row)
    record = {'schema_version': 1, 'items': rows,
              'inputs_sha256': {'grant.json': audit.sha(tmp_path / 'grant.json')}}
    path = tmp_path / 'review.json'
    path.write_text(json.dumps(record))
    return tmp_path, path, record


def test_intact_materials_keep_three_external_grants_open(dossier):
    root, path, _ = dossier
    result = audit.verify(path, root)
    assert result['materials_verified'] and not result['d04_closed']
    assert {row['id'] for row in result['unresolved']} == audit.ITEMS - {'D04-EGO-PORT'}


def test_changed_source_is_not_covered_by_the_old_review(dossier):
    root, path, _ = dossier
    (root / 'D04-IKD/original.c').write_text('changed source\n')
    with pytest.raises(ValueError, match='covered source changed'):
        audit.verify(path, root)


def test_new_source_cannot_escape_an_exact_inventory(dossier):
    root, path, _ = dossier
    (root / 'D04-LKH/new.c').write_text('unreviewed addition\n')
    with pytest.raises(ValueError, match='inventory changed'):
        audit.verify(path, root)


def test_owner_confirmation_cannot_close_a_third_party_grant(dossier):
    root, path, record = dossier
    row = next(row for row in record['items'] if row['id'] == 'D04-IKD')
    row.update(status='resolved_scoped_owner_grant', owner_grant='grant.json')
    path.write_text(json.dumps(record))
    with pytest.raises(ValueError, match='unsupported scope'):
        audit.verify(path, root)


@pytest.mark.parametrize('change', [
    {'license': 'MIT'}, {'scope': 'all_third_party_code'},
    {'preserve_inherited_licenses': False},
])
def test_owner_grant_scope_stays_narrow_even_after_rehash(dossier, change):
    root, path, record = dossier
    grant_path = root / 'grant.json'
    grant = json.loads(grant_path.read_text())
    grant.update(change)
    grant_path.write_text(json.dumps(grant))
    record['inputs_sha256']['grant.json'] = audit.sha(grant_path)
    path.write_text(json.dumps(record))
    with pytest.raises(ValueError, match='unsupported scope'):
        audit.verify(path, root)


def test_paths_cannot_escape_the_dossier_root(dossier):
    root, path, record = dossier
    record['inputs_sha256']['../outside'] = '0' * 64
    path.write_text(json.dumps(record))
    with pytest.raises(ValueError, match='uncontained'):
        audit.verify(path, root)


def test_duplicate_json_fields_are_not_accepted(dossier):
    root, path, _ = dossier
    path.write_text('{"schema_version": 1, "schema_version": 2}')
    with pytest.raises(ValueError, match='duplicate JSON field'):
        audit.verify(path, root)


def test_strict_cli_rejects_open_grants_without_rejecting_intact_materials(dossier):
    root, path, _ = dossier
    command = [sys.executable, str(Path(audit.__file__)), '--manifest', str(path),
               '--root', str(root)]
    normal = subprocess.run(command, capture_output=True, text=True, check=False)
    strict = subprocess.run(command + ['--require-closed'], capture_output=True,
                            text=True, check=False)
    assert normal.returncode == 0 and strict.returncode == 2
    assert json.loads(normal.stdout) == json.loads(strict.stdout)

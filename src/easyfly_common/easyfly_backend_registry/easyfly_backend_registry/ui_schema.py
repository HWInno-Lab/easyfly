# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_backend_registry/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
"""Shared UI generation: stdlib only, also used by the CMake generator."""
import copy

def generated_ui(base, entries):
    ui = copy.deepcopy(base)
    for field in ui['fields']:
        capability = 'localization' if field['key'] == 'slam' else field.get('capability')
        if not capability: continue
        labels = dict(field['choices'])
        prefix = ['profile', 'disabled'] if field['key'] == 'slam' else ['none'] if field['key'] == 'exploration' else ['profile']
        field['choices'] = [[key, labels.get(key, key)] for key in prefix] + [
            [row['id'], labels.get(row['id'], row['id'])] for row in entries if row['capability'] == capability]
    return ui

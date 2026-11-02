# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
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

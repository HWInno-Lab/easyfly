# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_backend_registry/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
"""Generate C++ lookup from the same metadata consumed by Python and the UI."""
import json
import hashlib
from pathlib import Path
import sys

data, output = map(Path, sys.argv[1:])
entries = json.loads((data / 'backends.json').read_text())['backends']
# Header-only lookup definitions differ between installed registry snapshots.
# Give each snapshot its own portable inline namespace so ELF weak/unique symbol
# interposition cannot substitute an older DSO's schema for a newly added backend.
snapshot = json.dumps(entries, sort_keys=True).encode()
snapshot += b''.join((data / row['parameter_schema']).read_bytes() for row in entries)
metadata_namespace = 'metadata_' + hashlib.sha256(snapshot).hexdigest()
lines = ['// Generated from backends.json; do not edit.', '#pragma once',
         '#include <easyfly_backend_registry/configuration.hpp>', '#include <stdexcept>', '#include <cmath>', '#include <string>', 'namespace easyfly::backends { inline namespace ' + metadata_namespace + ' {',
         'struct Descriptor {const char * capability; const char * id; const char * package; const char * entry;};',
         'inline constexpr Descriptor descriptors[] = {']
seen = set()
for row in entries:
    pair = row['capability'], row['id']
    if pair in seen:
        raise ValueError('duplicate backend: ' + str(pair))
    seen.add(pair)
    e = row['entrypoint']
    lines.append('  {' + ','.join(json.dumps(v) for v in [*pair, e['package'], e.get('symbol', e.get('file', e.get('executable')))]) + '},')
lines += ['};', 'inline const Descriptor & require_backend(const std::string & capability, const std::string & id) {',
          '  for (const auto & item : descriptors) {if (capability == item.capability && id == item.id) {return item;}}',
          '  throw std::invalid_argument("unknown " + capability + " backend: " + id);', '}', '}']
# Validate existing strong parameter structures at their actual creation boundary.
lines.pop()  # namespace close
for capability, backend in [('multi_tracking', 'sort'), ('grid_planning', 'astar')]:
    row = next(r for r in entries if r['capability'] == capability and r['id'] == backend)
    schema = json.loads((data / row['parameter_schema']).read_text())
    lines.append('template<class Params> inline void validate_' + backend + '(const Params & p) {')
    for name, spec in schema['properties'].items():
        checks = ['!std::isfinite(static_cast<double>(p.' + name + '))']
        if 'minimum' in spec: checks.append('p.' + name + ' < ' + str(spec['minimum']))
        if 'maximum' in spec: checks.append('p.' + name + ' > ' + str(spec['maximum']))
        lines.append('  if (' + ' || '.join(checks) + ') {throw std::invalid_argument("invalid ' + backend + '/' + name + '");}')
    lines.append('}')
# Only the four synchronous computation boundaries have generic C++ parameter maps.
def cpp_value(value):
    if isinstance(value, bool): return 'Value{' + str(value).lower() + '}'
    if isinstance(value, int): return 'Value{std::int64_t{' + str(value) + '}}'
    if isinstance(value, float): return 'Value{double{' + repr(value) + '}}'
    if isinstance(value, str): return 'Value{std::string{' + json.dumps(value) + '}}'
    if isinstance(value, list): return 'Value{std::vector<std::string>{' + ','.join(json.dumps(x) for x in value) + '}}'
    raise ValueError(value)
def bound(value, default):
    return str(value) if value is not None else default
lines.append('inline const Schema & parameter_schema(const std::string & capability, const std::string & backend) {')
for row in entries:
    if row['capability'] not in ('multi_tracking', 'grid_planning', 'detection', 'single_tracking'): continue
    schema = json.loads((data / row['parameter_schema']).read_text())
    lines.append('  if (capability == ' + json.dumps(row['capability']) + ' && backend == ' + json.dumps(row['id']) + ') {')
    lines.append('    static const Schema schema{' + json.dumps(row['capability']) + ',' + json.dumps(row['id']) + ', {')
    for name, spec in schema['properties'].items():
        supported = {'type', 'minimum', 'maximum', 'minLength', 'maxLength', 'minItems', 'maxItems', 'items', 'default', 'title', 'description'}
        if set(spec) - supported or spec.get('type') not in {'number', 'integer', 'boolean', 'string', 'array'}:
            raise ValueError('extend the C++ schema bridge before adding this constraint: ' + name)
        if spec['type'] == 'array' and (spec.get('items', {}).get('type') != 'string' or set(spec['items']) - {'type', 'maxLength'}):
            raise ValueError('C++ bridge currently supports string arrays only: ' + name)
        values = [json.dumps(name), json.dumps(spec['type']), bound(spec.get('minimum'), '-std::numeric_limits<double>::infinity()'),
            bound(spec.get('maximum'), 'std::numeric_limits<double>::infinity()'),
            str(spec.get('minLength', spec.get('minItems', 0))), str(spec.get('maxLength', spec.get('maxItems', 18446744073709551615))) + 'ULL',
            str(spec.get('items', {}).get('maxLength', 18446744073709551615)) + 'ULL',
            cpp_value(spec['default']) if 'default' in spec else 'std::nullopt', str(name in schema.get('required', [])).lower()]
        lines.append('      {' + ','.join(values) + '},')
    lines.append('    }}; return schema; }')
lines.append('  throw std::invalid_argument("no C++ parameter schema: " + capability + "/" + backend);')
lines.append('}')
lines.append('inline Parameters resolve_parameters(const std::string & capability, const std::string & backend, const Parameters & input) {return resolve(parameter_schema(capability, backend), input);}')
lines.append('} }')
header = '\n'.join(lines) + '\n'
if not output.exists() or output.read_text() != header: output.write_text(header)

import importlib.util
spec = importlib.util.spec_from_file_location('backend_ui_schema', data.parent / 'ui_schema.py')
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
ui = module.generated_ui(json.loads((data / 'algorithm_ui.json').read_text()), entries)
ui_file = output.parent / 'algorithm_ui.json'
ui_text = json.dumps(ui, ensure_ascii=False, indent=2) + '\n'
if not ui_file.exists() or ui_file.read_text() != ui_text: ui_file.write_text(ui_text)

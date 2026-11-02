#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Replay a captured CTest group, remapping conflicting DDS domains in fixtures.

Only generated CTest files change. Production CMake/source and the original
discovery/log stay intact. Call after other baseline tests have finished.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from extension_baseline import active_domains, run, write


def quote(value):
    value = str(value)
    if ']==]' in value:
        raise ValueError('unsupported CMake bracket terminator')
    return '[==[' + value + ']==]'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--package', required=True)
    parser.add_argument('--replacement-domain', type=int, required=True)
    parser.add_argument('--test-name', help='Replay only one exact captured test name')
    args = parser.parse_args()
    if not 1 <= args.replacement_domain <= 232:
        parser.error('use a nonzero domain in 1..232')
    baseline = args.baseline.resolve()
    discovery = json.loads((baseline / ('discover-' + args.package + '.log')).read_text())
    if not discovery.get('tests'):
        parser.error('no captured tests')
    if args.test_name:
        discovery['tests'] = [t for t in discovery['tests'] if t['name'] == args.test_name]
        if not discovery['tests']:
            parser.error('requested name is not in the captured test group')
    occupied = active_domains()
    replacement = str(args.replacement_domain)
    if replacement in occupied or '218' in occupied:
        parser.error('replacement/default DDS domain is already in use')
    suffix = '-' + args.test_name if args.test_name else ''
    out = baseline / ('replay-' + args.package + suffix)
    out.mkdir(exist_ok=False)
    lines, changes = [], []
    for test in discovery['tests']:
        lines.append('add_test(' + quote(test['name']) + ' ' +
                     ' '.join(quote(x) for x in test['command']) + ')')
        for prop in test.get('properties', []):
            if prop['name'] not in {'LABELS', 'REQUIRED_FILES', 'TIMEOUT',
                                    'WORKING_DIRECTORY', 'ENVIRONMENT', 'RUN_SERIAL'}:
                raise ValueError('unhandled test property: ' + prop['name'])
            value = prop['value']
            if prop['name'] == 'ENVIRONMENT':
                updated = []
                for entry in value:
                    if entry.startswith('ROS_DOMAIN_ID=') and entry.split('=', 1)[1] in occupied:
                        changes.append({'test': test['name'], 'old': entry,
                                        'new': 'ROS_DOMAIN_ID=' + replacement})
                        entry = 'ROS_DOMAIN_ID=' + replacement
                    updated.append(entry)
                value = updated
            if isinstance(value, list):
                value = ';'.join(str(x) for x in value)
            if isinstance(value, bool):
                value = 'TRUE' if value else 'FALSE'
            lines.append('set_tests_properties(' + quote(test['name']) +
                         ' PROPERTIES ' + prop['name'] + ' ' + quote(value) + ')')
    (out / 'CTestTestfile.cmake').write_text('\n'.join(lines) + '\n')
    record = {'scope': 'Replay original test commands with explicit DDS fixture-domain remap',
              'commands': [], 'remaps': changes, 'original_discovery': str(
                  baseline / ('discover-' + args.package + '.log')),
              'selected_tests': [t['name'] for t in discovery['tests']]}
    command = ['bash', '--noprofile', '--norc', '-c',
               'source "$1/local_setup.bash" && shift && exec "$@"',
               'bash', str(baseline / 'install'), 'ctest', '--test-dir', str(out),
               '--output-on-failure', '-j1', '--timeout', '180', '--output-junit', str(out / 'results.xml')]
    code = run(out, record, 'replay', command)
    cases = list(ET.parse(out / 'results.xml').getroot().iter('testcase'))
    record['ctest_entries'] = len(cases)
    record['passed'] = code == 0 and len(cases) == len(discovery['tests']) and all(
        c.find('failure') is None and c.find('skipped') is None for c in cases)
    write(out / 'summary.json', record)
    return 0 if record['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

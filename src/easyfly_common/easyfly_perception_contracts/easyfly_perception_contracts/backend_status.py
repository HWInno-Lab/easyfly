# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_perception_contracts/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
"""Read-only verified backend diagnostics. This never grants control permission."""
import copy
import time
from diagnostic_msgs.msg import DiagnosticArray
from easyfly_backend_registry import Registry, ROLES, loads
from . import NativeSubscription, writer, stamp


class BackendMonitor:
    def __init__(self, node, resolved):
        self.node = node
        self.registry = Registry()
        self.expected = copy.deepcopy(resolved['selection']['backends'])
        self.enabled = set(resolved['enabled_roles'])
        self.records, self.retired, self.current = {}, {}, {}
        self.errors = {}
        self.subscriptions = []
        for role in sorted(self.enabled):
            if role not in ('detection', 'multi_tracking', 'single_tracking', 'global_grid', 'combined_grid'): continue
            topic = '/platform/backends/' + role
            self.subscriptions.append(NativeSubscription(node, DiagnosticArray, topic,
                lambda message, info, key=role: self.receive(key, message, info)))

    def receive(self, role, message, info):
        try:
            source = writer(self.node, '/platform/backends/' + role, info)
            if not source: raise ValueError('ambiguous_or_unverified_writer')
            if len(message.status) != 1: raise ValueError('invalid_status_envelope')
            status = message.status[0]
            if status.name != role or not status.hardware_id or len(status.hardware_id) > 128 or len(status.values) > 16: raise ValueError('invalid_status_identity')
            values = {}
            for item in status.values:
                if len(item.key) > 64 or len(item.value) > 2*1024*1024 or item.key in values: raise ValueError('invalid_status_fields')
                values[item.key] = item.value
            if set(values) != {'schema_version','requested_backend','active_backend','created','phase','provider','effective_parameters','sequence'}: raise ValueError('invalid_status_fields')
            if not values['requested_backend'] or len(values['requested_backend']) > 64 or len(values['active_backend']) > 64: raise ValueError('invalid_backend_identity')
            if values['schema_version'] != '1' or values['created'] not in ('true','false') or values['phase'] not in ('creating','waiting_input','ready','failed'): raise ValueError('invalid_status_version_or_phase')
            if len(status.message) > 4096 or len(values['provider']) > 256: raise ValueError('invalid_status_detail')
            level = status.level[0] if isinstance(status.level, bytes) else int(status.level)
            if level != (0 if values['phase'] == 'ready' else 2 if values['phase'] == 'failed' else 1): raise ValueError('status_level_phase_disagree')
            sequence = int(values['sequence'])
            if sequence <= 0 or sequence > 2**64-1: raise ValueError('invalid_status_sequence')
            generation = (source, status.hardware_id)
            old = self.current.get(role)
            retired = self.retired.setdefault(role, set())
            if generation in retired: return
            if old and old['generation'] == generation and sequence <= old['sequence']: return
            parameters = loads(values['effective_parameters'])
            if values['created'] == 'true':
                checked = self.registry.parameters(ROLES[role], values['active_backend'], parameters)
                if checked != parameters: raise ValueError('incomplete_effective_parameters')
            record = dict(generation=generation, sequence=sequence, received=time.monotonic(), stamp=stamp(message.header),
                requested=values['requested_backend'], active=values['active_backend'], created=values['created']=='true',
                phase=values['phase'], provider=values['provider'], parameters=parameters, reason=status.message)
            if old and old['generation'] != generation:
                retired.add(old['generation'])
                if len(retired) > 128: raise ValueError('status_source_history_exhausted')
            self.current[role] = dict(generation=generation, sequence=sequence)
            self.records[role] = record
            self.errors.pop(role, None)
        except Exception as error:
            # Bad diagnostics invalidate the display, not the platform's independent control guards.
            self.errors[role] = str(error)[:4096]
            self.records.pop(role, None)

    def snapshot(self):
        now = time.monotonic(); ros_now = self.node.get_clock().now().nanoseconds
        rows = {}
        for role, expected in self.expected.items():
            row = dict(requested_backend=expected['backend'], enabled=role in self.enabled,
                       created=False, active_backend='', provider='', ready=False,
                       phase='disabled' if role not in self.enabled else 'not_observed',
                       reason='role_not_started_by_profile' if role not in self.enabled else 'runtime_report_not_observed')
            if role in self.enabled:
                record = self.records.get(role)
                if role in self.errors: row.update(phase='unavailable', reason=self.errors[role])
                elif record:
                    age = (ros_now - record['stamp']) / 1e9
                    row.update(created=record['created'], active_backend=record['active'], provider=record['provider'],
                        phase=record['phase'], reason=record['reason'], effective_parameters=record['parameters'])
                    if self.node.count_publishers('/platform/backends/' + role) != 1:
                        row.update(phase='unavailable', reason='ambiguous_or_missing_writer')
                    elif now-record['received'] > 2.5 or record['stamp'] <= 0 or age < -.2 or age > 2.5:
                        row.update(phase='stale', reason='runtime_report_stale')
                    elif record['requested'] != expected['backend'] or (record['created'] and (record['active'] != expected['backend'] or record['parameters'] != expected['parameters'])):
                        row.update(phase='mismatch', reason='requested_and_runtime_configuration_differ')
                    else: row['ready'] = record['created'] and record['phase'] == 'ready'
                    if record['reason'] == 'role_not_selected_by_strategy' and row['phase'] == 'waiting_input': row['phase'] = 'not_selected'
                    row['provider_fallback'] = bool(role == 'detection' and expected['parameters'].get('use_gpu') and record['provider'] == 'CPUExecutionProvider')
            if role not in ('detection', 'multi_tracking', 'single_tracking', 'global_grid', 'combined_grid'):
                row.update(enabled=True, created=None, ready=None, phase='dedicated_adapter', reason='use_existing_guarded_adapter_health')
            rows[role] = row
        return rows

# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Durable exact-intent mission ownership on top of the native reviewed service."""

import fcntl
import hashlib
import json
import os
from pathlib import Path
import sqlite3
import time
import uuid

from easyfly_connection_manager.remote_service import private_directory

from .configuration import HEX


def digest(value):
    """Hash stable content so request reuse cannot silently change an intent."""
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


class Operations:
    """Persist ownership before dispatch and resolve timeouts using read-only native lookup."""

    def __init__(self, config, bridge):
        self.config, self.bridge = config, bridge
        self.boot = uuid.uuid4().hex
        root = Path(config['state_directory'])
        private_directory(root)
        self.fd = os.open(root / 'web.lock', os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
        fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.db = sqlite3.connect(root / 'intents.sqlite3')
        (root / 'intents.sqlite3').chmod(0o600)
        self.db.execute('PRAGMA synchronous=FULL')
        self.db.execute('CREATE TABLE IF NOT EXISTS intents '
                        '(id TEXT PRIMARY KEY, owner TEXT NOT NULL, client TEXT NOT NULL, '
                        'digest TEXT NOT NULL, body TEXT NOT NULL)')
        self.db.commit()
        self.reviews = {}

    def validate(self, value, fields):
        """Accept exact schemas and browser intent IDs, never arbitrary ROS field mappings."""
        if not isinstance(value, dict) or set(value) != set(fields):
            raise ValueError('invalid request fields')
        identities = {'client_id', 'request_id', 'boot_id', 'review_id', 'execution_id'}
        for key in set(fields) & identities:
            if not isinstance(value[key], str) or not HEX.fullmatch(value[key]):
                raise ValueError('invalid request identity')
        if 'boot_id' in fields and value['boot_id'] != self.boot:
            raise ValueError('gateway restarted; refresh and resolve the original intent')

    def review(self, actor, value):
        """Bind exact returned bytes and native server epoch to a short-lived personal review."""
        self.validate(value, {'mission_name', 'client_id'})
        name = value['mission_name']
        if name not in self.config['missions']:
            raise ValueError('mission is outside the administrator allowlist')
        details = self.bridge.call('details', {'mission_name': name})
        if (not details['success'] or details['mission_name'] != name or
                len(details['content'].encode()) > 65536 or
                hashlib.sha256(details['content'].encode()).hexdigest() !=
                details['content_sha256']):
            raise ValueError('native mission content unavailable or integrity check failed')
        session = self.bridge.call('reviewed', {'operation': 'session'})
        if not session['success'] or not HEX.fullmatch(session['server_instance_id']):
            raise ValueError('native mission session is unavailable')
        now = time.monotonic()
        self.reviews = {key: row for key, row in self.reviews.items() if row['expires'] > now}
        if len(self.reviews) >= 64:
            raise ValueError('review capacity exhausted; wait for previous reviews to expire')
        identity = uuid.uuid4().hex
        row = dict(owner=actor['name'], client=value['client_id'], expires=now + 120.,
                   mission_name=name, content_sha256=details['content_sha256'],
                   server_instance_id=session['server_instance_id'])
        self.reviews[identity] = row
        return dict(review_id=identity, content=details['content'], **row)

    def record(self, actor, value):
        """Check personal account and browser intent ownership before querying or canceling."""
        row = self.db.execute('SELECT owner,client,body FROM intents WHERE id=?',
                              (value['request_id'],)).fetchone()
        if not row or row[0] != actor['name'] or row[1] != value['client_id']:
            raise ValueError('unknown or unowned request identity')
        return json.loads(row[2])

    def save(self, value):
        """Commit evidence without converting an absent reply into a terminal success."""
        self.db.execute('UPDATE intents SET body=? WHERE id=?',
                        (json.dumps(value, allow_nan=False), value['request_id']))
        self.db.commit()
        return value

    def native(self, row, operation, execution_id=''):
        """Send only the original native request identity and reviewed server instance."""
        fields = dict(operation=operation, server_instance_id=row['server_instance_id'],
                      client_request_id=row['native_request_id'], mission_name=row['mission_name'],
                      expected_content_sha256=row['content_sha256'], execution_id=execution_id)
        try:
            reply = self.bridge.call('reviewed', fields)
            if (reply['server_instance_id'] != row['server_instance_id'] or
                    reply['client_request_id'] != row['native_request_id']):
                raise RuntimeError('native server identity changed; original work unresolved')
            row.update(reply=reply, reply_received_ns=time.time_ns(), error=None)
        except (RuntimeError, OSError) as exc:
            row.update(error=str(exc), reply=None)
        return self.save(row)

    def start(self, actor, value):
        """Reserve one immutable intent durably before the first and only native start call."""
        self.validate(value, {'boot_id', 'client_id', 'request_id', 'review_id'})
        if actor['role'] != 'operator' or not self.config['enable_execution']:
            raise PermissionError('this account or deployment is read-only')
        previous = self.db.execute('SELECT digest FROM intents WHERE id=?',
                                   (value['request_id'],)).fetchone()
        if previous:
            row = self.record(actor, value)
            # A reconnect changes gateway boot, not the original personal intent.
            if previous[0] != digest({key: val for key, val in value.items() if key != 'boot_id'}):
                raise ValueError('request ID already belongs to different reviewed content')
            return row
        review = self.reviews.get(value['review_id'])
        if (not review or review['owner'] != actor['name'] or
                review['client'] != value['client_id'] or review['expires'] < time.monotonic()):
            raise ValueError('review expired or belongs to another account/browser')
        if self.db.execute('SELECT COUNT(*) FROM intents').fetchone()[0] >= 256:
            raise ValueError('intent capacity exhausted; archive during offline maintenance')
        row = dict(request_id=value['request_id'], native_request_id=uuid.uuid4().hex,
                   mission_name=review['mission_name'], content_sha256=review['content_sha256'],
                   server_instance_id=review['server_instance_id'], reply=None,
                   cancel_requested=False, error='dispatch unresolved', created_ns=time.time_ns())
        self.db.execute('INSERT INTO intents VALUES (?,?,?,?,?)',
                        (value['request_id'], actor['name'], value['client_id'],
                         digest({key: val for key, val in value.items() if key != 'boot_id'}),
                         json.dumps(row)))
        self.db.commit()
        self.reviews.pop(value['review_id'])
        return self.native(row, 'start')

    def lookup(self, actor, value):
        """Resolve an existing personal intent without resending its start or cancel operation."""
        self.validate(value, {'client_id', 'request_id'})
        return self.native(self.record(actor, value), 'lookup')

    def cancel(self, actor, value):
        """Cancel only the exact execution returned for this account and original request."""
        self.validate(value, {'client_id', 'request_id', 'boot_id', 'execution_id'})
        if actor['role'] != 'operator' or not self.config['enable_execution']:
            raise PermissionError('this account or deployment is read-only')
        row = self.record(actor, value)
        reply = row.get('reply')
        if (not reply or reply['execution_id'] != value['execution_id'] or
                not reply['admission_resolved'] or not reply['admission_success']):
            raise ValueError('exact admitted execution identity is required')
        if row['cancel_requested']:
            return self.native(row, 'lookup')
        row['cancel_requested'] = True
        self.save(row)
        return self.native(row, 'cancel', value['execution_id'])

    def close(self):
        """Close local durable ownership; never imply the remote task has stopped."""
        self.db.close()
        os.close(self.fd)

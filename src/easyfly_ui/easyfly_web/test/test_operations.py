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

"""Exercise durable ownership across uncertain replies and gateway/native restarts."""

import hashlib
import uuid

from easyfly_web.operations import Operations
import pytest


class Native:
    """Inject a missing native reply after dispatch without pretending a task completed."""

    def __init__(self):
        self.calls = []
        self.content = 'type=hover\nwait=0.1\n'
        self.server = 'a' * 32
        self.lost = True

    def call(self, kind, fields):
        """Record external effects; resolve the same identity on later read-only lookup."""
        self.calls.append((kind, dict(fields)))
        if kind == 'details':
            return dict(success=True, mission_name=fields['mission_name'], content=self.content,
                        content_sha256=hashlib.sha256(self.content.encode()).hexdigest())
        if fields['operation'] == 'start' and self.lost:
            raise RuntimeError('reply lost after dispatch')
        return dict(success=True, server_instance_id=self.server,
                    client_request_id=fields.get('client_request_id', ''),
                    execution_id='b' * 32, admission_resolved=True, admission_success=True,
                    execution_pending=True)


def provision(tmp_path):
    """Provision a private gateway store and explicit personal accounts."""
    config = dict(state_directory=str(tmp_path / 'state'), enable_execution=True,
                  missions=['wait.mission'])
    native = Native()
    owner = Operations(config, native)
    actor = dict(name='alice', role='operator')
    client = 'c' * 32
    review = owner.review(actor, dict(client_id=client, mission_name='wait.mission'))
    intent = dict(boot_id=owner.boot, client_id=client,
                  request_id=uuid.uuid4().hex, review_id=review['review_id'])
    return config, native, owner, actor, intent


def test_uncertain_start_survives_restart_without_resend(tmp_path):
    """An accepted-side effect with lost reply is resolved by lookup after gateway restart."""
    config, native, owner, actor, intent = provision(tmp_path)
    row = owner.start(actor, intent)
    assert row['reply'] is None and 'reply lost' in row['error']
    assert owner.start(actor, intent) == row
    owner.close()
    owner = Operations(config, native)
    try:
        intent['boot_id'] = owner.boot
        assert owner.start(actor, intent) == row
        result = owner.lookup(actor, {key: intent[key] for key in ['client_id', 'request_id']})
        assert result['reply']['execution_id'] == 'b' * 32
        assert sum(fields.get('operation') == 'start' for _, fields in native.calls) == 1
        assert result['native_request_id'] == row['native_request_id']
    finally:
        owner.close()


def test_roles_review_binding_and_exact_cancel(tmp_path):
    """Read-only accounts, other browsers and forged execution IDs never reach mutation."""
    _, native, owner, actor, intent = provision(tmp_path)
    try:
        with pytest.raises(PermissionError):
            owner.start(dict(name='alice', role='viewer'), intent)
        with pytest.raises(ValueError):
            owner.start(actor, dict(intent, client_id='d' * 32))
        with pytest.raises(ValueError):
            owner.start(dict(name='bob', role='operator'), intent)
        native.lost = False
        owner.start(actor, intent)
        lookup = {key: intent[key] for key in ['client_id', 'request_id']}
        with pytest.raises(ValueError):
            owner.lookup(dict(name='bob', role='operator'), lookup)
        with pytest.raises(ValueError):
            owner.start(actor, dict(intent, review_id='e' * 32))
        cancel = dict(lookup, boot_id=owner.boot, execution_id='f' * 32)
        with pytest.raises(ValueError):
            owner.cancel(actor, cancel)
        cancel['execution_id'] = 'b' * 32
        owner.cancel(actor, cancel)
        owner.cancel(actor, cancel)
        assert sum(fields.get('operation') == 'cancel' for _, fields in native.calls) == 1
    finally:
        owner.close()


def test_native_restart_cannot_adopt_original_intent(tmp_path):
    """A changed native provider epoch remains unresolved and never receives a new start."""
    _, native, owner, actor, intent = provision(tmp_path)
    try:
        owner.start(actor, intent)
        native.server = 'd' * 32
        result = owner.lookup(actor, {key: intent[key] for key in ['client_id', 'request_id']})
        assert result['reply'] is None and 'identity changed' in result['error']
        assert sum(fields.get('operation') == 'start' for _, fields in native.calls) == 1
    finally:
        owner.close()


def test_review_expiry_and_catalog_allowlist(tmp_path):
    """Expired or unconfigured content does not acquire any durable execution intent."""
    _, native, owner, actor, intent = provision(tmp_path)
    try:
        owner.reviews[intent['review_id']]['expires'] = 0
        with pytest.raises(ValueError):
            owner.start(actor, intent)
        with pytest.raises(ValueError):
            owner.review(actor, dict(client_id=intent['client_id'], mission_name='../escape'))
        assert not any(fields.get('operation') == 'start' for _, fields in native.calls)
    finally:
        owner.close()

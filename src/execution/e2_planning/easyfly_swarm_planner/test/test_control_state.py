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

"""Unknown native responses and persistent ownership cannot become permission to resend."""

from concurrent.futures import Future
import fcntl
import json
import os
from types import SimpleNamespace

from easyfly_swarm_planner.control import Controller, Operation
import pytest


def completed(value):
    """Construct a completed protocol response for deterministic ownership-state tests."""
    future = Future()
    future.set_result(value)
    return future


def owner(response):
    """Count all action admissions and persistence operations on a minimal transport boundary."""
    sent, persisted = [], []

    def send(value, goal_uuid):
        sent.append((value, goal_uuid))
        return completed(response)

    value = SimpleNamespace(operations=[], clients={
        'alpha': {'trajectory': SimpleNamespace(send_goal_async=send)}},
        save=lambda: persisted.append(True))
    return value, sent, persisted


def test_missing_admission_response_is_not_rejection_or_release():
    """An unexpected empty reply preserves uncertainty and never triggers another goal send."""
    manager, sent, persisted = owner(None)
    operation = Operation(manager, 'alpha', 'trajectory', object())
    for _ in range(3):
        operation.poll()
    assert operation.status == 'admission_unknown' and not operation.terminal()
    assert len(sent) == 1 and len(persisted) >= 1
    assert bytes(sent[0][1].uuid).hex() == operation.record()['goal_id']


def test_unknown_result_retains_ownership_and_cancels_only_once():
    """A native unknown-goal result cannot be treated as successful remote retirement."""
    canceled = []
    handle = SimpleNamespace(
        accepted=True,
        get_result_async=lambda: completed(SimpleNamespace(status=0, result=None)),
        cancel_goal_async=lambda: canceled.append(True))
    manager, sent, _ = owner(handle)
    operation = Operation(manager, 'alpha', 'trajectory', object())
    operation.poll()
    assert operation.status == 'result_unknown' and not operation.terminal()
    operation.cancel_requested = True
    for _ in range(3):
        operation.poll()
    assert len(sent) == 1 and len(canceled) == 1 and not operation.terminal()


def test_unresolved_ledger_and_live_lock_block_new_controller(tmp_path):
    """Neither a previous unresolved task nor a live coordinator can be silently replaced."""
    fleet = SimpleNamespace(path=tmp_path / 'manifest.json')
    ledger = tmp_path / 'swarm-control.json'
    ledger.write_text(json.dumps({'busy': True}))
    with pytest.raises(RuntimeError, match='unresolved'):
        Controller(fleet, {}, tmp_path, None)
    ledger.write_text(json.dumps({'busy': False}))
    descriptor = os.open(tmp_path / 'swarm-control.lock', os.O_RDWR)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(BlockingIOError):
            Controller(fleet, {}, tmp_path, None)
    finally:
        os.close(descriptor)


def test_result_and_cancel_transport_exceptions_preserve_ownership():
    """A broken native client must not interrupt retirement of other vehicle goals."""
    attempts = []

    def unavailable():
        raise RuntimeError('result service unavailable')

    def cancel_unavailable():
        attempts.append(True)
        raise RuntimeError('cancel service unavailable')

    handle = SimpleNamespace(accepted=True, get_result_async=unavailable,
                             cancel_goal_async=cancel_unavailable)
    manager, sent, _ = owner(handle)
    operation = Operation(manager, 'alpha', 'trajectory', object())
    operation.poll()
    assert operation.status == 'result_unknown' and not operation.terminal()
    operation.cancel_requested = True
    for _ in range(3):
        operation.poll()
    assert len(attempts) == 1 and len(sent) == 1 and not operation.terminal()
    assert 'cancel transport unknown' in operation.message

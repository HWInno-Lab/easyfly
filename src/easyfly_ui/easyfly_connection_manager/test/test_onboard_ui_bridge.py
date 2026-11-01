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
"""Adapter contracts: private drafts, exact task identity and no implicit replay."""

import copy
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import uuid

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from easyfly_connection_manager import onboard_ui_bridge as module  # noqa: E402


def request(op, data=None):
    return dict(id=uuid.uuid4().hex, op=op, data=data or {})


class Client:
    def __init__(self, config):
        self.config = config
        self.url = SimpleNamespace(hostname="aircraft.test")
        self.calls = []
        self.lose_reply = False

    def status(self):
        return dict(
            v=2,
            server_id=self.config["server_id"],
            boot_id="b" * 32,
            ros_domain_id=42,
            local_only=False,
            recipe=module.profiles.template(),
            recipe_hash="saved-version",
            state=dict(phase="running"),
        )

    def operation(self, snapshot, op, values, request_id):
        self.calls.append((op, copy.deepcopy(values), request_id))
        if self.lose_reply:
            raise OSError("arranged reply loss")
        if op == "mission_read":
            return dict(
                accepted=True, name=values["name"], sha256="c" * 64, content="mission bytes"
            )
        if op == "mission_lookup":
            return dict(
                accepted=True,
                task=dict(
                    request_id=values["request_id"], execution_pending=False, execution_id=7
                ),
            )
        return dict(accepted=True, phase="request_unresolved")


@pytest.fixture
def bridge(tmp_path, monkeypatch):
    settings = dict(
        server_id="a" * 32,
        client_id="d" * 32,
        url="https://aircraft.test:1234",
        ca_file="/private/ca",
        token_file="/private/operator.token",
    )
    monkeypatch.setattr(module, "client_settings", lambda path: settings)
    adapter = module.Bridge(tmp_path / "ui", client_factory=Client)
    adapter.connect_path(tmp_path / "client.json", "Aircraft")
    return adapter


def test_lost_task_response_is_persisted_and_never_resubmitted(bridge, monkeypatch):
    bridge.review = dict(name="mission.yaml", sha256="c" * 64)
    bridge.client.lose_reply = True
    start = request(
        "operate",
        dict(
            operation="mission_start",
            values=dict(name="mission.yaml", sha256="c" * 64, confirmed=True),
        ),
    )
    with pytest.raises(OSError):
        bridge.handle(start)
    assert bridge.task_request == start["id"]
    assert module.read_private(bridge.task_path())["pending"]
    # A new desktop adapter recovers the identity, not the start command.
    recovered = module.Bridge(bridge.directory, client_factory=Client)
    recovered.connect_path(bridge.connection_path)
    assert recovered.task_request == start["id"]
    assert recovered.client.calls == []
    recovered.review = bridge.review
    with pytest.raises(ValueError, match="原任务"):
        recovered.handle(request("operate", start["data"]))
    lookup = recovered.handle(request("operate", dict(operation="mission_lookup", values={})))
    assert recovered.client.calls == [
        ("mission_lookup", {"request_id": start["id"]}, recovered.client.calls[0][2])
    ]
    assert not lookup["task_pending"]
    assert lookup["task_request"] == start["id"]


def test_task_requires_exact_read_review_and_bounded_body(bridge):
    values = dict(name="mission.yaml", sha256="c" * 64, confirmed=True)
    with pytest.raises(ValueError, match="审阅"):
        bridge.handle(request("operate", dict(operation="mission_start", values=values)))
    bridge.handle(
        request("operate", dict(operation="mission_read", values={"name": "mission.yaml"}))
    )
    result = bridge.handle(request("operate", dict(operation="mission_start", values=values)))
    assert result["task_pending"]
    count = len(bridge.client.calls)
    with pytest.raises(ValueError, match="大小"):
        bridge.handle(
            request(
                "operate",
                dict(operation="mission_save", values=dict(name="big.yaml", content="汉" * 65536)),
            )
        )
    assert len(bridge.client.calls) == count


def test_local_incomplete_draft_does_not_mutate_platform_and_preserves_target(bridge):
    incomplete = {"name": "not complete yet", "parameters": "[unfinished"}
    before = copy.deepcopy(bridge.snapshot)
    draft = request(
        "save_draft",
        dict(
            recipe=incomplete,
            expected_hash="old-version",
            mission=dict(name="scratch.yaml", content="steps: ["),
            editor_overrides={"component/array": "[unfinished"},
        ),
    )
    bridge.handle(draft)
    restored = bridge.handle(request("load_draft"))
    assert restored["recipe"] == incomplete
    assert restored["target"]["server_id"] == "a" * 32
    assert restored["expected_hash"] == "old-version"
    assert restored["editor_overrides"]["component/array"] == "[unfinished"
    assert bridge.snapshot == before
    assert bridge.client.calls == []
    serialized = (bridge.directory / "draft.json").read_text()
    assert "operator.token" not in serialized and '"ca_file"' not in serialized
    assert (bridge.directory / "draft.json").stat().st_mode & 0o077 == 0


def test_recent_platforms_are_reused_without_exporting_credentials(bridge):
    bridge.connect_path(bridge.connection_path, "Aircraft renamed")
    rows = bridge.recent()
    assert len(rows) == 1
    assert rows[0]["alias"] == "Aircraft renamed"
    assert "token_file" not in json.dumps(rows)
    assert "ca_file" not in json.dumps(rows)


def test_private_draft_rejects_symlinks_and_oversize(bridge, tmp_path):
    outside = tmp_path / "outside.json"
    outside.write_text("{}")
    outside.chmod(0o600)
    (bridge.directory / "draft.json").symlink_to(outside)
    with pytest.raises(OSError):
        bridge.handle(request("load_draft"))
    with pytest.raises(ValueError, match="大小"):
        bridge.handle(request("save_draft", dict(recipe={"large": "x" * module.MAX_DRAFT})))


def test_operations_use_the_authenticated_boot_and_original_id(bridge):
    original = request("operate", dict(operation="check", values={"recipe_hash": "saved-version"}))
    bridge.handle(original)
    assert bridge.client.calls[-1] == ("check", {"recipe_hash": "saved-version"}, original["id"])
    bridge.fresh_at = 0
    with pytest.raises(ValueError, match="新鲜"):
        bridge.handle(request("operate", original["data"]))
    assert len(bridge.client.calls) == 1


def test_template_configuration_requires_capability_without_upload_replay(bridge):
    """A template save is one configure operation, never a sequence of uploads."""
    recipe = module.profiles.template()
    recipe['components']['color_line']['enabled'] = True
    values = dict(recipe=recipe, expected_hash='saved-version', templates=['color_line'])
    with pytest.raises(ValueError, match='更新并重启'):
        bridge.handle(request('operate', dict(operation='configure', values=values)))
    assert bridge.client.calls == []
    bridge.snapshot['capabilities'] = dict(parameter_templates=1)
    operation = request('operate', dict(operation='configure', values=values))
    bridge.handle(operation)
    assert bridge.client.calls == [('configure', values, operation['id'])]
    bridge.client.calls.clear()
    bridge.handle(request('operate', dict(
        operation='configure', values=dict(values, templates=[]))))
    assert 'templates' not in bridge.client.calls[0][1]


def test_template_source_draft_stays_local_and_hello_exposes_defaults(bridge):
    """Source choices survive offline drafts without extending remote recipe schema."""
    hello = bridge.handle(request('hello'))
    assert hello['parameter_templates']['fast_lio']['required']['mapping.extrinsic_T']
    original = module.profiles.template()
    bridge.handle(request('save_draft', dict(
        recipe=original, parameter_sources={'color_line': 'custom'})))
    result = bridge.handle(request('load_draft'))
    assert result['parameter_sources'] == {'color_line': 'custom'}
    assert result['recipe'] == original
    assert bridge.client.calls == []


def test_template_numbers_are_normalized_before_recipe_validation_and_transmission(bridge):
    """Mixed JSON physical arrays and integral doubles reach one typed configure call."""
    recipe = module.profiles.template()
    recipe['components']['map_odom']['enabled'] = True
    recipe['components']['map_odom']['parameters']['body_to_base_translation'] = [0, 0, 0.1]
    recipe['components']['flight_control']['enabled'] = True
    recipe['components']['flight_control']['parameters']['control_rate'] = 25
    bridge.snapshot['capabilities'] = dict(parameter_templates=1)
    bridge.handle(request('operate', dict(operation='configure', values=dict(
        recipe=recipe, expected_hash='saved-version', templates=['map_odom', 'flight_control']))))
    assert len(bridge.client.calls) == 1
    sent = bridge.client.calls[0][1]['recipe']['components']
    assert all(type(value) is float for value in sent['map_odom']['parameters'][
        'body_to_base_translation'])
    assert type(sent['flight_control']['parameters']['control_rate']) is float

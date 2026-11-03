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
"""JSON-lines UI fixture: real recipe helpers, no platform service or ROS nodes."""

import copy
import importlib.util
import hashlib
import json
import os
from pathlib import Path
import sys
import time


def main():
    spec = importlib.util.spec_from_file_location(
        "fixture_profiles", os.environ["EASYFLY_TEST_ONBOARD_PROFILES"]
    )
    profiles = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(profiles)
    control_path = Path(os.environ["EASYFLY_TEST_ONBOARD_CONTROL"])
    log_path = Path(os.environ["EASYFLY_TEST_ONBOARD_LOG"])
    selected = "A"
    recipes = {}
    saved_missions = {"A": {}, "B": {}}
    for key in ("A", "B"):
        recipes[key] = profiles.template()
        recipes[key]["name"] = "platform-" + key

    def control():
        try:
            return json.loads(control_path.read_text())
        except (OSError, ValueError):
            return {}

    def wait_for_release(key):
        deadline = time.monotonic() + 8
        while control().get(key, False) and time.monotonic() < deadline:
            time.sleep(0.01)

    def current_hash():
        return control().get("hash_" + selected, selected.lower() * 64)

    def missions():
        return dict(control().get("mission_catalog", {}), **saved_missions[selected])

    def state():
        recipe = copy.deepcopy(recipes[selected])
        name = control().get("name_" + selected)
        if name:
            recipe["name"] = name
        target = dict(
            server_id=selected.lower() * 32,
            boot_id=control().get("boot_" + selected, ("1" if selected == "A" else "2") * 32),
            client_id="c" * 32,
            ros_domain_id=187,
            local_only=False,
            local=selected == "A",
            alias="fixture-" + selected,
            url="https://fixture-" + selected.lower() + ":443",
        )
        return dict(
            target=target,
            snapshot=dict(
                v=2,
                server_id=target["server_id"],
                boot_id=target["boot_id"],
                ros_domain_id=187,
                local_only=False,
                recipe=recipe,
                recipe_hash=current_hash(),
                state=dict(phase="stopped", session="", components={}, tasks={}),
                conditions=dict(can_stop=False, grounded=True, mission_stack_ready=False),
                missions=[dict(name=name) for name in missions()],
                capabilities=control().get("capabilities", {}),
                task_step_support=control().get("task_step_support", {}),
                maintenance=False,
                mission_execution_enabled=False,
            ),
            recent=recent(),
            task_request=control().get("task_request", ""),
            task_pending=control().get("task_pending", False),
        )

    def recent():
        return [dict(alias="fixture-B", url="https://fixture-b:443", path="/fixture/B.json")]

    for line in sys.stdin:
        try:
            request = json.loads(line)
            with log_path.open("a") as log:
                log.write(json.dumps(request, separators=(",", ":")) + "\n")
            operation, data = request["op"], request.get("data", {})
            if operation == "hello":
                value = dict(
                    catalog=profiles.CATALOG, template=profiles.template(), recent=recent(),
                    parameter_templates=control().get("parameter_templates", {})
                )
            elif operation in ("connect_local", "connect_remote"):
                selected = "A" if operation == "connect_local" else "B"
                value = state()
            elif operation == "status":
                wait_for_release("hold_status")
                if control().get("fail_status", False):
                    raise OSError("arranged status failure")
                value = state()
            elif operation == "prepare":
                wait_for_release("hold_prepare")
                value = profiles.enable_dependencies(data["recipe"], data.get("enable", []))
                value["startup_order"] = profiles.order(value["recipe"])
            elif operation == "operate":
                expected = data.get("expected_target")
                actual = state()["target"]
                if expected != {key: actual[key] for key in ("server_id", "boot_id", "client_id")}:
                    raise ValueError("arranged target mismatch")
                wait_for_release("hold_operate")
                values = data.get("values", {})
                action = data["operation"]
                if action == "configure":
                    accepted = values.get("expected_hash") == current_hash()
                    if accepted:
                        recipes[selected] = copy.deepcopy(values["recipe"])
                    receipt = dict(
                        accepted=accepted, recipe_hash=current_hash(), phase="configured"
                    )
                    if accepted and values.get("templates"):
                        for name in values["templates"]:
                            recipes[selected]["components"][name]["parameter_file"] = (
                                "/fixture/" + "a" * 16 + "-easyfly-template-" + name + ".yaml"
                            )
                        receipt["recipe"] = copy.deepcopy(recipes[selected])
                elif action in ("start", "check"):
                    accepted = values.get("recipe_hash") == current_hash()
                    receipt = dict(accepted=accepted, phase="starting" if accepted else "changed")
                elif action == "mission_read":
                    content = missions()[values["name"]]
                    receipt = dict(
                        accepted=True, phase="mission_read", name=values["name"],
                        content=content, sha256=hashlib.sha256(content.encode()).hexdigest(),
                        step_types=control().get("mission_step_types", ["land"]),
                    )
                elif action == "mission_save":
                    old = missions().get(values["name"])
                    accepted = old is None or old == values["content"]
                    if accepted:
                        saved_missions[selected][values["name"]] = values["content"]
                    receipt = dict(
                        accepted=accepted, phase="mission_saved", name=values["name"],
                        sha256=hashlib.sha256(values["content"].encode()).hexdigest())
                elif action == "configuration_upload":
                    receipt = dict(
                        accepted=not control().get("reject_upload", False),
                        phase="configuration_uploaded", path="/fixture/" + values["name"],
                        reason="arranged upload rejection",
                    )
                else:
                    receipt = dict(accepted=False, reason="fixture operation unsupported")
                value = dict(state(), result=receipt)
            else:
                raise ValueError("fixture operation unsupported: " + operation)
            reply = dict(id=request["id"], ok=True, value=value)
        except Exception as error:
            reply = dict(id=request.get("id", ""), ok=False, error=str(error))
        print(json.dumps(reply, ensure_ascii=False, separators=(",", ":")), flush=True)


if __name__ == "__main__":
    main()

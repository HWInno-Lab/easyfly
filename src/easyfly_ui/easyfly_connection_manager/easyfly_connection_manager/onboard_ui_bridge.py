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
"""
Bounded, ordered desktop adapter for the existing authenticated onboard API.

This process owns no aircraft processes. Closing its stdin only ends this adapter;
platform services and their precise ownership remain with the onboard runtime.
"""

import argparse
import copy
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix

from . import onboard_profiles as profiles
from .onboard_parameter_templates import catalog as parameter_templates
from .onboard_parameter_templates import normalize_parameters
from .onboard_client import PlatformClient
from .onboard_deployment import client_settings, export_client, install_user_service
from .onboard_service import bootstrap_local
from .remote_service import atomic, private_directory

MAX_LINE = 524288
MAX_DRAFT = 262144
OPERATIONS = {
    "configure",
    "check",
    "start",
    "stop",
    "configuration_upload",
    "mission_save",
    "mission_read",
    "mission_start",
    "mission_lookup",
    "mission_cancel",
}


def read_private(path, maximum=MAX_DRAFT):
    """Read bounded state without following links or accepting other owners."""
    with os.fdopen(os.open(path, os.O_RDONLY | os.O_NOFOLLOW), "r") as stream:
        info = os.fstat(stream.fileno())
        if (
            not stat.S_ISREG(info.st_mode)
            or info.st_uid != os.geteuid()
            or info.st_mode & 0o077
            or info.st_size > maximum
        ):
            raise ValueError("本地状态须为当前用户私有的有界文件")
        return json.load(stream)


def require_size(value, maximum):
    if len(json.dumps(value, ensure_ascii=False, allow_nan=False).encode()) > maximum:
        raise ValueError("内容超过允许大小，未提交或截断")


class Bridge:
    """Keep one authenticated target, local drafts and unresolved task identity."""

    def __init__(self, directory, client_factory=PlatformClient):
        self.directory = Path(directory).absolute()
        private_directory(self.directory)
        self.client_factory = client_factory
        self.client = None
        self.snapshot = {}
        self.fresh_at = 0.0
        self.local_root = None
        self.connection_path = ""
        self.alias = ""
        self.task_request = ""
        self.pending_task = False
        self.review = None

    def recent(self):
        path = self.directory / "platforms.json"
        return read_private(path) if path.exists() else []

    def remember(self):
        rows = [r for r in self.recent() if r.get("server_id") != self.snapshot["server_id"]]
        rows.insert(
            0,
            dict(
                alias=self.alias,
                path=self.connection_path,
                server_id=self.snapshot["server_id"],
                url=self.client.config["url"],
            ),
        )
        atomic(self.directory / "platforms.json", rows[:20])

    def task_path(self):
        if not self.client:
            raise ValueError("请先连接目标平台")
        return self.directory / (
            "task-"
            + self.client.config["server_id"]
            + "-"
            + self.client.config["client_id"]
            + ".json"
        )

    def save_task(self):
        atomic(
            self.task_path(),
            dict(
                server_id=self.client.config["server_id"],
                client_id=self.client.config["client_id"],
                request_id=self.task_request,
                pending=self.pending_task,
            ),
        )

    def target(self):
        if not self.client:
            return {}
        return dict(
            server_id=self.snapshot["server_id"],
            boot_id=self.snapshot["boot_id"],
            client_id=self.client.config["client_id"],
            url=self.client.config["url"],
            alias=self.alias,
            local=bool(self.local_root),
            ros_domain_id=self.snapshot["ros_domain_id"],
            local_only=self.snapshot.get("local_only", False),
        )

    def state(self, result=None):
        return dict(
            snapshot=self.snapshot,
            target=self.target(),
            recent=self.recent(),
            task_request=self.task_request,
            task_pending=self.pending_task,
            review=self.review,
            result=result or {},
        )

    def refresh(self):
        # A failed poll must also revoke freshness for a foreground operation
        # already queued behind it in the serial JSON-lines transport.
        self.fresh_at = 0.0
        if not self.client:
            raise ValueError("尚未连接平台")
        self.snapshot = self.client.status()
        self.fresh_at = time.monotonic()
        return self.state()

    def require_target(self, data):
        """Bind queued actions to the identity the desktop actually confirmed."""
        self.fresh()
        expected = data.get("expected_target")
        current = self.target()
        keys = {"server_id", "boot_id", "client_id"}
        if (
            not isinstance(expected, dict)
            or set(expected) != keys
            or any(not expected[key] or expected[key] != current[key] for key in keys)
        ):
            raise ValueError("平台身份或服务会话已变化，请重新查询并确认；未提交操作")

    def connect_path(self, path, alias="", local_root=None):
        settings = client_settings(path)
        client = self.client_factory(settings)
        snapshot = client.status()
        self.client, self.snapshot = client, snapshot
        self.connection_path = str(Path(path).absolute())
        self.alias = alias.strip()[:64] or client.url.hostname
        self.local_root = local_root
        self.fresh_at = time.monotonic()
        self.review = None
        self.task_request, self.pending_task = "", False
        if self.task_path().exists():
            saved = read_private(self.task_path())
            if (
                saved.get("server_id") == settings["server_id"]
                and saved.get("client_id") == settings["client_id"]
            ):
                self.task_request = saved.get("request_id", "")
                self.pending_task = bool(saved.get("pending"))
        self.remember()
        return self.state()

    def connect_local(self, data):
        root = Path(
            os.environ.get(
                "EASYFLY_ONBOARD_CONFIG_ROOT", str(Path.home() / ".config/easyfly/onboard")
            )
        ).absolute()
        server, client_path = bootstrap_local(
            root, data.get("domain", 0), data.get("allow_missions", False), data.get("host", "")
        )
        try:
            return self.connect_path(client_path, "本机平台", root)
        except (OSError, ValueError):
            program = (
                Path(get_package_prefix("easyfly_connection_manager"))
                / "lib/easyfly_connection_manager/easyfly_onboard_service"
            )
            with (root / "service.log").open("ab") as log:
                process = subprocess.Popen(
                    [str(program), "--config", str(server)],
                    stdin=subprocess.DEVNULL,
                    stdout=log,
                    stderr=log,
                    start_new_session=True,
                )
            for _ in range(60):
                if process.poll() is not None:
                    raise RuntimeError("本机管理服务未启动，请查看 " + str(root / "service.log"))
                try:
                    return self.connect_path(client_path, "本机平台", root)
                except (OSError, ValueError):
                    time.sleep(0.1)
            raise RuntimeError("本机管理服务尚未就绪，请检查服务日志后重新查询")

    def fresh(self):
        if not self.client or time.monotonic() - self.fresh_at > 2.5:
            raise ValueError("请先取得新鲜、经过认证的平台状态")

    def prepare(self, data):
        recipe = copy.deepcopy(data["recipe"])
        added = []
        if data.get("enable"):
            if hasattr(profiles, "enable_dependencies"):
                prepared = profiles.enable_dependencies(recipe, data["enable"])
                recipe, added = prepared["recipe"], prepared["added"]
            else:
                pending = list(data["enable"])
                visited = set()
                while pending:
                    name = pending.pop()
                    if name in visited:
                        continue
                    visited.add(name)
                    entry = recipe["components"][name]
                    if not entry["enabled"]:
                        added.append(name)
                    entry["enabled"] = True
                    pending.extend(entry["depends_on"])
        if data.get("validate", True):
            profiles.validate(recipe)
        return dict(recipe=recipe, added=added, startup_order=profiles.order(recipe))

    def operate(self, data, request_id):
        self.require_target(data)
        operation, values = data["operation"], data.get("values", {})
        if operation not in OPERATIONS:
            raise ValueError("未支持的机载操作")
        if operation == "configure":
            if "templates" in values:
                values = dict(values)
                values["recipe"] = normalize_parameters(values["recipe"], values["templates"])
            profiles.validate(values["recipe"])
            if values.get("templates"):
                if self.snapshot.get("capabilities", {}).get("parameter_templates") != 1:
                    raise ValueError("目标机载服务尚不支持内置参数模板，请更新并重启机载服务；自定义文件仍可使用")
            elif "templates" in values:
                values = dict(values)
                values.pop("templates")
        if operation in ("mission_lookup", "mission_cancel"):
            if not self.task_request:
                raise ValueError("没有原任务请求可查询")
            values = dict(request_id=self.task_request)
        if operation == "mission_start":
            if self.pending_task:
                raise ValueError("原任务尚未确认完成，请先查询原请求，未提交新任务")
            if (
                not self.review
                or values.get("confirmed") is not True
                or self.review != dict(name=values.get("name"), sha256=values.get("sha256"))
            ):
                raise ValueError("任务需先读取机载版本并审阅确认")
        body = dict(
            v=2,
            server_id=self.snapshot["server_id"],
            boot_id=self.snapshot["boot_id"],
            id=request_id,
            client_id=self.client.config["client_id"],
            op=operation,
            data=values,
        )
        require_size(body, 131072)
        if operation == "mission_start":
            self.task_request, self.pending_task = request_id, True
            self.save_task()  # Persist before the external effect, even on response loss.
        try:
            result = self.client.operation(self.snapshot, operation, values, request_id=request_id)
        except Exception:
            self.fresh_at = 0.0
            raise
        if operation == "mission_start" and not result.get("accepted"):
            self.pending_task = result.get("phase") == "request_unresolved"
            if not self.pending_task:
                self.task_request = ""
            self.save_task()
        if result.get("accepted") and operation == "mission_read":
            self.review = dict(name=result["name"], sha256=result["sha256"])
        if operation == "mission_save":
            self.review = None
        if "task" in result:
            task = result["task"]
            self.task_request = task["request_id"]
            self.pending_task = bool(task.get("execution_pending", True))
            self.save_task()
        return self.state(result)

    def handle(self, request):
        if (
            not isinstance(request, dict)
            or set(request) != {"id", "op", "data"}
            or not re.fullmatch("[a-f0-9]{32}", request["id"])
            or not isinstance(request["data"], dict)
        ):
            raise ValueError("无效的本地请求")
        op, data = request["op"], request["data"]
        if op == "hello":
            return dict(
                catalog=profiles.CATALOG, template=profiles.template(), recent=self.recent(),
                parameter_templates=parameter_templates(),
            )
        if op == "connect_local":
            return self.connect_local(data)
        if op == "connect_remote":
            return self.connect_path(data["path"], data.get("alias", ""))
        if op == "status":
            return self.refresh()
        if op == "prepare":
            return self.prepare(data)
        if op == "operate":
            return self.operate(data, request["id"])
        if op == "clear_review":
            self.review = None
            return {}
        if op == "logs":
            self.require_target(data)
            return self.state(self.client.logs(data["component"]))
        if op == "export_connection":
            self.require_target(data)
            if not self.local_root:
                raise ValueError("只有本机部署可以导出连接包")
            return dict(path=export_client(self.local_root, data["directory"]))
        if op == "install_service":
            self.require_target(data)
            if not self.local_root:
                raise ValueError("只有本机部署可以启用本机服务")
            result = install_user_service(self.local_root, self.client, self.snapshot)
            self.fresh_at = 0.0
            return self.state(result)
        if op == "save_draft":
            # A draft may be incomplete. Never validate it as an applied recipe.
            sources = data.get("parameter_sources", {})
            if (not isinstance(sources, dict) or
                    any(key not in profiles.CATALOG or value not in ('builtin', 'custom')
                        for key, value in sources.items())):
                raise ValueError("参数来源草稿无效")
            draft = dict(
                v=1,
                target=data.get("draft_target") or self.target(),
                expected_hash=data.get("expected_hash", ""),
                recipe=data["recipe"],
                mission=data.get("mission", {}),
                editor_overrides=data.get("editor_overrides", {}),
                parameter_sources=sources,
            )
            require_size(draft, MAX_DRAFT)
            atomic(self.directory / "draft.json", draft)
            return dict(path=str(self.directory / "draft.json"))
        if op == "load_draft":
            return read_private(self.directory / "draft.json")
        raise ValueError("未支持的本地操作")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state-directory", required=True)
    args = parser.parse_args()
    bridge = Bridge(args.state_directory)
    while True:
        line = sys.stdin.buffer.readline(MAX_LINE + 1)
        if not line:
            return
        request = {}
        try:
            if len(line) > MAX_LINE or not line.endswith(b"\n"):
                raise ValueError("本地请求过大或缺少完整分帧，未执行")
            request = json.loads(line)
            if not isinstance(request, dict):
                request = {}
                raise ValueError("本地请求须为 JSON 对象")
            value = bridge.handle(request)
            reply = dict(id=request["id"], ok=True, value=value)
        except Exception as error:
            reply = dict(
                id=request.get("id", ""),
                ok=False,
                error=str(error)[:2048],
                task_request=bridge.task_request,
                task_pending=bridge.pending_task,
            )
        try:
            require_size(reply, MAX_LINE)
        except ValueError:
            reply = dict(id=request.get("id", ""), ok=False, error="响应超过本地显示大小限制，请读取单个模块日志")
        sys.stdout.write(json.dumps(reply, ensure_ascii=False, allow_nan=False) + "\n")
        sys.stdout.flush()
        if len(line) > MAX_LINE:
            # Close framing after an oversized line; never parse its tail as a new request.
            return


if __name__ == "__main__":
    main()

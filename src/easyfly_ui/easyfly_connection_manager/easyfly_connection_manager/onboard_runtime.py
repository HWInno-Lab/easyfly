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
"""Own a complete onboard recipe and reviewed mission intents across GUI disconnects."""

import copy
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import sqlite3
import subprocess
import threading
import time
import uuid

from ament_index_python.packages import get_package_prefix

from .onboard_observer import PlatformObserver
from .onboard_profiles import (
    CATALOG, complete_recipe, component_dependencies, digest, order, preflight, template, validate,
)
from .onboard_parameter_templates import materialize, normalize_parameters
from .remote_profiles import private_json
from .remote_service import Owner as AlgorithmOwner
from .remote_service import atomic, group_members, private_directory


class PlatformOwner:
    """One domain owner; recipes are configured once and started as one managed platform."""

    def __init__(self, config, observer_factory=PlatformObserver, popen=subprocess.Popen):
        self.config, self.popen = config, popen
        self.boot = uuid.uuid4().hex
        self.root = Path(config["state_directory"])
        private_directory(self.root)
        self.lock_fd = os.open(
            self.root / "owner.lock", os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600
        )
        fcntl.flock(self.lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        locks = Path("/tmp") / ("easyfly-remote-owner-" + str(os.geteuid()))
        private_directory(locks)
        self.domain_fd = os.open(
            locks / ("domain-%d.lock" % config["ros_domain_id"]),
            os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW,
            0o600,
        )
        fcntl.flock(self.domain_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.recipe_path = self.root / "recipe.json"
        self.recipe = (
            complete_recipe(private_json(self.recipe_path))
            if self.recipe_path.exists() else template()
        )
        if self.recipe_path.exists():
            atomic(self.recipe_path, self.recipe)
        self.missions = self.root / "missions"
        private_directory(self.missions)
        self.state_path = self.root / "platform-state.json"
        self.state = (
            private_json(self.state_path)
            if self.state_path.exists()
            else {"phase": "stopped", "session": "", "components": {}, "tasks": {}}
        )
        if self.state["phase"] not in ("stopped", "failed"):
            self.state["phase"] = "unresolved_after_service_restart"
        for task in self.state["tasks"].values():
            if task.get("execution_pending", True):
                task["unresolved_after_service_restart"] = True
        atomic(self.state_path, self.state)
        self.database = sqlite3.connect(self.root / "platform-requests.sqlite3")
        (self.root / "platform-requests.sqlite3").chmod(0o600)
        self.database.execute("PRAGMA synchronous=FULL")
        self.database.execute(
            "CREATE TABLE IF NOT EXISTS receipts "
            "(id TEXT PRIMARY KEY, digest TEXT NOT NULL, result TEXT NOT NULL)"
        )
        self.database.commit()
        self.mutex = threading.RLock()
        self.children, self.log_threads, self.log_errors = {}, {}, {}
        self.worker = None
        self.flight_fd = None
        self.abort = threading.Event()
        self.ever_connected = False
        self.observer_factory = observer_factory
        self.observer = observer_factory(self.recipe)
        self.last_authenticated = 0.0
        self.exit_requested = False

    def save(self):
        """Store ownership before process or task side effects."""
        atomic(self.state_path, self.state)

    def refresh(self):
        """Require every owned descendant to end before recording completion."""
        with self.mutex:
            observed = self.observer.snapshot("platform")
            self.ever_connected |= bool(getattr(self.observer, "ever_connected", False))
            changed = False
            for name, process in list(self.children.items()):
                code, members = process.poll(), group_members(process.pid)
                if code is not None and not members:
                    self.log_threads.pop(name).join()
                    self.children.pop(name)
                    self.state["components"][name].update(
                        phase="stopped" if code == 0 else "failed", returncode=code, members={}
                    )
                    changed = True
                    if self.state["phase"] not in ("stopping", "stopped"):
                        self.state["phase"] = "failed_owned"
                        self.state["error"] = name + " exited before requested stop"
                elif code is not None:
                    self.state["components"][name]["phase"] = "owned_descendants_remain"
                    changed = True
            if self.state["phase"] in ("running", "degraded"):
                ready = True
                for name in self.children:
                    entry = self.state["components"][name]
                    valid = self.observer.ready(name, entry.get("started_steady", 0.0))
                    phase = "ready" if valid else "waiting_for_data"
                    changed |= entry.get("phase") != phase
                    entry["phase"] = phase
                    ready &= valid
                phase = "running" if ready else "degraded"
                changed |= self.state["phase"] != phase
                self.state["phase"] = phase
            if self.state["phase"] == "stopping" and not self.children and not self.worker_alive():
                self.state["phase"] = "stopped"
                changed = True
            if changed:
                self.save()
            if (
                not self.children
                and not self.worker_alive()
                and self.state["phase"] in ("stopped", "failed")
            ):
                if self.flight_fd is not None:
                    os.close(self.flight_fd)
                    self.flight_fd = None
            return observed

    def worker_alive(self):
        """Keep one in-flight start/stop worker at most."""
        return self.worker is not None and self.worker.is_alive()

    def task_pending(self):
        """Unresolved admissions and draining child work retain platform ownership."""
        return any(task.get("execution_pending", True) for task in self.state["tasks"].values())

    def task_stack_ready(self):
        """Never submit a flight task to an unrelated externally discovered executor."""
        required = {
            "mavlink",
            "coord",
            "mission_services",
            "mission_fsm",
            "flight_control",
            "navigation",
        }
        return required <= self.children.keys() and all(
            self.state["components"].get(name, {}).get("phase") == "ready" for name in required
        )

    def task_step_support(self):
        """Separate adaptation, configured dependencies and observed managed readiness."""
        adapters = {
            "track": {"target_tracking"},
            "precision_land": {"precision_landing"},
            "explore": {"fuel", "gcopter", "lidar_rays"},
        }
        base = {"mission_fsm", "mission_services"}
        result = {}
        for kind in ("takeoff", "hover", "waypoint", "navigate", "track", "search", "land",
                     "precision_land", "explore"):
            if kind == "explore" and self.recipe["components"].get(
                    "navigation", {}).get("parameters", {}).get("strategy") != "gcopter":
                result[kind] = dict(state="not_ready", reason="自主探索需要选择 GCOPTER / FUEL 导航策略。")
                continue
            required = set(base) | adapters.get(kind, set())
            if kind != "hover":
                required.add("flight_control")
            if kind in ("navigate", "search"):
                required.add("navigation")
            pending = list(required)
            while pending:
                name = pending.pop()
                for dependency in component_dependencies(self.recipe, name):
                    if dependency not in required:
                        required.add(dependency)
                        pending.append(dependency)
            disabled = [name for name in sorted(required)
                        if not self.recipe["components"].get(name, {}).get("enabled")]
            missing = [name for name in sorted(required)
                       if name not in self.children or
                       self.state["components"].get(name, {}).get("phase") != "ready"]
            if disabled:
                text = "未启用所需模块：" + "、".join(CATALOG[name]["title"] for name in disabled)
            elif missing:
                text = "所需受管模块尚未就绪：" + "、".join(
                    CATALOG[name]["title"] for name in missing)
            else:
                observe = getattr(self.observer, "task_backend_ready", None)
                if observe is None:
                    result[kind] = dict(state="unknown", reason="尚未取得任务动作接口的实际观测。")
                    continue
                try:
                    ready = observe(kind)
                except Exception:
                    ready = False
                if ready:
                    result[kind] = dict(
                        state="supported", reason="已适配且所需受管模块与接口已就绪；执行仍需实时准入。")
                    continue
                text = "未发现唯一、新鲜的受管任务接口：" + kind
            result[kind] = dict(state="not_ready", reason=text)
        return result

    def maintenance(self):
        """Keep the existing QGC maintenance/managed-task separation."""
        link = self.recipe["components"].get("mavlink", {})
        return bool(link.get("enabled") and link.get("parameters", {}).get("gcs_url"))

    def can_stop(self, observed):
        """Unknown flight state after a connection is never a disarmed observation."""
        return bool(
            not self.task_pending()
            and (
                observed["grounded"]
                or (not self.ever_connected and observed["startup_abort_safe"])
            )
        )

    def status(self):
        """Return actual states, saved choices and capabilities to local or remote EasyFly."""
        observed = self.refresh()
        with self.mutex:
            state = copy.deepcopy(self.state)
            state["tasks"] = dict(list(state["tasks"].items())[-8:])
            return dict(
                v=2,
                server_id=self.config["server_id"],
                boot_id=self.boot,
                ros_domain_id=self.config["ros_domain_id"],
                local_only=self.config["local_only"],
                hardware=True,
                capabilities=dict(parameter_templates=1, task_adapters=1),
                task_step_support=self.task_step_support(),
                recipe=copy.deepcopy(self.recipe),
                recipe_hash=digest(self.recipe),
                catalog=CATALOG,
                configuration_roots=self.config["configuration_roots"],
                state=state,
                log_errors=dict(self.log_errors),
                conditions=dict(
                    observed,
                    can_stop=self.can_stop(observed),
                    mission_stack_ready=self.task_stack_ready(),
                ),
                missions=self.mission_list(),
                mission_execution_enabled=self.config["allow_mission_execution"]
                and not self.maintenance(),
                maintenance=self.maintenance(),
            )

    def mutate(self, request):
        """Resolve each authenticated intent once; replay cannot reapply a side effect."""
        if (
            not isinstance(request, dict)
            or set(request) != {"v", "server_id", "boot_id", "id", "client_id", "op", "data"}
            or request["v"] != 2
            or type(request["v"]) is not int
            or request["server_id"] != self.config["server_id"]
            or request["boot_id"] != self.boot
            or any(
                not isinstance(request[key], str) or not re.fullmatch("[a-f0-9]{32}", request[key])
                for key in ("id", "client_id")
            )
            or not isinstance(request["data"], dict)
        ):
            raise ValueError("invalid or stale platform request")
        fingerprint = digest(request)
        row = self.database.execute(
            "SELECT digest,result FROM receipts WHERE id=?", (request["id"],)
        ).fetchone()
        if row:
            if row[0] != fingerprint:
                raise ValueError("request identity already used for different content")
            return json.loads(row[1])
        if self.database.execute("SELECT COUNT(*) FROM receipts").fetchone()[0] >= 10000:
            raise ValueError("platform receipt capacity exhausted")
        result = dict(accepted=False, phase="request_unresolved", id=request["id"])
        self.database.execute(
            "INSERT INTO receipts VALUES (?,?,?)", (request["id"], fingerprint, json.dumps(result))
        )
        self.database.commit()
        try:
            result = dict(self.apply(request), id=request["id"])
        except (
            ValueError,
            OSError,
            RuntimeError,
            TimeoutError,
            subprocess.SubprocessError,
        ) as error:
            result = dict(accepted=False, id=request["id"], reason=str(error))
        self.database.execute(
            "UPDATE receipts SET result=? WHERE id=?",
            (json.dumps(result, ensure_ascii=False), request["id"]),
        )
        self.database.commit()
        return result

    def apply(self, request):
        """Apply an explicit recipe, platform lifecycle or reviewed task operation."""
        observed = self.refresh()
        operation, data = request["op"], request["data"]
        if operation == "service_exit":
            if data or not self.idle():
                raise ValueError("先正常停止平台并处理未决任务，再退出平台服务")
            self.exit_requested = True
            return dict(accepted=True, phase="service_exiting")
        if operation == "configuration_upload":
            if set(data) != {"name", "content"} or not isinstance(data["content"], str):
                raise ValueError("invalid configuration upload")
            if not re.fullmatch(r"[\w-]{1,80}\.(yaml|yml|json)", data["name"]):
                raise ValueError("配置文件名需要 .yaml/.yml/.json 后缀")
            content = data["content"].encode()
            if not 0 < len(content) <= 65536:
                raise ValueError("配置上传大小必须在 1–65536 字节内")
            identity = hashlib.sha256(content).hexdigest()
            path = Path(self.config["configuration_roots"][0]) / (
                identity[:16] + "-" + data["name"]
            )
            if path.exists():
                if path.is_symlink() or path.read_bytes() != content:
                    raise ValueError("已有不同内容的配置，未覆盖")
            else:
                with path.open("xb") as stream:
                    stream.write(content)
                path.chmod(0o600)
            return dict(
                accepted=True, phase="configuration_uploaded", path=str(path), sha256=identity
            )
        if operation in ("configure", "check", "start"):
            if (
                self.children
                or self.worker_alive()
                or self.state["phase"] not in ("stopped", "failed")
            ):
                raise ValueError("先正常停止当前平台；未决状态不能替换配置或重复启动")
            if self.task_pending():
                raise ValueError("仍有未决任务，先查询并排空原任务")
            if operation == "configure":
                allowed = ({"recipe", "expected_hash"}, {"recipe", "expected_hash", "templates"})
                if set(data) not in allowed or data["expected_hash"] != digest(self.recipe):
                    raise ValueError("运行方案已被其他客户端修改，请重新读取")
                configured = copy.deepcopy(data["recipe"])
                if "templates" in data:
                    configured = normalize_parameters(configured, data["templates"])
                validate(configured)
                configured = complete_recipe(configured)
                if "templates" in data:
                    configured = materialize(
                        configured, data["templates"], self.config["configuration_roots"][0])
                # Saving a draft can precede attaching devices/configuration.
                # Starting always performs the stronger file/package preflight.
                with self.mutex:
                    atomic(self.recipe_path, configured)
                    self.recipe = configured
                    self.observer.close()
                    self.observer = self.observer_factory(self.recipe)
                return dict(accepted=True, phase="configured", recipe_hash=digest(self.recipe),
                            recipe=copy.deepcopy(self.recipe))
            if data != {"recipe_hash": digest(self.recipe)}:
                raise ValueError("请先读取、保存并核对当前运行方案")
            if not order(self.recipe):
                raise ValueError("请至少选择一个传感器、算法或平台模块")
            _, files = preflight(self.recipe, self.config["configuration_roots"])
            if operation == "check":
                return dict(
                    accepted=True,
                    phase="checked",
                    configuration_files=files,
                    startup_order=order(self.recipe),
                )
            if not observed["observer_alive"] or not (
                observed["isolated"] or observed["grounded"]
            ):
                raise ValueError("平台状态未知、已有活动任务或不在地面，不能启动另一套模块")
            for name in order(self.recipe):
                if self.observer.source_conflict(name):
                    raise ValueError("已有外部模块/输出，不能重复启动或接管：" + name)
            self.begin_start(request["client_id"])
            return dict(accepted=True, phase="starting", session=self.state["session"])
        if operation == "stop":
            if data != {"session": self.state["session"]} or request[
                "client_id"
            ] != self.state.get("client_id"):
                raise ValueError("停止需要本客户端的精确平台会话")
            if not self.can_stop(observed):
                raise ValueError("需要新鲜的未解锁/在地面状态且无未决任务；未知状态不停止")
            self.abort.set()
            if self.worker_alive():
                # A startup worker sees abort and returns; stop is scheduled by
                # the main loop afterward, never a concurrent mutation worker.
                self.state["stop_requested"] = True
            else:
                self.begin_stop()
            return dict(accepted=True, phase="stopping", session=self.state["session"])
        if operation == "mission_save":
            return self.save_mission(data)
        if operation == "mission_read":
            return self.read_mission(data)
        if operation in ("mission_start", "mission_lookup", "mission_cancel"):
            return self.mission_operation(request, observed)
        raise ValueError("unknown platform operation")

    def begin_start(self, client_id):
        """Freeze a complete dependency plan before starting its first owned child."""
        session = uuid.uuid4().hex
        directory = self.root / session
        directory.mkdir(mode=0o700)
        recipe, files = preflight(self.recipe, self.config["configuration_roots"], directory)
        if "mavlink" in order(recipe):
            lock_root = Path("/tmp") / ("easyfly-communication-" + str(os.geteuid()))
            private_directory(lock_root)
            descriptor = os.open(
                lock_root / "backend.lock", os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600
            )
            try:
                fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                os.close(descriptor)
                raise ValueError("本机已有仿真或飞控通信后台，先正常停止；ROS domain 不隔离飞控端口")
            self.flight_fd = descriptor
        atomic(directory / "recipe.json", dict(recipe=recipe, files=files))
        self.state.update(
            phase="starting",
            session=session,
            client_id=client_id,
            recipe_hash=digest(self.recipe),
            components={},
            error="",
            stop_requested=False,
        )
        self.abort.clear()
        self.save()
        self.worker = threading.Thread(
            target=self.start_worker, args=(recipe, directory), name="onboard-start"
        )
        self.worker.start()

    def start_worker(self, recipe, directory):
        """Wait for each selected sensor/algorithm output, without fixed startup sleeps."""
        try:
            for name in order(recipe):
                if self.abort.is_set():
                    return
                if self.observer.source_conflict(name):
                    raise ValueError("模块输出已被占用：" + name)
                entry = recipe["components"][name]
                component_dir = directory / name
                component_dir.mkdir(mode=0o700)
                manifest = component_dir / "component.json"
                atomic(
                    manifest,
                    dict(kind=name, component=entry, mission_directory=str(self.missions)),
                )
                argv = [
                    "ros2",
                    "launch",
                    "easyfly_connection_manager",
                    "onboard_component.launch.py",
                    "component_file:=" + str(manifest),
                ]
                with self.mutex:
                    self.state["components"][name] = dict(
                        phase="starting", argv=argv, started_steady=time.monotonic()
                    )
                    self.save()
                    process = self.popen(
                        argv,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        stdin=subprocess.DEVNULL,
                        start_new_session=True,
                        cwd=component_dir,
                    )
                    self.children[name] = process
                    self.state["components"][name].update(
                        pid=process.pid, members=group_members(process.pid)
                    )
                    thread = threading.Thread(
                        target=AlgorithmOwner.drain_log,
                        args=(self, name, process, component_dir),
                        name="onboard-log-" + name,
                    )
                    self.log_threads[name] = thread
                    thread.start()
                    self.save()
                began = time.monotonic()
                deadline = began + entry["startup_timeout_s"]
                while not self.abort.wait(0.1):
                    if process.poll() is not None:
                        raise RuntimeError(name + " 启动后异常退出，查看模块日志")
                    if self.observer.ready(name, began):
                        break
                    if time.monotonic() >= deadline:
                        raise TimeoutError(name + " 等待传感器/算法输出超时，保留所属进程和日志")
                if self.abort.is_set():
                    return
                with self.mutex:
                    self.state["components"][name]["phase"] = "ready"
                    self.save()
            with self.mutex:
                self.state["phase"] = "running"
                self.save()
        except Exception as error:
            with self.mutex:
                self.state.update(
                    phase="failed_owned" if self.children else "failed", error=str(error)
                )
                self.save()

    def begin_stop(self):
        """Stop only owned processes, in reverse dependency order, without escalation."""
        self.state.update(phase="stopping", stop_requested=False)
        self.save()
        self.worker = threading.Thread(target=self.stop_worker, name="onboard-stop")
        self.worker.start()

    def stop_worker(self):
        """Retain ownership on a slow or unknown process exit."""
        with self.mutex:
            children = list(self.children.items())
        for name, process in reversed(children):
            if not self.can_stop(self.observer.snapshot("platform")):
                with self.mutex:
                    self.state.update(phase="stop_unknown", error="停止期间状态不再满足，保留其余模块")
                    self.save()
                return
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                if process.poll() is not None and not group_members(process.pid):
                    break
                time.sleep(0.1)
            else:
                with self.mutex:
                    self.state.update(phase="stop_unknown", error=name + " 尚未正常退出")
                    self.save()
                return

    def tick(self):
        """Finish a startup abort only after its original worker has returned."""
        self.refresh()
        if self.state.get("stop_requested") and not self.worker_alive():
            if self.can_stop(self.observer.snapshot("platform")):
                self.begin_stop()

    def mission_list(self):
        """Return only owned catalog basenames and exact content identities."""
        return [
            dict(name=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest())
            for path in sorted(self.missions.glob("*.yaml"))
            if path.is_file() and not path.is_symlink() and path.stat().st_size <= 65536
        ][:256]

    def mission_path(self, name):
        """Restrict every task operation to the platform's private mission catalog."""
        if (
            not isinstance(name, str)
            or len(name.encode()) > 128
            or not re.fullmatch(r"[\w-]{1,100}\.yaml", name)
        ):
            raise ValueError("任务名只能包含文字、数字、下划线、连字符并以 .yaml 结尾")
        path = self.missions / name
        if path.is_symlink():
            raise ValueError("任务不允许符号链接")
        return path

    def save_mission(self, data):
        """Validate native YAML semantics and publish a new immutable mission file."""
        if set(data) != {"name", "content"} or not isinstance(data["content"], str):
            raise ValueError("invalid mission document")
        content = data["content"].encode()
        if not 0 < len(content) <= 65536:
            raise ValueError("任务内容必须在 1–65536 字节内")
        path = self.mission_path(data["name"])
        if path.exists():
            if path.read_bytes() == content:
                return dict(
                    accepted=True,
                    phase="mission_saved",
                    name=path.name,
                    sha256=hashlib.sha256(content).hexdigest(),
                )
            raise ValueError("已有同名任务，另存为新名称；不会覆盖原任务")
        if len(self.mission_list()) >= 256:
            raise ValueError("任务目录已达 256 项上限")
        temporary = self.root / ("mission-" + uuid.uuid4().hex + ".yaml")
        try:
            with temporary.open("xb") as stream:
                stream.write(content)
            temporary.chmod(0o600)
            executable = (
                Path(get_package_prefix("easyfly_mission_format"))
                / "lib/easyfly_mission_format/mission_validate"
            )
            result = subprocess.run(
                [str(executable), str(temporary)],
                capture_output=True,
                text=True,
                timeout=3,
                check=False,
            )
            if result.returncode:
                raise ValueError("任务校验失败：" + result.stderr[-2048:])
            os.link(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)
        return dict(
            accepted=True,
            phase="mission_saved",
            name=path.name,
            sha256=hashlib.sha256(content).hexdigest(),
        )

    def read_mission(self, data):
        """Read a configured task without starting an action."""
        if set(data) != {"name"}:
            raise ValueError("invalid mission read")
        path = self.mission_path(data["name"])
        if path.stat().st_size > 65536:
            raise ValueError("oversized mission")
        with path.open("rb") as stream:
            raw = stream.read(65537)
        if not 0 < len(raw) <= 65536:
            raise ValueError("oversized or empty mission")
        content = raw.decode("utf-8")
        return dict(
            accepted=True,
            phase="mission_read",
            name=path.name,
            content=content,
            sha256=hashlib.sha256(content.encode()).hexdigest(),
            step_types=self.mission_step_types(raw),
        )

    def mission_step_types(self, content):
        """Extract types only through the same native parser used by task execution."""
        temporary = self.root / ("mission-review-" + uuid.uuid4().hex + ".yaml")
        try:
            with temporary.open("xb") as stream:
                stream.write(content)
            temporary.chmod(0o600)
            executable = (Path(get_package_prefix("easyfly_mission_format")) /
                          "lib/easyfly_mission_format/mission_validate")
            result = subprocess.run(
                [str(executable), "--step-types", str(temporary)], capture_output=True,
                text=True, timeout=3, check=False)
            if result.returncode:
                raise ValueError("任务步骤校验失败：" + result.stderr[-2048:])
            types = json.loads(result.stdout)
            if not isinstance(types, list) or not types or any(
                    not isinstance(kind, str) for kind in types):
                raise ValueError("原生任务校验器未返回有效步骤类型")
            return types
        finally:
            temporary.unlink(missing_ok=True)

    def mission_operation(self, request, observed):
        """Bind task review, execution, lookup and cancellation to the original owner."""
        operation, data = request["op"], request["data"]
        if operation == "mission_start":
            if (
                set(data) != {"name", "sha256", "confirmed"}
                or data["confirmed"] is not True
                or not self.config["allow_mission_execution"]
                or self.maintenance()
                or not self.task_stack_ready()
                or self.state["phase"] != "running"
                or self.task_pending()
                or not observed["grounded"]
            ):
                raise ValueError("任务执行需启用机载任务、审阅确认、完整平台就绪及新鲜地面状态")
            content = self.read_mission(dict(name=data["name"]))
            if content["sha256"] != data["sha256"]:
                raise ValueError("任务已变化，请重新读取审阅")
            details = self.observer.mission_details(data["name"])
            if details["content_sha256"] != data["sha256"]:
                raise ValueError("机载任务服务与所审阅目录不一致")
            supported = self.task_step_support()
            problems = [supported.get(kind, dict(state="unknown", reason="未知任务类型：" + kind))
                        for kind in dict.fromkeys(content["step_types"])]
            reasons = [item["reason"] for item in problems if item["state"] != "supported"]
            if reasons:
                raise ValueError("任务步骤尚不能执行：" + "；".join(reasons))
            session = self.observer.call("reviewed", dict(operation="session"))
            if not session["success"]:
                raise ValueError("机载任务会话不可用")
            intent = dict(
                client_id=request["client_id"],
                name=data["name"],
                sha256=data["sha256"],
                server_instance_id=session["server_instance_id"],
                request_id=request["id"],
                execution_pending=True,
                phase="request_unresolved",
            )
            for old in list(self.state["tasks"])[:-63]:
                if not self.state["tasks"][old].get("execution_pending", True):
                    self.state["tasks"].pop(old)
            self.state["tasks"][request["id"]] = intent
            self.save()
            reply = self.observer.call(
                "reviewed",
                dict(
                    operation="start",
                    server_instance_id=intent["server_instance_id"],
                    client_request_id=request["id"],
                    mission_name=data["name"],
                    expected_content_sha256=data["sha256"],
                ),
            )
        else:
            if set(data) != {"request_id"}:
                raise ValueError("查询/取消需要原始任务请求 ID")
            intent = self.state["tasks"].get(data["request_id"])
            if not intent or intent["client_id"] != request["client_id"]:
                raise ValueError("任务不属于本客户端")
            reply = self.observer.call(
                "reviewed",
                dict(
                    operation="lookup" if operation == "mission_lookup" else "cancel",
                    server_instance_id=intent["server_instance_id"],
                    client_request_id=intent["request_id"],
                    execution_id=intent.get("execution_id", ""),
                ),
            )
        # Unknown/restarted service replies must never clear unresolved work.
        if (
            reply.get("server_instance_id") == intent["server_instance_id"]
            and reply.get("client_request_id") == intent["request_id"]
            and (
                reply.get("success")
                or (reply.get("admission_resolved") and not reply.get("admission_success"))
            )
        ):
            intent.update(reply)
            intent["phase"] = reply["status"]
            self.save()
        return dict(accepted=True, phase=intent["phase"], task=copy.deepcopy(intent))

    def logs(self, name):
        """Read a bounded tail of one owned module log, never an arbitrary remote path."""
        if name not in self.state["components"] or name not in CATALOG:
            raise ValueError("当前会话尚未启动该模块")
        path = self.root / self.state["session"] / name / "process.log"
        with os.fdopen(os.open(path, os.O_RDONLY | os.O_NOFOLLOW), "rb") as stream:
            stream.seek(0, 2)
            stream.seek(max(0, stream.tell() - 16384))
            content = stream.read(16384).decode(errors="replace")
        return dict(
            accepted=True,
            phase="logs",
            component=name,
            session=self.state["session"],
            content=content,
        )

    def idle(self):
        """Report idle only after owned processes and task intents end."""
        return not self.children and not self.worker_alive() and not self.task_pending()

    def close(self):
        """Release resources only after every owned process and task has ended."""
        if not self.idle():
            raise RuntimeError("platform still owns processes or unresolved tasks")
        self.observer.close()
        self.database.close()
        if self.flight_fd is not None:
            os.close(self.flight_fd)
        os.close(self.domain_fd)
        os.close(self.lock_fd)

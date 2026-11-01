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
"""EasyFly UI for a complete local or remote onboard recipe and mission catalog."""

from concurrent.futures import ThreadPoolExecutor
import argparse
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

from ament_index_python.packages import get_package_prefix
from PyQt5.QtCore import Qt, QSignalBlocker, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QSplitter,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from .onboard_client import PlatformClient
from .onboard_deployment import client_settings, export_client, install_user_service
from .onboard_profiles import CATALOG, template, validate
from .onboard_service import bootstrap_local


MISSION_TEMPLATE = """schema_version: 1
frame_id: odom
units: {position: m, angle: rad, time: s}
failure_policy: abort
steps:
  - {type: takeoff, altitude: 1.5, timeout: 60}
  - {type: navigate, x: 1.5, y: 0.0, z: 1.5, timeout: 60}
  - {type: land, timeout: 90}
"""


class PlatformWindow(QWidget):
    """One platform workflow, not one terminal or lifecycle window per algorithm."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("EasyFly · 机载平台：算法与任务")
        self.resize(1100, 800)
        self.setStyleSheet(
            "QWidget {background:#f3f6fb;color:#263244;font-size:14px;}"
            "QLineEdit,QComboBox,QPlainTextEdit,QListWidget {background:white;"
            "border:1px solid #ccd7e5;padding:5px;}"
            "QPushButton {padding:7px 12px;border:1px solid #8ea7c8;border-radius:4px;}"
            "QPushButton:focus,QLineEdit:focus,QPlainTextEdit:focus {border:2px solid #2563eb;}"
            "QPushButton:disabled {color:#64748b;border-color:#d7e0ec;}"
        )
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future, self.client, self.snapshot = None, None, None
        self.fresh_at, self.next_poll = 0.0, 0.0
        self.loading, self.dirty, self.closing = False, False, False
        self.recipe, self.loaded_hash = template(), ""
        self.selected_component, self.parameter_widgets = "", {}
        self.review, self.task_request = None, ""
        self.mission_dirty = False
        self.local_root = None
        layout = QVBoxLayout(self)
        title = QLabel("机载平台 · 算法与任务")
        title.setStyleSheet("font-size:24px;font-weight:600")
        layout.addWidget(title)
        hint = QLabel("配置一次完整运行方案，由机载服务统一启动。关闭地面窗口后平台继续运行。")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        self.mission_policy = QLabel("连接后可编辑、校验和保存任务；执行还需完整平台与明确授权。")
        self.mission_policy.setWordWrap(True)
        layout.addWidget(self.mission_policy)
        connection = QHBoxLayout()
        self.domain = QSpinBox()
        self.domain.setRange(0, 232)
        self.domain.setValue(int(os.environ.get("ROS_DOMAIN_ID", "0")))
        self.domain.setPrefix("本机 ROS domain ")
        self.allow_missions = QCheckBox("首次部署允许受管任务执行")
        self.local = QPushButton("初始化 / 连接本机平台")
        remote = QPushButton("连接远端平台…")
        for widget in (self.domain, self.allow_missions, self.local, remote):
            connection.addWidget(widget)
        layout.addLayout(connection)
        deployment = QHBoxLayout()
        self.advertised_host = QLineEdit()
        self.advertised_host.setPlaceholderText("首次部署机载地址（IPv4/DNS）；留空仅本机管理")
        self.export = QPushButton("导出地面连接配置…")
        self.install_boot = QPushButton("启用本机启动服务")
        deployment.addWidget(self.advertised_host, 1)
        deployment.addWidget(self.export)
        deployment.addWidget(self.install_boot)
        layout.addLayout(deployment)
        self.export.clicked.connect(self.export_connection)
        self.install_boot.clicked.connect(self.install_service)
        self.identity = QLabel("尚未连接机载平台；可以先编辑运行方案")
        self.identity.setTextFormat(Qt.PlainText)
        self.identity.setWordWrap(True)
        layout.addWidget(self.identity)
        self.tabs = QTabWidget()
        layout.addWidget(self.tabs, 1)
        self.build_recipe_tab()
        self.build_mission_tab()
        self.status = QPlainTextEdit()
        self.status.setReadOnly(True)
        diagnostics = QWidget()
        diagnostics_layout = QVBoxLayout(diagnostics)
        diagnostics_layout.addWidget(self.status, 1)
        log_actions = QHBoxLayout()
        self.log_component = QComboBox()
        self.log_component.setMinimumWidth(220)
        self.read_log = QPushButton("读取模块日志")
        log_actions.addWidget(self.log_component)
        log_actions.addWidget(self.read_log)
        diagnostics_layout.addLayout(log_actions)
        self.log_text = QPlainTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setPlaceholderText("启动模块后，选择模块读取最近的运行日志。")
        diagnostics_layout.addWidget(self.log_text, 2)
        self.read_log.clicked.connect(
            lambda: self.submit("logs", lambda: self.client.logs(self.log_component.currentData()))
        )
        self.tabs.addTab(diagnostics, "平台状态与故障")
        self.result = QLabel("启动平台不会自动执行任务。配置、启动、执行分别由明确操作触发。")
        self.result.setWordWrap(True)
        self.result.setTextFormat(Qt.PlainText)
        layout.addWidget(self.result)
        self.local.clicked.connect(self.connect_local)
        remote.clicked.connect(self.connect_remote)
        self.load_recipe(self.recipe)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)
        self.tick()

    def build_recipe_tab(self):
        """Expose module choices, typed parameters, calibration files and one platform start."""
        page = QWidget()
        layout = QVBoxLayout(page)
        row = QHBoxLayout()
        self.recipe_name = QLineEdit()
        self.autostart = QCheckBox("平台服务启动时自动运行此方案（不执行任务）")
        row.addWidget(QLabel("方案名称"))
        row.addWidget(self.recipe_name, 1)
        row.addWidget(self.autostart)
        layout.addLayout(row)
        splitter = QSplitter()
        self.components = QListWidget()
        self.components.setObjectName("onboardComponents")
        splitter.addWidget(self.components)
        area = QScrollArea()
        area.setWidgetResizable(True)
        panel = QWidget()
        self.component_form = QFormLayout(panel)
        self.component_form.setRowWrapPolicy(QFormLayout.WrapLongRows)
        area.setWidget(panel)
        splitter.addWidget(area)
        splitter.setStretchFactor(1, 2)
        layout.addWidget(splitter, 1)
        actions = QHBoxLayout()
        self.read = QPushButton("读取当前方案")
        self.save = QPushButton("保存到机载端")
        self.check = QPushButton("检查设备配置与依赖")
        self.start = QPushButton("统一启动平台")
        self.stop = QPushButton("正常停止本平台")
        advanced = QPushButton("高级方案 JSON…")
        for button in (self.read, self.save, self.check, self.start, self.stop, advanced):
            actions.addWidget(button)
        layout.addLayout(actions)
        self.tabs.addTab(page, "设备与算法方案")
        self.components.currentRowChanged.connect(self.select_component)
        self.components.itemChanged.connect(self.toggle_component)
        self.recipe_name.textEdited.connect(self.mark_dirty)
        self.autostart.toggled.connect(self.mark_dirty)
        self.read.clicked.connect(self.read_recipe)
        self.save.clicked.connect(self.save_recipe)
        self.check.clicked.connect(
            lambda: self.operate("check", dict(recipe_hash=self.loaded_hash))
        )
        self.start.clicked.connect(
            lambda: self.operate("start", dict(recipe_hash=self.loaded_hash))
        )
        self.stop.clicked.connect(
            lambda: self.operate("stop", dict(session=self.snapshot["state"]["session"]))
        )
        advanced.clicked.connect(self.advanced_recipe)

    def build_mission_tab(self):
        """Edit, validate, save, review and explicitly execute an onboard mission."""
        page = QWidget()
        layout = QVBoxLayout(page)
        row = QHBoxLayout()
        self.missions = QComboBox()
        self.missions.setObjectName("onboardMissionCatalog")
        self.missions.setPlaceholderText("尚无机载任务，请编辑并保存新任务")
        read = QPushButton("读取并审阅")
        row.addWidget(QLabel("机载任务"))
        row.addWidget(self.missions, 1)
        row.addWidget(read)
        layout.addLayout(row)
        self.mission_name = QLineEdit("mission_001.yaml")
        layout.addWidget(self.mission_name)
        self.mission_text = QPlainTextEdit(MISSION_TEMPLATE)
        self.mission_text.setObjectName("onboardMissionEditor")
        layout.addWidget(self.mission_text, 1)
        hint = QLabel("编辑后保存为机载任务。已有同名且内容不同的文件不覆盖，请另存新名称。" "执行前须重新读取、审阅机载版本；示例坐标应按真实环境修改。")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        actions = QHBoxLayout()
        self.mission_save = QPushButton("校验并保存新任务")
        self.permit = QCheckBox("已审阅本次任务，允许提交执行")
        self.mission_start = QPushButton("执行所审阅任务")
        self.mission_lookup = QPushButton("查询原任务")
        self.mission_cancel = QPushButton("取消本客户端任务")
        for widget in (
            self.mission_save,
            self.permit,
            self.mission_start,
            self.mission_lookup,
            self.mission_cancel,
        ):
            actions.addWidget(widget)
        layout.addLayout(actions)
        self.tabs.addTab(page, "任务配置与执行")
        read.clicked.connect(self.read_task)
        self.mission_save.clicked.connect(
            lambda: self.operate(
                "mission_save",
                dict(name=self.mission_name.text(), content=self.mission_text.toPlainText()),
            )
        )
        self.mission_start.clicked.connect(self.start_task)
        self.mission_lookup.clicked.connect(
            lambda: self.operate("mission_lookup", dict(request_id=self.task_request))
        )
        self.mission_cancel.clicked.connect(
            lambda: self.operate("mission_cancel", dict(request_id=self.task_request))
        )
        self.mission_text.textChanged.connect(self.clear_review)
        self.mission_name.textChanged.connect(self.clear_review)

    def mark_dirty(self, *_):
        """Mark local edits without replacing them during polling."""
        if not self.loading:
            self.dirty = True

    def load_recipe(self, recipe):
        """Populate the editor only after explicit read, initial connection or own save."""
        self.loading = True
        self.recipe = copy.deepcopy(recipe)
        self.recipe_name.setText(recipe["name"])
        self.autostart.setChecked(recipe["autostart"])
        self.selected_component = ""
        self.components.clear()
        for name in CATALOG:
            if name not in self.recipe["components"]:
                self.recipe["components"][name] = template()["components"][name]
            item = QListWidgetItem(CATALOG[name]["title"])
            item.setData(Qt.UserRole, name)
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(
                Qt.Checked if self.recipe["components"][name]["enabled"] else Qt.Unchecked
            )
            self.components.addItem(item)
        self.loading = False
        self.components.setCurrentRow(0)
        self.dirty = False

    def toggle_component(self, item):
        """Enable known dependencies together; an operator may explicitly use external inputs."""
        if self.loading:
            return
        name = item.data(Qt.UserRole)
        self.recipe["components"][name]["enabled"] = item.checkState() == Qt.Checked
        if item.checkState() == Qt.Checked:
            pending, visited = [name], set()
            while pending:
                key = pending.pop()
                if key in visited:
                    continue
                visited.add(key)
                for dependency in self.recipe["components"][key]["depends_on"]:
                    for index in range(self.components.count()):
                        other = self.components.item(index)
                        if other.data(Qt.UserRole) == dependency:
                            self.recipe["components"][dependency]["enabled"] = True
                            with QSignalBlocker(self.components):
                                other.setCheckState(Qt.Checked)
                    pending.append(dependency)
        self.mark_dirty()

    def commit_component(self):
        """Keep a typed local draft without sending any live parameter mutation."""
        if not self.selected_component:
            return
        entry = self.recipe["components"][self.selected_component]
        entry["parameter_file"] = self.parameter_file.text()
        if CATALOG[self.selected_component]["output"]:
            entry["output_topic"] = self.output_topic.text()
        parameters = {}
        for key, (widget, kind) in self.parameter_widgets.items():
            if kind is bool:
                parameters[key] = widget.isChecked()
            elif kind in (int, float):
                parameters[key] = widget.value()
            elif kind is list:
                parameters[key] = json.loads(widget.text())
            else:
                parameters[key] = widget.text()
        entry["parameters"] = parameters

    def select_component(self, row):
        """Use native typed fields; raw JSON is optional advanced editing only."""
        if self.loading or row < 0:
            return
        try:
            self.commit_component()
        except ValueError as error:
            self.result.setText("参数数组格式错误：" + str(error))
            return
        self.loading = True
        while self.component_form.rowCount():
            self.component_form.removeRow(0)
        self.selected_component = self.components.item(row).data(Qt.UserRole)
        entry = self.recipe["components"][self.selected_component]
        self.parameter_widgets = {}
        self.parameter_file = QLineEdit(entry["parameter_file"])
        self.parameter_file.textEdited.connect(self.mark_dirty)
        self.component_form.addRow("ROS 参数文件（机载路径）", self.parameter_file)
        upload = QPushButton("上传设备 / 标定 / 参数文件…")
        upload.clicked.connect(self.upload_configuration)
        self.component_form.addRow(upload)
        supports_file = self.selected_component not in ("mavlink", "local_planner")
        self.parameter_file.setEnabled(supports_file)
        upload.setEnabled(supports_file)
        if not supports_file:
            self.parameter_file.setPlaceholderText("此模块通过下方参数配置")
        self.output_topic = QLineEdit(entry["output_topic"])
        self.output_topic.textEdited.connect(self.mark_dirty)
        if CATALOG[self.selected_component]["output"]:
            self.component_form.addRow("就绪检测话题", self.output_topic)
            self.output_topic.setToolTip("须与模块实际输出一致；修改输出还需设置模块参数或 remappings")
        for key, value in entry["parameters"].items():
            kind = type(value)
            if kind is bool:
                widget = QCheckBox()
                widget.setChecked(value)
                widget.toggled.connect(self.mark_dirty)
            elif kind in (int, float):
                widget = QSpinBox() if kind is int else QDoubleSpinBox()
                widget.setRange(-1000000000, 1000000000)
                if kind is float:
                    widget.setDecimals(8)
                widget.setValue(value)
                widget.valueChanged.connect(self.mark_dirty)
            else:
                widget = QLineEdit(json.dumps(value) if kind is list else value)
                widget.textEdited.connect(self.mark_dirty)
            self.component_form.addRow(key, widget)
            self.parameter_widgets[key] = (widget, kind)
        dependencies = QLabel("依赖：" + "、".join(entry["depends_on"]) + "\n启动仅使用系统时间；硬件参数文件需填写实际标定。")
        dependencies.setWordWrap(True)
        self.component_form.addRow(dependencies)
        self.loading = False

    def edited_recipe(self):
        """Validate the complete graph before saving or previewing it."""
        self.commit_component()
        self.recipe["name"] = self.recipe_name.text()
        self.recipe["autostart"] = self.autostart.isChecked()
        return validate(copy.deepcopy(self.recipe))

    def advanced_recipe(self):
        """Allow explicit advanced remappings, dependencies and extra native parameters."""
        try:
            self.commit_component()
            text, accepted = QInputDialog.getMultiLineText(
                self,
                "高级机载运行方案",
                "JSON（保存到机载端前仍会完整校验）",
                json.dumps(self.recipe, ensure_ascii=False, indent=2),
            )
            if accepted:
                self.load_recipe(validate(json.loads(text)))
                self.dirty = True
        except (ValueError, TypeError) as error:
            self.result.setText("方案未更改：" + str(error))

    def submit(self, kind, function):
        """One request at a time keeps operations ordered and the Qt event loop responsive."""
        if self.future or self.closing:
            return
        self.kind = kind
        self.future = self.pool.submit(function)

    def connect_local(self):
        """Start the local platform service automatically; never start a recipe implicitly."""
        root = Path(
            os.environ.get(
                "EASYFLY_ONBOARD_CONFIG_ROOT", str(Path.home() / ".config/easyfly/onboard")
            )
        ).absolute()
        if not self.confirm_replace_draft():
            return
        domain, allow = self.domain.value(), self.allow_missions.isChecked()
        host = self.advertised_host.text().strip()
        self.local_root = root

        def connect():
            server, client_path = bootstrap_local(root, domain, allow, host)
            client = PlatformClient(client_settings(client_path))
            try:
                return client, client.status()
            except (OSError, ValueError):
                # A domain/state lock in the daemon prevents creating duplicate
                # owners. Credentials bind a listener to this exact platform.
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
                        raise RuntimeError("本机平台服务未启动，请查看 " + str(root / "service.log"))
                    try:
                        return client, client.status()
                    except (OSError, ValueError):
                        time.sleep(0.1)
                raise RuntimeError("本机平台服务尚未就绪；查看服务日志后重新查询")

        self.submit("connect", connect)

    def connect_remote(self):
        """Use the same configuration and task workflow for the ground computer."""
        if not self.confirm_replace_draft():
            return
        path, _ = QFileDialog.getOpenFileName(self, "选择机载端提供的连接配置", "", "JSON (*.json)")
        if not path:
            return
        self.local_root = None

        def connect():
            client = PlatformClient(client_settings(path))
            return client, client.status()

        self.submit("connect", connect)

    def confirm_replace_draft(self):
        """Confirm before discarding a local draft while changing platforms."""
        return (
            not (self.dirty or self.mission_dirty)
            or QMessageBox.question(self, "切换平台", "放弃当前本地未保存修改并连接另一平台？") == QMessageBox.Yes
        )

    def export_connection(self):
        """Create a private portable connection bundle for this operator's ground UI."""
        parent = QFileDialog.getExistingDirectory(self, "选择地面连接包的保存目录")
        if parent and self.local_root:
            try:
                target = Path(parent) / ("easyfly-ground-" + self.snapshot["server_id"][:8])
                self.result.setText("已导出，请完整复制到地面电脑：" + export_client(self.local_root, target))
            except (OSError, ValueError) as error:
                self.result.setText(str(error))

    def install_service(self):
        """Enable the local platform service only after an explicit deployment action."""
        if not self.local_root or not self.snapshot:
            return
        if (
            QMessageBox.question(
                self, "启用本机平台服务", "安装并启用当前用户的平台启动服务？\n" "方案的自动运行选项决定服务启动后是否运行算法；不会自动执行任务。"
            )
            != QMessageBox.Yes
        ):
            return
        self.submit(
            "install_service",
            lambda: install_user_service(self.local_root, self.client, self.snapshot),
        )

    def operate(self, operation, data):
        """Bind every mutation to fresh platform identity; never retry on reply loss."""
        if not self.snapshot or self.future or time.monotonic() - self.fresh_at > 2.5:
            self.result.setText("请先连接并取得新鲜的平台状态")
            return
        request_id = uuid.uuid4().hex
        if operation == "mission_start":
            self.task_request = request_id
        self.result.setText("正在提交：" + operation + "；请求 " + request_id)
        self.submit(
            operation,
            lambda: self.client.operation(self.snapshot, operation, data, request_id=request_id),
        )

    def save_recipe(self):
        """Editing and saving a scheme do not start sensors or flight tasks."""
        try:
            recipe = self.edited_recipe()
            self.operate("configure", dict(recipe=recipe, expected_hash=self.loaded_hash))
        except (ValueError, TypeError) as error:
            self.result.setText("运行方案未保存：" + str(error))

    def read_recipe(self):
        """Explicitly replace the local draft with the last authenticated snapshot."""
        if self.snapshot:
            if (
                self.dirty
                and QMessageBox.question(self, "读取机载方案", "放弃本地未保存修改，读取机载方案？") != QMessageBox.Yes
            ):
                return
            self.load_recipe(self.snapshot["recipe"])
            self.loaded_hash = self.snapshot["recipe_hash"]

    def upload_configuration(self):
        """Import a local parameter file into the agent-owned configuration directory."""
        path, _ = QFileDialog.getOpenFileName(self, "上传硬件配置或标定", "", "配置 (*.yaml *.yml *.json)")
        if not path:
            return
        try:
            self.commit_component()
            source = Path(path)
            if source.stat().st_size > 65536:
                raise ValueError("文件超过 64 KiB")
            self.upload_component = self.selected_component
            self.operate(
                "configuration_upload", dict(name=source.name, content=source.read_text())
            )
        except (ValueError, OSError) as error:
            self.result.setText(str(error))

    def clear_review(self):
        """Any local edit invalidates the reviewed onboard content identity."""
        self.review = None
        self.mission_dirty = True
        self.permit.setChecked(False)

    def read_task(self):
        """Read exact saved bytes before enabling a task submission."""
        if self.missions.currentText():
            self.operate("mission_read", dict(name=self.missions.currentText()))

    def start_task(self):
        """Require deliberate review and bind the exact task hash, not editable text."""
        if not self.review or not self.permit.isChecked():
            return
        name, identity = self.review
        if (
            QMessageBox.question(
                self,
                "执行机载任务",
                "平台 %s\n任务 %s\n内容 %s\n提交本次任务？" % (self.snapshot["server_id"], name, identity),
            )
            == QMessageBox.Yes
        ):
            self.operate("mission_start", dict(name=name, sha256=identity, confirmed=True))

    def handle_result(self, value):
        """Display the receipt without confusing process start with algorithm readiness."""
        if not value.get("accepted"):
            self.result.setText("未执行：" + value.get("reason", str(value)))
            return
        self.result.setText("已受理：" + value.get("phase", ""))
        if self.kind == "logs":
            self.log_text.setPlainText(value["content"] or "模块尚无日志输出。")
        elif self.kind == "configure":
            self.loaded_hash = value["recipe_hash"]
            self.dirty = False
        elif self.kind == "mission_read":
            self.mission_name.setText(value["name"])
            self.mission_text.setPlainText(value["content"])
            self.review = (value["name"], value["sha256"])
            self.mission_dirty = False
        elif self.kind == "configuration_upload":
            target = self.recipe["components"][self.upload_component]
            key = {"camera_calib": "calib_file", "livox": "user_config_path"}.get(
                self.upload_component
            )
            if key:
                target["parameters"][key] = value["path"]
            else:
                target["parameter_file"] = value["path"]
            self.selected_component = ""
            self.select_component(self.components.currentRow())
            self.dirty = True
            self.result.setText("已上传配置：" + value["path"])
        if "task" in value:
            self.task_request = value["task"]["request_id"]
        if self.kind == "mission_save":
            self.mission_dirty = False

    def render(self):
        """Update runtime state without overwriting a local draft."""
        snapshot = self.snapshot
        self.identity.setText(
            "平台 %s · ROS domain %s · 系统时间 · %s\n%s"
            % (
                snapshot["server_id"],
                snapshot["ros_domain_id"],
                snapshot["state"]["phase"],
                snapshot["state"].get("error", ""),
            )
        )
        if not self.loaded_hash:
            self.load_recipe(snapshot["recipe"])
            self.loaded_hash = snapshot["recipe_hash"]
        elif snapshot["recipe_hash"] != self.loaded_hash and not self.dirty:
            self.load_recipe(snapshot["recipe"])
            self.loaded_hash = snapshot["recipe_hash"]
        choice = self.missions.currentText()
        self.missions.clear()
        self.missions.addItems([row["name"] for row in snapshot["missions"]])
        if self.missions.findText(choice) >= 0:
            self.missions.setCurrentText(choice)
        self.status.setPlainText(json.dumps(snapshot["state"], ensure_ascii=False, indent=2))
        selected_log = self.log_component.currentData()
        self.log_component.clear()
        for key in snapshot["state"].get("components", {}):
            self.log_component.addItem(CATALOG[key]["title"], key)
        previous = self.log_component.findData(selected_log)
        if previous >= 0:
            self.log_component.setCurrentIndex(previous)

    def tick(self):
        """Expire all operation permissions on loss of authenticated fresh state."""
        now = time.monotonic()
        if self.future and self.future.done():
            future, self.future = self.future, None
            try:
                value = future.result()
                if self.kind == "connect":
                    self.client, self.snapshot = value
                    self.loaded_hash = ""
                    self.fresh_at, self.next_poll = now, now + 0.8
                    self.render()
                elif self.kind == "status":
                    self.snapshot = value
                    self.fresh_at, self.next_poll = now, now + 0.8
                    self.render()
                else:
                    self.handle_result(value)
                    self.next_poll = 0.0
            except Exception as error:
                self.fresh_at, self.next_poll = 0.0, now + 1.0
                self.result.setText("请求未确认；先查询原会话，不自动重发。" + str(error))
        fresh = bool(self.snapshot and now - self.fresh_at <= 2.5)
        available = bool(fresh and not self.future and not self.closing)
        state = self.snapshot["state"] if self.snapshot else {}
        idle = state.get("phase") in ("stopped", "failed")
        for button in (self.save, self.check, self.start):
            button.setEnabled(available and idle)
        self.check.setEnabled(self.check.isEnabled() and not self.dirty)
        self.start.setEnabled(self.start.isEnabled() and not self.dirty)
        self.read.setEnabled(available)
        self.read_log.setEnabled(available and self.log_component.count() > 0)
        self.local.setEnabled(not self.future and not self.closing)
        self.export.setEnabled(bool(available and self.local_root))
        self.install_boot.setEnabled(bool(available and idle and self.local_root))
        editing = not self.future or self.kind == "status"
        for widget in (
            self.components,
            self.recipe_name,
            self.autostart,
            self.component_form.parentWidget(),
            self.mission_text,
            self.mission_name,
        ):
            widget.setEnabled(editing and not self.closing)
        self.stop.setEnabled(
            bool(
                available
                and state.get("client_id")
                == (self.client.config["client_id"] if self.client else "")
                and self.snapshot["conditions"]["can_stop"]
                and state.get("phase") not in ("stopped", "failed")
            )
        )
        self.mission_save.setEnabled(available)
        if fresh:
            if self.snapshot.get("maintenance"):
                self.mission_policy.setText("当前启用了 QGC 转发，处于维护配置；关闭转发后可使用受管任务。")
            elif not self.snapshot["mission_execution_enabled"]:
                self.mission_policy.setText("当前部署允许任务配置；任务执行未启用。首次部署可选择受管任务执行。")
            elif not self.snapshot["conditions"].get("mission_stack_ready", False):
                self.mission_policy.setText("执行前需在方案中启用并统一启动导航、飞控控制、任务状态机和任务服务。")
            elif not self.snapshot["conditions"]["grounded"]:
                self.mission_policy.setText("等待唯一、新鲜的未解锁/在地面状态；仍可编辑和保存任务。")
            else:
                self.mission_policy.setText("平台满足提交条件；读取机载任务并审阅确认后，可提交本次任务。")
        self.mission_start.setEnabled(
            bool(
                available
                and self.review
                and self.permit.isChecked()
                and self.snapshot["mission_execution_enabled"]
                and state.get("phase") == "running"
                and self.snapshot["conditions"]["grounded"]
                and self.snapshot["conditions"].get("mission_stack_ready", False)
            )
        )
        self.mission_lookup.setEnabled(available and bool(self.task_request))
        self.mission_cancel.setEnabled(available and bool(self.task_request))
        if not fresh and self.snapshot:
            self.identity.setText("连接或状态已过期；机载运行状态未知，编辑草稿仍保留")
        if self.client and not self.future and not self.closing and now >= self.next_poll:
            self.submit("status", self.client.status)
        if self.closing and not self.future:
            self.close()

    def closeEvent(self, event):  # noqa: N802
        """Close only this UI; the platform and task owner stay on the aircraft."""
        self.closing = True
        if self.future:
            event.ignore()
            return
        if (self.dirty or self.mission_dirty) and QMessageBox.question(
            self, "未保存修改", "放弃本地未保存的方案/任务修改并关闭窗口？"
        ) != QMessageBox.Yes:
            self.closing = False
            event.ignore()
            return
        self.timer.stop()
        self.pool.shutdown(wait=True)
        event.accept()


def main():
    """Open the local/remote platform workflow from the existing EasyFly connection page."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "0")))
    args = parser.parse_args()
    app = QApplication(sys.argv[:1])
    window = PlatformWindow()
    window.domain.setValue(args.domain)
    window.show()
    raise SystemExit(app.exec_())

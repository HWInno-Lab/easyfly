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
"""Native Qt remote algorithm management with fresh status and owned-session actions."""

from concurrent.futures import ThreadPoolExecutor
import sys
import time

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QFileDialog, QHBoxLayout, QLabel, QLineEdit, QPushButton,
    QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

from .remote_client import Client
from .remote_profiles import private_json


class RemoteWindow(QWidget):
    """Keep network operations off the UI thread and never turn stale status into permission."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle('EasyFly · 远端算法管理')
        self.resize(920, 580)
        self.setStyleSheet(
            'QWidget {background:#f3f6fb;color:#263244;font-size:14px;}'
            'QLineEdit,QComboBox,QTableWidget {background:white;border:1px solid '
            '#ccd7e5;padding:6px;} QPushButton {padding:8px 14px;border:1px solid '
            '#8ea7c8;border-radius:4px;} QPushButton:focus {border:2px solid #2563eb;}'
            'QPushButton:disabled {color:#8190a5;border-color:#d7e0ec;}')
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.kind = ''
        self.client = None
        self.snapshot = None
        self.fresh_at = 0.
        self.next_poll = 0.
        self.closing = False
        layout = QVBoxLayout(self)
        title = QLabel('远端算法管理')
        title.setStyleSheet('font-size:24px;font-weight:600;')
        layout.addWidget(title)
        hint = QLabel('选择管理员提供的连接配置，核对平台后启动或停止算法。关闭窗口后远端继续运行。')
        hint.setWordWrap(True)
        layout.addWidget(hint)
        row = QHBoxLayout()
        self.path = QLineEdit()
        self.path.setObjectName('remoteSettingsPath')
        self.path.setPlaceholderText('私有连接配置 JSON 文件')
        choose = QPushButton('选择配置…')
        self.connect_button = QPushButton('连接并核对平台')
        row.addWidget(self.path, 1)
        row.addWidget(choose)
        row.addWidget(self.connect_button)
        layout.addLayout(row)
        self.identity = QLabel('尚未连接平台')
        self.identity.setTextFormat(Qt.PlainText)
        self.identity.setWordWrap(True)
        layout.addWidget(self.identity)
        self.connection = QLabel('等待连接')
        self.connection.setTextFormat(Qt.PlainText)
        layout.addWidget(self.connection)
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(['预设 / 算法', '进程状态', '输出活动', '操作条件', '会话'])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        layout.addWidget(self.table, 1)
        controls = QHBoxLayout()
        self.profile = QComboBox()
        self.start = QPushButton('启动所选算法')
        self.stop = QPushButton('停止本客户端会话')
        self.refresh = QPushButton('重新查询')
        for widget in (self.profile, self.start, self.stop, self.refresh):
            controls.addWidget(widget)
        layout.addLayout(controls)
        self.result = QLabel('尚未提交操作')
        self.result.setTextFormat(Qt.PlainText)
        self.result.setWordWrap(True)
        layout.addWidget(self.result)
        choose.clicked.connect(self.choose)
        self.connect_button.clicked.connect(self.connect_remote)
        self.start.clicked.connect(lambda: self.operate('start'))
        self.stop.clicked.connect(lambda: self.operate('stop'))
        self.refresh.clicked.connect(lambda: self.submit('status', self.client.status))
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)
        self.tick()

    def choose(self):
        """Select a file; no remote operation happens merely because a file was chosen."""
        path, _ = QFileDialog.getOpenFileName(self, '选择远端连接配置', '', 'JSON (*.json)')
        if path:
            self.path.setText(path)

    def submit(self, kind, function):
        """Keep one in-flight request; operations are never retried automatically."""
        if self.future or self.closing:
            return
        self.kind = kind
        self.future = self.pool.submit(function)

    def connect_remote(self):
        """Load credentials privately and request a verified current platform snapshot."""
        if self.future:
            return
        self.snapshot = None
        self.fresh_at = 0.
        self.client = None
        self.table.setRowCount(0)
        self.profile.clear()
        path = self.path.text()

        def connect():
            client = Client(private_json(path))
            return client, client.status()

        self.submit('connect', connect)

    def operate(self, operation):
        """Bind an explicit click to the exact platform boot, preset and owned session."""
        if not self.snapshot or time.monotonic() - self.fresh_at > 2.5 or self.future:
            return
        name = self.profile.currentText()
        state = self.snapshot['sessions'].get(name, {})
        session = state.get('session', '') if operation == 'stop' else ''
        snapshot = self.snapshot
        self.result.setText('请求已提交；正在等待远端确认…')
        self.submit('operation', lambda: self.client.operation(snapshot, operation, name, session))

    def render(self):
        """Show process and output activity separately; neither implies flight readiness."""
        snapshot = self.snapshot
        self.identity.setText('平台 %s · 服务会话 %s' % (
            snapshot['server_id'], snapshot['boot_id']))
        selected = self.profile.currentText()
        self.profile.clear()
        self.profile.addItems(snapshot['profiles'])
        if selected in snapshot['profiles']:
            self.profile.setCurrentText(selected)
        self.table.setRowCount(len(snapshot['profiles']))
        for row, (name, data) in enumerate(snapshot['profiles'].items()):
            state = snapshot['sessions'].get(name, {})
            permit = data['grounded'] if data['policy'] == 'grounded' else data['isolated']
            values = [name + ' / ' + data['algorithm'], state.get('phase', '未启动'),
                      '近期有输出' if data['output_receipt_fresh'] else '未收到近期输出',
                      '条件满足' if permit else '等待有效状态', state.get('session', '—')]
            for column, value in enumerate(values):
                self.table.setItem(row, column, QTableWidgetItem(value))
        self.table.resizeColumnsToContents()

    def tick(self):
        """Expire permissions on lost status and retain remote ownership on window close."""
        now = time.monotonic()
        if self.future and self.future.done():
            future, self.future = self.future, None
            try:
                value = future.result()
                if self.kind == 'operation':
                    self.result.setText('已受理：' + value.get('phase', '') if value.get('accepted')
                                        else '操作未执行：' + value.get('reason', str(value)))
                    self.next_poll = 0.
                else:
                    if self.kind == 'connect':
                        self.client, value = value
                    self.snapshot = value
                    self.fresh_at = now
                    self.next_poll = now + .8
                    self.render()
            except Exception as exc:
                self.fresh_at = 0.
                self.next_poll = now + 1.
                prefix = '操作结果未知；请先查询原会话。' if self.kind == 'operation' else '连接不可用：'
                self.result.setText(prefix + str(exc))
        fresh = bool(self.snapshot and now - self.fresh_at <= 2.5)
        self.connection.setText('已连接 · 状态有效' if fresh else '未连接或状态过期 · 远端状态未知')
        if not fresh and self.snapshot:
            for row in range(self.table.rowCount()):
                previous = self.table.item(row, 1)
                if previous and not previous.text().startswith('上次：'):
                    previous.setText('上次：' + previous.text())
                self.table.setItem(row, 2, QTableWidgetItem('状态过期'))
                self.table.setItem(row, 3, QTableWidgetItem('状态未知'))
        available = fresh and not self.future and not self.closing
        name = self.profile.currentText()
        data = self.snapshot['profiles'].get(name, {}) if self.snapshot else {}
        state = self.snapshot['sessions'].get(name, {}) if self.snapshot else {}
        permit = data.get('grounded') if data.get('policy') == 'grounded' else data.get('isolated')
        self.start.setEnabled(bool(
            available and permit and not data.get('output_publishers') and
            state.get('phase', 'stopped') in ('stopped', 'failed')))
        self.stop.setEnabled(bool(
            available and permit and state.get('phase') == 'running' and
            state.get('client_id') == self.client.config['client_id']))
        self.connect_button.setEnabled(not self.future and not self.closing)
        self.refresh.setEnabled(bool(self.client and not self.future and not self.closing))
        if self.client and not self.future and not self.closing and now >= self.next_poll:
            self.submit('status', self.client.status)
        if self.closing and not self.future:
            self.close()

    def closeEvent(self, event):  # noqa: N802
        """Wait for an already submitted request; never send a remote stop on GUI exit."""
        self.closing = True
        if self.future:
            event.ignore()
        else:
            self.timer.stop()
            self.pool.shutdown(wait=True)
            event.accept()


def main():
    """Start the native client without creating an in-process ROS context."""
    app = QApplication(sys.argv[:1])
    window = RemoteWindow()
    window.show()
    raise SystemExit(app.exec_())

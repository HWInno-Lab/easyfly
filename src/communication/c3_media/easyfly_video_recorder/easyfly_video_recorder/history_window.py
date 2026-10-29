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

"""Local historical image/trajectory window and explicit independent recorder."""

from bisect import bisect_right
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path
import signal
import sys
import uuid

import cv2
from PyQt5.QtCore import QPointF, QProcess, Qt, QTimer
from PyQt5.QtGui import QColor, QImage, QPainter, QPainterPath, QPen, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QFileDialog, QFormLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QPushButton, QSlider, QSplitter, QVBoxLayout, QWidget,
)

from .replay import isolated_command
from .storage import inspect_session


def load_history(path):
    """Validate all fingerprints and bound retained metadata before displaying it."""
    path = Path(path)
    record = inspect_session(path)
    rows = []
    for filename, limit in [('frames.jsonl', 16 * 1024**2), ('trajectory.jsonl', 64 * 1024**2)]:
        payload = path / filename
        if payload.stat().st_size > limit:
            raise ValueError('history metadata exceeds viewer limit')
        values = [json.loads(line) for line in payload.read_text().splitlines()]
        if len(values) > 200000:
            raise ValueError('history sample count exceeds viewer limit')
        stamps = [r['source_ns'] for r in values]
        if any(b <= a for a, b in zip(stamps, stamps[1:])):
            raise ValueError('history timestamps are not strictly increasing')
        rows.append(values)
    frames, poses = rows
    if len(frames) != record['frames'] or len(poses) != record['poses'] or not frames:
        raise ValueError('history manifest sample count mismatch')
    for row in frames:
        if row['segment'] not in record['segments'] or row['segment'] not in record['files']:
            raise ValueError('unknown image segment')
    for row in poses:
        if len(row['pose']) != 7 or not all(math.isfinite(v) for v in row['pose']):
            raise ValueError('invalid recorded pose')
    return record, frames, poses


def decode_frame(root, row):
    """Perform file seek and image decode off the GUI thread."""
    cap = cv2.VideoCapture(str(root / row['segment']), cv2.CAP_OPENCV_MJPEG)
    try:
        cap.set(cv2.CAP_PROP_POS_FRAMES, row['segment_frame'])
        ok, frame = cap.read()
        if not ok:
            raise ValueError('recorded video frame could not be decoded')
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    finally:
        cap.release()


class Trajectory(QWidget):
    """Draw metric XY history with a source-time-selected native position."""

    def __init__(self):
        super().__init__()
        self.setObjectName('historyTrajectory')
        self.setMinimumSize(320, 220)
        self.rows = []
        self.source = 0

    def paintEvent(self, event):  # noqa: N802
        """Fit the recorded path while preserving equal X/Y meter scales."""
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor('#ffffff'))
        painter.setPen(QColor('#1f2937'))
        if not self.rows:
            painter.drawText(self.rect(), Qt.AlignCenter, '没有记录原生轨迹')
            return
        points = [(r['pose'][0], r['pose'][1]) for r in self.rows]
        xs, ys = zip(*points)
        xmin, xmax, ymin, ymax = min(xs), max(xs), min(ys), max(ys)
        scale = min((self.width() - 64) / max(.5, xmax - xmin),
                    (self.height() - 80) / max(.5, ymax - ymin))

        def point(value):
            return QPointF(self.width() / 2 + (value[0] - (xmin + xmax) / 2) * scale,
                           self.height() / 2 - (value[1] - (ymin + ymax) / 2) * scale)

        path = QPainterPath()
        stride = max(1, len(points) // 10000)
        path.moveTo(point(points[0]))
        for value in points[::stride]:
            path.lineTo(point(value))
        path.lineTo(point(points[-1]))
        painter.setPen(QPen(QColor('#1769df'), 2))
        painter.drawPath(path)
        index = max(0, bisect_right([r['source_ns'] for r in self.rows], self.source) - 1)
        selected = self.rows[index]
        painter.setPen(QColor('#b45309'))
        painter.setBrush(QColor('#b45309'))
        painter.drawEllipse(point(points[index]), 5, 5)
        painter.drawText(16, 24, '历史 XY 轨迹 · ' + selected['frame_id'] + ' · 米')
        painter.drawText(16, self.height() - 14,
                         'X %.3f   Y %.3f   Z %.3f' % tuple(selected['pose'][:3]))


class HistoryWindow(QWidget):
    """Keep live recording controls and historical replay explicitly separated."""

    def __init__(self, executable=None):
        super().__init__()
        self.executable = Path(executable or sys.argv[0]).resolve()
        self.setObjectName('historyWindow')
        self.setWindowTitle('EasyFly · 原始记录与历史轨迹')
        self.resize(1200, 800)
        self.setMinimumSize(1024, 720)
        self.setStyleSheet('''
            QWidget {background:#f3f6fb;color:#1f2937;font-size:14px;}
            QLineEdit,QComboBox {background:#ffffff;border:1px solid #cbd5e1;padding:6px;}
            QPushButton {padding:8px 14px;border:1px solid #94a3b8;border-radius:4px;}
            QPushButton:hover {background:#eaf0f8;}
            QPushButton:disabled {color:#64748b;border-color:#dce3eb;}
            QPushButton:focus,QLineEdit:focus,QComboBox:focus {border:2px solid #1769df;}
            QGroupBox {margin-top:12px;padding-top:14px;}
        ''')
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='history-files')
        self.loading = self.decoding = None
        self.root = None
        self.frames, self.poses = [], []
        self.selected, self.displayed = 0, -1
        self.closing = False
        self.record_process, self.replay_process = QProcess(self), QProcess(self)
        self.record_buffer = self.replay_buffer = b''
        self.last_record = ''
        self.record_stop_pending = self.replay_stop_pending = False
        self.record_signal_pid = 0
        layout = QVBoxLayout(self)
        title = QLabel('原始记录与历史轨迹')
        title.setStyleSheet('font-size:24px;font-weight:600;')
        layout.addWidget(title)
        description = QLabel('历史窗口不发送飞行指令。原始数据回放在独立网络中运行。')
        description.setWordWrap(True)
        layout.addWidget(description)
        group = QGroupBox('独立录制 · 订阅当前 ROS 数据')
        form = QFormLayout(group)
        self.destination = QLineEdit(str(Path.home() / 'easyfly-history'))
        self.destination.setObjectName('historyRecordDirectory')
        self.image_topic = QLineEdit('/camera/color/image_raw')
        self.odom_topic = QLineEdit('/mavros/local_position/odom')
        self.simulation = QCheckBox('使用仿真时间')
        form.addRow('保存目录', self.destination)
        form.addRow('图像话题', self.image_topic)
        form.addRow('定位话题', self.odom_topic)
        form.addRow('', self.simulation)
        self.record_start = QPushButton('开始记录视频、轨迹与原始数据')
        self.record_stop = QPushButton('停止并保存')
        row = QHBoxLayout()
        row.addWidget(self.record_start)
        row.addWidget(self.record_stop)
        form.addRow(row)
        layout.addWidget(group)
        history = QHBoxLayout()
        self.path = QLineEdit()
        self.path.setObjectName('historySessionPath')
        self.path.setPlaceholderText('已完成的 UUID 会话目录')
        choose = QPushButton('选择历史记录…')
        self.open_button = QPushButton('校验并打开')
        self.open_button.setObjectName('historyOpen')
        history.addWidget(self.path, 1)
        history.addWidget(choose)
        history.addWidget(self.open_button)
        layout.addLayout(history)
        self.status = QLabel('请选择记录；打开后默认暂停。')
        self.status.setObjectName('historyStatus')
        self.status.setTextFormat(Qt.PlainText)
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        split = QSplitter(Qt.Horizontal)
        self.image = QLabel('历史图像')
        self.image.setAlignment(Qt.AlignCenter)
        self.image.setMinimumSize(320, 200)
        self.image.setObjectName('historyImage')
        self.trajectory = Trajectory()
        split.addWidget(self.image)
        split.addWidget(self.trajectory)
        layout.addWidget(split, 1)
        self.timeline = QSlider(Qt.Horizontal)
        self.timeline.setObjectName('historyTimeline')
        layout.addWidget(self.timeline)
        self.source_label = QLabel('源时间：尚未选择记录')
        self.source_label.setObjectName('historySourceTime')
        layout.addWidget(self.source_label)
        controls = QHBoxLayout()
        self.previous, self.next = QPushButton('上一帧'), QPushButton('下一帧')
        self.rate = QComboBox()
        self.rate.addItems(['0.25', '0.5', '1', '2', '4'])
        self.rate.setCurrentText('1')
        self.play = QPushButton('启动隔离回放（暂停）')
        self.resume, self.pause, self.stop_replay = (
            QPushButton('播放'), QPushButton('暂停'), QPushButton('结束回放'))
        for widget in [self.previous, self.next, self.rate, self.play,
                       self.resume, self.pause, self.stop_replay]:
            controls.addWidget(widget)
        layout.addLayout(controls)
        choose.clicked.connect(self.choose)
        self.open_button.clicked.connect(self.open_history)
        self.timeline.valueChanged.connect(self.select_frame)
        self.timeline.sliderReleased.connect(self.seek)
        self.previous.clicked.connect(lambda: self.timeline.setValue(self.timeline.value() - 1))
        self.next.clicked.connect(lambda: self.timeline.setValue(self.timeline.value() + 1))
        self.play.clicked.connect(self.start_replay)
        self.resume.clicked.connect(lambda: self.command('resume'))
        self.pause.clicked.connect(lambda: self.command('pause'))
        self.stop_replay.clicked.connect(lambda: self.command('stop'))
        self.record_start.clicked.connect(self.start_recording)
        self.record_stop.clicked.connect(self.stop_recording)
        for process, kind in [(self.record_process, 'record'), (self.replay_process, 'replay')]:
            process.setProcessChannelMode(QProcess.MergedChannels)
            process.readyReadStandardOutput.connect(lambda k=kind: self.receive(k))
            process.finished.connect(lambda code, status, k=kind: self.finished(k, code))
            process.errorOccurred.connect(lambda error, k=kind: self.status.setText(
                k + ' 进程错误：' + str(error)))
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)

    def choose(self):
        """Select an existing session without altering any local record."""
        path = QFileDialog.getExistingDirectory(self, '选择 UUID 会话目录')
        if path:
            self.path.setText(path)

    def open_history(self):
        """Perform hashing and indexing asynchronously before allowing playback."""
        if self.loading or self.decoding or self.replay_process.state() != QProcess.NotRunning:
            return
        self.root = Path(self.path.text())
        self.frames, self.poses = [], []
        self.trajectory.rows = []
        self.trajectory.update()
        self.image.clear()
        self.image.setText('正在校验历史记录…')
        self.source_label.setText('源时间：尚未选择记录')
        self.displayed = -1
        self.status.setText('正在校验文件和源时间…')
        self.loading = self.pool.submit(load_history, self.root)

    def select_frame(self, index):
        """Seek local display by its original image source timestamp."""
        self.selected = index
        if self.frames:
            self.source_label.setText('第 %d / %d 帧 · 源时间 %.9f 秒 · %s' % (
                index + 1, len(self.frames), self.frames[index]['source_ns'] / 1e9,
                self.frames[index]['frame_id']))
            self.trajectory.source = self.frames[index]['source_ns']
            self.trajectory.update()

    def seek(self):
        """Request a new source time only in the isolated player."""
        if self.frames:
            self.command('seek', source_ns=self.frames[self.selected]['source_ns'])

    def start_recording(self):
        """Start an owned recorder with a unique service namespace and explicit inputs."""
        self.last_record = ''
        self.record_stop_pending = False
        self.record_signal_pid = 0
        self.record_buffer = b''
        command = self.executable.with_name('video_recorder')
        args = ['--ros-args', '-r', '__ns:=/history_recorder_' + uuid.uuid4().hex,
                '-p', 'directory:=' + self.destination.text(),
                '-p', 'image_topic:=' + self.image_topic.text(),
                '-p', 'odom_topic:=' + self.odom_topic.text(),
                '-p', 'use_sim_time:=' + str(self.simulation.isChecked()).lower(),
                '-p', 'auto_start:=true', '-p', 'record_bag:=true',
                '-p', 'report_stdout:=true']
        self.record_process.start(str(command), args)

    def stop_recording(self):
        """Ask only this window's child to finalize and exit normally."""
        if self.record_process.state() == QProcess.Starting:
            self.record_stop_pending = True
            return
        pid = self.record_process.processId()
        if pid and pid != self.record_signal_pid:
            os.kill(pid, signal.SIGINT)
            self.record_signal_pid = pid
        self.record_stop_pending = False

    def start_replay(self):
        """Launch the verified bag through the mandatory namespace boundary."""
        try:
            command = isolated_command(self.executable, self.root, float(self.rate.currentText()))
            self.replay_stop_pending = False
            self.replay_process.start(command[0], command[1:])
        except Exception as exc:
            self.status.setText('无法启动回放：' + str(exc))

    def command(self, action, **fields):
        """Send a bounded control line to the owned replay process only."""
        if action == 'stop' and self.replay_process.state() == QProcess.Starting:
            self.replay_stop_pending = True
            return
        if self.replay_process.state() == QProcess.Running:
            if action == 'stop':
                self.replay_stop_pending = False
            self.replay_process.write((json.dumps({'action': action, **fields}) + '\n').encode())

    def receive(self, kind):
        """Render actual child state, not the fact that a process was started."""
        process = self.record_process if kind == 'record' else self.replay_process
        name = kind + '_buffer'
        buffer = getattr(self, name) + bytes(process.readAllStandardOutput())
        lines = buffer.split(b'\n')
        setattr(self, name, lines.pop()[-8192:])
        for line in lines:
            try:
                value = json.loads(line)
            except (ValueError, UnicodeDecodeError):
                continue
            if not isinstance(value, dict) or 'phase' not in value:
                continue
            if kind == 'record':
                self.last_record = value.get('directory', '') or self.last_record
                self.status.setText('录制：%s · %d 帧 · %d 个轨迹点 · %s' % (
                    value['phase'], value.get('frames', 0), value.get('poses', 0),
                    value.get('reason', '')))
            else:
                source = value.get('source_ns', 0)
                self.status.setText('隔离回放：%s · 已发布 %d 条 · 源时间 %.3f 秒' % (
                    value['phase'], value.get('published', 0), source / 1e9))
                if self.frames:
                    index = max(0, bisect_right([r['source_ns'] for r in self.frames], source) - 1)
                    self.timeline.setValue(min(index, len(self.frames) - 1))

    def finished(self, kind, code):
        """Expose exit status and make the completed recorder directory accessible."""
        self.status.setText('%s 进程已退出，返回值 %d' % (kind, code))
        if kind == 'record' and self.last_record:
            self.path.setText(self.last_record)

    def tick(self):
        """Collect background results and keep controls consistent with actual state."""
        try:
            if self.loading and self.loading.done():
                future, self.loading = self.loading, None
                record, self.frames, self.poses = future.result()
                self.trajectory.rows = self.poses
                self.timeline.setRange(0, len(self.frames) - 1)
                self.timeline.setValue(0)
                self.selected = 0
                self.select_frame(0)
                self.trajectory.source = self.frames[0]['source_ns']
                self.trajectory.update()
                self.status.setText('校验通过：%d 帧，%d 个原生轨迹点；默认暂停。' % (
                    record['frames'], record['poses']))
            if self.decoding and self.decoding[1].done():
                index, future = self.decoding
                self.decoding = None
                pixels = future.result()
                image = QImage(pixels.data, pixels.shape[1], pixels.shape[0],
                               pixels.strides[0], QImage.Format_RGB888).copy()
                self.image.setPixmap(QPixmap.fromImage(image).scaled(
                    self.image.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
                self.displayed = index
            if self.frames and not self.decoding and self.selected != self.displayed:
                self.decoding = (self.selected, self.pool.submit(
                    decode_frame, self.root, self.frames[self.selected]))
        except Exception as exc:
            self.status.setText('历史记录不可用：' + str(exc))
            self.frames, self.poses = [], []
            self.trajectory.rows = []
            self.trajectory.update()
            self.image.clear()
            self.image.setText('历史数据不可用')
            self.source_label.setText('源时间：不可用')
        if self.record_stop_pending and self.record_process.state() == QProcess.Running:
            self.stop_recording()
        if self.replay_stop_pending and self.replay_process.state() == QProcess.Running:
            self.command('stop')
        recording = self.record_process.state() != QProcess.NotRunning
        replaying = self.replay_process.state() != QProcess.NotRunning
        self.record_start.setEnabled(not recording and not self.closing)
        self.record_stop.setEnabled(recording)
        for control in [self.destination, self.image_topic, self.odom_topic, self.simulation]:
            control.setEnabled(not recording)
        self.open_button.setEnabled(not self.loading and not self.decoding and not replaying)
        self.play.setEnabled(bool(self.frames) and not replaying and not self.closing)
        self.rate.setEnabled(not replaying)
        for control in [self.resume, self.pause, self.stop_replay]:
            control.setEnabled(replaying)
        for control in [self.timeline, self.previous, self.next]:
            control.setEnabled(bool(self.frames))
        if (self.closing and not recording and not replaying and
                not self.loading and not self.decoding):
            self.close()

    def closeEvent(self, event):  # noqa: N802
        """Finalize owned recordings and stop replay before destroying QProcess."""
        self.closing = True
        self.stop_recording()
        self.command('stop')
        if (self.record_process.state() != QProcess.NotRunning or
                self.replay_process.state() != QProcess.NotRunning or
                self.loading or self.decoding):
            event.ignore()
        else:
            self.timer.stop()
            self.pool.shutdown(wait=True)
            event.accept()


def run():
    """Open a native Qt history window without initializing ROS in the GUI."""
    app = QApplication(sys.argv[:1])
    window = HistoryWindow()
    window.show()
    raise SystemExit(app.exec_())

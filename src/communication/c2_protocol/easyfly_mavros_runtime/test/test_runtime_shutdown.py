# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Synthetic UDP traffic through actual installed MAVROS plugins and owned I/O."""
import os
from pathlib import Path
import pty
import signal
import socket
import struct
import subprocess
import threading
import time
import tty

import pytest


def heartbeat(sequence):
    payload = struct.pack('<IBBBBB', 0, 2, 12, 0, 3, 3)
    body = bytes([len(payload), sequence % 256, 1, 1, 0]) + payload
    crc = 0xffff
    for value in body + bytes([50]):
        value ^= crc & 0xff
        value ^= (value << 4) & 0xff
        crc = ((crc >> 8) ^ (value << 8) ^ (value << 3) ^ (value >> 4)) & 0xffff
    return b'\xfe' + body + struct.pack('<H', crc)


@pytest.mark.parametrize('stop_signal', [signal.SIGINT, signal.SIGTERM])
@pytest.mark.parametrize('repeat', range(3))
def test_udp_callbacks_retire_before_library_unload(tmp_path, stop_signal, repeat):
    executable = os.environ['RUNTIME_EXECUTABLE']
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        sender.bind(('127.0.0.1', 0))
        remote = sender.getsockname()[1]
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
            reserve.bind(('127.0.0.1', 0))
            local = reserve.getsockname()[1]
        env = dict(os.environ, ROS_DOMAIN_ID='202', ROS_LOCALHOST_ONLY='1')
        path = tmp_path/'runtime.log'
        stop = threading.Event()
        sent = [0]

        def transmit():
            while not stop.is_set():
                sender.sendto(heartbeat(sent[0]), ('127.0.0.1', local))
                sent[0] += 1
                stop.wait(.001)

        worker = threading.Thread(target=transmit)
        with path.open('w') as output:
            process = subprocess.Popen([executable, '--ros-args', '-p',
                f'fcu_url:=udp://127.0.0.1:{local}@127.0.0.1:{remote}'],
                env=env, stdout=output, stderr=subprocess.STDOUT)
            worker.start()
            try:
                deadline = time.monotonic()+15
                while 'MAVROS UAS via' not in path.read_text():
                    assert process.poll() is None, path.read_text()
                    assert time.monotonic() < deadline, path.read_text()
                    time.sleep(.02)
                maps = Path(f'/proc/{process.pid}/maps').read_text()
                assert '/opt/ros/humble/lib/libmavros.so' not in maps
                assert 'libmavros.so' in maps
                time.sleep(.15+repeat*.05)
                process.send_signal(stop_signal)
                assert process.wait(timeout=10) == 0, path.read_text()
                assert 'I/O joined, Router/UAS destroyed' in path.read_text()
                assert sent[0] > 20
            finally:
                stop.set()
                worker.join(timeout=2)
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=3)


@pytest.mark.parametrize('stop_signal', [signal.SIGINT, signal.SIGTERM])
@pytest.mark.parametrize('disconnect', [False, True])
@pytest.mark.parametrize('repeat', range(3))
def test_serial_callbacks_drain_and_join(tmp_path, stop_signal, disconnect, repeat):
    """Actual serial transport: cancellation and peer-loss callbacks both retire."""
    master, slave = pty.openpty()
    tty.setraw(master)
    tty.setraw(slave)
    os.set_blocking(master, False)
    stop = threading.Event()
    sent = [0]
    path = tmp_path/'serial.log'
    executable = os.environ['RUNTIME_EXECUTABLE']

    def exchange():
        while not stop.is_set():
            try:
                os.write(master, heartbeat(sent[0]))
                sent[0] += 1
            except BlockingIOError:
                pass
            try:
                os.read(master, 65535)
            except BlockingIOError:
                pass
            stop.wait(.001)

    worker = threading.Thread(target=exchange)
    with path.open('w') as output:
        process = subprocess.Popen([executable, '--ros-args', '-p',
            f'fcu_url:={os.ttyname(slave)}:921600'],
            env=dict(os.environ, ROS_DOMAIN_ID='203', ROS_LOCALHOST_ONLY='1'),
            stdout=output, stderr=subprocess.STDOUT)
        worker.start()
        try:
            deadline = time.monotonic()+15
            while 'MAVROS UAS via' not in path.read_text():
                assert process.poll() is None, path.read_text()
                assert time.monotonic() < deadline, path.read_text()
                time.sleep(.02)
            maps = Path(f'/proc/{process.pid}/maps').read_text()
            assert '/opt/ros/humble/lib/libmavconn.so' not in maps
            assert '/runtime/libmavconn.so' in maps
            time.sleep(.15+repeat*.05)
            if disconnect:
                stop.set()
                worker.join(timeout=2)
                assert not worker.is_alive()
                os.close(master)
                master = None
                deadline = time.monotonic()+3
                while 'serial0: receive:' not in path.read_text():
                    assert process.poll() is None, path.read_text()
                    assert time.monotonic() < deadline, path.read_text()
                    time.sleep(.02)
            process.send_signal(stop_signal)
            assert process.wait(timeout=10) == 0, path.read_text()
            assert 'I/O joined, Router/UAS destroyed' in path.read_text()
            assert sent[0] > 20
        finally:
            stop.set()
            worker.join(timeout=2)
            if process.poll() is None:
                process.kill()
                process.wait(timeout=3)
            if master is not None:
                os.close(master)
            os.close(slave)


@pytest.mark.parametrize('stop_signal', [signal.SIGINT, signal.SIGTERM])
@pytest.mark.parametrize('marker', ['Starting mavros uas node', 'Plugin gps_input created'])
def test_stop_during_plugin_construction(tmp_path, stop_signal, marker):
    """Interrupt native startup before plugin construction has completed, without context loss."""
    path = tmp_path / 'startup-stop.log'
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
        reserve.bind(('127.0.0.1', 0))
        local = reserve.getsockname()[1]
    with path.open('w') as output:
        process = subprocess.Popen([
            os.environ['RUNTIME_EXECUTABLE'], '--ros-args', '-p',
            f'fcu_url:=udp://127.0.0.1:{local}@127.0.0.1:14999'],
            env=dict(os.environ, ROS_DOMAIN_ID='204', ROS_LOCALHOST_ONLY='1'),
            stdout=output, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 15
            while marker not in path.read_text():
                assert process.poll() is None, path.read_text()
                assert time.monotonic() < deadline, path.read_text()
                time.sleep(.001)
            before = path.read_text()
            assert 'MAVROS UAS via' not in before, before
            process.send_signal(stop_signal)
            assert process.wait(timeout=10) == 0, path.read_text()
            assert 'I/O joined, Router/UAS destroyed' in path.read_text()
            assert 'context is invalid' not in path.read_text()
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=10)

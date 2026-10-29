#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Opt-in semantic-only stack capture; forward shutdown to the exact owned inferior."""

import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time


def identity(pid, executable):
    """Reject reused PIDs and any process not executing the requested semantic binary."""
    base = Path('/proc') / str(pid)
    try:
        if base.joinpath('exe').resolve(strict=True) != executable:
            return None
        ticks = base.joinpath('stat').read_text().rsplit(')', 1)[1].split()[19]
        return pid, ticks
    except (FileNotFoundError, ProcessLookupError):
        return None


def signal_owned(owned, executable, stop_signal):
    """Bind the signal to a pidfd so PID reuse after validation cannot change its target."""
    descriptor = os.pidfd_open(owned[0])
    try:
        if identity(owned[0], executable) != owned:
            raise RuntimeError('inferior changed before shutdown; refuse signaling')
        signal.pidfd_send_signal(descriptor, stop_signal)
    finally:
        os.close(descriptor)


def cleanup_debugger(debugger, inferior, executable):
    """Reap the owned debugger even if exact-inferior cleanup fails."""
    if debugger.poll() is not None:
        return
    print('semantic capture forced cleanup: FAIL', flush=True)
    try:
        if inferior is not None and identity(inferior[0], executable) == inferior:
            try:
                signal_owned(inferior, executable, signal.SIGKILL)
            except ProcessLookupError:
                pass  # Inferior exited after identity validation.
    finally:
        try:
            os.killpg(debugger.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass  # The newly-created debugger session has already exited.
        debugger.wait(timeout=3)


def main():
    """Start one debugger, never attach to or signal unrelated processes."""
    if len(sys.argv) < 2:
        raise ValueError('semantic executable required')
    executable = Path(sys.argv[1]).resolve(strict=True)
    if executable.name != 'semantic_map_node':
        raise ValueError('only semantic_map_node is supported')
    debugger_path = shutil.which('gdb')
    if not debugger_path:
        raise RuntimeError('requested semantic capture requires gdb; no silent fallback')
    pending = []
    for stop_signal in (signal.SIGINT, signal.SIGTERM):
        signal.signal(stop_signal, lambda sig, frame: pending.append(sig))
    command = [debugger_path, '--batch', '--return-child-result', '-ex',
               'set pagination off', '-ex', 'set debuginfod enabled off', '-ex',
               'handle SIGINT nostop noprint pass', '-ex',
               'handle SIGTERM nostop noprint pass', '-ex', 'run', '-ex',
               'thread apply all bt 24', '--args', str(executable), *sys.argv[2:]]
    # GDB quotes --args for its default startup shell. Disabling that shell
    # leaves escape characters in paths containing spaces and splits ROS args.
    # Pass an argv list to GDB; never assemble an interpolated shell command.
    debugger = subprocess.Popen(command, start_new_session=True)
    began = time.monotonic()
    inferior = None
    sent = None
    try:
        while debugger.poll() is None:
            if inferior is None:
                children = Path(f'/proc/{debugger.pid}/task/{debugger.pid}/children')
                try:
                    for child in children.read_text().split():
                        inferior = identity(int(child), executable)
                        if inferior is not None:
                            print(f'semantic capture inferior: pid={inferior[0]} '
                                  f'start_ticks={inferior[1]} executable={executable}', flush=True)
                            break
                except FileNotFoundError:
                    pass
            if pending and inferior is not None and sent is None:
                signal_owned(inferior, executable, pending[0])
                sent = time.monotonic()
                print(f'semantic capture forwarded: signal={pending[0]} '
                      f'pid={inferior[0]}', flush=True)
            if inferior is None and time.monotonic() - began > 10:
                raise TimeoutError('semantic debugger inferior never started')
            if sent is not None and time.monotonic() - sent > 20:
                raise TimeoutError('semantic debugger did not finish after shutdown')
            time.sleep(.02)
        return debugger.returncode
    finally:
        cleanup_debugger(debugger, inferior, executable)


if __name__ == '__main__':
    raise SystemExit(main())

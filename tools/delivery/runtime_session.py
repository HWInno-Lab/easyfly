#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Keep a PID namespace alive until its foreground and detached descendants finish."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    """Reap adopted children without canceling detached managed application sessions."""
    if os.getpid() != 1 or len(sys.argv) < 2:
        raise SystemExit('runtime supervisor must be PID 1 with an explicit command')
    child = subprocess.Popen(sys.argv[1:], start_new_session=True)
    initial_result = None

    def forward(number, _frame):
        if initial_result is None:
            try:
                os.killpg(child.pid, number)
            except ProcessLookupError:
                pass
        else:
            print('EasyFly background processes remain; reopen this installation to stop them normally.',
                  file=sys.stderr, flush=True)

    signal.signal(signal.SIGINT, forward)
    signal.signal(signal.SIGTERM, forward)
    while True:
        while True:
            try:
                pid, status = os.waitpid(-1, os.WNOHANG)
            except ChildProcessError:
                break
            if pid == 0:
                break
            if pid == child.pid:
                initial_result = os.waitstatus_to_exitcode(status)
                child.returncode = initial_result
        # Joined commands may have a parent outside this PID namespace. They also
        # keep this session alive, even when they are not direct adopted children.
        others = False
        for entry in Path('/proc').iterdir():
            if not entry.name.isdecimal() or entry.name == '1':
                continue
            try:
                status = (entry / 'stat').read_text().rsplit(')', 1)[1].split()[0]
            except (FileNotFoundError, ProcessLookupError):
                continue
            if status != 'Z':
                others = True
                break
        if initial_result is not None and not others:
            return max(0, initial_result) if initial_result >= 0 else 128 - initial_result
        time.sleep(0.05)


if __name__ == '__main__':
    raise SystemExit(main())

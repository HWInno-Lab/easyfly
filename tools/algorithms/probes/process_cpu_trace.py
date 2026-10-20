#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Read only owned process CPU/RSS/thread counters; never enable kernel tracing."""

import os
from pathlib import Path
import time


def snapshot(processes):
    start = time.monotonic_ns()
    rows = []
    for pid, _ in processes:
        try:
            proc = Path('/proc') / str(pid)
            data = (proc / 'stat').read_text()
            end_name = data.rindex(')')
            fields = data[end_name+2:].split()
            rows.append(dict(pid=pid, name=data[data.index('(')+1:end_name],
                             executable=str((proc / 'exe').resolve(strict=True)),
                             state=fields[0], start_ticks=int(fields[19]),
                             user_ticks=int(fields[11]), system_ticks=int(fields[12]),
                             threads=int(fields[17]), nice=int(fields[16]), rss_pages=int(fields[21]),
                             allowed_cpus=sorted(os.sched_getaffinity(pid))))
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    return dict(begin_ns=start, end_ns=time.monotonic_ns(), processes=rows)

#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Bounded, process-local CPU contention for isolated simulation experiments."""

import argparse
import json
import signal
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, required=True)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 300:
        parser.error('bounded duration required')
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    begin = time.monotonic()
    cpu_begin = time.process_time()
    cycles = 0
    while not stopped and time.monotonic() - begin < args.seconds:
        cycle = time.monotonic()
        while not stopped and time.monotonic() - cycle < .075:
            cycles += 1
        time.sleep(max(0, .1 - (time.monotonic() - cycle)))
    print(json.dumps(dict(wall_s=time.monotonic()-begin,
                          cpu_s=time.process_time()-cpu_begin, iterations=cycles)))


if __name__ == '__main__':
    main()

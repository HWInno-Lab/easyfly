#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Summarize owned process counters and measured source/wall rate, without a profiler."""
import argparse
import json
from pathlib import Path

import numpy as np


def summarize(case):
    data = json.loads((case/'process-budget.json').read_text())
    samples = data['samples']
    series = {}
    for sample in samples:
        for process in sample['processes']:
            key = (process['pid'], process['start_ticks'])
            series.setdefault(key, []).append((sample['begin_ns']/1e9, process))
    processes = []
    for (pid, start), rows in series.items():
        cores = []
        for (ta, a), (tb, b) in zip(rows, rows[1:]):
            ticks = b['user_ticks']+b['system_ticks']-a['user_ticks']-a['system_ticks']
            if tb > ta and ticks >= 0:
                cores.append(ticks/data['clock_ticks_per_second']/(tb-ta))
        rss = np.array([r['rss_pages']*data['page_size']/2**20 for _, r in rows])
        window = max(1, min(30, len(rss)//3))
        processes.append(dict(
            pid=pid, start_ticks=start, executable=rows[0][1]['executable'],
            name=rows[0][1]['name'], samples=len(rows),
            core_mean=float(np.mean(cores)) if cores else None,
            core_p95=float(np.percentile(cores, 95)) if cores else None,
            rss_peak_mib=float(max(rss)),
            rss_tail_median_mib=float(np.median(rss[-window:])),
            rss_first_median_mib=float(np.median(rss[:window])),
            rss_tail_minus_first_mib=float(np.median(rss[-window:])-np.median(rss[:window])),
            thread_peak=max(r['threads'] for _, r in rows),
            cpu_affinity=rows[0][1]['allowed_cpus'],
            nice_values=sorted({r['nice'] for _, r in rows})))
    report = dict(
        wall_observation_seconds=(samples[-1]['begin_ns']-samples[0]['begin_ns'])/1e9,
        source_observation_seconds=(samples[-1]['clock_ns']-samples[0]['clock_ns'])/1e9,
        source_per_wall=(samples[-1]['clock_ns']-samples[0]['clock_ns']) /
        (samples[-1]['begin_ns']-samples[0]['begin_ns']),
        peak_sum_process_rss_mib=max(sum(
            p['rss_pages']*data['page_size']/2**20 for p in s['processes']) for s in samples),
        rss_caveat='Sum of process RSS counts shared mappings repeatedly; not physical RAM usage.',
        window_definition='first/tail median of up to 30 samples; includes all retained processes',
        processes=processes)
    (case/'resources.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', required=True, type=Path)
    args = parser.parse_args()
    report = summarize(args.case)
    print(json.dumps({k: v for k, v in report.items() if k != 'processes'}, indent=2))


if __name__ == '__main__':
    main()

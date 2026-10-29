#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Compare the installed existing 2D A* on exactly the recorded GCOPTER requests."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def compare(cases, executable):
    records = []
    for case in cases:
        plans = [
            json.loads(line) for line in (case / "polynomials.jsonl").read_text().splitlines()
        ]
        for index, request in enumerate(sorted(case.glob("gcopter-*/job-*.request"))):
            run = subprocess.run(
                [str(executable.resolve()), str(request.resolve())],
                capture_output=True,
                text=True,
                check=True,
                timeout=5,
            )
            record = json.loads(run.stdout)
            record.update(
                case=case.name,
                request=str(request),
                request_sha256=hashlib.sha256(request.read_bytes()).hexdigest(),
                unknown_blocked=True,
                speed_limit=0.6,
                acceleration_limit=2.0,
                jerk_limit=8.0,
                astar_adapter="fixed start-altitude slice; linear endpoint height interpolation",
                astar_timing="quintic Hermite rest joins; exact measured initial velocity",
            )
            if index < len(plans):
                plan = plans[index]
                record["gcopter"] = {
                    k: plan[k] for k in ["max_speed", "max_acceleration", "max_jerk"]
                }
                record["gcopter"]["duration_s"] = sum(plan["durations"])
            records.append(record)
    return records


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--consumer", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("cases", nargs="+", type=Path)
    args = parser.parse_args()
    with args.output.open("x") as stream:
        stream.write(json.dumps(compare(args.cases, args.consumer), indent=2) + "\n")

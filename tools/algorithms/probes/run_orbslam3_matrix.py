#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""One managed simulation at a time; stop on the first failed runtime/evaluation."""
import argparse
import json
from pathlib import Path
import subprocess
import sys


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--root", type=Path, required=True)
    p.add_argument("--suffix", default="verified")
    p.add_argument(
        "--cases",
        nargs="+",
        choices=['camera-loss', 'depth-loss', 'map-loss', 'backend-exit', 'cancel',
                 'visual-loss', 'clock-reset', 'long-run', 'astar', 'ego'],
        default=[
            "camera-loss",
            "depth-loss",
            "map-loss",
            "backend-exit",
            "cancel",
            "visual-loss",
            "clock-reset",
            "long-run",
        ],
    )
    a = p.parse_args()
    root = a.root.resolve()
    records = []
    for case in a.cases:
        scenario = (
            "localization-loss" if case in ("camera-loss", "depth-loss") else case
        )
        if case in ("astar", "ego"):
            scenario = "mission"
        name = case + "-" + a.suffix
        command = [
            sys.executable,
            str(Path(__file__).with_name("orbslam3_simulation.py")),
            "--output-dir",
            str(root / name),
            "--px4-dir",
            "artifacts/scene-s4-20261017/px4-mag-lpf-v2/px4-input",
            "--truth-executable",
            "artifacts/new-algorithms/openvins-sim-acceptance-20261020/native-long-build/gazebo_pose_receipts",
            "--scenario",
            scenario,
            "--visual-feature-board",
            "--execute-simulation",
        ]
        if case in ("camera-loss", "depth-loss"):
            command += ["--fault-kind", case.split("-")[0]]
        if case in ("astar", "ego"):
            command += ["--strategy", case, "--managed-profile"]
        if case == "long-run":
            command += ["--hold-source-seconds", "120", "--managed-profile"]
        with (root / (name + "-driver.log")).open("x") as stream:
            result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT)
        if result.returncode:
            print(json.dumps(dict(case=case, passed=False, stage="runtime")), flush=True)
            return 1
        evaluation = subprocess.run(
            [
                sys.executable,
                str(Path(__file__).with_name("evaluate_orbslam3_simulation.py")),
                str(root / name),
            ],
            capture_output=True,
            text=True,
        )
        (root / (name + "-evaluation.log")).write_text(evaluation.stdout + evaluation.stderr)
        row = dict(case=case, passed=evaluation.returncode == 0)
        records.append(row)
        (root / ("matrix-progress-" + a.suffix + ".json")).write_text(
            json.dumps(records, indent=2) + "\n"
        )
        print(json.dumps(row), flush=True)
        if evaluation.returncode:
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Run explicit fresh GCOPTER cases sequentially; stop on the first failed case."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--px4-dir", type=Path, required=True)
    parser.add_argument("--truth-executable", type=Path, required=True)
    parser.add_argument("--cases", nargs="+", required=True, help="scenario:unique-directory-name")
    parser.add_argument("--execute-simulation", action="store_true", required=True)
    args = parser.parse_args()
    status = args.root / "remaining-matrix-status.json"
    if status.exists():
        parser.error("matrix status already exists; use a fresh matrix root")
    args.root.mkdir(exist_ok=True, parents=True)
    entries = []
    for selection in args.cases:
        scenario, name = selection.split(":", 1)
        if Path(name).name != name or (args.root / name).exists():
            parser.error("each case requires a fresh simple directory name")
        entry = dict(scenario=scenario, case=name, running=True)
        entries.append(entry)
        status.write_text(json.dumps(entries, indent=2) + "\n")
        command = [
            sys.executable,
            str(Path(__file__).with_name("gcopter_simulation.py")),
            "--px4-dir",
            str(args.px4_dir),
            "--output-dir",
            str(args.root / name),
            "--truth-executable",
            str(args.truth_executable),
            "--scenario",
            scenario,
            "--execute-simulation",
        ]
        if scenario in ["channel", "map-change"]:
            command.append("--managed-profile")
        with (args.root / (name + ".runner.log")).open("x") as log:
            run = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, cwd=ROOT)
        entry.update(running=False, simulation_returncode=run.returncode)
        if run.returncode == 0:
            with (args.root / (name + ".evaluation.log")).open("x") as log:
                checked = subprocess.run(
                    [
                        sys.executable,
                        str(Path(__file__).with_name("evaluate_gcopter.py")),
                        str(args.root / name),
                    ],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                )
            entry["evaluation_returncode"] = checked.returncode
        entry["passed"] = run.returncode == 0 and entry.get("evaluation_returncode") == 0
        status.write_text(json.dumps(entries, indent=2) + "\n")
        if not entry["passed"]:
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

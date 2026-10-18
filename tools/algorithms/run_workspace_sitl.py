#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""Explicit serial ALG-9 SITL matrix using one verified installation.

Sends simulation flight goals. Never run by CTest. Existing entry points retain
their vehicle reservation, input guards, draining and shutdown policy.
"""
import argparse
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import time

from verify_api_install import clean_environment

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--installation", type=Path, required=True)
    parser.add_argument("--px4-dir", type=Path, required=True)
    parser.add_argument("--hrt-compat", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--execute-sitl", action="store_true", required=True)
    args = parser.parse_args()
    prefix, out = args.installation.resolve(strict=True), args.output.resolve()
    if out.exists():
        parser.error("output must be new; do not replace previous evidence")
    nav = (
        prefix
        / "easyfly_sim_launcher/lib/easyfly_sim_launcher/navigation_sitl"
    )
    d5 = prefix / "easyfly_sim_launcher/lib/easyfly_sim_launcher/d5_simulation"
    for entry in (nav, d5):
        if not entry.is_file():
            parser.error("verified simulation entry is missing: " + str(entry))
    out.mkdir(parents=True)
    packages = sorted(p for p in prefix.iterdir() if (p / "share").is_dir())
    roots = ":".join(str(p) for p in packages)
    libs = ":".join(str(p / "lib") for p in packages)
    python = ":".join(
        str(p)
        for package in packages
        for pattern in (
            "local/lib/python*/dist-packages",
            "lib/python*/site-packages",
        )
        for p in package.glob(pattern)
    )
    env = clean_environment()
    env.update(
        ROS_LOCALHOST_ONLY="1",
        EASYFLY_PX4_HRT_COMPAT_DIR=str(args.hrt_compat.resolve(strict=True)),
    )
    tasks = [
        (f"navigation-{strategy}-happy", strategy, "happy")
        for strategy in ("astar", "ego", "combined")
    ]
    tasks += [
        (f"navigation-combined-{scenario}", "combined", scenario)
        for scenario in ("cancel", "map-loss", "mission", "search")
    ]
    tasks += [
        ("follow-perception-cancel", "astar", "follow"),
        ("precision-mission", "precision", "mission"),
    ]
    record = {
        "schema_version": 1,
        "installation": str(prefix),
        "cases": [],
        "script_sha256": hashlib.sha256(
            Path(__file__).read_bytes()
        ).hexdigest(),
        "scope": "Bounded local PX4/Gazebo; no hardware qualification.",
    }

    def save():
        (out / "summary.json").write_text(json.dumps(record, indent=2) + "\n")

    for index, (name, strategy, scenario) in enumerate(tasks):
        directory = out / name
        directory.mkdir()
        domain, port = 210 + index, 11510 + index
        if strategy == "precision":
            command = [
                "python3",
                str(d5),
                "--px4-dir",
                str(args.px4_dir),
                "--directory",
                str(directory),
                "--use-bundled-dds-profile",
                "--mission",
            ]
            # D5's engine explicitly selects these fixed isolated values.
            domain, port = 122, 11357
        else:
            entry = (
                ROOT / "tools/algorithms/follow_sitl.py"
                if scenario == "follow"
                else nav
            )
            command = [
                "python3",
                str(entry),
                "--px4-dir",
                str(args.px4_dir),
                "--output-dir",
                str(directory),
                "--domain",
                str(domain),
                "--gazebo-port",
                str(port),
            ]
            if scenario != "follow":
                command += ["--strategy", strategy, "--scenario", scenario]
        script = (
            "source /opt/ros/humble/setup.bash && "
            'export AMENT_PREFIX_PATH="$1:$AMENT_PREFIX_PATH" '
            'CMAKE_PREFIX_PATH="$1:$CMAKE_PREFIX_PATH" '
            'LD_LIBRARY_PATH="$2:$LD_LIBRARY_PATH" '
            'PYTHONPATH="$3:$PYTHONPATH" && shift 3 && exec "$@"'
        )
        actual = [
            "bash",
            "--noprofile",
            "--norc",
            "-c",
            script,
            "bash",
            roots,
            libs,
            python,
            *command,
        ]
        process_env = dict(
            env,
            ROS_DOMAIN_ID=str(domain),
            GAZEBO_MASTER_URI=f"http://127.0.0.1:{port}",
        )
        print(name, flush=True)
        began, timed_out = time.monotonic(), False
        with (out / (name + ".log")).open("w") as log:
            process = subprocess.Popen(
                actual,
                cwd=ROOT,
                env=process_env,
                stdout=log,
                stderr=subprocess.STDOUT,
            )
            try:
                code = process.wait(timeout=600)
            except (subprocess.TimeoutExpired, KeyboardInterrupt):
                timed_out = True
                # Signal only the supervisor we created; its own finally block
                # drains goals and stops its identified child sessions.
                process.send_signal(signal.SIGINT)
                try:
                    code = process.wait(timeout=60)
                except subprocess.TimeoutExpired:
                    record["supervisor_failed_to_drain"] = process.pid
                    save()
                    raise RuntimeError(
                        "SITL supervisor did not drain; matrix stopped"
                    )
        result = (
            json.loads((directory / "result.json").read_text())
            if (directory / "result.json").exists()
            else None
        )
        record["cases"].append(
            {
                "name": name,
                "argv": actual,
                "returncode": code,
                "elapsed_seconds": round(time.monotonic() - began, 3),
                "domain": domain,
                "gazebo_port": port,
                "timed_out": timed_out,
                "result": str(directory / "result.json"),
                "passed": code == 0
                and not timed_out
                and result is not None
                and result.get("passed") is True,
            }
        )
        save()
        # Stop on unknown/forced cleanup; never retry the original failure.
        if (
            timed_out
            or result is None
            or result
            and (result.get("leftovers") or result.get("forced"))
        ):
            record["stopped_after_unclean_case"] = name
            break
    record["all_cases_executed"] = len(record["cases"]) == len(tasks)
    record["passed"] = record["all_cases_executed"] and all(
        c["passed"] for c in record["cases"]
    )
    save()
    return 0 if record["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

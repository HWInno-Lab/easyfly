#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""Build pinned OpenVINS against ROS Humble and an optional private Ceres prefix.

Only builds the supplied upstream checkout. No source import, package install,
device access or flight actions. The runtime probe uses an isolated ROS domain.
"""

import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from verify_api_install import clean_environment


ROOT = Path(__file__).resolve().parents[2]
REVISION = "69488123ed9362dd44b6f28e7f4680abbff1442b"


def fingerprint(source):
    files = subprocess.check_output(
        ["git", "-C", str(source), "ls-files", "-z"]
    ).decode().split("\0")
    return {
        name: hashlib.sha256((source / name).read_bytes()).hexdigest()
        for name in files if name
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--dependency-prefix", type=Path)
    parser.add_argument("--patch", type=Path, help="Apply a reviewed patch in an isolated clone")
    parser.add_argument("--resume", action="store_true", help="Preserve earlier attempts")
    args = parser.parse_args()
    source = args.source.resolve(strict=True)
    out = args.output.resolve()
    revision = subprocess.check_output(
        ["git", "-C", str(source), "rev-parse", "HEAD"], text=True
    ).strip()
    if revision != REVISION:
        parser.error("unexpected upstream revision: " + revision)
    if subprocess.check_output(
        ["git", "-C", str(source), "status", "--porcelain"], text=True
    ).strip():
        parser.error("upstream checkout has local changes")
    if out.exists() and not args.resume:
        parser.error("output already exists; use a new directory or --resume")
    if args.resume and not (out / "summary.json").is_file():
        parser.error("resume requires an existing summary")
    out.mkdir(parents=True, exist_ok=args.resume)
    (out / "COLCON_IGNORE").touch()
    base_source = source
    patch_sha256 = None
    if args.patch:
        patch = args.patch.resolve(strict=True)
        patch_sha256 = hashlib.sha256(patch.read_bytes()).hexdigest()
        source = out / "patched-source"
        if not args.resume:
            subprocess.run(["git", "clone", "--quiet", "--shared",
                            str(base_source), str(source)], check=True)
            subprocess.run(["git", "-C", str(source), "apply", "--check", str(patch)],
                           check=True)
            subprocess.run(["git", "-C", str(source), "apply", str(patch)], check=True)
        actual_patch = subprocess.check_output(["git", "-C", str(source), "diff", "--binary"])
        if actual_patch != patch.read_bytes():
            parser.error("isolated source changes do not match the supplied patch")
    hashes = fingerprint(source)
    env = clean_environment()
    env.update(ROS_LOCALHOST_ONLY="1", ROS_DOMAIN_ID="231",
               ROS_LOG_DIR=str(out / "ros-log"))
    prefix = args.dependency_prefix.resolve(strict=True) if args.dependency_prefix else None
    if prefix:
        env["LD_LIBRARY_PATH"] = os.pathsep.join(
            str(prefix / p) for p in ("lib/x86_64-linux-gnu", "lib")
        )
    if args.resume:
        record = json.loads((out / "summary.json").read_text())
        if record["source_files"] != hashes or record["dependency_prefix"] != str(prefix):
            parser.error("source/dependency identity changed since previous attempt")
        if record.get("patch_sha256") != patch_sha256:
            parser.error("patch changed since previous attempt")
    else:
        record = dict(
            source=str(source), upstream="https://github.com/rpng/open_vins.git",
            revision=revision, dependency_prefix=str(prefix), source_files=hashes,
            base_source=str(base_source), patch_sha256=patch_sha256,
            started_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
            scope=("Isolated ROS2 build and executable checks; "
                   "not camera VIO or flight qualification."),
            commands=[], complete=False,
        )

    def save():
        (out / "summary.json").write_text(json.dumps(record, indent=2) + "\n")

    def run(name, command, installed=False):
        script = 'source /opt/ros/humble/setup.bash && '
        paths = []
        if installed:
            script += 'source "$1/local_setup.bash" && shift && '
            paths.append(str(out / "install"))
        actual = ["bash", "--noprofile", "--norc", "-c", script + 'exec "$@"',
                  "bash", *paths, *map(str, command)]
        attempt = sum(c["name"] == name for c in record["commands"]) + 1
        log = out / f"{name}-{attempt}.log"
        started = time.monotonic()
        print("START", name, attempt, flush=True)
        with log.open("w") as stream:
            result = subprocess.run(actual, cwd=ROOT, env=env,
                                    stdout=stream, stderr=subprocess.STDOUT)
        record["commands"].append(dict(
            name=name, attempt=attempt, argv=actual, returncode=result.returncode,
            elapsed_seconds=round(time.monotonic() - started, 3), log=str(log),
            log_sha256=hashlib.sha256(log.read_bytes()).hexdigest(),
        ))
        save()
        print("END", name, result.returncode, flush=True)
        if result.returncode:
            raise RuntimeError(f"{name} failed; see {log}")

    save()
    flags = ["-DCMAKE_BUILD_TYPE=RelWithDebInfo",
             "-DCMAKE_CXX_FLAGS_RELWITHDEBINFO=-O2 -g",
             "-DDISABLE_MATPLOTLIB=ON", "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"]
    if prefix:
        flags.append("-DCMAKE_PREFIX_PATH=" + str(prefix))
    run("build", ["colcon", "--log-base", out / "colcon-log", "build",
                  "--base-paths", source, "--build-base", out / "build",
                  "--install-base", out / "install", "--executor", "sequential",
                  "--packages-select", "ov_core", "ov_init", "ov_msckf",
                  "--cmake-args", *flags])
    executable = out / "install/ov_msckf/lib/ov_msckf/run_subscribe_msckf"
    run("linked-libraries", ["ldd", executable], installed=True)
    if "not found" in Path(record["commands"][-1]["log"]).read_text():
        raise RuntimeError("installed executable has unresolved libraries")
    runtime_attempt = sum(c["name"] == "subscriber" for c in record["commands"]) + 1
    run("subscriber", [sys.executable, ROOT / "tools/algorithms/probes/openvins_runtime_probe.py",
                       "--installation", out / "install",
                       "--output", out / f"runtime-{runtime_attempt}"], True)
    after = fingerprint(source)
    record["changed_inputs"] = [p for p, value in hashes.items() if after.get(p) != value]
    if record["changed_inputs"]:
        save()
        raise RuntimeError("upstream inputs changed during verification")
    record["complete"] = True
    record["finished_at"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
    save()
    print("PASS isolated OpenVINS build and subscriber lifecycle", flush=True)


if __name__ == "__main__":
    main()

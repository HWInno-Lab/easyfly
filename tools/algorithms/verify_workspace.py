#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""ALG-9 clean workspace matrix, without an existing EasyFly overlay.

Runs builds and software checks; SITL is a separate explicit step.
Preserves logs, failures and source fingerprints. No system installation.
"""
import argparse
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

from verify_api_install import clean_environment

ROOT = Path(__file__).resolve().parents[2]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mode", choices=("symlink", "copy"), required=True)
    parser.add_argument(
        "--px4-gazebo-build",
        type=Path,
        required=True,
        help="Reviewed PX4 Classic protobuf build; no PX4 rebuild",
    )
    parser.add_argument(
        "--resume-build",
        action="store_true",
        help="Retry a failed build in its initially clean tree; keep old logs",
    )
    parser.add_argument(
        "--build-only", action="store_true", help="Defer software checks"
    )
    parser.add_argument(
        "--tests-only", action="store_true", help="Check a completed build"
    )
    parser.add_argument(
        "--continue-tests",
        action="store_true",
        help="Resume recorded checks after an interrupted scheduler",
    )
    parser.add_argument(
        "--fastdds-archive", type=Path, help="Optional pinned local archive"
    )
    parser.add_argument(
        "--skip-source-lint", action="store_true",
        help="Second mode: source lint is recorded in the first mode",
    )
    args = parser.parse_args()
    px4_gazebo = args.px4_gazebo_build.resolve()
    if not all(
        (px4_gazebo / n).is_file() for n in ("Imu.pb.h", "libsensor_msgs.so")
    ):
        parser.error("PX4 Classic Imu.pb.h and libsensor_msgs.so are required")
    out = args.output.resolve()
    reuse = args.resume_build or args.tests_only or args.continue_tests
    if (
        sum((args.resume_build, args.tests_only, args.continue_tests)) > 1
        or args.build_only
        and (args.tests_only or args.continue_tests)
    ):
        parser.error("incompatible execution options")
    if out.exists() and not reuse:
        parser.error("output must be new; preserve previous evidence")
    out.mkdir(parents=True, exist_ok=reuse)
    lock = (out / ".verification.lock").open("a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error("another verifier owns this matrix directory")
    env = clean_environment()
    env.update(
        ROS_DOMAIN_ID="226",
        ROS_LOCALHOST_ONLY="1",
        QT_QPA_PLATFORM="offscreen",
        EASYFLY_PX4_HRT_COMPAT_DIR=str(ROOT / "build/easyfly_px4_hrt_compat"),
    )
    # RealSense's private installation must not update the system linker cache.
    private = out / "private-bin"
    private.mkdir(exist_ok=reuse)
    stub = private / "ldconfig"
    stub.write_text(
        '#!/bin/sh\necho "ALG-9: private install; system ldconfig skipped"\n'
    )
    stub.chmod(0o755)
    env["PATH"] = str(private) + os.pathsep + env["PATH"]
    tracked = (
        subprocess.check_output(["git", "ls-files", "-z"], cwd=ROOT)
        .decode()
        .split("\0")
    )
    protected = {n: sha(ROOT / n) for n in tracked if (ROOT / n).is_file()}
    record = {
        "schema_version": 1,
        "mode": args.mode,
        "started_utc": datetime.datetime.now(
            datetime.timezone.utc
        ).isoformat(),
        "source_commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip(),
        "tracked_inputs": protected,
        "commands": [],
        "tests": [],
    }
    attempt = ""
    if reuse:
        record = json.loads((out / "summary.json").read_text())
        if (
            record["mode"] != args.mode
            or record["tests"]
            and not args.continue_tests
        ):
            parser.error(
                "reuse requires the same mode and no checks already run"
            )
        if args.resume_build and record.get("build_passed"):
            parser.error("resume-build requires a failed build")
        if (args.tests_only or args.continue_tests) and not record.get(
            "build_passed"
        ):
            parser.error("tests-only requires a completed build")
        if record["tracked_inputs"] != protected:
            parser.error("tracked inputs changed; use a new clean matrix")
        attempt = (
            "-continue" + str(len(record["commands"]))
            if args.continue_tests
            else "-tests"
            if args.tests_only
            else "-retry"
            + str(
                sum(c["name"].startswith("build") for c in record["commands"])
            )
        )

    def save():
        (out / "summary.json").write_text(json.dumps(record, indent=2) + "\n")

    def run(name, command, overlay=False):
        if (
            name in ("build", "discovery", "runtime-environment")
            or args.continue_tests
        ):
            name += attempt
        setup = "source /opt/ros/humble/setup.bash && "
        if overlay:
            setup += 'source "$1/local_setup.bash" && shift && '
        actual = [
            "bash",
            "--noprofile",
            "--norc",
            "-c",
            setup + 'exec "$@"',
            "bash",
        ]
        if overlay:
            actual.append(str(out / "install"))
        actual += [str(p) for p in command]
        print(name, flush=True)
        started = time.monotonic()
        with (out / (name + ".log")).open("w") as stream:
            code = subprocess.call(
                actual,
                cwd=ROOT,
                env=env,
                stdout=stream,
                stderr=subprocess.STDOUT,
            )
        record["commands"].append(
            {
                "name": name,
                "argv": actual,
                "returncode": code,
                "elapsed_seconds": round(time.monotonic() - started, 3),
                "log_sha256": sha(out / (name + ".log")),
            }
        )
        save()
        return code

    run("discovery", ["colcon", "list", "--base-paths", "src"])
    packages = [
        line.split()[0]
        for line in (out / ("discovery" + attempt + ".log"))
        .read_text()
        .splitlines()
        if line.strip()
    ]
    assert len(packages) == len(set(packages)) == 91
    flags = [
        "-DBUILD_TESTING=ON",
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
        "-DEASYFLY_PX4_GAZEBO_BUILD=" + str(px4_gazebo),
        "-DCMAKE_INSTALL_LIBDIR=lib",
        "-DFASTLIO_OUTPUT_ROOT=" + str(out / "fastlio-runtime"),
        "-DCMAKE_PROJECT_realsense2_INCLUDE="
        + str(ROOT / "tools/algorithms/sdk_offline.cmake"),
        "-DALG83_JSON_SOURCE="
        + str(ROOT / "artifacts/algorithms/alg8-3-20261018/json-source"),
        "-DBUILD_EXAMPLES=OFF",
        "-DBUILD_GRAPHICAL_EXAMPLES=OFF",
        "-DBUILD_TOOLS=OFF",
        "-DBUILD_GLSL_EXTENSIONS=OFF",
        "-DBUILD_UNIT_TESTS=OFF",
        "-DIMPORT_DEPTH_CAM_FW=OFF",
        "-DCHECK_FOR_UPDATES=OFF",
        "-DBUILD_WITH_DDS=OFF",
        "-DENABLE_CCACHE=OFF",
    ]
    if args.fastdds_archive:
        flags.append(
            "-DEASYFLY_FASTDDS_ARCHIVE="
            + str(args.fastdds_archive.resolve(strict=True))
        )
    command = [
        "colcon",
        "--log-base",
        out / "log",
        "build",
        "--base-paths",
        "src",
        "--build-base",
        out / "build",
        "--install-base",
        out / "install",
        "--parallel-workers",
        "2",
        "--continue-on-error",
    ]
    if args.mode == "symlink":
        command.append("--symlink-install")
    if not (args.tests_only or args.continue_tests):
        code = run("build", command + ["--cmake-args", *flags])
        record["build_passed"] = code == 0
    if record["build_passed"] and not args.build_only:
        selection = []
        if args.skip_source_lint:
            pattern = ("^(copyright|cpplint|pep257|uncrustify|flake8|xmllint|"
                       "lint_cmake|cppcheck)$")
            selection = ["-E", pattern]
            record["source_lint_excluded_in_this_mode"] = pattern
        run(
            "runtime-environment",
            [
                "python3",
                "-c",
                "import os,json; print(json.dumps({k:v "
                "for k,v in os.environ.items() "
                'if k.endswith("PATH") or '
                'k.startswith(("ROS_","RMW_"))},indent=2))',
            ],
            True,
        )
        for package in packages:
            if package == "realsense2_camera":
                record["excluded_upstream_suites"] = {
                    package: "Upstream device/rosbag suite downloads datasets "
                    "and uses name-based camera cleanup. SDK geometry, "
                    "messages and installed component "
                    "loading are covered separately without devices."
                }
                save()
                continue
            if package in {p["package"] for p in record["tests"]}:
                continue
            build = out / "build" / package
            if not (build / "CTestTestfile.cmake").exists():
                continue
            junit = out / (
                package + (attempt if args.continue_tests else "") + ".xml"
            )
            code = run(
                "test-" + package,
                [
                    "ctest",
                    "--test-dir",
                    build,
                    "-j1",
                    "--output-on-failure",
                    "--output-junit",
                    junit,
                    *selection,
                ],
                True,
            )
            tests = []
            if junit.exists():
                for case in ET.parse(junit).getroot().iter("testcase"):
                    tests.append(
                        {
                            "name": case.get("name"),
                            "time": case.get("time"),
                            "status": "failed"
                            if case.find("failure") is not None
                            else "skipped"
                            if case.find("skipped") is not None
                            else "passed",
                        }
                    )
            record["tests"].append(
                {"package": package, "returncode": code, "cases": tests}
            )
            save()
    record["source_changes_during_run"] = [
        n
        for n, digest in protected.items()
        if not (ROOT / n).is_file() or sha(ROOT / n) != digest
    ]
    record["finished_utc"] = datetime.datetime.now(
        datetime.timezone.utc
    ).isoformat()
    record["counts"] = {
        status: sum(
            c["status"] == status for p in record["tests"] for c in p["cases"]
        )
        for status in ("passed", "failed", "skipped")
    }
    record["software_checks_executed"] = bool(record["tests"])
    record["passed"] = (
        record["build_passed"]
        and record["software_checks_executed"]
        and not record["source_changes_during_run"]
        and all(p["returncode"] == 0 for p in record["tests"])
    )
    save()
    print(
        json.dumps(
            {k: record[k] for k in ("build_passed", "counts", "passed")}
        ),
        flush=True,
    )
    return (
        0
        if record["passed"] or args.build_only and record["build_passed"]
        else 1
    )


if __name__ == "__main__":
    raise SystemExit(main())

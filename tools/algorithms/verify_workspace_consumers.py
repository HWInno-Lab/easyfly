#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""Consume the ALG-9 installation using the existing independent fixtures.

For copy mode, move the entire installation before configuring any consumer.
No old EasyFly overlay, source includes, device nodes or flight goals.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from verify_api_install import clean_environment

ROOT = Path(__file__).resolve().parents[2]
FIXTURES = (
    "consumer",
    "tracking_consumer",
    "planning_consumer",
    "prediction_filter_consumer",
    "vision_consumer",
    "projection_consumer",
    "ground_consumer",
    "obstacle_consumer",
    "occupancy_consumer",
    "ego_consumer",
    "fastlio_consumer",
    "sdk_consumer",
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matrix", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--relocate", action="store_true")
    args = parser.parse_args()
    matrix, out = args.matrix.resolve(), args.output.resolve()
    if out.exists():
        parser.error("output must be new")
    summary = json.loads((matrix / "summary.json").read_text())
    if not summary["build_passed"]:
        parser.error("workspace build must pass first")
    out.mkdir(parents=True)
    prefix = matrix / "install"
    if args.relocate:
        moved = matrix / "relocated-install"
        if moved.exists():
            parser.error("relocation destination already exists")
        prefix.rename(moved)
        prefix = moved
    packages = sorted(
        p for p in prefix.iterdir() if p.is_dir() and (p / "share").exists()
    )
    env = clean_environment()
    env.update(ROS_DOMAIN_ID="227", ROS_LOCALHOST_ONLY="1")
    record = {
        "schema_version": 1,
        "installation": str(prefix),
        "relocated_before_configuration": args.relocate,
        "commands": [],
        "consumers": [],
    }

    def save():
        (out / "summary.json").write_text(json.dumps(record, indent=2) + "\n")

    def run(name, command, ros=False):
        process_env = dict(env)
        for var, suffix in [
            ("CMAKE_PREFIX_PATH", ""),
            ("AMENT_PREFIX_PATH", ""),
            ("LD_LIBRARY_PATH", "/lib"),
        ]:
            process_env[var] = ":".join(str(p) + suffix for p in packages)
        process_env["PYTHONPATH"] = ":".join(
            str(p)
            for package in packages
            for pattern in (
                "local/lib/python*/dist-packages",
                "lib/python*/site-packages",
            )
            for p in package.glob(pattern)
        )
        actual = [str(p) for p in command]
        if ros:
            # Prepend this matrix after initializing the system ROS setup.
            script = (
                "source /opt/ros/humble/setup.bash && "
                'export CMAKE_PREFIX_PATH="$1:$CMAKE_PREFIX_PATH" '
                'AMENT_PREFIX_PATH="$1:$AMENT_PREFIX_PATH" '
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
                process_env["CMAKE_PREFIX_PATH"],
                process_env["LD_LIBRARY_PATH"],
                process_env["PYTHONPATH"],
                *actual,
            ]
            process_env = dict(env)
        print(name, flush=True)
        with (out / (name + ".log")).open("w") as stream:
            code = subprocess.call(
                actual,
                cwd=out,
                env=process_env,
                stdout=stream,
                stderr=subprocess.STDOUT,
            )
        record["commands"].append(
            {
                "name": name,
                "argv": actual,
                "returncode": code,
                "log_sha256": hashlib.sha256(
                    (out / (name + ".log")).read_bytes()
                ).hexdigest(),
            }
        )
        save()
        return code == 0

    for fixture in FIXTURES:
        source, build = out / fixture / "source", out / fixture / "build"
        shutil.copytree(ROOT / "tools/algorithms" / fixture, source)
        for probe in (ROOT / "tools/algorithms/probes").glob("*.cpp"):
            shutil.copy2(probe, source / probe.name)
        ros = fixture in ("ego_consumer", "fastlio_consumer", "sdk_consumer")
        flags = ["-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"]
        if fixture == "sdk_consumer":
            flags += [
                "-DBUILD_DRIVER_CONSUMER=ON",
                "-DLIVOX_LIBRARY="
                + str(prefix / "livox_sdk2/lib/liblivox_lidar_sdk_shared.so"),
                "-DLIVOX_INCLUDE=" + str(prefix / "livox_sdk2/include"),
            ]
        passed = run(
            fixture + "-configure",
            ["cmake", "-S", source, "-B", build, *flags],
            ros,
        )
        if passed:
            passed = run(
                fixture + "-build",
                ["cmake", "--build", build, "--parallel", "2"],
                ros,
            )
        source_free = False
        if passed:
            source_free = (
                str(ROOT / "src")
                not in (build / "compile_commands.json").read_text()
            )
            if fixture == "sdk_consumer":
                passed = run(fixture + "-run", [build / "sdk_consumer"], ros)
                passed &= run(
                    fixture + "-drivers",
                    [
                        build / "driver_consumer",
                        prefix
                        / "livox_ros_driver2/lib/liblivox_ros_driver2.so",
                        prefix
                        / "realsense2_camera/lib/librealsense2_camera.so",
                    ],
                    ros,
                )
            else:
                passed = run(
                    fixture + "-test",
                    [
                        "ctest",
                        "--test-dir",
                        build,
                        "--output-on-failure",
                        "--output-junit",
                        out / (fixture + ".xml"),
                    ],
                    ros,
                )
            if fixture == "projection_consumer":
                passed &= run(
                    "python-consumers",
                    [
                        "/usr/bin/python3",
                        "-I",
                        source / "python_consumer.py",
                        prefix / "easyfly_projection_py",
                        prefix / "easyfly_obstacle_association",
                    ],
                )
            libraries = {}
            for binary in build.iterdir():
                if binary.is_file() and binary.read_bytes()[:4] == b"\x7fELF":
                    name = fixture + "-ldd-" + binary.name
                    linked = run(name, ["ldd", binary], ros)
                    text = (out / (name + ".log")).read_text()
                    libraries[binary.name] = text
                    passed &= linked and "not found" not in text
                    if not ros:
                        passed &= not any(
                            v in text
                            for v in (
                                "librclcpp",
                                "librcl.so",
                                "librmw",
                                "librosidl",
                            )
                        )
            record["consumers"].append(
                {
                    "fixture": fixture,
                    "passed": bool(passed and source_free),
                    "no_source_includes": source_free,
                    "libraries": libraries,
                }
            )
        else:
            record["consumers"].append({"fixture": fixture, "passed": False})
        save()
    record["passed"] = all(v["passed"] for v in record["consumers"])
    save()
    print("Consumers passed:", record["passed"], flush=True)
    return 0 if record["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Consume installed Vikit/Sophus targets numerically in a clean environment with explicit build dependency prefixes."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--prefix", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--mode", choices=["copy", "symlink", "relocated"], required=True)
    a = p.parse_args()
    prefix = a.prefix.resolve(strict=True)
    a.output.mkdir(parents=True, exist_ok=False)
    out = a.output.resolve()
    shutil.copytree(ROOT / "tools/algorithms/fastlivo_consumer", out / "consumer-source")
    env = dict(PATH="/usr/bin:/bin", LC_ALL="C.UTF-8")
    cmds = [
        [
            "cmake",
            "-S",
            str(out / "consumer-source"),
            "-B",
            str(out / "build"),
            "-DCMAKE_PREFIX_PATH=" + str(prefix / "vikit_common") + ";" + str(prefix / "sophus"),
        ],
        ["cmake", "--build", str(out / "build"), "--parallel", "2"],
        ["ctest", "--test-dir", str(out / "build"), "-V"],
    ]
    for index, cmd in enumerate(cmds):
        with (out / (str(index) + ".log")).open("x") as log:
            subprocess.run(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
    commands = json.loads((out / "build/compile_commands.json").read_text())
    assert all("src/third_party" not in c["command"] for c in commands)
    ldd = subprocess.check_output(["ldd", str(out / "build/consumer")], env=env, text=True)
    assert "librclcpp" not in ldd and "librclpy" not in ldd
    (out / "ldd.log").write_text(ldd)
    libraries = [
        l.strip().split(" => ")[1].split()[0] for l in ldd.splitlines() if "libvikit_common" in l
    ]
    assert len(libraries) == 1 and libraries[0].startswith(str(prefix))
    if a.mode != "symlink":
        assert not (prefix / "vikit_common/include/vikit/pinhole_camera.h").is_symlink()
    report = dict(
        passed=True,
        mode=a.mode,
        prefix=str(prefix),
        consumer="actual installed camera + Sophus rigid transform",
        library=libraries[0],
        library_realpath=os.path.realpath(libraries[0]),
        library_sha256=hashlib.sha256(Path(libraries[0]).read_bytes()).hexdigest(),
        clean_environment=env,
        compile_commands=commands,
    )
    (out / "result.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()

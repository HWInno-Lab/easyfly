#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Consume installed ORB-SLAM3 projection and real feature extraction without ROS/source includes."""
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
    p.add_argument("--pangolin", type=Path, required=True)
    p.add_argument("--build-base", type=Path)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--mode", choices=["copy", "symlink", "relocated"], required=True)
    a = p.parse_args()
    prefix = a.prefix.resolve(strict=True)
    a.output.mkdir(parents=True, exist_ok=False)
    out = a.output.resolve()
    shutil.copytree(ROOT / "tools/algorithms/orbslam3_consumer", out / "consumer-source")
    # Pangolin is an explicitly supplied external runtime dependency, installed
    # privately rather than registered in this machine's global ldconfig cache.
    env = dict(PATH="/usr/bin:/bin", LC_ALL="C.UTF-8",
               LD_LIBRARY_PATH=str(a.pangolin.resolve(strict=True) / 'lib'))
    cmds = [
        [
            "cmake",
            "-S",
            str(out / "consumer-source"),
            "-B",
            str(out / "build"),
            "-DCMAKE_PREFIX_PATH=" + str(prefix / "easyfly_orbslam3_core") + ";" + str(a.pangolin.resolve(strict=True)),
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
        l.strip().split(" => ")[1].split()[0] for l in ldd.splitlines() if "libeasyfly_orbslam3.so" in l
    ]
    assert len(libraries) == 1 and libraries[0].startswith(str(prefix))
    pangolin_libs = [l.strip().split(" => ")[1].split()[0] for l in ldd.splitlines() if "libpangolin" in l]
    assert len(pangolin_libs) == 1 and pangolin_libs[0].startswith(str(a.pangolin.resolve()))
    if a.mode == 'symlink':
        assert a.build_base is not None
        cache = a.build_base / 'easyfly_orbslam3_core/CMakeCache.txt'
        assert 'AMENT_CMAKE_SYMLINK_INSTALL:BOOL=1' in cache.read_text()
        assert (prefix / 'easyfly_orbslam3_integration/lib/easyfly_orbslam3_integration/orbslam3_rgbd').is_symlink()
    if a.mode != "symlink":
        assert not (prefix / "easyfly_orbslam3_core/include/easyfly_orbslam3/include/CameraModels/Pinhole.h").is_symlink()
    report = dict(
        passed=True,
        mode=a.mode,
        prefix=str(prefix),
        consumer="actual installed Pinhole projection and real upstream ORB extractor",
        library=libraries[0],
        library_realpath=os.path.realpath(libraries[0]),
        library_sha256=hashlib.sha256(Path(libraries[0]).read_bytes()).hexdigest(),
        clean_environment=env,
        compile_commands=commands,
        core_library_is_symlink=(prefix / 'easyfly_orbslam3_core/lib/libeasyfly_orbslam3.so').is_symlink(),
        install_semantics='Humble ament copies EXPORT targets and FILES_MATCHING headers even in symlink configuration; wrapper executable is symlinked' if a.mode == 'symlink' else 'regular copy',
    )
    (out / "result.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()

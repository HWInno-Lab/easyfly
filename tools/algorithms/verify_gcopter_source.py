#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Reconstruct the selected upstream GCOPTER files and verify the complete local delta."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def digest(data):
    return hashlib.sha256(data).hexdigest()


def verify(upstream):
    manifest = json.loads(
        (ROOT / "src/third_party/algorithms/planning/gcopter.provenance.json").read_text()
    )
    revision = subprocess.check_output(
        ["git", "-C", str(upstream), "rev-parse", "HEAD"], text=True
    ).strip()
    assert revision == manifest["revision"], "upstream revision mismatch"
    destination = ROOT / manifest["current_path"]
    actual = {str(p.relative_to(destination)) for p in destination.rglob("*") if p.is_file()}
    assert actual == set(manifest["files"]), "unregistered vendor files"
    with tempfile.TemporaryDirectory(prefix="easyfly-gcopter-source-") as temporary:
        tree = Path(temporary)
        for relative, expected in manifest["upstream_files"].items():
            source = relative if relative in ["LICENSE", "README.md"] else "gcopter/" + relative
            data = subprocess.check_output(
                ["git", "-C", str(upstream), "show", revision + ":" + source]
            )
            assert digest(data) == expected, relative + " upstream checksum"
            target = tree / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
        for patch in manifest["patches"]:
            file = ROOT / patch["path"]
            assert digest(file.read_bytes()) == patch["sha256"], "patch checksum"
            subprocess.run(
                ["patch", "--batch", "--fuzz=0", "-p1", "-i", str(file)], cwd=tree, check=True
            )
        for relative, expected in manifest["files"].items():
            rebuilt, current = (tree / relative).read_bytes(), (
                destination / relative
            ).read_bytes()
            assert rebuilt == current and digest(current) == expected, (
                relative + " reconstruction mismatch"
            )
    return dict(
        passed=True, revision=revision, files=len(actual), patches=len(manifest["patches"])
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(verify(args.upstream), indent=2))

#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Reconstruct pinned FAST-LIVO2 and dependencies from archives plus zero-fuzz patches."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def verify(manifest):
    record = json.loads(manifest.read_text())
    count = 0
    for component in record["components"]:
        archive = ROOT / component["archive"]
        patch = ROOT / component["patch"]
        assert sha(archive) == component["archive_sha256"]
        assert sha(patch) == component["patch_sha256"]
        with tempfile.TemporaryDirectory(prefix="easyfly-fastlivo-source-") as tmp:
            dst = Path(tmp)
            with tarfile.open(archive) as t:
                for member in t:
                    if member.isfile():
                        path = dst / member.name
                        assert path.resolve().is_relative_to(dst)
                        path.parent.mkdir(parents=True, exist_ok=True)
                        path.write_bytes(t.extractfile(member).read())
            if patch.stat().st_size:
                result = subprocess.run(
                    ["patch", "--batch", "--fuzz=0", "-p1", "-i", str(patch.resolve())],
                    cwd=dst,
                    capture_output=True,
                    text=True,
                )
                if result.returncode:
                    raise RuntimeError(result.stdout + result.stderr)
            for name, digest in component["installed_source_sha256"].items():
                actual = ROOT / component["destination"] / name
                assert sha(dst / name) == digest == sha(actual), str(actual)
                count += 1
    return dict(passed=True, files=count, components=len(record["components"]), fuzz=0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--manifest",
        type=Path,
        default=ROOT / "src/third_party/algorithms/localization/fast_livo2.provenance.json",
    )
    args = parser.parse_args()
    print(json.dumps(verify(args.manifest), indent=2))


if __name__ == "__main__":
    main()

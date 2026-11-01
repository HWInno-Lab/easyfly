# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0.
"""Private evidence storage; managed launch supplies a manifest-derived directory."""
import os
from pathlib import Path
import stat


def private_evidence_directory(value):
    path = Path(value).absolute()
    path.mkdir(mode=0o700, exist_ok=True)
    info = path.lstat()
    if (
        not stat.S_ISDIR(info.st_mode)
        or info.st_uid != os.getuid()
        or stat.S_IMODE(info.st_mode) & 0o077
    ):
        raise ValueError("FUEL evidence_directory must be an owner-private non-symlink directory")
    return path.resolve()

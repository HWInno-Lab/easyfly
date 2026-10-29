# Copyright 2026 EasyFly Project
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

"""Load an explicitly labeled multi-view reference gallery with frozen file hashes."""

import hashlib
import io
import json
import os
from pathlib import Path
import re
import stat

import cv2
import numpy as np
from PIL import Image

from .appearance import MODEL_SHA256


def bounded_file(path, limit):
    """Read a bounded regular file without following its final path component."""
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    try:
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or not 0 < info.st_size <= limit:
            raise ValueError('gallery input must be a bounded regular file')
        with os.fdopen(descriptor, 'rb', closefd=False) as stream:
            return stream.read(limit + 1)
    finally:
        os.close(descriptor)


def load(path, model):
    """Verify all labeled crop bytes before any identity can appear in output."""
    path = Path(path).absolute()
    raw = bounded_file(path, 65536)
    value = json.loads(raw)
    if (set(value) != {'version', 'model_sha256', 'vehicles'} or value['version'] != 1 or
            value['model_sha256'] != MODEL_SHA256 or
            not isinstance(value['vehicles'], list) or not 1 <= len(value['vehicles']) <= 64):
        raise ValueError('invalid enrollment version, model identity or gallery size')
    references = {}
    for vehicle in value['vehicles']:
        if set(vehicle) != {'identity', 'views'}:
            raise ValueError('invalid enrollment vehicle fields')
        identity = vehicle['identity']
        if (not re.fullmatch('[a-z][a-z0-9_-]{0,47}', identity) or identity in references or
                not isinstance(vehicle['views'], list) or not 1 <= len(vehicle['views']) <= 8):
            raise ValueError('invalid or duplicate enrollment identity')
        vectors = []
        seen = set()
        for view in vehicle['views']:
            if set(view) != {'file', 'sha256'} or not re.fullmatch('[0-9a-f]{64}', view['sha256']):
                raise ValueError('each reference image requires an exact SHA-256')
            relative = Path(view['file'])
            if relative.is_absolute() or '..' in relative.parts:
                raise ValueError('reference paths must remain inside the enrollment directory')
            candidate = path.parent / relative
            if not candidate.resolve().is_relative_to(path.parent.resolve()):
                raise ValueError('reference path escapes enrollment directory')
            data = bounded_file(candidate, 4 * 1024 * 1024)
            digest = hashlib.sha256(data).hexdigest()
            if digest != view['sha256'] or digest in seen:
                raise ValueError('reference image hash mismatch or duplicate view')
            seen.add(digest)
            with Image.open(io.BytesIO(data)) as header:
                if (header.format not in ('PNG', 'JPEG') or
                        not 24 <= header.width <= 1920 or not 24 <= header.height <= 1080):
                    raise ValueError('reference image format or decoded size is invalid')
            image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('reference image decode failed')
            vectors.append(model.embed(image[:, :, ::-1]))
        references[identity] = vectors
    return references, hashlib.sha256(raw).hexdigest()

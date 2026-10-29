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

"""Run the pinned OSNet model on original RGB crops and compare learned embeddings."""

import hashlib
import os
from pathlib import Path
import stat
import uuid

import cv2
import numpy as np

MODEL_SHA256 = '4aaad3e5db648618b0df3d2ff21c61323985ff9e50194c3d2edd4fb87c92d91f'


class AppearanceModel:
    """Verify model bytes before creating a bounded CPU-only inference session."""

    def __init__(self, path):
        import onnxruntime as ort
        # ament symlink installations legitimately link immutable package assets.
        # Resolve the asset, then verify the opened regular file's exact bytes.
        path = Path(path).resolve(strict=True)
        descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
        try:
            info = os.fstat(descriptor)
            if not stat.S_ISREG(info.st_mode) or info.st_size != 8836743:
                raise ValueError('model must be the pinned regular ONNX file')
            with os.fdopen(descriptor, 'rb', closefd=False) as stream:
                data = stream.read(8836744)
        finally:
            os.close(descriptor)
        if hashlib.sha256(data).hexdigest() != MODEL_SHA256:
            raise ValueError('model content does not match the declared vehicle OSNet model')
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.inter_op_num_threads = 1
        options.log_severity_level = 3
        self.session = ort.InferenceSession(data, options, providers=['CPUExecutionProvider'])
        self.runtime_version = ort.__version__
        self.path = str(Path(path).resolve())

    def embed(self, rgb):
        """Return a unit 512-vector; input normalization is part of the exported graph."""
        if (not isinstance(rgb, np.ndarray) or rgb.dtype != np.uint8 or
                rgb.ndim != 3 or rgb.shape[2] != 3 or min(rgb.shape[:2]) < 24 or
                max(rgb.shape[:2]) > 1920):
            raise ValueError('expected a bounded RGB uint8 vehicle crop of at least 24 pixels')
        # Reject blank/very low-texture crops before allowing gallery creation.
        if float(np.std(rgb.astype(np.float32), axis=(0, 1)).max()) < 8.:
            raise ValueError('vehicle crop contains insufficient visual texture')
        resized = cv2.resize(rgb, (208, 208), interpolation=cv2.INTER_LINEAR)
        blob = np.ascontiguousarray(resized.transpose(2, 0, 1)[None], dtype=np.float32)
        vector = self.session.run(['output'], {'input': blob})[0].reshape(-1)
        norm = float(np.linalg.norm(vector))
        if vector.shape != (512,) or not np.isfinite(vector).all() or norm < 1e-8:
            raise ValueError('model produced an invalid appearance embedding')
        return vector / norm


def normalized(vector):
    """Reject malformed embeddings at the association boundary."""
    value = np.asarray(vector, dtype=np.float32)
    norm = float(np.linalg.norm(value))
    if value.shape != (512,) or not np.isfinite(value).all() or not .99 <= norm <= 1.01:
        raise ValueError('association requires finite unit 512-dimensional embeddings')
    return value / norm


class Gallery:
    """Associate by appearance only, preserving ambiguity and one identity per frame."""

    def __init__(self, threshold=.25, margin=.05, capacity=128, lifetime_s=30.):
        if not (0. < threshold <= .5 and .01 <= margin <= .2 and
                2 <= capacity <= 512 and 1 <= lifetime_s <= 120):
            raise ValueError('invalid bounded appearance gallery configuration')
        self.threshold, self.margin = threshold, margin
        self.capacity, self.lifetime_ns = capacity, int(lifetime_s * 1e9)
        self.entries = {}
        self.last_source = {}

    def seed(self, references):
        """Keep explicit enrollment features separate from bounded live observations."""
        if self.entries or len(references) > min(64, self.capacity):
            raise ValueError('reference gallery must be bounded and installed before observations')
        for identity, vectors in references.items():
            if not identity or not 1 <= len(vectors) <= 8:
                raise ValueError('invalid enrollment identity or reference count')
            self.entries[identity] = dict(vectors=[], references=[normalized(v) for v in vectors],
                                          at=0., source_ns=0)

    def associate(self, camera, stamp_ns, vectors, now):
        """Return new/matched/ambiguous without trusting input tracker IDs as identity."""
        if (not camera or len(camera) > 32 or
                (camera not in self.last_source and len(self.last_source) >= 4) or
                stamp_ns <= self.last_source.get(camera, 0) or len(vectors) > 8):
            raise ValueError('nonincreasing camera source time or excessive vehicle count')
        vectors = [normalized(value) for value in vectors]
        self.last_source[camera] = stamp_ns
        self.entries = {key: entry for key, entry in self.entries.items()
                        if entry.get('references') or (
                            now - entry['at'] <= self.lifetime_ns / 1e9 and
                            stamp_ns - entry['source_ns'] <= self.lifetime_ns)}
        identities = list(self.entries)
        costs = np.array([[min(1. - float(np.dot(vector, candidate))
                               for candidate in (self.entries[key].get('references', []) +
                                                 self.entries[key]['vectors']))
                           for key in identities] for vector in vectors], dtype=np.float64)
        proposals, result = [], []
        for index in range(len(vectors)):
            order = sorted(range(len(identities)), key=lambda j: costs[index, j])
            best = max(0., float(costs[index, order[0]])) if order else 2.
            second = float(costs[index, order[1]]) if len(order) > 1 else 2.
            gap = max(0., second - best)
            if best > self.threshold:
                proposals.append(None)
                result.append(dict(identity='', state='new', distance=best, margin=gap))
            elif gap < self.margin:
                proposals.append(None)
                result.append(dict(identity='', state='ambiguous', distance=best, margin=gap))
            else:
                proposals.append(identities[order[0]])
                result.append(dict(identity=identities[order[0]], state='matched',
                                   distance=best, margin=gap))
        # Multiple observations claiming the same identity are all left unknown.
        for index, key in enumerate(proposals):
            if key is not None and proposals.count(key) > 1:
                result[index].update(identity='', state='ambiguous')
        for vector, row in zip(vectors, result):
            if row['state'] == 'new':
                if len(self.entries) >= self.capacity:
                    row['state'] = 'capacity_exhausted'
                    continue
                row['identity'] = uuid.uuid4().hex
                self.entries[row['identity']] = dict(vectors=[], at=now, source_ns=stamp_ns)
            if row['state'] in ('new', 'matched'):
                entry = self.entries[row['identity']]
                entry.update(at=now, source_ns=max(stamp_ns, entry['source_ns']))
                entry['vectors'].append(vector.copy())
                del entry['vectors'][:-8]
        return result

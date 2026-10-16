# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Apply one checked change to the hash-pinned complete upstream source."""
import hashlib
from pathlib import Path
import sys

path = Path(sys.argv[1]) / 'src/cpp/rtps/builtin/discovery/endpoint/EDP.cpp'
before = 'c98c42fd227b5e0e7bd7ce7a093a611e234c31e4d5e06b1ed88a1c2d83d41a1d'
after = 'c347aca0b447b3e7ad45ece94500b52bc96628f751481b8cb8dfd2e1116d623c'
data = path.read_bytes()
digest = hashlib.sha256(data).hexdigest()
if digest == after:
    sys.exit(0)
if digest != before:
    raise RuntimeError('Unexpected upstream EDP.cpp; refusing patch')
start = data.index(b'bool EDP::unpairWriterProxy(')
end = data.index(b'bool EDP::unpairReaderProxy(', start)
chunk = data[start:end]
old = b'                    //MATCHED AND ADDED CORRECTLY:\n                    if (r.getListener() != nullptr)'
new = b"                    // The participant's endpoint-list shared lock keeps the reader and\n                    // its DDS owner alive until this traversal finishes. Snapshot under\n                    // the same mutex as setListener, without holding it across callbacks\n                    // or the PDP status lock (which would invert discovery lock order).\n                    ReaderListener* listener = nullptr;\n                    {\n                        std::lock_guard<RecursiveTimedMutex> reader_lock(r.getMutex());\n                        listener = r.getListener();\n                    }\n                    if (listener != nullptr)"
if chunk.count(old) != 1:
    raise RuntimeError('Unexpected patch context')
chunk = chunk.replace(old, new).replace(b'r.getListener()->onReaderMatched', b'listener->onReaderMatched')
data = data[:start] + chunk + data[end:]
if hashlib.sha256(data).hexdigest() != after:
    raise RuntimeError('Unexpected patched source')
path.write_bytes(data)

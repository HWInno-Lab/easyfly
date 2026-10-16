# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Apply checked listener changes to the hash-pinned complete upstream source."""
import hashlib
from pathlib import Path
import sys

def reader_patch():
    path = Path(sys.argv[1]) / 'src/cpp/rtps/builtin/discovery/endpoint/EDP.cpp'
    before = 'c98c42fd227b5e0e7bd7ce7a093a611e234c31e4d5e06b1ed88a1c2d83d41a1d'
    after = 'c347aca0b447b3e7ad45ece94500b52bc96628f751481b8cb8dfd2e1116d623c'
    data = path.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    if digest == after:
        return
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

root = Path(sys.argv[1])
path = root / "src/cpp/rtps/builtin/discovery/endpoint/EDP.cpp"
if hashlib.sha256(path.read_bytes()).hexdigest() != '6efc6392bea2bba8c5a8f3b6a471e0c062f244ef28932f1d298b61746c7357e5':
    reader_patch()
    data = path.read_bytes()
    start = data.index(b"bool EDP::unpairReaderProxy(")
    end = data.index(b"bool EDP::validMatching(", start)
    chunk = data[start:end]
    old = b'                    //MATCHED AND ADDED CORRECTLY:\n                    if (w.getListener() != nullptr)'
    new = b'                    // The endpoint traversal retains the DDS owner. Synchronize\n                    // with set_listener, then release before callbacks/PDP locks.\n                    WriterListener* listener = nullptr;\n                    {\n                        std::lock_guard<RecursiveTimedMutex> writer_lock(w.getMutex());\n                        listener = w.getListener();\n                    }\n                    if (listener != nullptr)'
    if chunk.count(old) != 1:
        raise RuntimeError("Unexpected writer patch context")
    chunk = chunk.replace(old, new).replace(b"w.getListener()->onWriterMatched", b"listener->onWriterMatched")
    data = data[:start] + chunk + data[end:]
    if hashlib.sha256(data).hexdigest() != '6efc6392bea2bba8c5a8f3b6a471e0c062f244ef28932f1d298b61746c7357e5':
        raise RuntimeError("Unexpected patched writer source")
    path.write_bytes(data)

path = root / "include/fastdds/rtps/writer/RTPSWriter.h"
data = path.read_bytes()
digest = hashlib.sha256(data).hexdigest()
if digest != 'afe142d08866f61f842cb9b618ffb00d81d0ba7ffb45ca3216e5c100514f8a4d':
    if digest != '4414c565be56eb9240f54df7ad2ed77217ae9fccaa05fde7fce74a16aebcfde5':
        raise RuntimeError("Unexpected writer header")
    old = b'    {\n        mp_listener = listener;\n        return true;\n    }'
    new = b'    {\n        std::lock_guard<RecursiveTimedMutex> guard(mp_mutex);\n        mp_listener = listener;\n        return true;\n    }'
    if data.count(old) != 1:
        raise RuntimeError("Unexpected writer setter context")
    data = data.replace(old, new)
    if hashlib.sha256(data).hexdigest() != 'afe142d08866f61f842cb9b618ffb00d81d0ba7ffb45ca3216e5c100514f8a4d':
        raise RuntimeError("Unexpected patched writer header")
    path.write_bytes(data)

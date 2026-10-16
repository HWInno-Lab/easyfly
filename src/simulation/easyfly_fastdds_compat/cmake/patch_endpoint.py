# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Apply checked listener changes to the hash-pinned complete upstream source."""
import hashlib
from pathlib import Path
import sys

root = Path(sys.argv[1])

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


def discovery_patch(class_name, original_hash, patched_hash):
    """Snapshot all three reader discovery events without extending lock scope."""
    path = root / f'src/cpp/rtps/reader/{class_name}.cpp'
    data = path.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    if digest == patched_hash:
        return
    if digest != original_hash:
        raise RuntimeError(f'Unexpected upstream {class_name}.cpp; refusing patch')
    start = data.index(f'bool {class_name}::matched_writer_add('.encode())
    middle = data.index(f'bool {class_name}::matched_writer_remove('.encode(), start)
    end = data.index(f'bool {class_name}::matched_writer_is_matched('.encode(), middle)
    add, remove = data[start:middle], data[middle:end]
    old = b'    {\n        std::unique_lock<RecursiveTimedMutex> guard(mp_mutex);'
    if add.count(old) != 1 or add.count(b'if (nullptr != mp_listener)') != 2:
        raise RuntimeError('Unexpected reader discovery add context')
    add = add.replace(old, b'    ReaderListener* listener = nullptr;\n' + old +
                      b'\n        listener = mp_listener;', 1)
    add = add.replace(b'if (nullptr != mp_listener)', b'if (nullptr != listener)')
    add = add.replace(b'mp_listener->on_writer_discovery', b'listener->on_writer_discovery')
    if remove.count(b'if (nullptr != mp_listener)') != 1:
        raise RuntimeError('Unexpected reader discovery remove context')
    indent = b'            ' if class_name == 'StatefulReader' else b'                '
    remove = remove.replace(b'if (nullptr != mp_listener)',
                            b'// Capture under the reader mutex before releasing it for discovery.\n' +
                            indent + b'ReaderListener* listener = mp_listener;\n' +
                            indent + b'if (nullptr != listener)')
    remove = remove.replace(b'mp_listener->on_writer_discovery', b'listener->on_writer_discovery')
    data = data[:start] + add + remove + data[end:]
    if hashlib.sha256(data).hexdigest() != patched_hash:
        raise RuntimeError(f'Unexpected patched {class_name}.cpp')
    path.write_bytes(data)


discovery_patch('StatefulReader',
                'aca08de4053e27fdf21ae1f60336b6c027616bb98226edda136e9b13666666d6',
                '605e7710db34068442fb291a1f299bfc4597fc74c626b35e1c1fd4965de6f03b')
discovery_patch('StatelessReader',
                'c526380b5631d71411bc54d0f202e283fb8a4745683cc7cb3803e6071504ac77',
                '153a3f2a5ff14e65b5013d9331866f1ddc1190d32360efcfb4254b5c3bdb4350')

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

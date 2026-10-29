#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Verify pinned deployment bytes and the semantics of a separately exported ONNX."""
import argparse
import copy
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def canonical_graph(graph):
    """Ignore diagnostic names and optional shape annotations, never operators/data."""
    graph = copy.deepcopy(graph)
    graph.ClearField('value_info')
    graph.ClearField('doc_string')
    names = {item.name: item.name for item in graph.input}
    names.update({item.name: item.name for item in graph.initializer})
    for index, node in enumerate(graph.node):
        node.ClearField('name')
        node.ClearField('doc_string')
        for slot, name in enumerate(node.input):
            if name:
                node.input[slot] = names[name]
        for slot, name in enumerate(node.output):
            symbol = f'node_{index}_output_{slot}'
            names[name] = symbol
            node.output[slot] = symbol
    for output in graph.output:
        output.name = names[output.name]
    return graph.SerializeToString(deterministic=True)


def verify(candidate):
    import onnx
    manifest = json.loads((ROOT / 'src/third_party/model-assets.provenance.json').read_text())
    for name, fingerprint in manifest['artifacts'].items():
        path = ROOT / 'models' / name
        if hashlib.sha256(path.read_bytes()).hexdigest() != fingerprint['sha256']:
            raise ValueError('deployment asset fingerprint differs: ' + name)
    original = onnx.load(ROOT / 'models/yolo11n.onnx')
    exported = onnx.load(candidate)
    onnx.checker.check_model(original)
    onnx.checker.check_model(exported)
    if not all(7 <= model.ir_version <= 10 for model in (original, exported)):
        raise ValueError('ONNX IR version outside verified runtime compatibility')
    if original.functions != exported.functions:
        raise ValueError('local functions differ')
    if original.opset_import != exported.opset_import:
        raise ValueError('operator set differs')
    if ([item.name for item in original.graph.output] !=
            [item.name for item in exported.graph.output]):
        raise ValueError('public output names differ')
    baseline = canonical_graph(original.graph)
    actual = canonical_graph(exported.graph)
    if baseline != actual:
        raise ValueError('model operators, connectivity, attributes or data differ')
    return {'passed': True, 'candidate': str(candidate),
            'deployment_ir_version': original.ir_version,
            'candidate_ir_version': exported.ir_version,
            'canonical_graph_sha256': hashlib.sha256(actual).hexdigest(),
            'initializer_count': len(original.graph.initializer),
            'candidate_file_sha256': hashlib.sha256(candidate.read_bytes()).hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('candidate', type=Path)
    args = parser.parse_args()
    print(json.dumps(verify(args.candidate)))


if __name__ == '__main__':
    main()

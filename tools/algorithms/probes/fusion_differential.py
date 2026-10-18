#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""Compare preserved full projection/classification outputs to installed adapters."""
import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import random
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Transform
from sensor_msgs.msg import Image, CameraInfo
from rosidl_runtime_py.convert import message_to_ordereddict
from easyfly_lidar_camera_fusion import projection as new_projection
from easyfly_obstacle_classification import core as new_classification


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def canonical(message):
    return json.dumps(message_to_ordereddict(message), sort_keys=True, separators=(',', ':')).encode()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--legacy-dir', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    old_projection = load('old_projection', args.legacy_dir / 'easyfly_lidar_camera_fusion/easyfly_lidar_camera_fusion/projection.py')
    old_classification = load('old_classification', args.legacy_dir / 'easyfly_semantic_map/easyfly_obstacle_classification/core.py')
    fixtures = load('old_fixtures', args.legacy_dir / 'easyfly_semantic_map/test/test_classification.py').fixtures
    rng = random.Random(618)
    trace_projection, trace_classification = hashlib.sha256(), hashlib.sha256()
    for index in range(240):
        encoding = ('rgb8', 'bgr8', 'mono8')[index % 3]
        channels = 1 if encoding == 'mono8' else 3
        image = Image(width=13, height=9, encoding=encoding, step=13*channels+2,
                      data=bytes(rng.randrange(256) for _ in range((13*channels+2)*9)))
        info = CameraInfo(width=13, height=9, distortion_model='plumb_bob',
                          k=[6.,0.,6.,0.,6.,4.,0.,0.,1.],
                          d=[rng.uniform(-.1,.1),0.,0.,0.,0.])
        transform = Transform()
        angle = rng.uniform(-.5,.5)
        transform.rotation.z, transform.rotation.w = math.sin(angle/2), math.cos(angle/2)
        transform.translation.x = rng.uniform(-1,1)
        points = [(rng.uniform(-4,4), rng.uniform(-3,3), rng.uniform(-1,6)) for _ in range(80)]
        points += [(0.,0.,2.),(0.,0.,4.),(0.,0.,2.)]
        if index % 12 == 0:
            info.k[0] = float('nan')
        elif index % 12 == 1:
            transform.rotation.w = 0.
        elif index % 12 == 2:
            image.step = 1
        elif index % 12 == 3:
            info.d[0] = 101.
        results = []
        for module in (old_projection, new_projection):
            try:
                colors, depths, count = module.project(points, image, module.camera_model(info), transform)
                results.append((colors, depths, count))
            except (ValueError, IndexError) as error:
                results.append((type(error).__name__, str(error)))
        assert results[0] == results[1], ('projection', index)
        trace_projection.update(repr(results[0]).encode())
    for index in range(320):
        geometry, targets = fixtures()
        original_obstacle = copy.deepcopy(geometry.obstacles[0])
        original_target = copy.deepcopy(targets.targets[0])
        geometry.obstacles = []
        targets.targets = []
        for identity in range(rng.randrange(1,9)):
            obstacle = copy.deepcopy(original_obstacle)
            obstacle.obstacle_id = identity
            obstacle.center_x = rng.uniform(-3,3)
            obstacle.center_y = rng.uniform(-2,2)
            obstacle.size_x = rng.uniform(.1,3)
            obstacle.velocity_y = rng.uniform(-2,2)
            geometry.obstacles.append(obstacle)
        for identity in range(rng.randrange(9)):
            target = copy.deepcopy(original_target)
            target.track_id = identity
            target.position_x = rng.uniform(-3,3)
            target.position_y = rng.uniform(-2,2)
            target.confidence = rng.choice([.1,.7,.9])
            target.class_name = rng.choice(['person','car','unknown',''])
            targets.targets.append(target)
        case = index % 10
        if case == 0:
            geometry.header.frame_id = 'map'
        elif case == 1:
            geometry.obstacles[0].size_x = -1.
        elif case == 2:
            targets.header.stamp.sec = 9
        elif case == 3:
            targets.header.stamp.nanosec = 200000000
        elif case == 4 and targets.targets:
            targets.targets[0].position_x = float('nan')
        elif case == 5:
            geometry.obstacles.append(copy.deepcopy(geometry.obstacles[0]))
        elif case == 6 and targets.targets:
            targets.targets.append(copy.deepcopy(targets.targets[0]))
        before = (canonical(geometry), canonical(targets))
        now = 10000000000 + rng.randrange(0,700000000)
        options = dict(ttl=rng.choice([.1,.5,.8]), margin=rng.choice([0.,.3,.8]))
        expected = old_classification.classify(geometry, targets, now, **options)
        actual = new_classification.classify(geometry, targets, now, **options)
        assert actual == expected, ('classification', index)
        assert (canonical(geometry), canonical(targets)) == before
        # Compare every message field without depending on unspecified CDR alignment padding.
        serialized = canonical(actual)
        assert serialized == canonical(expected), ('serialized classification', index)
        trace_classification.update(serialized)
    result = {'projection_cases':240, 'classification_cases':320,
              'projection_trace_sha256':trace_projection.hexdigest(),
              'classification_trace_sha256':trace_classification.hexdigest(),
              'projection_adapter':new_projection.__file__, 'classification_adapter':new_classification.__file__,
              'prefixes':{p:get_package_prefix(p) for p in ['easyfly_lidar_camera_fusion','easyfly_semantic_map']},
              'passed':True}
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()

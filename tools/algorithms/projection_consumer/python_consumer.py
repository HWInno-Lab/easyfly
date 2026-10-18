# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""Run with python -I: explicitly installed pure modules, no ROS overlay."""
import json
from pathlib import Path
import sys

prefixes = [Path(p).resolve() for p in sys.argv[1:]]
for prefix in prefixes:
    paths = [path for pattern in ('lib/python*/site-packages',
                                   'lib/python*/dist-packages',
                                   'local/lib/python*/dist-packages')
             for path in prefix.glob(pattern)]
    assert len(paths) == 1, prefix
    sys.path.insert(0, str(paths[0]))
from easyfly_projection_py import core as projection
from easyfly_obstacle_association import core as association

image = projection.ImageView(3, 3, 'rgb8', 9, bytes([10, 20, 30]*9))
model = projection.camera_model(3, 3, (1.,0.,1.,0.,1.,1.,0.,0.,1.), (0.,)*5, 'plumb_bob')
transform = projection.TransformView((0.,0.,0.,1.), (1.,0.,0.))
colors, depths = projection.project([(-1.,0.,2.),(-1.,0.,4.)], image, model, transform)
assert colors == [((-1.,0.,2.), (10,20,30))] and depths[4] == 2
boxes = [association.Obstacle((0.,0.,1.), (2.,2.,2.))]
targets = [association.Target(3, 'person', .9, (0.,0.,1.))]
assert association.associate(boxes, targets)[0].target_id == 3
assert association.associate(boxes*2, targets) == []
assert not any(n.split('.')[0] in {'rclpy', 'rclcpp', 'sensor_msgs', 'easyfly_msgs', 'builtin_interfaces'} for n in sys.modules)
paths = [projection.__file__, association.__file__]
for path, prefix in zip(paths, prefixes):
    assert str(path).startswith(str(prefix)+'/'), (path, prefix)
print(json.dumps({'installed_modules':paths, 'no_ros_imports':True, 'passed':True}))

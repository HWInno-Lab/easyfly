# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
from easyfly_obstacle_association.core import Obstacle, Target, associate


def target(identity=9, x=0., confidence=.9, name='person'):
    return Target(identity, name, confidence, (x, 0., 1.))


def test_unambiguous_fields_and_inputs_unchanged():
    obstacles = [Obstacle((0., 0., 1.), (2., 2., 2.))]
    targets = [target()]
    result = associate(obstacles, targets)
    assert len(result) == 1
    assert (result[0].obstacle_index, result[0].target_index, result[0].target_id) == (0, 0, 9)
    assert result[0].class_name == 'person' and result[0].confidence == .9
    assert targets == [target()] and obstacles[0].size == (2., 2., 2.)


def test_bidirectional_ambiguity_counts_all_candidates():
    obstacles = [Obstacle((0., 0., 1.), (2., 2., 2.)), Obstacle((2., 0., 1.), (2., 2., 2.))]
    assert associate(obstacles, [target(x=1.)]) == []
    assert associate(obstacles, [target(x=1.), target(10, -1.)]) == []
    assert associate(obstacles[:1], [target(), target(10)]) == []


def test_inclusive_margin_and_confidence_threshold():
    box = [Obstacle((0., 0., 1.), (2., 2., 2.))]
    assert len(associate(box, [target(x=1.3, confidence=.7)])) == 1
    assert associate(box, [target(x=1.30001)]) == []
    assert associate(box, [target(confidence=.6999)]) == []


def test_unknown_empty_and_no_geometry():
    box = [Obstacle((0., 0., 1.), (2., 2., 2.))]
    assert associate(box, [target(name='unknown')]) == []
    assert associate(box, [target(name='')]) == []
    assert associate([], [target()]) == [] and associate(box, []) == []

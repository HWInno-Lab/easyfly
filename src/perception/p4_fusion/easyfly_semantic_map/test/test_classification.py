# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_semantic_map/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
#

import copy
import unittest
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from easyfly_msgs.msg import (
    ObstacleInfo,
    ObstacleInfoArray,
    Target3D,
    Target3DArray,
)
from easyfly_obstacle_classification.core import classify


def fixtures():
    header = Header(stamp=Time(sec=10), frame_id='odom')
    obstacle = ObstacleInfo(
        header=header,
        obstacle_id=7,
        center_z=1.0,
        size_x=2.0,
        size_y=2.0,
        size_z=2.0,
        obstacle_type='unknown',
        is_dynamic=True,
        velocity_x=1.0,
    )
    target = Target3D(
        header=header,
        track_id=9,
        class_name='person',
        confidence=0.9,
        position_z=1.0,
        position_valid=True,
    )
    return ObstacleInfoArray(
        header=header, obstacles=[obstacle]
    ), Target3DArray(header=header, targets=[target])


class Classification(unittest.TestCase):
    def test_class_confidence_expiry_and_geometry_unchanged(self):
        geometry, targets = fixtures()
        original = copy.deepcopy(geometry)
        output = classify(geometry, targets, 10000000000)
        self.assertTrue(output.geometry_valid)
        item = output.obstacles[0]
        self.assertTrue(item.association_valid)
        self.assertEqual(item.target_id, 9)
        self.assertAlmostEqual(item.confidence, 0.9)
        self.assertEqual(item.obstacle.obstacle_type, 'person')
        self.assertEqual(item.expires_at, Time(sec=10, nanosec=500000000))
        item.obstacle.obstacle_type = 'unknown'
        self.assertEqual(item.obstacle, original.obstacles[0])
        self.assertEqual(geometry, original)

    def test_stale_empty_invalid_low_confidence_keep_geometry_unknown(self):
        geometry, targets = fixtures()
        for mutate in (
            lambda t: t.targets.clear(),
            lambda t: setattr(t.targets[0], 'position_valid', False),
            lambda t: setattr(t.targets[0], 'confidence', 0.1),
            lambda t: setattr(t.targets[0], 'position_x', float('nan')),
        ):
            snapshot = copy.deepcopy(targets)
            mutate(snapshot)
            output = classify(geometry, snapshot, 10000000000)
            self.assertTrue(output.geometry_valid)
            self.assertEqual(len(output.obstacles), 1)
            self.assertFalse(output.obstacles[0].association_valid)
            self.assertEqual(
                output.obstacles[0].obstacle.obstacle_type, 'unknown'
            )
        targets.header = Header(stamp=Time(sec=9), frame_id='odom')
        self.assertFalse(
            classify(geometry, targets, 10000000000)
            .obstacles[0]
            .association_valid
        )

    def test_ambiguity_in_both_directions_even_when_other_box_is_ambiguous(
        self,
    ):
        geometry, targets = fixtures()
        geometry.obstacles.append(copy.deepcopy(geometry.obstacles[0]))
        geometry.obstacles[1].obstacle_id = 8
        self.assertFalse(
            any(
                o.association_valid
                for o in classify(geometry, targets, 10000000000).obstacles
            )
        )
        geometry.obstacles[1].center_x = 2.0
        targets.targets[0].position_x = 1.0
        second = copy.deepcopy(targets.targets[0])
        second.track_id, second.position_x = 10, -1.0
        targets.targets.append(second)
        self.assertFalse(
            any(
                o.association_valid
                for o in classify(geometry, targets, 10000000000).obstacles
            )
        )

    def test_invalid_geometry_has_no_trusted_sidecopy(self):
        geometry, targets = fixtures()
        geometry.obstacles[0].size_x = -1.0
        result = classify(geometry, targets, 10000000000)
        self.assertFalse(result.geometry_valid)
        self.assertFalse(result.obstacles)


if __name__ == '__main__':
    unittest.main()

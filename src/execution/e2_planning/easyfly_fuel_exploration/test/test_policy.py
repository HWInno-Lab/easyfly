"""Safety/coverage contracts on tiny independent grids, not sensor acceptance."""
import copy
import unittest
import numpy as np
from easyfly_msgs.msg import VoxelGrid
from easyfly_fuel_exploration.policy import Policy, validate_pair


def grid():
    g=VoxelGrid();g.header.frame_id='odom';g.header.stamp.sec=1
    g.session_id='sensor';g.generation=1;g.revision=2;g.valid=True;g.resolution=.2
    g.size_x=g.size_y=20;g.size_z=10;g.origin.x=g.origin.y=-2.;g.origin.z=.4
    g.data=[-1]*4000
    return g


class PolicyTest(unittest.TestCase):
    def test_inflation_does_not_observe_unknown_space(self):
        raw=grid();inflated=copy.deepcopy(raw);inflated.inflation_radius=.65;inflated.data=[100]*4000
        policy=Policy(raw,[-1.8,-1.8,.6],[1.8,1.8,2.4],[])
        a,b,c=policy.apply(raw,inflated)
        self.assertEqual(c,0.);self.assertTrue((b==100).all())
        self.assertTrue((np.asarray(raw.data)==-1).all())
        raw.data=[0]*4000;_,_,c=policy.apply(raw,inflated);self.assertEqual(c,1.)

    def test_no_fly_masks_owned_copies_and_adds_body_clearance(self):
        raw=grid();raw.data=[0]*4000;inflated=copy.deepcopy(raw);inflated.inflation_radius=.65
        policy=Policy(raw,[-1.8,-1.8,.6],[1.8,1.8,2.4],[[0.,0.,1.],[.2,.2,2.]])
        a,b,c=policy.apply(raw,inflated)
        self.assertEqual(c,1.);self.assertGreater(np.count_nonzero(b==100),np.count_nonzero(a==100))
        self.assertFalse(policy.contains([-.4,0.,1.4]));self.assertTrue(policy.contains([-1.2,0.,1.4]))
        self.assertEqual(raw.data,inflated.data)

    def test_source_pair_cannot_mix_revisions_or_free_unobserved(self):
        raw=grid();inflated=copy.deepcopy(raw);inflated.inflation_radius=.65
        inflated.revision+=1
        with self.assertRaisesRegex(ValueError,'IDENTITY'):validate_pair(raw,inflated)
        inflated.revision=raw.revision;inflated.data[0]=0
        with self.assertRaisesRegex(ValueError,'VALUES'):validate_pair(raw,inflated)

    def test_bounds_and_epoch_changes_fail_closed(self):
        raw=grid();inflated=copy.deepcopy(raw);inflated.inflation_radius=.65
        with self.assertRaises(ValueError):Policy(raw,[-3.,-1.,.6],[1.,1.,2.4],[])
        with self.assertRaises(ValueError):Policy(raw,[-1.,-1.,.4],[1.,1.,2.4],[])
        policy=Policy(raw,[-1.,-1.,.6],[1.,1.,2.4],[])
        raw.origin.x+=.2;inflated.origin.x+=.2
        with self.assertRaisesRegex(ValueError,'GEOMETRY'):policy.apply(raw,inflated)

    def test_unknown_outside_task_reserves_body_inside_without_changing_coverage(self):
        raw=grid();raw.data=[0]*4000
        # Physical unknown at x=1.3; task ends at x=1.0. Masking the task first
        # would erase this evidence and let a body centre approach unseen space.
        source=16+20*(10+20*5);near=14+20*(10+20*5);far=8+20*(10+20*5)
        raw.data[source]=-1;inflated=copy.deepcopy(raw);inflated.inflation_radius=.65
        policy=Policy(raw,[-1.8,-1.8,.6],[1.,1.8,2.4],[])
        a,b,coverage=policy.apply(raw,inflated)
        guarded_a,guarded,guarded_coverage=policy.apply(raw,inflated,reserve_unknown=True)
        self.assertEqual(coverage,1.);self.assertEqual(guarded_coverage,coverage)
        np.testing.assert_array_equal(a,guarded_a)
        self.assertEqual(b[near],0);self.assertEqual(guarded[near],100)
        self.assertEqual(guarded[far],0);self.assertEqual(raw.data[source],-1)
        # Separable dilation agrees with an independent direct cube oracle.
        expected=b.copy()
        for z in range(10):
            for y in range(20):
                for x in range(20):
                    index=x+20*(y+20*z)
                    if max(abs(x-16),abs(y-10),abs(z-5))<=5 and expected[index]==0:
                        expected[index]=100
        np.testing.assert_array_equal(guarded,expected)


if __name__=='__main__':unittest.main()

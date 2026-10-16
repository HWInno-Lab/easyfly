"""Pure geometry contracts: no ROS, simulator, estimator or flight is started."""
import copy
import math
import pytest
from easyfly_px4_sitl.scene_alignment import compare_alignment, validate_targets

REGION = dict(min=[-.3, -.3, .5], max=[1.8, .3, 2.3])
IDENTITY = dict(translation=[0., 0., 0.], yaw=0.)


def test_default_preserves_exact_legacy_translation_and_wrapped_yaw():
    a=dict(translation=[1.,2.,3.],yaw=math.pi-.01)
    b=dict(translation=[4.,6.,3.],yaw=-math.pi+.01)
    result=compare_alignment(a,b)
    assert result['policy']=='world_origin' and result['maximum_position_error_m']==5.
    assert result['translation_component_error_m']==5. and len(result['samples'])==1
    assert result['yaw_error_rad']==pytest.approx(.02)


def test_region_checks_real_domain_away_from_world_origin():
    theta=.03; x,y=-6.,10.
    a=dict(translation=[6.,-10.,0.],yaw=0.)
    b=dict(translation=[.02-math.cos(theta)*x+math.sin(theta)*y,
                        -math.sin(theta)*x-math.cos(theta)*y,.05],yaw=theta)
    result=compare_alignment(a,b,policy='recommended_region',region=REGION)
    assert result['translation_component_error_m']>.3
    assert result['maximum_position_error_m']<.12 and result['yaw_error_rad']==pytest.approx(theta)
    assert len(result['samples'])==8


@pytest.mark.parametrize('offset', [[100.,-200.,50.],[-10000.,5000.,-300.]])
def test_region_error_is_invariant_to_world_origin(offset):
    a=dict(translation=[6.,-10.,-.05],yaw=-.015)
    b=dict(translation=[6.1,-9.9,-.1],yaw=.01)
    before=compare_alignment(a,b,policy='recommended_region',region=REGION)
    def shifted(t):
        x,y,z=offset;c,s=math.cos(t['yaw']),math.sin(t['yaw'])
        return dict(translation=[t['translation'][0]+c*x-s*y,t['translation'][1]+s*x+c*y,
                                 t['translation'][2]+z],yaw=t['yaw'])
    after=compare_alignment(shifted(a),shifted(b),policy='recommended_region',region=REGION)
    assert after['maximum_position_error_m']==pytest.approx(before['maximum_position_error_m'],abs=1e-9)
    assert after['yaw_error_rad']==before['yaw_error_rad']
    assert after['translation_component_error_m']!=pytest.approx(before['translation_component_error_m'])


def test_real_translation_and_yaw_excess_remain_observable():
    shifted=dict(translation=[.201,0.,0.],yaw=0.)
    assert compare_alignment(IDENTITY,shifted,policy='recommended_region',region=REGION)['maximum_position_error_m']>.2
    turned=dict(translation=[0.,0.,0.],yaw=.1001)
    assert compare_alignment(IDENTITY,turned,policy='recommended_region',region=REGION)['yaw_error_rad']>.1


def test_region_rejects_far_corner_even_when_world_origin_is_exact():
    turned=dict(translation=[0.,0.,0.],yaw=.09)
    assert compare_alignment(IDENTITY,turned)['maximum_position_error_m']==0.
    result=compare_alignment(IDENTITY,turned,policy='recommended_region',region=dict(min=[0.,0.,0.],max=[10.,1.,2.]))
    assert result['yaw_error_rad']<.1 and result['maximum_position_error_m']>.8


@pytest.mark.parametrize('bad', [None, {'min':[0,0,0],'max':[0,1,1]},
                               {'min':[0,0,0],'max':[1,float('nan'),1]},
                               {'min':[False,0,0],'max':[1,1,1]},
                               {'min':[0,0],'max':[1,1,1]}])
def test_region_alignment_requires_valid_finite_bounds(bad):
    with pytest.raises(ValueError):compare_alignment(IDENTITY,IDENTITY,policy='recommended_region',region=bad)


@pytest.mark.parametrize('change', ['policy','yaw','translation','missing'])
def test_invalid_transform_and_unknown_policy_fail_closed(change):
    measured=copy.deepcopy(IDENTITY);policy='recommended_region'
    if change=='policy':policy='guess'
    elif change=='yaw':measured['yaw']=float('inf')
    elif change=='translation':measured['translation']=[0,0]
    else:measured.pop('yaw')
    with pytest.raises(ValueError):compare_alignment(IDENTITY,measured,policy=policy,region=REGION)


def test_mission_cannot_shrink_checked_region_away_from_actual_targets():
    validate_targets(REGION,[[1.5,0,1.5],[0,0,1.5]])
    for targets in [[],[[2.,0,1.5]],[[0,0,3.]],[[float('nan'),0,1.5]]]:
        with pytest.raises(ValueError):validate_targets(REGION,targets)

# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Camera geometry and source leases; synthetic contract tests, not flight evidence."""
from collections import OrderedDict
from types import SimpleNamespace
import time

from geometry_msgs.msg import TransformStamped
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo, Image

from easyfly_fuel_exploration.camera import CameraView, calibration, transform


def info():
    value=CameraInfo(width=640,height=480,distortion_model='plumb_bob',d=[0.]*5)
    value.k=[400.,0.,320.,0.,400.,240.,0.,0.,1.]
    return value


@pytest.mark.parametrize('fault', ['nan', 'distortion', 'roi', 'principal', 'skew', 'focal'])
def test_unsupported_calibration_is_rejected(fault):
    value=info()
    if fault=='nan':value.k[0]=float('nan')
    if fault=='distortion':value.d[0]=.02
    if fault=='roi':value.roi.width=640
    if fault=='principal':value.k[2]=700.
    if fault=='skew':value.k[1]=1.
    if fault=='focal':value.k[4]=0.
    with pytest.raises(ValueError):calibration(value)


def test_optical_geometry_uses_observed_extrinsic_and_real_intrinsics():
    matrix,horizontal,vertical=calibration(info())
    assert .5<horizontal<.7 and .4<vertical<.6
    t=TransformStamped();t.transform.translation.x=.2825;t.transform.translation.z=.22
    t.transform.rotation.x=-.5;t.transform.rotation.y=.5;t.transform.rotation.z=-.5;t.transform.rotation.w=.5
    rotation,translation=transform(t)
    np.testing.assert_allclose(rotation,[[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
    view=CameraView.__new__(CameraView)
    view.sample=lambda:dict(source_ns=101,rotation=rotation,translation=translation,matrix=matrix,width=640,height=480)
    points=[[2.2825,0.,.22],[2.2825,.1,.22],[2.2825,-.1,.22],[2.2825,0.,.32],[-2.,0.,.22],[2.2825,3.,.22]]
    witness=view.witness(points,100)
    assert witness['after_arrival'] and witness['visible_count']==4 and witness['native_count']==6
    np.testing.assert_allclose(witness['pixels'][0],[320.,240.])
    assert not view.witness(points,101)['after_arrival']


def test_duplicate_image_never_renews_source_receipt():
    view=CameraView.__new__(CameraView);view.streams={'image':OrderedDict()}
    message=Image(width=160,height=120,encoding='rgb8',step=480,data=bytes(160*120*3))
    message.header.stamp.sec=10
    view.receive('image',message)
    first=view.streams['image'][10**10][0]
    time.sleep(.01);view.receive('image',message)
    assert view.streams['image'][10**10][0]==first
    message.header.stamp.sec=9;view.receive('image',message)
    assert len(view.streams['image'])==1


def test_changed_camera_writer_latches_even_after_original_returns():
    view=CameraView.__new__(CameraView);view.topics={'image':'/image','info':'/info'}
    endpoints={'/image':[b'a'],'/info':[b'b'],'/tf':[b'c'],'/tf_static':[b'd']}
    view.node=SimpleNamespace(get_parameter=lambda _:SimpleNamespace(value=False),
        get_publishers_info_by_topic=lambda topic:[SimpleNamespace(endpoint_gid=x) for x in endpoints[topic]])
    view.writers={key:[x.hex() for x in rows] for key,rows in endpoints.items()}
    view.candidates=None;view.fault='';view.discovery_at=0.
    endpoints['/image']=[b'a',b'e'];view.discover(1.)
    assert 'duplicated' in view.fault
    endpoints['/image']=[b'a'];view.discover(2.)
    assert view.fault

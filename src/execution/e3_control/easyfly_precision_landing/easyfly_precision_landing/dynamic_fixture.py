# Copyright 2026 EasyFly Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Explicit opt-in physical gimbal and slow collision-pad simulation fixture."""

import math
import xml.etree.ElementTree as ET


def plugin(parent, name, mode):
    """Give each actual actuator an independent ROS node identity."""
    item = ET.SubElement(parent, 'plugin', name=name, filename='libeasyfly_landing_fixture.so')
    ET.SubElement(item, 'mode').text = mode
    ET.SubElement(ET.SubElement(item, 'ros'), 'namespace').text = '/landing_fixture/'+mode


def camera(model_path):
    """Move only the generated down sensor onto a physical measured revolute joint."""
    tree = ET.parse(model_path)
    model = tree.find('model')
    body = model.find("link[@name='base_link']")
    sensor = body.find("sensor[@name='down_color']")
    if sensor is None or model.find("link[@name='landing_camera']") is not None:
        raise ValueError('requires fresh generated down-camera model')
    body.remove(sensor)
    link = ET.SubElement(model, 'link', name='landing_camera')
    ET.SubElement(link, 'pose').text = '0 0 -0.08 0 0 0'
    ET.SubElement(link, 'gravity').text = 'false'
    inertia = ET.SubElement(link, 'inertial')
    ET.SubElement(inertia, 'mass').text = '0.005'
    tensor = ET.SubElement(inertia, 'inertia')
    for key in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'):
        ET.SubElement(tensor, key).text = '0.00001' if key in ('ixx', 'iyy', 'izz') else '0'
    sensor.find('pose').text = '0 0 0 0 1.5707963267948966 0'
    link.append(sensor)
    joint = ET.SubElement(model, 'joint', name='landing_gimbal_pitch', type='revolute')
    ET.SubElement(joint, 'parent').text = 'base_link'
    ET.SubElement(joint, 'child').text = 'landing_camera'
    axis = ET.SubElement(joint, 'axis')
    ET.SubElement(axis, 'xyz').text = '0 1 0'
    limits = ET.SubElement(axis, 'limit')
    for key, value in [('lower', '-0.35'), ('upper', '0.35'), ('effort', '0.02'),
                       ('velocity', '1.5')]:
        ET.SubElement(limits, key).text = value
    plugin(model, 'landing_gimbal_encoder', 'gimbal')
    tree.write(model_path, encoding='utf-8', xml_declaration=True)


def pad(world):
    """Keep native rendered tag cells, adding a 2m physical moving deck."""
    model = world.find("./world/model[@name='static_tag_7']")
    model.find('static').text = 'false'
    ET.SubElement(model, 'allow_auto_disable').text = 'false'
    # Orthogonal tag tilt keeps planar PnP observable throughout pitch sweep.
    roll = math.radians(8.)
    model.find('pose').text = f'0.15 0 0.04 {roll} 0 0'
    link = model.find('link')
    ET.SubElement(link, 'kinematic').text = 'true'
    ET.SubElement(link, 'gravity').text = 'false'
    inertia = ET.SubElement(link, 'inertial')
    ET.SubElement(inertia, 'mass').text = '100'
    tensor = ET.SubElement(inertia, 'inertia')
    for key in ('ixx', 'iyy', 'izz'):
        ET.SubElement(tensor, key).text = '100'
    for kind in ('collision', 'visual'):
        part = ET.SubElement(link, kind, name='moving_deck')
        # Cancel the model roll for the physical level deck (world top .017m).
        dy, dz = -.033 * math.sin(roll), -.033 * math.cos(roll)
        ET.SubElement(part, 'pose').text = f'0 {dy} {dz} {-roll} 0 0'
        box = ET.SubElement(ET.SubElement(part, 'geometry'), 'box')
        ET.SubElement(box, 'size').text = '2 2 0.02'
        if kind == 'visual':
            material = ET.SubElement(part, 'material')
            ET.SubElement(material, 'ambient').text = '0.25 0.25 0.25 1'
            ET.SubElement(material, 'diffuse').text = '0.25 0.25 0.25 1'
    plugin(model, 'landing_moving_deck', 'pad')


def configure(document):
    """Declare the measured calibration and bounded moving-target contract."""
    transform = document['landing_transform']['ros__parameters']
    transform['dynamic_camera'] = True
    document['landing_camera_mount'] = {'ros__parameters': dict(
        configured=True, joint_name='landing_gimbal_pitch',
        optical_frame=transform['optical_frame'], pivot_translation_flu=[0., 0., -.08],
        optical_offset_flu_at_zero=[0., 0., 0.], joint_axis_flu=[0., 1., 0.],
        camera_quaternion_xyzw_zero=list(transform['camera_quaternion_xyzw']))}
    document['flight_controller']['ros__parameters'].update(
        precision_moving_target=True, precision_moving_max_speed=.06,
        precision_moving_fit_error=.04, precision_moving_land_timeout=8.,
        precision_moving_usable_radius=.65)

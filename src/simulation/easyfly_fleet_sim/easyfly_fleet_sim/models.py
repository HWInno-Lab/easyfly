# Copyright 2026 EasyFly Project
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

"""Derive per-vehicle names and ports while preserving the reviewed Iris physics and sensors."""

import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET


def namespace_plugins(root, vehicle):
    """Bind each ROS plugin node, topic and frame to one explicit fleet member."""
    prefix = '/fleet/' + vehicle
    for plugin in root.iter('plugin'):
        ros = plugin.find('ros')
        if ros is not None:
            namespace = ros.find('namespace')
            if namespace is not None:
                namespace.text = prefix + (namespace.text or '')
            for topic in ros.findall('.//topic'):
                if topic.get('name', '').startswith('/'):
                    topic.set('name', prefix + topic.get('name'))
        for tag in ('frame_name', 'frameName'):
            frame = plugin.find(tag)
            if frame is not None:
                frame.text = vehicle + '/' + frame.text
        topic = plugin.find('ros_topic')
        if topic is not None:
            topic.text = prefix + topic.text
            ET.SubElement(plugin, 'ros_namespace').text = prefix + '/livox'


def derive(model_path, lidar_path, root, vehicle, camera_rate):
    """Write immutable derived inputs into the private session, leaving source models untouched."""
    root = Path(root)
    model = ET.parse(model_path)
    lidar = ET.parse(lidar_path)
    name, slot = vehicle['id'], vehicle['instance']
    namespace_plugins(model.getroot(), name)
    namespace_plugins(lidar.getroot(), name)
    model.getroot().find('model').set('name', name)
    for sensor in model.getroot().iter('sensor'):
        if sensor.get('type') in ('camera', 'depth'):
            sensor.find('update_rate').text = str(camera_rate)
    interface = model.getroot().find("model/plugin[@name='mavlink_interface']")
    for tag, value in [('mavlink_tcp_port', 4560 + slot), ('mavlink_udp_port', 14560 + slot),
                       ('sdk_udp_port', 14540 + slot)]:
        interface.find(tag).text = str(value)
    # Inline the complete MID-360 model so two vehicles cannot share a global ROS plugin name.
    parent = model.getroot().find('model')
    include = next(item for item in parent.findall('include')
                   if item.findtext('uri') == 'model://mid360')
    child = lidar.getroot().find('model')
    child.insert(0, include.find('pose'))
    parent.remove(include)
    parent.append(child)
    target = root / (name + '.sdf')
    model.write(target, encoding='utf-8', xml_declaration=True)
    return {'path': str(target), 'sha256': hashlib.sha256(target.read_bytes()).hexdigest(),
            'sources': {str(Path(p).resolve()): hashlib.sha256(Path(p).read_bytes()).hexdigest()
                        for p in (model_path, lidar_path)},
            'changes': ['per-vehicle ROS namespace and frames', 'per-instance simulator ports',
                        'explicit camera update rate', 'inline complete MID-360 model']}

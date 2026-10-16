"""Pinned indoor resource contracts and upstream defect fixes; never launch flight."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from easyfly_px4_sitl.scene_catalog import SceneCatalog

WORKSPACE = Path(__file__).resolve().parents[5]
MODELS = WORKSPACE / 'src/simulation/easyfly_gazebo_models'
WORLDS = WORKSPACE / 'src/simulation/easyfly_gazebo_worlds'


@pytest.mark.parametrize('kind,count', [('house',63), ('bookstore',26)])
def test_pinned_indoor_files_and_relocatable_closure(kind, count):
    origin = json.loads((MODELS/f'licenses/aws_{kind}/origins.json').read_text())
    assert len(origin['included_models']) == count
    assert origin['declared_license'] == 'MIT-0'
    for item in origin['files']:
        root = WORLDS if item['path'].startswith('worlds/') else MODELS
        path = root / item['path']
        assert hashlib.sha256(path.read_bytes()).hexdigest() == item['sha256']
    catalog = SceneCatalog({'easyfly_gazebo_models':MODELS, 'easyfly_gazebo_worlds':WORLDS})
    report = catalog.validate(f'{kind}_aws_v1').as_dict()
    assert report['scene_resources_complete'], report['issues']
    assert not [issue for issue in report['issues'] if issue['scope'] == 'licenses']
    # Runtime roots are deliberately absent; offline checks never become an
    # assertion of live flight readiness.
    assert not report['runtime_verified']


def test_house_world_keeps_static_layout_and_bookstore_removes_only_outlier():
    house = WORLDS/'worlds/indoor/residential/house_aws_v1.world'
    assert house.read_bytes() == (MODELS/'licenses/aws_house/upstream-small_house.world').read_bytes()
    for model in ET.parse(house).findall('.//world/model'):
        assert model.findtext('static').strip() in ('true','1')
    old = ET.parse(MODELS/'licenses/aws_bookstore/upstream-bookstore.world').getroot()
    new = ET.parse(WORLDS/'worlds/indoor/commercial/bookstore_aws_v1.world').getroot()
    old_world = old.find('world')
    outlier = next(m for m in old_world.findall('model') if m.get('name') == 'BookE_01_009')
    assert float(outlier.findtext('pose').split()[2]) == 343582.718718
    old_world.remove(outlier)
    def semantic(node):
        return node.tag, node.attrib, (node.text or '').strip(), [semantic(c) for c in node]
    assert semantic(old) == semantic(new)

"""Offline catalog contracts; never launch ROS, Gazebo or PX4."""

import copy
import hashlib
from pathlib import Path
import shutil

import pytest
import yaml

from easyfly_sim_launcher.scene_catalog import CatalogError, SceneCatalog


WORKSPACE = Path(__file__).resolve().parents[5]
SOURCE_WORLDS = WORKSPACE / 'src/simulation/easyfly_gazebo_worlds'


def write_yaml(path, data):
    path.write_text(yaml.safe_dump(data, sort_keys=False))


@pytest.fixture
def catalog_data(tmp_path):
    worlds = tmp_path / 'worlds'
    models = tmp_path / 'models'
    for path in (worlds / 'scenes', models / 'manifests'):
        path.mkdir(parents=True)
    shutil.copytree(SOURCE_WORLDS / 'schemas', worlds / 'schemas')
    (worlds / 'LICENSE').write_text('Test license evidence')
    (worlds / 'test.world').write_text('<sdf><world name="test"/></sdf>')
    data = yaml.safe_load((SOURCE_WORLDS / 'scenes/empty.yaml').read_text())
    data.update(id='test', status='registered', supported_profiles=['base_sim'])
    data['world'] = dict(package='easyfly_gazebo_worlds', path='test.world',
                         sha256=hashlib.sha256((worlds / 'test.world').read_bytes()).hexdigest())
    data['provenance']['evidence'] = [{'package':'easyfly_gazebo_worlds', 'path':'LICENSE'}]
    data['navigation']['world_to_navigation'] = dict(translation=[0,0,0], yaw=0)
    data['runtime'] = dict(entrypoint=dict(package='easyfly_gazebo_worlds',path='test.world'),
                           asset_sets=[], required_plugins=[], additional_files=[], notes=[])
    write_yaml(worlds / 'scenes/test.yaml', data)
    shares = dict(easyfly_gazebo_worlds=worlds, easyfly_gazebo_models=models)
    return shares, data


def save(catalog_data):
    shares, data = catalog_data
    write_yaml(shares['easyfly_gazebo_worlds'] / 'scenes/test.yaml', data)
    return SceneCatalog(shares)


def test_positive_offline_is_not_runtime_verification(catalog_data):
    result = save(catalog_data).validate('test').as_dict()
    assert all(result[k] for k in ('scene_resources_complete', 'runtime_resources_complete',
                                  'licenses_complete', 'registration_complete'))
    assert result['runtime_verified'] is False


def test_qualification_dependency_requires_reviewed_px4_feature(catalog_data):
    shares, data = catalog_data
    data['verification']['required_px4_compatibility'] = ['mag_lpf_interval_v1']
    catalog = save(catalog_data)
    report = catalog.validate('test').as_dict()
    assert not report['runtime_resources_complete']
    assert any('mag_lpf_interval_v1' in issue['message'] for issue in report['issues'])
    checked = SceneCatalog(shares, px4_compatibility=['mag_lpf_interval_v1'])
    assert checked.validate('test').as_dict()['runtime_resources_complete']


def test_unknown_scene(catalog_data):
    with pytest.raises(CatalogError, match='unknown scene'):
        save(catalog_data).validate('../../test')


@pytest.mark.parametrize('change', ['nan', 'unknown_field', 'wrong_type', 'absolute', 'traversal'])
def test_invalid_schema_or_numbers(catalog_data, change):
    _, data = catalog_data
    if change == 'nan':
        data['spawn']['yaw'] = float('nan')
    elif change == 'unknown_field':
        data['unexpected'] = True
    elif change == 'wrong_type':
        data['schema_version'] = True
    else:
        data['world']['path'] = '/tmp/a.world' if change == 'absolute' else '../a.world'
    with pytest.raises(CatalogError):
        save(catalog_data)


@pytest.mark.parametrize('tail', ['id: test\n', 'name: &x test\ntags: [*x]\n'])
def test_duplicate_keys_and_aliases_rejected(catalog_data, tail):
    shares, _ = catalog_data
    p = shares['easyfly_gazebo_worlds'] / 'scenes/test.yaml'
    p.write_text(p.read_text() + tail)
    with pytest.raises(CatalogError):
        SceneCatalog(shares)


def test_duplicate_scene_id(catalog_data):
    shares, _ = catalog_data
    scenes = shares['easyfly_gazebo_worlds'] / 'scenes'
    shutil.copyfile(scenes / 'test.yaml', scenes / 'duplicate.yaml')
    with pytest.raises(CatalogError, match='duplicate id'):
        SceneCatalog(shares)


@pytest.mark.parametrize('kind', ['missing', 'hash', 'symlink'])
def test_world_failure(catalog_data, tmp_path, kind):
    shares, data = catalog_data
    p = shares['easyfly_gazebo_worlds'] / 'test.world'
    if kind == 'missing':
        p.unlink()
    elif kind == 'hash':
        p.write_text('<sdf/>')
    else:
        outside = tmp_path / 'outside.world'
        outside.write_bytes(p.read_bytes())
        p.unlink()
        p.symlink_to(outside)
    assert not save(catalog_data).validate('test').as_dict()['scene_resources_complete']


@pytest.mark.parametrize('filename,content', [('image.png', b'not a world'), ('wrong.world', b'<sdf><model/></sdf>')])
def test_world_requires_world_document(catalog_data, filename, content):
    shares, data = catalog_data
    (shares['easyfly_gazebo_worlds'] / filename).write_bytes(content)
    data['world'].update(path=filename, sha256=hashlib.sha256(content).hexdigest())
    assert not save(catalog_data).validate('test').as_dict()['scene_resources_complete']


def test_draft_and_unknown_license(catalog_data):
    _, data = catalog_data
    data['status'] = 'draft'
    data['provenance']['status'] = 'unverified'
    data['navigation']['world_to_navigation'] = None
    result = save(catalog_data).validate('test').as_dict()
    assert result['scene_resources_complete']
    assert not result['licenses_complete']
    assert not result['registration_complete']


def add_model(catalog_data, content, name='box'):
    shares, data = catalog_data
    models = shares['easyfly_gazebo_models']
    directory = models / name
    directory.mkdir(exist_ok=True)
    (directory / 'model.config').write_text('<model><sdf>model.sdf</sdf></model>')
    (directory / 'model.sdf').write_text(content)
    data['asset_sets'] = ['objects']
    world = shares['easyfly_gazebo_worlds'] / 'test.world'
    world.write_text(f'<sdf><world><include><uri>model://{name}</uri></include></world></sdf>')
    data['world']['sha256'] = hashlib.sha256(world.read_bytes()).hexdigest()
    asset = dict(schema_version=1, asset_set_id='objects',
                 model_root=dict(package='easyfly_gazebo_models',path='.'),
                 provenance=copy.deepcopy(data['provenance']),
                 models=[dict(name=name,sdf='model.sdf',provenance=copy.deepcopy(data['provenance']))],
                 files=[dict(path=f'{name}/{f.name}',sha256=hashlib.sha256(f.read_bytes()).hexdigest())
                        for f in directory.iterdir()], local_changes=[],storage='existing',inventory_note='test')
    # Expand shared provenance values to avoid YAML anchors in the test data.
    write_yaml(models / 'manifests/objects.yaml', asset)
    return directory, asset


def test_recursive_model_and_config(catalog_data):
    add_model(catalog_data, '<sdf><model name="box"/></sdf>')
    result = save(catalog_data).validate('test').as_dict()
    assert result['scene_resources_complete']
    assert any(p.endswith('model.config') for p in result['checked_files'])


def test_transitive_missing_model(catalog_data):
    add_model(catalog_data, '<sdf><model><include><uri>model://absent</uri></include></model></sdf>')
    result = save(catalog_data).validate('test').as_dict()
    assert not result['scene_resources_complete']
    assert any('absent' in i['message'] for i in result['issues'])


def test_duplicate_model_across_scene_and_runtime(catalog_data):
    _, asset = add_model(catalog_data, '<sdf/>')
    shares, data = catalog_data
    asset['asset_set_id'] = 'duplicate'
    write_yaml(shares['easyfly_gazebo_models'] / 'manifests/duplicate.yaml', asset)
    data['runtime']['asset_sets'] = ['duplicate']
    result = save(catalog_data).validate('test').as_dict()
    assert not result['runtime_resources_complete']
    assert any('duplicate model name' in i['message'] for i in result['issues'])


@pytest.mark.parametrize('kind', ['changed', 'unlocked'])
def test_model_inventory(catalog_data, kind):
    directory, _ = add_model(catalog_data, '<sdf/>')
    (directory / ('model.sdf' if kind == 'changed' else 'extra.png')).write_text('changed')
    assert not save(catalog_data).validate('test').as_dict()['scene_resources_complete']


def test_remote_uri_not_downloaded(catalog_data):
    add_model(catalog_data, '<sdf><uri>https://example.invalid/model</uri></sdf>')
    result = save(catalog_data).validate('test').as_dict()
    assert any('remote URI' in i['message'] for i in result['issues'])


def refresh_inventory(catalog_data, asset):
    shares, _ = catalog_data
    root = shares['easyfly_gazebo_models']
    asset['files'] = [dict(path=f.relative_to(root).as_posix(),
                          sha256=hashlib.sha256(f.read_bytes()).hexdigest())
                      for f in sorted((root / 'box').rglob('*')) if f.is_file()]
    write_yaml(root / 'manifests/objects.yaml', asset)


@pytest.mark.parametrize('kind', ['dae', 'obj'])
def test_mesh_texture_closure(catalog_data, kind):
    directory, asset = add_model(catalog_data, f'<sdf><uri>model://box/mesh.{kind}</uri></sdf>')
    (directory / 'materials/textures').mkdir(parents=True)
    texture = directory / 'materials/textures/paint.png'
    texture.write_bytes(b'test image placeholder')
    if kind == 'dae':
        (directory / 'mesh.dae').write_text(
            '<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema">'
            '<library_images><image><init_from>paint.png</init_from></image></library_images></COLLADA>')
    else:
        (directory / 'mesh.obj').write_text('mtllib mesh.mtl\n')
        (directory / 'mesh.mtl').write_text('map_Kd paint.png\n')
    refresh_inventory(catalog_data, asset)
    assert save(catalog_data).validate('test').as_dict()['scene_resources_complete']
    texture.unlink()
    refresh_inventory(catalog_data, asset)
    result = save(catalog_data).validate('test').as_dict()
    assert not result['scene_resources_complete']
    assert any('paint.png' in i['message'] for i in result['issues'])


def test_relative_texture_cannot_escape_model(catalog_data):
    directory, asset = add_model(catalog_data, '<sdf><uri>model://box/mesh.dae</uri></sdf>')
    (directory / 'mesh.dae').write_text(
        '<COLLADA><image><init_from>../secret.png</init_from></image></COLLADA>')
    refresh_inventory(catalog_data, asset)
    result = save(catalog_data).validate('test').as_dict()
    assert any('escapes model' in i['message'] for i in result['issues'])


def test_unconfigured_root(catalog_data):
    _, data = catalog_data
    data['runtime']['additional_files'] = [{'external':'not_configured','path':'a'}]
    result = save(catalog_data).validate('test').as_dict()
    assert result['scene_resources_complete']
    assert not result['runtime_resources_complete']


def test_ogre_script_is_not_silently_complete(catalog_data):
    directory, asset = add_model(catalog_data, '<sdf><uri>model://box/a.material</uri></sdf>')
    (directory / 'a.material').write_text('material Test { }')
    refresh_inventory(catalog_data, asset)
    result = save(catalog_data).validate('test').as_dict()
    assert not result['scene_resources_complete']
    assert any('OGRE' in i['message'] for i in result['issues'])


@pytest.mark.parametrize('body,complete', [('technique { pass { diffuse 1 1 1 1 } }', True),
                                         ('technique { pass { vertex_program_ref Foo {} } }', False),
                                         ('technique { pass { texture_unit {\ntexture missing.png\n} } }', False)])
def test_selected_ogre_material(catalog_data, body, complete):
    directory, asset = add_model(catalog_data,
        '<sdf><material><script><uri>model://box/a.material</uri><name>Test/White</name></script></material></sdf>')
    (directory / 'a.material').write_text('material Test/White {' + body + '}')
    refresh_inventory(catalog_data, asset)
    assert save(catalog_data).validate('test').as_dict()['scene_resources_complete'] is complete


def test_name_only_builtin_material_keeps_existing_behavior(catalog_data):
    add_model(catalog_data, '<sdf><material><script><name>Gazebo/Black</name></script></material></sdf>')
    assert save(catalog_data).validate('test').as_dict()['scene_resources_complete']


@pytest.mark.parametrize('fault', [None, 'missing_texture', 'missing_name', 'duplicate',
                                  'advanced', 'remote', 'traversal', 'missing_directory',
                                  'symlink', 'directory_mesh'])
def test_ogre_directory_locations_keep_closed_dependencies(catalog_data, tmp_path, fault):
    content = ('<sdf><material><script>'
               '<uri>model://box/materials/scripts/</uri>'
               '<uri>model://box/materials/textures/</uri>'
               '<name>Pine/Branch</name></script></material></sdf>')
    directory, asset = add_model(catalog_data, content)
    scripts = directory / 'materials/scripts'; scripts.mkdir(parents=True)
    textures = directory / 'materials/textures'; textures.mkdir()
    material = scripts / 'pine.material'
    material.write_text('material Pine/Branch { technique { pass { '
                        'alpha_rejection greater 128\ntexture_unit {\ntexture branch.png\n} } } }')
    (textures / 'branch.png').write_bytes(b'locked test texture')
    if fault == 'missing_texture':
        (textures / 'branch.png').unlink()
    elif fault == 'missing_name':
        material.write_text('material Other {}')
    elif fault == 'duplicate':
        (scripts / 'duplicate.material').write_bytes(material.read_bytes())
    elif fault == 'advanced':
        material.write_text('material Pine/Branch { technique { pass { vertex_program_ref Foo {} } } }')
    elif fault in ('remote', 'traversal', 'missing_directory'):
        replacement = {'remote':'https://example.com/script/', 'traversal':'model://box/../scripts/',
                       'missing_directory':'model://box/absent/'}[fault]
        (directory / 'model.sdf').write_text(content.replace('model://box/materials/scripts/', replacement))
    elif fault == 'symlink':
        outside = tmp_path / 'outside.material'; outside.write_bytes(material.read_bytes())
        material.unlink(); material.symlink_to(outside)
    elif fault == 'directory_mesh':
        (directory / 'model.sdf').write_text(content.replace('</sdf>', '<mesh><uri>model://box/materials/scripts/</uri></mesh></sdf>'))
    refresh_inventory(catalog_data, asset)
    result = save(catalog_data).validate('test').as_dict()
    assert result['scene_resources_complete'] is (fault is None), result['issues']
    if fault is None:
        assert str(material) in result['checked_files']
        assert str(textures / 'branch.png') in result['checked_files']


def test_symlink_install_with_individual_model_files(catalog_data, tmp_path):
    add_model(catalog_data, '<sdf><model name="box"/></sdf>')
    save(catalog_data)
    shares, _ = catalog_data
    source = shares['easyfly_gazebo_models']
    installed = tmp_path / 'installed_models'
    installed.mkdir()
    (installed / 'manifests').symlink_to(source / 'manifests', target_is_directory=True)
    (installed / 'box').mkdir()
    for p in (source / 'box').iterdir():
        (installed / 'box' / p.name).symlink_to(p)
    shares['easyfly_gazebo_models'] = installed
    result = SceneCatalog(shares, trusted_sources=[source]).validate('test').as_dict()
    assert result['scene_resources_complete']


def test_invalid_region(catalog_data):
    _, data = catalog_data
    data['navigation']['recommended_region'] = {'min':[1,0,0], 'max':[0,1,1]}
    assert not save(catalog_data).validate('test').as_dict()['registration_complete']


def test_missing_plugin(catalog_data):
    _, data = catalog_data
    data['required_plugins'] = ['absent.so']
    result = save(catalog_data).validate('test').as_dict()
    assert not result['scene_resources_complete']


def test_symlink_install_requires_explicit_trusted_source(catalog_data, tmp_path):
    shares, _ = catalog_data
    installed = tmp_path / 'install'
    installed.mkdir()
    source = shares['easyfly_gazebo_worlds']
    for path in source.iterdir():
        (installed / path.name).symlink_to(path, target_is_directory=path.is_dir())
    shares['easyfly_gazebo_worlds'] = installed
    assert not SceneCatalog(shares).validate('test').as_dict()['scene_resources_complete']
    result = SceneCatalog(shares, trusted_sources=[source]).validate('test').as_dict()
    assert result['scene_resources_complete']


def test_repository_catalog_loads_without_runtime_roots():
    shares = dict(easyfly_gazebo_worlds=SOURCE_WORLDS,
                  easyfly_gazebo_models=WORKSPACE / 'src/simulation/easyfly_gazebo_models')
    catalog = SceneCatalog(shares)
    assert {'empty','perception_demo','precision_landing','warehouse_aws_v1','warehouse_aws_no_roof_v1'} <= set(catalog.scenes)
    assert catalog.validate('empty').as_dict()['scene_resources_complete']
    result = catalog.validate('perception_demo').as_dict()
    assert not result['scene_resources_complete']
    assert not result['resources_complete']
    assert any('suv' in i['message'] for i in result['issues'])
    assert not result['licenses_complete']

"""Exercise CLI, preflight and both process descriptions without starting processes."""

import importlib.util
import json
from pathlib import Path

import launch.logging
from launch import LaunchContext, LaunchService
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.utilities import perform_substitutions
import pytest

from easyfly_px4_sitl.scene_catalog import CatalogError
from easyfly_px4_sitl.scene_inputs import PREFIX, preflight_action, resolve_scene, simulation_preflight
from easyfly_px4_sitl.scene_environment import source_package
from easyfly_px4_sitl.session_guard import guarded_launch, SessionReservation
from easyfly_sim_launcher.scene_cli import main
from test_scene_catalog import catalog_data, add_model, save
from test_profile_launch import nested_entities


WORKSPACE = Path(__file__).resolve().parents[5]


@pytest.fixture(autouse=True, scope='module')
def temporary_launch_logs(tmp_path_factory):
    previous = launch.logging.launch_config._log_dir
    launch.logging.launch_config.log_dir = str(tmp_path_factory.mktemp('scene_launch_logs'))
    yield
    launch.logging.launch_config._log_dir = previous


@pytest.fixture
def ready_catalog(catalog_data):
    _, data = catalog_data
    add_model(catalog_data, '<sdf><model name="box"/></sdf>')
    data['runtime']['entrypoint'] = {'package':'easyfly_gazebo_models','path':'box/model.sdf'}
    data['runtime']['asset_sets'] = ['objects']
    data['spawn'].update(position=[1.25,-2.5,0.95],yaw=0.4)
    data['supported_profiles'] = ['base_sim','full_perception','navigation_dev','landing_dev']
    return save(catalog_data)


def load_launch(package, filename):
    parent = ('src/simulation/easyfly_px4_sitl' if package == 'easyfly_px4_sitl'
              else 'src/application/a3_mission/easyfly_sim_launcher')
    path = WORKSPACE / parent / 'launch' / filename
    spec = importlib.util.spec_from_file_location(filename.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('command', ['list','validate'])
def test_cli_success_json(ready_catalog, capsys, command, monkeypatch):
    import socket
    monkeypatch.setattr(socket, 'socket', lambda *a, **k: pytest.fail('CLI opened a socket'))
    args = [command] + (['test'] if command == 'validate' else []) + ['--json']
    assert main(args, catalog_factory=lambda **k:(ready_catalog, [])) == 0
    output = json.loads(capsys.readouterr().out)
    assert output['scenes'][0]['validation']['resources_complete']
    assert not output['scenes'][0]['validation']['runtime_verified']


@pytest.mark.parametrize('reason,code', [('draft',1),('missing',1),('license',1),('unknown',2)])
def test_cli_failure_results(ready_catalog, capsys, reason, code):
    scene = ready_catalog.scenes['test']
    if reason == 'draft':
        scene['status'] = 'draft'
    elif reason == 'missing':
        scene['world']['path'] = 'missing.world'
    elif reason == 'license':
        scene['provenance']['status'] = 'unverified'
    assert main(['validate','unknown' if reason == 'unknown' else 'test','--json'],
                catalog_factory=lambda **k:(ready_catalog, [])) == code
    assert json.loads(capsys.readouterr().out)


def test_cli_list_reports_incomplete_without_failing(ready_catalog, capsys):
    ready_catalog.scenes['test']['status'] = 'draft'
    assert main(['list'], catalog_factory=lambda **k:(ready_catalog, ['test missing optional input'])) == 0
    output = capsys.readouterr()
    assert 'draft' in output.out
    assert 'test missing optional input' in output.err


@pytest.mark.parametrize('reason', ['unknown','draft','profile','missing','license','spawn','alignment'])
def test_scene_preflight_rejects(ready_catalog, reason):
    scene = ready_catalog.scenes['test']
    profile, scene_id = 'base_sim', 'test'
    if reason == 'unknown':
        scene_id = 'absent'
    elif reason == 'draft':
        scene['status'] = 'draft'
    elif reason == 'profile':
        profile = 'not_supported'
    elif reason == 'missing':
        scene['world']['path'] = 'missing.world'
    elif reason == 'license':
        scene['provenance']['status'] = 'unverified'
    elif reason == 'spawn':
        scene['spawn']['position'] = None
    else:
        scene['navigation']['world_to_navigation'] = None
    with pytest.raises(CatalogError):
        resolve_scene(ready_catalog, scene_id, profile)


def test_search_root_conflict_even_for_undeclared_model(ready_catalog, tmp_path):
    # Each root exposes all its child model directories. An unregistered sibling
    # must not silently shadow the declared box model.
    root = tmp_path / 'external'
    (root / 'box').mkdir(parents=True)
    (root / 'box/model.config').write_text('<model/>')
    ready_catalog.external['extra'] = root
    asset = dict(ready_catalog.assets['objects'])
    asset.update(asset_set_id='extra', model_root={'external':'extra','path':'.'},models=[],files=[])
    ready_catalog.assets['extra'] = asset
    ready_catalog.scenes['test']['runtime']['asset_sets'].append('extra')
    with pytest.raises(CatalogError, match='duplicate model name'):
        resolve_scene(ready_catalog,'test','base_sim')


def test_scene_and_world_ambiguity_precedes_catalog(ready_catalog):
    context = LaunchContext()
    context.launch_configurations.update(scene='test', world='test.world')
    with pytest.raises(CatalogError, match='mutually exclusive'):
        simulation_preflight(context, px4_dir=Path('/tmp/unused'),default_world='empty.world',
            default_roots=[],default_model='default.sdf',allowed_profiles=('base_sim',),
            catalog_factory=lambda **k:pytest.fail('catalog loaded before conflict rejection'))


@pytest.mark.parametrize('rate', ['1', '5', '0', '6', '-1', '5.0', 'NaN'])
def test_raw_world_heartbeat_is_a_bounded_native_configuration(tmp_path, rate):
    context = LaunchContext()
    context.launch_configurations.update(world='inspection.world', debug_model_path=str(tmp_path),
                                        debug_heartbeat_rate_hz=rate)
    kwargs = dict(px4_dir=tmp_path, default_world='empty.world', default_roots=[],
                  default_model='default.sdf', allowed_profiles=('base_sim',),
                  catalog_factory=lambda **k:pytest.fail('raw world loaded managed catalog'))
    if rate not in ('1', '5'):
        with pytest.raises(CatalogError, match='must be 1 or 5'):
            simulation_preflight(context, **kwargs)
    else:
        for action in simulation_preflight(context, **kwargs):
            action.execute(context)
        assert context.launch_configurations[PREFIX+'heartbeat_rate_hz'] == rate


@pytest.mark.parametrize('missing', ['world', 'debug_model_path'])
def test_raw_heartbeat_requires_explicit_inspection_inputs(tmp_path, missing):
    context = LaunchContext()
    context.launch_configurations.update(world='inspection.world', debug_model_path=str(tmp_path),
                                        debug_heartbeat_rate_hz='5')
    context.launch_configurations.pop(missing)
    with pytest.raises(CatalogError, match='requires explicit world and debug_model_path'):
        simulation_preflight(context, px4_dir=tmp_path, default_world='empty.world',
                             default_roots=[], default_model='default.sdf', allowed_profiles=('base_sim',))


@pytest.mark.parametrize('override', ['model','entrypoint_profile','debug_roots','ground_basis','grid_mode','heartbeat','clock','battery','grid_height'])
def test_managed_scene_rejects_override(ready_catalog, override):
    context = LaunchContext()
    context.launch_configurations.update(scene='test',scene_profile='base_sim')
    if override == 'model':
        context.launch_configurations['model_sdf'] = '/tmp/other.sdf'
    elif override=='entrypoint_profile':
        context.launch_configurations['scene_profile'] = 'navigation_dev'
    elif override=='debug_roots':
        context.launch_configurations['debug_model_path']='/tmp'
    elif override == 'grid_mode':
        context.launch_configurations['grid_footprint_mode']='observed_cells'
    elif override == 'grid_height':
        context.launch_configurations['grid_height_limit_m']='2.9'
    elif override == 'heartbeat':
        context.launch_configurations['debug_heartbeat_rate_hz']='5'
    elif override == 'clock':
        context.launch_configurations['debug_clock_rate_hz']='100'
    elif override == 'battery':
        context.launch_configurations['debug_battery_rate_hz']='5'
    else:
        context.launch_configurations['ground_model_frame']='map'
    with pytest.raises(CatalogError):
        simulation_preflight(context,px4_dir=Path('/tmp/unused'),default_world='empty.world',
            default_roots=[],default_model='default.sdf',allowed_profiles=('base_sim',),
            catalog_factory=lambda **k:(ready_catalog,[]))


@pytest.mark.parametrize('filename,profile', [('px4_sitl.launch.py','base_sim'),
                                           ('perception_sitl.launch.py','navigation_dev')])
@pytest.mark.parametrize('managed', [False, True])
def test_both_entrypoints_effective_scene_inputs(ready_catalog, monkeypatch, filename, profile, managed):
    module = load_launch('easyfly_px4_sitl', filename)
    monkeypatch.setattr(module,'_resolve_px4_dir', lambda:Path('/tmp/nonexecuted-px4'))
    # Inspect descriptions only; native rejection has a separate guard test.
    native_inputs = []
    monkeypatch.setattr(module, 'require_native_imu', native_inputs.append)
    monkeypatch.setattr(module,'preflight_action',lambda **kw:preflight_action(
        catalog_factory=lambda **k:(ready_catalog,[]),**kw))
    context = LaunchContext()
    if managed:
        ready_catalog.scenes['test']['navigation']['ground_model_frame']='map'
        ready_catalog.scenes['test']['navigation']['grid_footprint_mode']='observed_cells'
        ready_catalog.scenes['test']['runtime']['heartbeat_rate_hz']=5
        ready_catalog.scenes['test']['runtime']['clock_rate_hz']=100
        context.launch_configurations.update(scene='test',scene_profile=profile)
    description = module.generate_launch_description()
    assert native_inputs == [Path('/tmp/nonexecuted-px4')]
    guard_index = None
    preflight_index = None
    for index, action in enumerate(description.entities):
        if isinstance(action,DeclareLaunchArgument):
            action.execute(context)
        elif isinstance(action,OpaqueFunction):
            function = action._OpaqueFunction__function
            if function.__module__ == 'easyfly_px4_sitl.scene_inputs':
                preflight_index = index
                for config in action.execute(context):
                    config.execute(context)
            elif function.__module__ == 'easyfly_px4_sitl.session_guard':
                guard_index = index
    assert preflight_index < guard_index
    processes = [a for a in nested_entities(description.entities) if type(a) is ExecuteProcess]
    commands = [[perform_substitutions(context, item) for item in a.cmd] for a in processes]
    spawn = next(cmd for cmd in commands if cmd[:2] == ['gz','model'])
    server = next(cmd for cmd in commands if '--verbose' in cmd)
    if managed:
        assert str(ready_catalog.resolve(ready_catalog.scenes['test']['world'])) in server
        assert spawn[spawn.index('-x')+1] == '1.25'
        assert spawn[spawn.index('-y')+1] == '-2.5'
        assert spawn[spawn.index('-z')+1] == '0.95'
        assert spawn[spawn.index('-Y')+1] == '0.4'
        assert any(v == '--spawn-file=' + str(ready_catalog.resolve(
            ready_catalog.scenes['test']['runtime']['entrypoint'])) for v in spawn)
    else:
        assert any(v.endswith('/empty.world' if profile == 'base_sim' else '/perception_demo.world') for v in server)
        assert spawn[spawn.index('-z')+1] == '0.83'
        assert spawn[spawn.index('-Y')+1] == '0.0'
    assert context.launch_configurations[PREFIX+'model_path']
    assert context.launch_configurations[PREFIX+'ground_model_frame']==('map' if managed else 'odom')
    assert context.launch_configurations[PREFIX+'grid_footprint_mode']==('observed_cells' if managed else 'cluster_boxes')
    assert context.launch_configurations[PREFIX+'heartbeat_rate_hz']==('5' if managed else '1')
    assert context.launch_configurations[PREFIX+'clock_rate_hz']==('100' if managed else '10')
    assert server[-3:] == ['--ros-args','--param','gazebo:publish_rate:='+('100.0' if managed else '10.0')]


def test_profile_scene_forwarding_and_headless(ready_catalog, monkeypatch):
    module = load_launch('easyfly_sim_launcher','profile.launch.py')
    monkeypatch.setattr(module,'installed_catalog',lambda:(ready_catalog,[]))
    context = LaunchContext()
    context.launch_configurations.update(profile='navigation_dev',scene='test',headless='false')
    includes = [a for a in nested_entities(module._launch_profile(context)) if isinstance(a,IncludeLaunchDescription)]
    args = dict(includes[0].launch_arguments)
    assert args['scene'] == 'test'
    assert args['scene_profile'] == 'navigation_dev'
    assert args['headless'] == 'false'
    assert args['local_planner_virtual_ceil_height'] == '3.2'
    assert args['navigation_strategy'] == 'combined'


def test_base_wrapper_forwards_scene_and_checks_before_includes(ready_catalog, monkeypatch):
    module = load_launch('easyfly_sim_launcher','easyfly_sim.launch.py')
    monkeypatch.setattr(module,'installed_catalog',lambda:(ready_catalog,[]))
    context = LaunchContext()
    context.launch_configurations.update(scene='test',headless='true')
    description = module.generate_launch_description()
    for action in description.entities:
        if isinstance(action,DeclareLaunchArgument):
            action.execute(context)
        elif isinstance(action,OpaqueFunction):
            assert action.execute(context) == []
            break
        else:
            pytest.fail('child include appeared before wrapper preflight')
    include = next(a for a in description.entities if isinstance(a,IncludeLaunchDescription))
    arguments = dict(include.launch_arguments)
    assert arguments['scene'].perform(context) == 'test'
    assert arguments['scene_profile'] == 'base_sim'
    assert arguments['gzclient'].perform(context) == 'false'


@pytest.mark.parametrize('configs', [dict(scene='test',world='test.world'),dict(world='/tmp/raw.world'),dict(headless='bad')])
def test_profile_rejects_ambiguity_raw_world_and_bad_headless(ready_catalog, configs):
    module = load_launch('easyfly_sim_launcher','profile.launch.py')
    context = LaunchContext()
    context.launch_configurations.update(profile='base_sim',**configs)
    with pytest.raises(ValueError):
        module._launch_profile(context)


def test_ament_source_metadata_identity(tmp_path):
    source = tmp_path/'correct_package'
    source.mkdir()
    (source/'package.xml').write_text('<package><name>wrong_package</name></package>')
    share = tmp_path/'installed'
    share.mkdir()
    (share/'package.xml').symlink_to(source/'package.xml')
    with pytest.raises(CatalogError,match='identity mismatch'):
        source_package(share,'correct_package')


def test_failed_preflight_never_reserves_or_starts_children(monkeypatch):
    reached = []
    def reject(context):
        raise CatalogError('test scene refused before reservation')
    monkeypatch.setattr(SessionReservation,'acquire',
                        lambda *a,**k:pytest.fail('session acquired before failed preflight'))
    service = LaunchService()
    service.include_launch_description(guarded_launch(
        [OpaqueFunction(function=lambda context: reached.append(True))], {},
        preflight=[OpaqueFunction(function=reject)]))
    assert service.run() == 1
    assert reached == []


@pytest.mark.parametrize('value', ['0 0 0.1', '0 0 nan 0', '0 0 inf 0', '0 0 0.1 yaw'])
def test_raw_inspection_spawn_rejects_bad_pose(value, tmp_path):
    context = LaunchContext()
    context.launch_configurations.update(world='fixture.world', debug_model_path=str(tmp_path),
                                         debug_spawn_pose=value)
    with pytest.raises(CatalogError, match='four finite'):
        simulation_preflight(context, px4_dir=Path('/tmp/unused'), default_world='empty.world',
                             default_roots=[], default_model='unused.sdf', allowed_profiles=('base_sim',))


def test_raw_inspection_spawn_is_explicit_and_cannot_override_managed_scene(tmp_path):
    context = LaunchContext()
    context.launch_configurations['debug_spawn_pose'] = '1.2 -2.3 0.1 0.4'
    kwargs = dict(px4_dir=Path('/tmp/unused'), default_world='empty.world',
                  default_roots=[], default_model='unused.sdf', allowed_profiles=('base_sim',))
    with pytest.raises(CatalogError, match='requires explicit world and debug_model_path'):
        simulation_preflight(context, **kwargs)
    context.launch_configurations.update(world='fixture.world', debug_model_path=str(tmp_path))
    for action in simulation_preflight(context, **kwargs):
        action.execute(context)
    assert [context.launch_configurations[PREFIX + name] for name in ['x','y','z','yaw']] == [
        '1.2','-2.3','0.1','0.4']
    context.launch_configurations.update(scene='test', world='', scene_profile='base_sim',debug_model_path='')
    with pytest.raises(CatalogError, match='managed scene cannot override debug_spawn_pose'):
        simulation_preflight(context, **kwargs)


@pytest.mark.parametrize('rate', ['10', '100', '0', '100.0', 'NaN', '1000'])
def test_raw_clock_is_native_and_requires_reviewed_rate(tmp_path, rate):
    context = LaunchContext()
    context.launch_configurations.update(world='inspection.world', debug_model_path=str(tmp_path),
                                        debug_clock_rate_hz=rate)
    kwargs = dict(px4_dir=tmp_path, default_world='empty.world', default_roots=[],
                  default_model='default.sdf', allowed_profiles=('base_sim',))
    if rate not in ('10', '100'):
        with pytest.raises(CatalogError, match='must be 10 or 100'):
            simulation_preflight(context, **kwargs)
    else:
        for action in simulation_preflight(context, **kwargs):
            action.execute(context)
        assert context.launch_configurations[PREFIX+'clock_rate_hz'] == rate


@pytest.mark.parametrize('missing', ['world', 'debug_model_path'])
def test_raw_clock_requires_explicit_inspection_inputs(tmp_path, missing):
    context = LaunchContext()
    context.launch_configurations.update(world='inspection.world', debug_model_path=str(tmp_path),
                                        debug_clock_rate_hz='100')
    context.launch_configurations.pop(missing)
    with pytest.raises(CatalogError, match='requires explicit world and debug_model_path'):
        simulation_preflight(context, px4_dir=tmp_path, default_world='empty.world',
                             default_roots=[], default_model='default.sdf', allowed_profiles=('base_sim',))


@pytest.mark.parametrize('rate,heartbeat', [('5','5'),('1','5'),('5','1')])
def test_raw_battery_requires_reviewed_native_configuration(tmp_path,rate,heartbeat):
    context=LaunchContext()
    context.launch_configurations.update(world='inspection.world',debug_model_path=str(tmp_path),
        debug_battery_rate_hz=rate,debug_heartbeat_rate_hz=heartbeat)
    kwargs=dict(px4_dir=tmp_path,default_world='empty.world',default_roots=[],
                default_model='default.sdf',allowed_profiles=('base_sim',))
    if rate=='5' and heartbeat=='5':
        for action in simulation_preflight(context,**kwargs):action.execute(context)
        assert context.launch_configurations[PREFIX+'battery_rate_hz']=='5'
    else:
        with pytest.raises(CatalogError,match='debug_battery_rate_hz requires'):
            simulation_preflight(context,**kwargs)


def test_managed_battery_cannot_silently_keep_upstream_heartbeat(ready_catalog):
    ready_catalog.scenes['test']['runtime']['battery_rate_hz']=5
    with pytest.raises(CatalogError,match='requires reviewed 5Hz heartbeat'):
        resolve_scene(ready_catalog,'test','base_sim')

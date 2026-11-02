# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""EXT-05 selection and live diagnostics. No flight processes or goals are started."""
import copy
import importlib.util
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import pytest
from launch import LaunchContext
from launch.actions import GroupAction, IncludeLaunchDescription
from easyfly_params.backends import resolve_selection, launch_values, parse_layers, check_dependencies, SelectionError
from test_ext02_native import Fixture, Producer, DetectionArrayV2, DetectionV2, PixelBoxV2
from easyfly_perception_contracts.backend_status import BackendMonitor
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Header


def load_profile_launch():
    path = Path(get_package_share_directory('easyfly_sim_launcher')) / 'launch/profile.launch.py'
    spec = importlib.util.spec_from_file_location('ext05_profile', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def test_terminal_and_desktop_share_effective_selection():
    from easyfly_connection_manager.backend import command
    options = dict(multi_tracking_backend='sort', multi_tracking_min_hits=5,
                   global_grid_max_cells=123456, combined_grid_unstick_cells=2, provider='cpu')
    profile = dict(mode='sitl', selected_profile='navigation_dev', algorithms=options,
        simulation=dict(scene='warehouse_aws_v1', gazebo_gui=False),
        fcu=dict(transport='udp', bind_host='127.0.0.1', bind_port=14540, remote_host='127.0.0.1', remote_port=14580, target_system=1),
        gcs=dict(enabled=False, host='127.0.0.1', port=14560, bind_port=14558))
    args = command(profile)
    transferred = json.loads(next(a.split(':=',1)[1] for a in args if a.startswith('algorithms_json:=')))
    desktop = resolve_selection('navigation_dev', transferred)
    terminal = resolve_selection('navigation_dev', options)
    assert desktop['selection'] == terminal['selection']
    context = LaunchContext(); context.launch_configurations.update(profile='navigation_dev', algorithms_json=json.dumps(options))
    actions = load_profile_launch()._launch_profile(context)
    group = next(a for a in actions if isinstance(a, GroupAction))
    include = next(a for a in group.get_sub_entities() if isinstance(a, IncludeLaunchDescription))
    actual = dict(include.launch_arguments)
    for key,value in launch_values(terminal).items(): assert actual[key] == value
    assert json.loads(actual['multi_tracking_parameters'])['min_hits'] == 5
    assert json.loads(actual['detection_parameters'])['use_gpu'] is False
    assert actual['navigation_strategy'] == 'combined'


def test_priority_defaults_and_dedicated_guards(tmp_path):
    file_value = {'backends': {'multi_tracking': {'parameters': {'min_hits': 4}}}}
    cli = {'backends': {'multi_tracking': {'parameters': {'min_hits': 6}}}}
    result = resolve_selection('full_perception', {'multi_tracking_min_hits':5}, file_value, cli)
    assert result['selection']['backends']['multi_tracking']['parameters']['min_hits'] == 6
    assert result['provenance']['/backends/multi_tracking/parameters/min_hits'] == 'cli'
    baseline = resolve_selection('navigation_dev')
    assert baseline['selection']['backends']['multi_tracking']['parameters']['iou_threshold'] == .15
    assert baseline['selection']['backends']['combined_grid']['parameters']['max_cells'] == 1000000
    assert not resolve_selection('base_sim')['enabled_roles']
    for text in ['{"backends":{},"backends":{}}', '{"schema_version":NaN}']:
        with pytest.raises(ValueError): parse_layers('', text)
    for patch in [dict(backends=dict(multi_tracking=dict(backend='missing',parameters={}))),
                  dict(platform=dict(multi_tracking=dict(image_width=123))),
                  dict(backends=dict(localization=dict(backend='openvins',parameters={})))] :
        with pytest.raises(Exception): resolve_selection('full_perception', cli_config=patch)
    bad = resolve_selection('full_perception', cli_config={'backends':{'detection':{'parameters':{'model_path':'/missing/ext05/model.onnx'}}}})
    with pytest.raises(SelectionError, match='dependency_missing'): check_dependencies(bad)


def test_live_sort_reports_actual_config_ready_then_stale(tmp_path):
    f = Fixture(tmp_path)
    try:
        resolved = resolve_selection('full_perception', {'multi_tracking_min_hits':1})
        resolved['enabled_roles'] = ['multi_tracking']
        monitor = BackendMonitor(f.node, resolved)
        values = launch_values(resolved)
        # Default installed YAML plus the very same effective mapping produced by the root.
        f.run('easyfly_multi_object_tracking', 'tracking_node', '-p', 'tracker_backend:=sort',
              '-p', 'backend_parameters:=' + json.dumps(values['multi_tracking_parameters']))
        f.until(lambda: monitor.snapshot()['multi_tracking']['created'])
        row = monitor.snapshot()['multi_tracking']
        assert row['active_backend'] == 'sort' and not row['ready']
        assert row['effective_parameters'] == resolved['selection']['backends']['multi_tracking']['parameters']
        publisher = f.node.create_publisher(DetectionArrayV2, '/perception/v2/detections', 10)
        f.until(lambda: publisher.get_subscription_count() > 0); f.spin(.3)
        producer = Producer()
        for _ in range(3):
            header=Header(stamp=f.node.get_clock().now().to_msg(), frame_id='camera_color_optical_frame')
            message=DetectionArrayV2(context=producer.next(header,'front',64,48),backend_id='onnx',active_provider='fixture')
            message.detections=[DetectionV2(box=PixelBoxV2(x1=8.,y1=8.,x2=24.,y2=24.),confidence_valid=True,confidence=.8,class_id=0,class_name='person')]
            publisher.publish(message);f.spin(.1)
        f.until(lambda: monitor.snapshot()['multi_tracking']['ready'])
        f.spin(2.8)
        assert not monitor.snapshot()['multi_tracking']['ready']
        assert monitor.snapshot()['multi_tracking']['phase'] == 'waiting_input'
    finally: f.close()


def test_status_mismatch_ambiguous_writer_and_replay_are_not_ready(tmp_path):
    f = Fixture(tmp_path)
    try:
        resolved=resolve_selection('full_perception'); resolved['enabled_roles']=['multi_tracking']
        monitor=BackendMonitor(f.node,resolved)
        topic='/platform/backends/multi_tracking'
        from rclpy.qos import QoSProfile, DurabilityPolicy
        publisher=f.node.create_publisher(DiagnosticArray,topic,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        f.spin(.4)
        def report(seq,session='fixture-a',params=None):
            fields=dict(schema_version='1',requested_backend='sort',active_backend='sort',created='true',phase='ready',
                provider='not_applicable',effective_parameters=json.dumps(params or resolved['selection']['backends']['multi_tracking']['parameters']),sequence=str(seq))
            publisher.publish(DiagnosticArray(header=Header(stamp=f.node.get_clock().now().to_msg()),status=[DiagnosticStatus(name='multi_tracking',hardware_id=session,message='fixture',values=[KeyValue(key=k,value=v) for k,v in fields.items()])]))
            f.spin(.1)
        report(1);f.until(lambda: monitor.snapshot()['multi_tracking']['ready'])
        bad=copy.deepcopy(resolved['selection']['backends']['multi_tracking']['parameters']);bad['min_hits']=7
        report(2,params=bad);assert monitor.snapshot()['multi_tracking']['phase']=='mismatch'
        report(1);assert not monitor.snapshot()['multi_tracking']['ready']
        report(1,'fixture-b');f.until(lambda: monitor.snapshot()['multi_tracking']['ready'])
        report(100,'fixture-a',bad);assert monitor.snapshot()['multi_tracking']['ready']
        extra=f.node.create_publisher(DiagnosticArray,topic,10);f.spin(.3);report(2,'fixture-b')
        assert not monitor.snapshot()['multi_tracking']['ready']
        f.node.destroy_publisher(extra)
    finally:f.close()


@pytest.mark.parametrize('role,package,executable,selector,parameters', [
    ('detection','easyfly_object_detection','detection_node','backend','backend_parameters'),
    ('single_tracking','easyfly_single_object_tracking','single_object_tracking_node','tracker_backend','backend_parameters'),
    ('global_grid','easyfly_global_planner','global_planner_node','planner_backend','planner_parameters'),
    ('combined_grid','easyfly_global_planner','navigation_coordinator_node','combined_planner_backend','combined_planner_parameters'),
])
def test_each_runtime_reports_the_selection_it_consumed(role,package,executable,selector,parameters,tmp_path):
    f=Fixture(tmp_path)
    try:
        resolved=resolve_selection('navigation_dev',{'provider':'cpu','global_grid_unstick_cells':2,'combined_grid_max_cells':123456})
        resolved['enabled_roles']=[role]
        monitor=BackendMonitor(f.node,resolved)
        choice=resolved['selection']['backends'][role]
        f.run(package,executable,'-p',selector+':='+choice['backend'],
            '-p',parameters+':='+json.dumps(json.dumps(choice['parameters'])))
        f.until(lambda: monitor.snapshot()[role]['created'])
        row=monitor.snapshot()[role]
        assert row['active_backend']==choice['backend']
        assert row['effective_parameters']==choice['parameters']
        assert not row['ready']
        if role=='detection': assert row['provider']=='CPUExecutionProvider'
    finally:f.close()


def test_effective_values_reach_real_leaf_launch_requests():
    from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
    import inspect
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters
    from launch_ros.actions.load_composable_nodes import get_composable_node_load_request
    resolved=resolve_selection('navigation_dev',{'provider':'cpu','multi_tracking_min_hits':7,'global_grid_unstick_cells':3,'combined_grid_max_cells':234567})
    context=LaunchContext();context.launch_configurations.update(launch_values(resolved))
    context.launch_configurations.update(perception_version='2',navigation_strategy='combined')
    def module_at(path):
        spec=importlib.util.spec_from_file_location('ext05_leaf_'+path.stem,path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
    def defaults(description,ctx):
        for entity in description.entities:
            if isinstance(entity,DeclareLaunchArgument):entity.execute(ctx)
    path=Path(get_package_share_directory('easyfly_px4_sitl'))/'launch/perception_sitl.launch.py'
    description=module_at(path).generate_launch_description();defaults(description,context)
    def walk(entities):
        for entity in entities:
            yield entity
            if isinstance(entity,TimerAction):yield from walk(entity.actions)
            elif isinstance(entity,GroupAction):yield from walk(entity.get_sub_entities())
            elif isinstance(entity,OpaqueFunction):
                function=entity._OpaqueFunction__function
                if function.__module__=='easyfly_px4_sitl.session_guard':
                    # Inspect the reservation closure only: never acquire its live lease or execute processes.
                    yield from walk(inspect.getclosurevars(function).nonlocals['actions'])
    leaf_roles={'detection.launch.py':('detection','backend','backend_parameters'),
                'tracking.launch.py':('multi_tracking','tracker_backend','backend_parameters'),
                'single_object_tracking.launch.py':('single_tracking','tracker_backend','backend_parameters')}
    checked=set()
    for include in walk(description.entities):
        if not isinstance(include,IncludeLaunchDescription):continue
        source=perform_substitutions(context,include.launch_description_source._LaunchDescriptionSource__location)
        filename=Path(source).name
        if filename not in leaf_roles and filename!='navigate.launch.py':continue
        child=LaunchContext();child.launch_configurations.update(context.launch_configurations)
        for key,value in include.launch_arguments:
            child.launch_configurations[key]=perform_substitutions(context,normalize_to_list_of_substitutions(value))
        leaf=module_at(Path(source)).generate_launch_description();defaults(leaf,child)
        if filename in leaf_roles:
            role,selector,parameters=leaf_roles[filename]
            if role=='single_tracking':
                node=next(entity for entity in leaf.entities if isinstance(entity,Node))
                data=evaluate_parameters(child,node._Node__parameters)[0]
            else:
                container=next(entity for entity in leaf.entities if hasattr(entity,'descriptions'))
                request=get_composable_node_load_request(container.descriptions[0],child)
                data={p.name:p.value.string_value if p.value.type==4 else p.value.integer_value for p in request.parameters}
            assert data[selector]==resolved['selection']['backends'][role]['backend']
            assert json.loads(data[parameters])==resolved['selection']['backends'][role]['parameters']
            assert data['perception_version']==2
            checked.add(role)
        else:
            for entity in leaf.entities:
                if not isinstance(entity,Node):continue
                name=perform_substitutions(child,normalize_to_list_of_substitutions(entity.node_executable))
                role='global_grid' if name=='global_planner_node' else 'combined_grid' if name=='navigation_coordinator_node' else None
                if not role:continue
                rows=evaluate_parameters(child,entity._Node__parameters)
                data=next(value for value in reversed(rows) if isinstance(value,dict))
                selector='planner_backend' if role=='global_grid' else 'combined_planner_backend'
                parameters='planner_parameters' if role=='global_grid' else 'combined_planner_parameters'
                assert data[selector]==resolved['selection']['backends'][role]['backend']
                assert json.loads(data[parameters])==resolved['selection']['backends'][role]['parameters']
                checked.add(role)
    assert checked==set(leaf_roles[name][0] for name in leaf_roles)|{'global_grid','combined_grid'}



def test_fuel_and_wire_version_keep_the_guarded_route(monkeypatch):
    from easyfly_connection_manager.backend import command
    resolved=resolve_selection('navigation_dev',{'strategy':'gcopter','exploration':'fuel'})
    assert resolved['selection']['backends']['exploration']['backend']=='fuel'
    with pytest.raises(Exception): resolve_selection('navigation_dev',{'strategy':'astar','exploration':'fuel'})
    from easyfly_params.algorithms import fields
    assert dict(next(f for f in fields() if f['key']=='exploration')['choices'])['fuel']
    # No runtime is started: this checks the same opt-in used by the desktop bridge.
    monkeypatch.setenv('EASYFLY_PERCEPTION_VERSION','2')
    profile=dict(mode='sitl',selected_profile='full_perception',algorithms={},simulation=dict(scene='warehouse_aws_v1',gazebo_gui=False),
        fcu=dict(transport='udp',bind_host='127.0.0.1',bind_port=14540,remote_host='127.0.0.1',remote_port=14580,target_system=1),
        gcs=dict(enabled=False,host='127.0.0.1',port=14560,bind_port=14558))
    assert 'perception_version:=2' in command(profile)


def test_installed_selection_cli_is_read_only_and_reports_preflight():
    import subprocess
    from ament_index_python.packages import get_package_prefix
    path=Path(get_package_prefix('easyfly_params'))/'lib/easyfly_params/backend_selection'
    result=subprocess.run([str(path),'--profile','full_perception','--algorithms-json',
        '{"multi_tracking_min_hits":7,"provider":"cpu"}','--check-dependencies'],capture_output=True,text=True,timeout=15)
    assert result.returncode==0,result.stdout+result.stderr
    value=json.loads(result.stdout)
    assert value['selection']['backends']['multi_tracking']['parameters']['min_hits']==7
    assert value['selection']['backends']['detection']['parameters']['use_gpu'] is False
    assert value['preflight']==dict(phase='configured',dependencies_present=True,runtime_ready=False)
    rejected=subprocess.run([str(path),'--profile','full_perception','--backends-json',
        '{"backends":{"multi_tracking":{"backend":"missing","parameters":{}}}}'],capture_output=True,text=True,timeout=15)
    assert rejected.returncode==2
    assert json.loads(rejected.stdout)['error_code']=='unsupported_configuration'

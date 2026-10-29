# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
import hashlib
from pathlib import Path
import pytest
from easyfly_fastdds_compat.runtime import verified_library, check_loaded


def test_modified_binary_is_rejected(tmp_path):
    root=tmp_path/'lib/easyfly_fastdds_compat';root.mkdir(parents=True)
    library=root/'libfastrtps.so.2.6';library.write_bytes(b'original')
    (root/'SHA256').write_text(hashlib.sha256(library.read_bytes()).hexdigest())
    assert verified_library(tmp_path)[0]==library
    library.write_bytes(b'modified')
    with pytest.raises(RuntimeError,match='fingerprint'):verified_library(tmp_path)


def test_preloaded_system_library_is_rejected(tmp_path):
    library=tmp_path/'libfastrtps.so.2.6'
    check_loaded(library,'000-fff r-xp 0 0:0 0 '+str(library))
    with pytest.raises(RuntimeError,match='already loaded'):
        check_loaded(library,'000-fff r-xp 0 0:0 0 /opt/ros/humble/lib/libfastrtps.so.2.6.11')


def test_component_profiles_preserve_input_and_reach_subprocess(tmp_path):
    import os,subprocess,sys,json,xml.etree.ElementTree as ET
    from easyfly_fastdds_compat.runtime import component_profile_environment
    profile=Path(__file__).resolve().parents[1]/'config/component_services.xml'
    original=tmp_path/'caller.xml'
    original.write_text('<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"><participant profile_name="caller" is_default_profile="true"><rtps><useBuiltinTransports>false</useBuiltinTransports></rtps></participant></profiles>')
    raw=original.read_bytes()
    env=component_profile_environment(profile,{'FASTRTPS_DEFAULT_PROFILES_FILE':str(original)})
    result=subprocess.check_output([sys.executable,'-c','import os,json;print(json.dumps({k:os.environ[k] for k in ("FASTRTPS_DEFAULT_PROFILES_FILE","FASTDDS_DEFAULT_PROFILES_FILE")}))'],env={**os.environ,**env},text=True)
    assert json.loads(result)==env
    root=ET.parse(env['FASTRTPS_DEFAULT_PROFILES_FILE']).getroot()
    ns={'d':'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    assert root.find('d:participant',ns).get('profile_name')=='caller'
    assert root.find('d:participant/d:rtps/d:useBuiltinTransports',ns).text=='false'
    assert len(root.findall('d:publisher',ns))==6
    assert original.read_bytes()==raw
    again=component_profile_environment(profile,env)
    assert len(ET.parse(again['FASTRTPS_DEFAULT_PROFILES_FILE']).getroot())==7
    assert Path(again['FASTRTPS_DEFAULT_PROFILES_FILE']).read_bytes()==Path(env['FASTRTPS_DEFAULT_PROFILES_FILE']).read_bytes()


def test_component_profiles_reject_conflicting_user_policy(tmp_path):
    from easyfly_fastdds_compat.runtime import component_profile_environment
    profile=Path(__file__).resolve().parents[1]/'config/component_services.xml'
    original=tmp_path/'conflict.xml'
    original.write_text(profile.read_text().replace('<sec>2</sec>','<sec>9</sec>'))
    with pytest.raises(RuntimeError,match='Conflicting component'):
        component_profile_environment(profile,{'FASTRTPS_DEFAULT_PROFILES_FILE':str(original)})
    with pytest.raises(RuntimeError,match='Conflicting Fast DDS'):
        component_profile_environment(profile,{'FASTRTPS_DEFAULT_PROFILES_FILE':str(original),'FASTDDS_DEFAULT_PROFILES_FILE':str(profile)})


def test_openvins_defaults_preserve_explicit_policy_and_network_scope(tmp_path):
    import xml.etree.ElementTree as ET
    from easyfly_fastdds_compat.runtime import (
        component_profile_environment, openvins_profile_environment)
    config = Path(__file__).resolve().parents[1]/'config'
    profile, component = config/'openvins_simulation.xml', config/'component_services.xml'
    ns = {'d': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    for locality in ('0', '1'):
        environment = {'ROS_LOCALHOST_ONLY': locality}
        # Ordinary activation before selection does not mask the default choice.
        environment.update(component_profile_environment(component, environment))
        selected = openvins_profile_environment(profile, environment, component)
        assert selected
        root = ET.parse(selected['FASTRTPS_DEFAULT_PROFILES_FILE']).getroot()
        assert (root.find('.//d:interfaceWhiteList', ns) is not None) == (locality == '1')
        assert root.find('.//d:segment_size', ns).text == '134217728'
        assert root.find('.//d:port_queue_capacity', ns).text == '4096'
        raw = Path(selected['FASTRTPS_DEFAULT_PROFILES_FILE']).read_bytes()
        assert openvins_profile_environment(profile, selected, component) == {}
        assert Path(selected['FASTRTPS_DEFAULT_PROFILES_FILE']).read_bytes() == raw
    explicit = tmp_path/'explicit.xml'
    explicit.write_text('<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">'
                        '<participant profile_name="caller"/></profiles>')
    assert openvins_profile_environment(profile, {
        'FASTRTPS_DEFAULT_PROFILES_FILE': str(explicit)}, component) == {}


def test_fuel_image_defaults_match_qualified_raw_image_transport():
    import xml.etree.ElementTree as ET
    from easyfly_fastdds_compat.runtime import openvins_profile_environment
    config=Path(__file__).resolve().parents[1]/'config'
    selected=openvins_profile_environment(config/'raw_image_transport.xml',{},config/'component_services.xml')
    root=ET.parse(selected['FASTRTPS_DEFAULT_PROFILES_FILE']).getroot()
    ns={'d':'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    assert root.find('.//d:segment_size',ns).text=='67108864'
    assert root.find('.//d:maxMessageSize',ns).text=='4194304'
    assert root.find('.//d:port_queue_capacity',ns).text=='512'

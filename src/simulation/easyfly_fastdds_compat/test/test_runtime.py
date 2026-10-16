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

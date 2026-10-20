# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Opt-in activation before DDS initialization, without replacing system files."""
import ctypes
import hashlib
import os
import tempfile
import copy
import xml.etree.ElementTree as ET
from pathlib import Path

_HANDLES = []
_PROFILE_DIRECTORIES = []


def component_profile_environment(profile, environment):
    """Preserve the caller's participant/transports; add only named replies."""
    keys = ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE')
    originals = {str(Path(environment[k]).resolve(strict=True)) for k in keys if environment.get(k)}
    if len(originals) > 1:
        raise RuntimeError('Conflicting Fast DDS profile inputs')
    additions = ET.parse(profile).getroot()
    root = ET.parse(next(iter(originals))).getroot() if originals else ET.Element(additions.tag)
    ns = '{http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles}'
    profiles = root if root.tag == ns+'profiles' else root.find(ns+'profiles')
    if profiles is None:
        raise RuntimeError('Fast DDS input has no profiles element')
    for addition in additions:
        name = addition.get('profile_name')
        matches = [v for v in profiles if v.tag == addition.tag and v.get('profile_name') == name]
        if matches:
            # Do not silently replace user policies, even when the name matches.
            def structure(v):
                return (v.tag,dict(v.attrib),(v.text or '').strip(),[structure(c) for c in v])
            if len(matches)!=1 or structure(matches[0])!=structure(addition):
                raise RuntimeError('Conflicting component reply profile: '+name)
        else:
            profiles.append(copy.deepcopy(addition))
    directory = tempfile.TemporaryDirectory(prefix='easyfly-scene-dds-')
    _PROFILE_DIRECTORIES.append(directory)  # Parent lifetime includes child joins.
    destination = Path(directory.name)/'profiles.xml'
    ET.register_namespace('',ns[1:-1])
    ET.ElementTree(root).write(destination,encoding='utf-8',xml_declaration=True)
    return {key:str(destination) for key in keys}


def openvins_profile_environment(profile, environment, component_profile):
    """Use measured simulation defaults only when the caller supplied no DDS policy.

    A previous activation may already contain exactly the component reply rules.
    Any other explicit policy, including a transport, remains the caller's choice.
    """
    keys = ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE')
    originals = {str(Path(environment[k]).resolve(strict=True))
                 for k in keys if environment.get(k)}
    if len(originals) > 1:
        raise RuntimeError('Conflicting Fast DDS profile inputs')
    ns = '{http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles}'
    if originals:
        existing = ET.parse(next(iter(originals))).getroot()
        existing = existing if existing.tag == ns+'profiles' else existing.find(ns+'profiles')
        approved = ET.parse(component_profile).getroot()
        def shape(node):
            return (node.tag, dict(node.attrib), (node.text or '').strip(),
                    [shape(child) for child in node])
        allowed = [shape(child) for child in approved]
        if existing is None or any(shape(child) not in allowed for child in existing):
            return {}  # Never silently replace user XML, even an empty participant.
    root = ET.parse(profile).getroot()
    if environment.get('ROS_LOCALHOST_ONLY') != '1':
        for descriptor in root.findall(ns+'transport_descriptors/'+ns+'transport_descriptor'):
            whitelist = descriptor.find(ns+'interfaceWhiteList')
            if whitelist is not None:
                descriptor.remove(whitelist)
    directory = tempfile.TemporaryDirectory(prefix='easyfly-vio-dds-')
    _PROFILE_DIRECTORIES.append(directory)
    destination = Path(directory.name)/'profiles.xml'
    ET.register_namespace('', ns[1:-1])
    ET.ElementTree(root).write(destination, encoding='utf-8', xml_declaration=True)
    return {key: str(destination) for key in keys}


def verified_library(prefix):
    directory = Path(prefix) / 'lib/easyfly_fastdds_compat'
    library = (directory / 'libfastrtps.so.2.6').resolve(strict=True)
    expected = (directory / 'SHA256').read_text().strip()
    if len(expected) != 64 or hashlib.sha256(library.read_bytes()).hexdigest() != expected:
        raise RuntimeError('Fast DDS compatibility library fingerprint mismatch')
    return library, expected


def check_loaded(library, maps):
    for line in maps.splitlines():
        path = line.split()[-1]
        if Path(path).name.startswith('libfastrtps.so.') and Path(path).resolve() != library:
            raise RuntimeError('Unpatched Fast DDS is already loaded; start a fresh managed simulation process')


def activate(prefix=None, openvins=False):
    if os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp') != 'rmw_fastrtps_cpp':
        raise RuntimeError('Managed scene DDS compatibility requires rmw_fastrtps_cpp')
    if prefix is None:
        from ament_index_python.packages import get_package_prefix
        prefix = get_package_prefix('easyfly_fastdds_compat')
    library, digest = verified_library(prefix)
    check_loaded(library, Path('/proc/self/maps').read_text())
    _HANDLES.append(ctypes.CDLL(str(library), mode=os.RTLD_NOW | os.RTLD_GLOBAL))
    check_loaded(library, Path('/proc/self/maps').read_text())
    profile = Path(prefix) / 'share/easyfly_fastdds_compat/config/component_services.xml'
    selected_environment = dict(os.environ)
    vio_environment = {}
    if openvins:
        vio_profile = Path(prefix) / 'share/easyfly_fastdds_compat/config/openvins_simulation.xml'
        vio_environment = openvins_profile_environment(vio_profile, selected_environment, profile)
        selected_environment.update(vio_environment)
    profile_environment = component_profile_environment(profile, selected_environment)
    os.environ.update(profile_environment)
    paths = os.environ.get('LD_LIBRARY_PATH', '').split(':')
    paths = [str(library.parent)] + [p for p in paths if p and p != str(library.parent)]
    os.environ['LD_LIBRARY_PATH'] = ':'.join(paths)
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    return dict(path=str(library), sha256=digest, implementation='rmw_fastrtps_cpp',
                source_version='2.6.11', openvins_defaults_selected=bool(vio_environment),
                fix='reader_writer_listener_snapshot',
                component_profiles_sha256=hashlib.sha256(profile.read_bytes()).hexdigest(),
                profile_environment=profile_environment)

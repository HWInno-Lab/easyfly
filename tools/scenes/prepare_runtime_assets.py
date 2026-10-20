#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Prepare reviewed, non-conflicting runtime assets from existing local sources."""
import argparse
import copy
import difflib
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

import yaml

PX4_REVISION='54f0455ffcd755534539a7cf33a09a20bf71d29d'
CLASSIC_REVISION='70683dc759cbd3ebccfa78429b564f8b1fb5d149'


def configure_mid360_imu(sensor):
    """Use SDFormat 1.6 axis noises and the actual IMU sampling frame."""
    imu_sensor = sensor.find(".//sensor[@name='imu_sensor']")
    imu = imu_sensor.find('imu')
    imu.clear()
    # Per-sample sigma in rad/s and m/s^2, not continuous-time densities.
    # Bias is sampled once at initialization; no dynamic bias diffusion is configured.
    for quantity, sigma, bias in [('angular_velocity', '0.00017', '0.00001'),
                                  ('linear_acceleration', '0.017', '0.001')]:
        field = ET.SubElement(imu, quantity)
        for axis in 'xyz':
            noise = ET.SubElement(ET.SubElement(field, axis), 'noise', type='gaussian')
            for key, value in [('mean', '0.0'), ('stddev', sigma),
                               ('bias_mean', '0.0'), ('bias_stddev', bias)]:
                ET.SubElement(noise, key).text = value
    plugin = imu_sensor.find("plugin[@name='imu_plugin']")
    plugin.find('frame_name').text = 'livox_imu_frame'
    plugin.set('filename', 'libeasyfly_gazebo_ros_imu.so')
    legacy_rate = plugin.find('updateRateHZ')
    if legacy_rate is not None:
        plugin.remove(legacy_rate)


class PlainDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def write_yaml(path,data):
    path.write_text(yaml.dump(data,Dumper=PlainDumper,allow_unicode=True,sort_keys=False))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def provenance(source,revision,license,evidence,note):
    return dict(status='recorded',source_url=source,revision=revision,declared_license=license,
                evidence=evidence,note=note)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-source',type=Path,required=True)
    parser.add_argument('--workspace',type=Path,default=Path(__file__).resolve().parents[2])
    args=parser.parse_args()
    ws=args.workspace.resolve(strict=True)
    px4=args.px4_source.resolve(strict=True)
    classic=px4/'Tools/simulation/gazebo-classic/sitl_gazebo-classic'
    for root,revision in [(px4,PX4_REVISION),(classic,CLASSIC_REVISION)]:
        actual=subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip()
        if actual!=revision:
            raise ValueError('unreviewed upstream revision: '+actual)
    models=ws/'src/simulation/easyfly_gazebo_models'
    worlds=ws/'src/simulation/easyfly_gazebo_worlds'
    airframe=ws/'src/simulation/easyfly_px4_sitl/models'
    runtime=models/'easyfly/runtime_models'
    for name in ['mid360','gps']:
        if (runtime/name).exists():
            raise ValueError('refusing to overwrite existing curated model: '+name)
    evidence=models/'licenses/runtime'
    evidence.mkdir(parents=True,exist_ok=True)
    shutil.copyfile(px4/'LICENSE',evidence/'PX4-LICENSE')
    shutil.copyfile(classic/'package.xml',evidence/'PX4-CLASSIC-package.xml')
    shutil.copyfile(classic/'models/iris/model.config',evidence/'PX4-IRIS-model.config')
    shutil.copyfile(classic/'models/gps/model.config',evidence/'PX4-GPS-model.config')
    shutil.copyfile(ws/'LICENSE',evidence/'PROJECT-LICENSE')
    shutil.copyfile('/usr/share/doc/gazebo-common/copyright',evidence/'GAZEBO-COPYRIGHT')
    for name in ['iris.stl','iris_prop_ccw.dae','iris_prop_cw.dae']:
        local=airframe/'easyfly_iris/meshes'/name
        upstream=classic/'models/iris/meshes'/name
        if digest(local)!=digest(upstream):
            raise ValueError('Iris mesh is not the recorded upstream bytes: '+name)
    shutil.copytree(classic/'models/gps',runtime/'gps')
    sensor=ET.parse(models/'mid360/mid360.sdf')
    configure_mid360_imu(sensor)
    visual=sensor.find('.//visual/geometry')
    visual.clear()
    cylinder=ET.SubElement(visual,'cylinder')
    ET.SubElement(cylinder,'radius').text='0.025'
    ET.SubElement(cylinder,'length').text='0.04'
    (runtime/'mid360').mkdir(parents=True)
    ET.indent(sensor,space='  ')
    (runtime/'mid360/mid360.sdf').write_text(
        '<?xml version="1.0"?>\n<!-- Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0 -->\n'+
        ET.tostring(sensor.getroot(),encoding='unicode')+'\n')
    (runtime/'mid360/model.config').write_text('''<?xml version="1.0"?>
<model><name>mid360</name><version>1.0</version><sdf version="1.6">mid360.sdf</sdf>
<author><name>EasyFly Team</name></author>
<description>Project sensor configuration; self-authored primitive visual, no vendor mesh.</description></model>
''')
    loc=lambda p:dict(package='easyfly_gazebo_models',path='licenses/runtime/'+p)
    source='https://github.com/PX4/PX4-SITL_gazebo-classic'
    iris_prov=provenance(source,CLASSIC_REVISION,'BSD-3-Clause AND Apache-2.0',
        [loc('PX4-LICENSE'),loc('PX4-CLASSIC-package.xml'),loc('PROJECT-LICENSE'),loc('ORIGINS.json')],
        'PX4 Iris meshes byte-identical to pinned source; retained PX4 notice and package BSD declaration. '+
        'Iris attribution also corroborated by PegasusSimulator docs/licenses/assets/iris-license.rst. '+
        'EasyFly sensor/physics modifications remain in maintained aircraft SDF; no parameter changed by this curation.')
    gps_prov=provenance(source,CLASSIC_REVISION,'BSD',
        [loc('PX4-LICENSE'),loc('PX4-CLASSIC-package.xml'),loc('ORIGINS.json')],
        'Unmodified GPS files from pinned Classic package declaring BSD; retain PX4 BSD-3-Clause notice; author Nuno Marques.')
    sensor_prov=provenance(None,'easyfly-mid360-primitive-v1','Apache-2.0',[loc('PROJECT-LICENSE'),loc('ORIGINS.json')],
        'Project sensor configuration, self-authored cylinder visual; SDFormat axis IMU noise and sampling frame corrected; physical poses/mass/inertia retained; vendor mesh excluded.')
    gazebo_prov=provenance('https://packages.ubuntu.com/jammy/gazebo-common','11.10.2+dfsg-1','Apache-2.0',
        [loc('GAZEBO-COPYRIGHT'),loc('ORIGINS.json')],
        'Installed Gazebo sun/ground_plane and selected gazebo.material; package Files:* declaration applies, no matching exception.')
    origins=dict(px4_revision=PX4_REVISION,classic_revision=CLASSIC_REVISION,
        iris_authors=['Lorenz Meier','Thomas Gubler'],gps_author='Nuno Marques',
        iris_license_corroboration='https://raw.githubusercontent.com/PegasusSimulator/PegasusSimulator/main/docs/licenses/assets/iris-license.rst',
        iris_meshes=[dict(path='models/iris/meshes/'+n,sha256=digest(classic/'models/iris/meshes'/n))
                     for n in ['iris.stl','iris_prop_ccw.dae','iris_prop_cw.dae']],
        gps_files=[dict(path=p.relative_to(classic).as_posix(),sha256=digest(p)) for p in sorted((classic/'models/gps').rglob('*')) if p.is_file()],
        original_mid360_sdf_sha256=digest(models/'mid360/mid360.sdf'),curated_mid360_sdf_sha256=digest(runtime/'mid360/mid360.sdf'),
        mid360_change='Visual geometry replaced by a project primitive; IMU uses SDFormat axis noise, livox_imu_frame and the coherent measurement-snapshot plugin; physical poses/mass/inertia retained; no original mesh imported.',
        gazebo_package='gazebo-common=11.10.2+dfsg-1',
        gazebo_material_sha256=digest(Path('/usr/share/gazebo-11/media/materials/scripts/gazebo.material')))
    (evidence/'ORIGINS.json').write_text(json.dumps(origins,indent=2)+'\n')
    template=classic/'models/iris/iris.sdf.jinja'
    (evidence/'iris-template-to-easyfly.diff').write_text(''.join(difflib.unified_diff(
        template.read_text().splitlines(keepends=True),(airframe/'easyfly_iris/easyfly_iris.sdf').read_text().splitlines(keepends=True),
        fromfile='PX4/models/iris/iris.sdf.jinja',tofile='EasyFly/easyfly_iris.sdf')))
    def manifest(id,root,folder,entries,prov,storage='existing',changes=None):
        files=[]
        for name,sdf,item_prov in entries:
            for path in sorted((folder/name).rglob('*')):
                if path.is_file():
                    files.append(dict(path=path.relative_to(folder).as_posix(),sha256=digest(path)))
        write_yaml(models/'manifests'/f'{id}.yaml',dict(schema_version=1,asset_set_id=id,model_root=root,
            provenance=prov,models=[dict(name=n,sdf=s,provenance=p) for n,s,p in entries],files=files,
            local_changes=changes or [],storage=storage,inventory_note='S2 reviewed fixed local snapshot; see retained notices and ORIGINS.json.'))
    manifest('easyfly_airframe_v1',dict(package='easyfly_px4_sitl',path='models'),airframe,
             [('easyfly_iris','easyfly_iris.sdf',iris_prov)],iris_prov)
    mixed=provenance(source,CLASSIC_REVISION,'BSD AND Apache-2.0',
                     [loc('PX4-LICENSE'),loc('PX4-CLASSIC-package.xml'),loc('PROJECT-LICENSE'),loc('ORIGINS.json')],
                     'Curated GPS and project MID360 only; no sun/ground_plane and no vendor lidar mesh.')
    manifest('easyfly_runtime_v1',dict(package='easyfly_gazebo_models',path='easyfly/runtime_models'),runtime,
             [('gps','gps.sdf',gps_prov),('mid360','mid360.sdf',sensor_prov)],mixed,'vendored',
             ['MID360 visual mesh replaced with own cylinder; physical poses, mass and inertia retained.',
              'VIO correction: SDFormat per-axis IMU noise and livox_imu_frame; removed unused plugin updateRateHZ.'])
    manifest('gazebo_builtin_v1',dict(external='gazebo_share',path='models'),Path('/usr/share/gazebo-11/models'),
             [('sun','model.sdf',gazebo_prov),('ground_plane','model.sdf',gazebo_prov)],gazebo_prov)
    aws=json.loads((models/'licenses/aws_warehouse/origins.json').read_text())
    aws_prov=provenance(aws['source_url'],aws['revision'],'MIT-0',
        [dict(package='easyfly_gazebo_models',path='licenses/aws_warehouse/LICENSE'),
         dict(package='easyfly_gazebo_models',path='licenses/aws_warehouse/origins.json')],
        'Only the selected world dependency closure from the pinned official archive; no external model references found.')
    manifest('aws_warehouse_v1',dict(package='easyfly_gazebo_models',path='third_party/aws/warehouse'),
             models/'third_party/aws/warehouse',[(n,'model.sdf',aws_prov) for n in aws['included_models']],aws_prov,'vendored',
             ['Model SDF file://models/ references normalized to model://; meshes, textures, collisions and poses unchanged.'])
    scene=yaml.safe_load((worlds/'scenes/empty.yaml').read_text())
    scene.update(id='warehouse_aws_v1',name='AWS 小型仓库',status='draft',category='indoor/warehouse',
        tags=['warehouse','navigation','inspection','static'],
        world=dict(package='easyfly_gazebo_worlds',path='worlds/indoor/warehouse/warehouse_aws_v1.world',
                   sha256=digest(worlds/'worlds/indoor/warehouse/warehouse_aws_v1.world')),
        provenance=aws_prov,asset_sets=['aws_warehouse_v1'],legacy_profiles=[],supported_profiles=[])
    scene['spawn'].update(position=[0.0,0.0,0.83],yaw=0.0,
        basis='Candidate central aisle; verify collision clearance, floor and odom alignment in S2 bootstrap before registration.')
    scene['runtime']['asset_sets']=['easyfly_airframe_v1','easyfly_runtime_v1','gazebo_builtin_v1']
    scene['navigation']['ground_model_frame']='map'
    scene['runtime']['notes']=['Curated GPS and primitive lidar visual; sensor and aircraft physics unchanged.',
        'No full PX4 model root and no legacy scene model root are exposed.']
    scene['verification']=dict(resources_complete=False,world_loaded=False,sensors_checked=False,navigation_tested=False,evidence_refs=[])
    write_yaml(worlds/'scenes/warehouse_aws_v1.yaml',scene)
    open_scene=copy.deepcopy(scene)
    open_scene.update(id='warehouse_aws_no_roof_v1',name='AWS 小型仓库（上游无屋顶版本）')
    path='worlds/indoor/warehouse/warehouse_aws_no_roof_v1.world'
    open_scene['world'].update(path=path,sha256=digest(worlds/path))
    open_scene['provenance']['note']+=' Official no_roof_small_warehouse.world variant, unchanged upstream bytes; separate ID from full-roof world.'
    write_yaml(worlds/'scenes/warehouse_aws_no_roof_v1.yaml',open_scene)
    print('curated runtime manifests and two draft warehouse scenes created')


if __name__=='__main__':
    main()

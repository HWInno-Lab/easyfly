#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Reproduce eight hospital resource repairs from the original reviewed archives.

The original assets/world/evidence remain unchanged. Explicit --prepare-cache
prepares three new versioned assets; no downloads or upstream code execution.
"""
import argparse
import copy
import json
from pathlib import Path
import sys
import tarfile
import zipfile

from import_aws_hospital import ROOT, MODELS, WORLDS, reviewed_outputs, sha, encoded, yaml_bytes


def repaired_outputs(upstream):
    original_outputs, originals = reviewed_outputs(upstream)
    source_assets = {a['asset_set_id']: (a, path) for a, path in originals}
    aws, aws_path = source_assets['aws_hospital_v1']
    with tarfile.open(aws_path) as archive:
        raw_aws = {e.name.split('/', 1)[1]: archive.extractfile(e).read()
                   for e in archive if e.isfile()}
    assets, records = [], []

    def update(asset, member, before, after, old, new):
        if before.count(old.encode()) != 1 or before.replace(old.encode(), new.encode()) != after:
            raise ValueError('reviewed single literal repair required')
        member.setdefault('replacements', []).append(dict(old=old, new=new, count=1))
        target = next(f for f in asset['files'] if f['path'] == member['target'])
        target['sha256'] = sha(after)
        records.append(dict(asset_set=asset['asset_set_id'], source=member['source'],
                            target=member['target'], source_sha256=sha(before),
                            sha256=sha(after), replacements=member['replacements']))

    aws = copy.deepcopy(aws)
    aws['asset_set_id'] = 'aws_hospital_repaired_v1'
    for portrait in ['PortraitA_01', 'PortraitD_01', 'PortraitD_02', 'PortraitE_01', 'PortraitE_02']:
        model = 'aws_robomaker_residential_' + portrait
        member = next(m for m in aws['archive']['members']
                      if m['target'].startswith(model + '/') and m['target'].endswith('_visual.DAE'))
        before = raw_aws[member['source']]
        old = '../../../../photos/' + portrait + '.jpg'
        new = '../materials/textures/' + portrait + '.jpg'
        update(aws, member, before, before.replace(old.encode(), new.encode()), old, new)
        source, target = 'photos/' + portrait + '.jpg', model + '/materials/textures/' + portrait + '.jpg'
        data = raw_aws[source]
        aws['archive']['members'].append(dict(source=source, target=target, source_sha256=sha(data), replacements=[]))
        aws['files'].append(dict(path=target, sha256=sha(data)))
        records.append(dict(asset_set=aws['asset_set_id'], source=source, target=target,
                            source_sha256=sha(data), sha256=sha(data), replacements=[]))
    aws['local_changes'].append('Copy five original archive JPEGs into their owning models and relocate five DAE image URIs; geometry arrays unchanged.')
    assets.append((aws, aws_path, {}))

    for name, old, new in [('BedTable', 'BedTable_Normal_Normal_Bump.tga', 'BedTable_Normal.png'),
                           ('BPCart', 'BP_Normal_Normal_Bump.tga', 'BP_BP_LP_Normal.png')]:
        asset, path = source_assets['fuel_hospital_' + name.lower() + '_v2']
        asset = copy.deepcopy(asset)
        asset['asset_set_id'] = 'fuel_hospital_' + name.lower() + '_repaired_v1'
        source = 'meshes/BedTable.mtl' if name == 'BedTable' else 'meshes/Cart_BP.mtl'
        member = next(m for m in asset['archive']['members'] if m['source'] == source)
        with zipfile.ZipFile(path) as archive:
            before = archive.read(source)
            archive.read('meshes/' + new)  # Candidate really exists in this exact original ZIP.
        update(asset, member, before, before.replace(old.encode(), new.encode()), old, new)
        asset['local_changes'] = ['Repair one MTL normal-map filename to the supplied same-model PNG; original image/OBJ/SDF bytes retained.']
        asset['provenance']['note'] = 'Derived resource repair. One MTL path changed; original license/author evidence retained. See hospital_repaired attribution/origins.'
        supplements = {}
        if name == 'BPCart':
            donor, donor_path = source_assets['fuel_hospital_cgmclassic_v4']
            donor_spec = copy.deepcopy(donor['archive'])
            filename = 'meshes/Wheels_Normal_Normal_Bump.tga'
            with zipfile.ZipFile(donor_path) as archive:
                pixels = archive.read(filename)
                donor_atlas = archive.read('meshes/Carts_Wheels_Diffuse.png')
            with zipfile.ZipFile(path) as archive:
                if archive.read('meshes/Carts_Wheels_Diffuse.png') != donor_atlas:
                    raise ValueError('reviewed shared wheel diffuse atlas differs')
            target = name + '/' + filename
            donor_spec['members'] = [dict(source=filename, target=target, source_sha256=sha(pixels), replacements=[])]
            asset['archive']['supplements'] = [dict(id='cgmclassic_v4', archive=donor_spec)]
            asset['files'].append(dict(path=target, sha256=sha(pixels)))
            asset['local_changes'].append('Copy the exact wheel normal TGA from independently attributed CGMClassic v4; wheel diffuse atlas is byte-identical in both models.')
            asset['provenance']['evidence'].append(dict(package='easyfly_gazebo_models', path='licenses/hospital_fuel/metadata/CGMClassic.json'))
            records.append(dict(asset_set=asset['asset_set_id'], source_archive='CGMClassic v4',
                                archive_sha256=donor_spec['sha256'], declared_license='CC-BY-4.0',
                                source=filename, target=target, source_sha256=sha(pixels),
                                sha256=sha(pixels), replacements=[], wheel_diffuse_atlas_sha256=sha(donor_atlas)))
            supplements['cgmclassic_v4'] = donor_path
        assets.append((asset, path, supplements))

    outputs = {}
    provenance_records = []
    for asset, _, _ in assets:
        for prov in [asset['provenance'], *[m['provenance'] for m in asset['models']]]:
            for suffix in ['ATTRIBUTION.md', 'origins.json']:
                location = dict(package='easyfly_gazebo_models', path='licenses/hospital_repaired/' + suffix)
                if location not in prov['evidence']:
                    prov['evidence'].append(location)
        asset['files'].sort(key=lambda f: f['path'])
        asset['archive']['members'].sort(key=lambda f: (f['source'], f['target']))
        asset['inventory_note'] = 'Derived resource-only repairs; immutable original archives and each source notice retained. No OBJ/collision geometry, SDF physics, texture pixels or original asset versions modified.'
        outputs[MODELS + 'manifests/' + asset['asset_set_id'] + '.yaml'] = yaml_bytes(asset)
        provenance_records.append(dict(asset_set=asset['asset_set_id'], provenance=asset['provenance'],
                                       source_archive_sha256=asset['archive']['sha256'],
                                       changes=asset['local_changes'], files=asset['files']))
    attribution = '''# Hospital resource repairs

AWS hospital models and the five original portrait JPEGs: Amazon.com, Inc. or
its affiliates, MIT-0. Original LICENSE and NOTICE remain in aws_hospital/.
Five DAE image references now point to copies of those original JPEGs within
their own model. The six prior description-only XML escapes remain recorded.

BedTable v2: Cole Biesemeyer; BPCart v2: publisher OpenRobotics (model author
not specified); CGMClassic v4: Roselle Carmen. All three publisher declarations
are CC-BY-4.0: https://creativecommons.org/licenses/by/4.0/ . Exact original
metadata, authors and descriptions remain in hospital_fuel/metadata/ and
hospital_fuel/origins.json; source URLs and versioned archives are locked in
each manifest. No endorsement is implied.

BedTable and BPCart each change one MTL filename to a supplied normal-map PNG.
BPCart additionally copies the original Wheels_Normal_Normal_Bump.tga from
CGMClassic v4. Both models supply the byte-identical Carts_Wheels_Diffuse.png
atlas, supporting this explicit shared-wheel material repair. The normal maps
were visually inspected and their dimensions/RGB statistics recorded.
The donor ZIP and its independent publisher notice stay in the composed cache.
This is an EasyFly material repair, not a claim of the original author's intent
or bit-identical original rendering. No texture pixels or mesh/UV arrays changed.
'''
    outputs[MODELS + 'licenses/hospital_repaired/ATTRIBUTION.md'] = attribution.encode()
    outputs[MODELS + 'licenses/hospital_repaired/origins.json'] = encoded(dict(
        assets=provenance_records, resource_changes=records,
        original_source_lock_sha256=sha((Path(__file__).with_name('hospital-source-lock.json')).read_bytes())))
    proof = dict(source_scene='hospital_aws_v1', repaired_asset_sets=[a['asset_set_id'] for a, _, _ in assets],
                 addressed_issues=8, original_asset_versions_preserved=True,
                 model_geometry_or_texture_pixels_changed=False, world_placements_changed=False,
                 changes=records, world_loaded=False, sensors_checked=False, navigation_tested=False)
    outputs[WORLDS + 'evidence/hospital_aws_v2/resource-repair-source.json'] = encoded(proof)
    return outputs, assets


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('upstream', type=Path)
    parser.add_argument('--workspace', type=Path, default=ROOT)
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--prepare-cache', action='store_true')
    parser.add_argument('--cache-root', type=Path)
    args = parser.parse_args()
    outputs, assets = repaired_outputs(args.upstream)
    for name, data in outputs.items():
        target = args.workspace / name
        if args.check:
            if target.read_bytes() != data:
                raise ValueError('recorded repair differs: ' + name)
        elif target.exists():
            raise ValueError('refusing overwrite: ' + name)
    if not args.check:
        for name, data in outputs.items():
            target = args.workspace / name
            target.parent.mkdir(parents=True, exist_ok=True)
            with target.open('xb') as stream:
                stream.write(data)
    results = []
    if args.prepare_cache:
        sys.path.insert(0, str(ROOT / 'src/simulation/easyfly_px4_sitl'))
        from easyfly_px4_sitl.asset_cache import default_root, fetch_asset
        for asset, path, supplements in assets:
            results.append(fetch_asset(asset, args.cache_root or default_root(), archive_path=path,
                                       supplement_paths=supplements))
    print(json.dumps(dict(repaired_resources=8, generated_files=len(outputs), check=args.check,
                          prepared_assets=results), indent=2))


if __name__ == '__main__':
    main()

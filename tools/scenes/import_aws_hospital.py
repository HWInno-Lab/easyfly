#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Reproduce the reviewed hospital dependency catalog from local locked sources.

No upstream code is executed and no network is used. --prepare-cache explicitly
prepares all 44 archives using the same atomic cache implementation as the CLI.
The original world remains a draft: this is not a geometry or flight acceptance.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path, PurePosixPath
import stat
import sys
import tarfile
import xml.etree.ElementTree as ET
import zipfile

import yaml

ROOT = Path(__file__).resolve().parents[2]
MODELS = 'src/simulation/easyfly_gazebo_models/'
WORLDS = 'src/simulation/easyfly_gazebo_worlds/'
SCENE = 'hospital_aws_v1'
AWS_ASSET = 'aws_hospital_v1'


class Dumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def sha(data):
    return hashlib.sha256(data).hexdigest()


def encoded(value):
    return (json.dumps(value, indent=2, ensure_ascii=False) + '\n').encode()


def yaml_bytes(value):
    return yaml.dump(value, Dumper=Dumper, sort_keys=False, allow_unicode=True).encode()


def locked(path, spec):
    data = path.read_bytes()
    if sha(data) != spec['sha256'] or len(data) != spec['size_bytes']:
        raise ValueError('reviewed source bytes required: ' + str(path))
    return data


def safe_name(name):
    path = PurePosixPath(name.rstrip('/'))
    if (path.is_absolute() or any(p in ('', '.', '..') for p in name.rstrip('/').split('/'))
            or any(c in name for c in ':\\%\x00')):
        raise ValueError('noncanonical source member: ' + name)
    return path.as_posix()


def provenance(url, revision, license_id, paths, note):
    return dict(status='recorded', source_url=url, revision=revision, declared_license=license_id,
                evidence=[dict(package='easyfly_gazebo_models', path=p) for p in paths], note=note)


def asset_record(asset_id, prov, models, inventory, spec, changes):
    return dict(schema_version=1, asset_set_id=asset_id,
                model_root=dict(external='asset_cache', path='models'), provenance=prov,
                models=[dict(name=n, sdf='model.sdf', provenance=prov) for n in sorted(models)],
                files=inventory, storage='cached', local_changes=changes,
                inventory_note='Only locked regular files enter the external cache. '
                               'World geometry/navigation qualification is independent.', archive=spec)


def reviewed_outputs(upstream):
    lock = json.loads((Path(__file__).with_name('hospital-source-lock.json')).read_text())
    revision = lock['revision']
    archive_path = upstream / 'hospital.tar.gz'
    locked(archive_path, lock['archive'])
    prefix = 'aws-robomaker-hospital-world-' + revision
    with tarfile.open(archive_path) as archive:
        entries = archive.getmembers()
        files, seen = {}, set()
        for entry in entries:
            name = safe_name(entry.name)
            if name in seen or not (entry.isfile() or entry.isdir()):
                raise ValueError('unsupported/duplicate source entry: ' + name)
            seen.add(name)
            if name != prefix and not name.startswith(prefix + '/'):
                raise ValueError('source archive prefix differs')
            if entry.isfile():
                files[name[len(prefix) + 1:]] = archive.extractfile(entry).read()
    world = files['worlds/hospital.world']
    document = ET.fromstring(world)
    if list(document.iter('plugin')):
        raise ValueError('unreviewed world plugin')
    all_models = {name.split('/')[1] for name in files if name.startswith('models/')}
    references = [e.text.removeprefix('model://') for e in document.iter('uri')]
    if any('/' in name or not name for name in references):
        raise ValueError('unreviewed world URI')
    local = set(references) & all_models
    external = set(references) - all_models - {'sun', 'ground_plane'}
    if len(local) != 26 or external != {r['name'] for r in lock['fuel_models']} or len(external) != 43:
        raise ValueError('reviewed hospital model closure differs')
    prov = provenance(lock['source_repository'], revision, 'MIT-0', [
        'licenses/aws_hospital/LICENSE', 'licenses/aws_hospital/NOTICE',
        'licenses/aws_hospital/origins.json'],
        'AWS repository content only; does not license the 43 separately recorded Fuel models.')
    outputs = {MODELS + 'licenses/aws_hospital/' + n: files[n] for n in ['LICENSE', 'NOTICE']}
    inventory, mapping, origins, xml_fixes = [], [], [], []
    for name, before in sorted(files.items()):
        if not name.startswith('models/') or name.split('/')[1] not in local:
            continue
        after, replacements = before, []
        if name.endswith('model.config') and b' & ' in before:
            replacements = [dict(old=' & ', new=' &amp; ', count=before.count(b' & '))]
            after = before.replace(b' & ', b' &amp; ')
            xml_fixes.append(name)
        if PurePosixPath(name).suffix.lower() in ('.sdf', '.config', '.dae'):
            node = ET.fromstring(after)
            if list(node.iter('plugin')):
                raise ValueError('unreviewed model plugin')
        target = name.removeprefix('models/')
        inventory.append(dict(path=target, sha256=sha(after)))
        mapping.append(dict(source=name, target=target, source_sha256=sha(before), replacements=replacements))
        origins.append(dict(source=name, target='cache:models/' + target,
                            source_sha256=sha(before), sha256=sha(after), replacements=replacements))
    if len(xml_fixes) != 6:
        raise ValueError('reviewed six description-only XML escapes required')
    spec = dict(format='tar.gz', urls=[
        'https://codeload.github.com/aws-robotics/aws-robomaker-hospital-world/tar.gz/' + revision],
        **lock['archive'], prefix=prefix, max_members=len(entries),
        max_unpacked_bytes=sum(e.size for e in entries),
        license=dict(source='LICENSE', sha256=sha(files['LICENSE'])), members=mapping)
    assets = [(asset_record(AWS_ASSET, prov, local, inventory, spec,
                           ['Escape bare ampersands in six model.config descriptions; no geometry changes.']), archive_path)]
    outputs[MODELS + 'licenses/aws_hospital/origins.json'] = encoded(dict(
        source_repository=lock['source_repository'], revision=revision, archive=lock['archive'],
        declared_license='MIT-0', world_sha256=sha(world), world_changed=False,
        local_models=sorted(local), description_xml_fixes=xml_fixes, files=origins))
    fuel_records = []
    for record in lock['fuel_models']:
        name, version = record['name'], record['version']
        raw_meta = (upstream / 'fuel-metadata' / (name + '.json')).read_bytes()
        if sha(raw_meta) != record['metadata_sha256']:
            raise ValueError('reviewed publisher metadata snapshot required: ' + name)
        meta = json.loads(raw_meta)
        if (meta['name'] != name or meta['owner'] != 'OpenRobotics' or meta['version'] != version
                or meta['license_name'] != 'Creative Commons Attribution 4.0 International'
                or meta['license_url'] != 'http://creativecommons.org/licenses/by/4.0/'):
            raise ValueError('reviewed per-model license/version differs: ' + name)
        path = upstream / 'fuel-archives' / f'{name}-v{version}.zip'
        locked(path, record)
        contents, seen = {}, set()
        with zipfile.ZipFile(path) as archive:
            entries = archive.infolist()
            for entry in entries:
                member = safe_name(entry.filename)
                kind = stat.S_IFMT(entry.external_attr >> 16)
                if member in seen or kind not in (0, stat.S_IFREG, stat.S_IFDIR) or entry.flag_bits & 1:
                    raise ValueError('unsupported/duplicate Fuel ZIP entry: ' + member)
                seen.add(member)
                if not entry.is_dir():
                    contents[member] = archive.read(entry)
        config = ET.fromstring(contents['model.config'])
        authors = [e.findtext('name') or '' for e in config.findall('author')]
        description = config.findtext('description') or ''
        for member, data in contents.items():
            if PurePosixPath(member).suffix.lower() in ('.sdf', '.config', '.dae'):
                node = ET.fromstring(data)
                if list(node.iter('plugin')):
                    raise ValueError('unreviewed Fuel plugin')
        asset_id = 'fuel_hospital_' + name.lower() + '_v' + str(version)
        meta_target = 'licenses/hospital_fuel/metadata/' + name + '.json'
        prov = provenance(record['metadata_url'], str(version), 'CC-BY-4.0', [
            meta_target, 'licenses/hospital_fuel/ATTRIBUTION.md', 'licenses/hospital_fuel/origins.json'],
            'Publisher API snapshot declares CC-BY-4.0 for this model/version. '
            'ZIP has no LICENSE; cache LICENSE explicitly contains that external declaration. '
            'Original model.config author/description and every ZIP file retained unchanged.')
        inventory = [dict(path=name + '/' + n, sha256=sha(data)) for n, data in sorted(contents.items())]
        spec = dict(format='zip', urls=[record['url']], sha256=record['sha256'],
                    size_bytes=record['size_bytes'], prefix='', max_members=len(entries),
                    max_unpacked_bytes=sum(e.file_size for e in entries),
                    license=dict(notice=raw_meta.decode(), sha256=sha(raw_meta)),
                    members=[dict(source=n, target=name + '/' + n, source_sha256=sha(data), replacements=[])
                             for n, data in sorted(contents.items())])
        assets.append((asset_record(asset_id, prov, [name], inventory, spec, []), path))
        outputs[MODELS + meta_target] = raw_meta
        fuel_records.append(dict(name=name, owner=meta['owner'], version=version,
                                 authors=authors, description=description, publisher_description=meta['description'],
                                 declared_license='CC-BY-4.0', license_url=meta['license_url'],
                                 asset_set=asset_id, metadata_url=record['metadata_url'],
                                 metadata_sha256=sha(raw_meta), archive_url=record['url'],
                                 archive_sha256=record['sha256'], archive_bytes=record['size_bytes'],
                                 archive_file_count=len(contents), files_unchanged=True,
                                 license_evidence='Publisher API snapshot; no license file in ZIP.'))
    outputs[MODELS + 'licenses/hospital_fuel/origins.json'] = encoded(fuel_records)
    attribution = '# Hospital Fuel attribution\n\n'
    attribution += ('Publisher: OpenRobotics. Each of these 43 independent models is declared\n'
                    'CC-BY-4.0 by its own retained publisher API response. License:\n'
                    'https://creativecommons.org/licenses/by/4.0/ . This is separate from\n'
                    'the MIT-0 AWS world and from the old script\'s generated CC-BY-3.0 database.\n'
                    'Model files, authors, descriptions and third-party source credits remain\n'
                    'unchanged. No endorsement is implied. See origins.json for exact versions,\n'
                    'archive hashes, authors and original descriptions (including Autodesk,\n'
                    'Mixamo and Cole Biesemeyer credits where supplied by the publisher).\n\n')
    attribution += '| Title | Fuel version | Authors as supplied | Source |\n|---|---|---|---|\n'
    for r in fuel_records:
        author = ', '.join(a for a in r['authors'] if a.strip()) or 'Not specified; publisher OpenRobotics'
        attribution += f"| {r['name']} | {r['version']} | {author} | [Fuel]({r['archive_url']}) |\n"
    outputs[MODELS + 'licenses/hospital_fuel/ATTRIBUTION.md'] = attribution.encode()
    for asset, _ in assets:
        outputs[MODELS + 'manifests/' + asset['asset_set_id'] + '.yaml'] = yaml_bytes(asset)
    outliers = []
    for model in document.findall('world/model'):
        for pose in model.iter('pose'):
            if pose.text and any(abs(float(v)) > 10000 for v in pose.text.split()):
                outliers.append(dict(instance=model.attrib['name'], original_pose=pose.text,
                                     action='Preserved in original draft world; correction requires a new revision.'))
    if len(outliers) != 3:
        raise ValueError('reviewed three anomalous placements required')
    report = dict(scene=SCENE, source_revision=revision, world_sha256=sha(world), world_changed=False,
                  source_model_instances=len(document.findall('world/model')), builtin_models=['sun', 'ground_plane'],
                  aws_model_count=26, fuel_model_count=43, asset_sets=44,
                  aws_locked_files=len(assets[0][0]['files']),
                  fuel_locked_files=sum(len(a['files']) for a, _ in assets[1:]),
                  fuel_archive_bytes=sum(r['archive_bytes'] for r in fuel_records),
                  aws_license='MIT-0', fuel_licenses={'CC-BY-4.0': 43},
                  fuel_versions={str(v): sum(r['version'] == v for r in fuel_records) for v in [2, 3, 4]},
                  xml_description_fixes=xml_fixes, anomalous_placements=outliers,
                  geometry_changed=False, world_loaded=False, sensors_checked=False, navigation_tested=False,
                  status='draft', geometry_qualification_pending=True,
                  limitations=['Only the single-floor hospital.world dependency graph selected.',
                               'No multi-floor/elevator behavior or whole-building flight acceptance.',
                               'License declarations recorded from publisher, not inferred from AWS root license.',
                               'Publisher metadata names its current version; immutable ZIP bytes and the matching '
                               'X-Ign-Resource-Version response were independently recorded during source retrieval.'])
    outputs[WORLDS + 'worlds/indoor/hospital/' + SCENE + '.world'] = world
    outputs[WORLDS + 'evidence/' + SCENE + '/import-source.json'] = encoded(report)
    # Copy the established runtime contract, not its office qualification flags or calibration.
    scene = copy.deepcopy(yaml.safe_load((ROOT / (WORLDS + 'scenes/office_willowgarage_v1.yaml')).read_text()))
    scene.update(id=SCENE, name='AWS 医院（上游原始草稿）', status='draft', category='indoor/hospital',
                 tags=['hospital', 'medical', 'static'], supported_profiles=[], recommended_missions=[])
    scene['world'] = dict(package='easyfly_gazebo_worlds', path='worlds/indoor/hospital/' + SCENE + '.world', sha256=sha(world))
    scene['provenance'] = provenance(lock['source_repository'], revision, 'MIT-0', [
        'licenses/aws_hospital/LICENSE', 'licenses/aws_hospital/origins.json'],
        'Original hospital.world unchanged. Fuel dependencies have separate CC-BY-4.0 declarations. '
        'Three anomalous source heights retained for review; draft cannot launch.')
    scene['asset_sets'] = [a['asset_set_id'] for a, _ in assets] + ['gazebo_builtin_v1']
    scene['spawn'] = dict(frame='world', position=None, yaw=None, basis='Pending native floor/clearance survey and corrected-world review.')
    scene['navigation'].update(world_to_navigation=None, recommended_region=None)
    scene['navigation'].pop('alignment_check', None)
    scene['verification'] = dict(resources_complete=False, world_loaded=False, sensors_checked=False,
                                 navigation_tested=False, evidence_refs=[dict(package='easyfly_gazebo_worlds',
                                 path='evidence/' + SCENE + '/import-source.json')],
                                 required_px4_compatibility=['mag_lpf_interval_v1'])
    outputs[WORLDS + 'scenes/' + SCENE + '.yaml'] = yaml_bytes(scene)
    return outputs, assets


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('upstream', type=Path, help='Directory containing locked hospital.tar.gz, fuel-metadata/ and fuel-archives/')
    parser.add_argument('--workspace', type=Path, default=ROOT)
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--prepare-cache', action='store_true')
    parser.add_argument('--cache-root', type=Path)
    args = parser.parse_args()
    outputs, assets = reviewed_outputs(args.upstream)
    for name, data in outputs.items():
        target = args.workspace / name
        if args.check:
            if target.read_bytes() != data:
                raise ValueError('recorded source import differs: ' + name)
        elif target.exists():
            raise ValueError('refusing to overwrite: ' + name)
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
        for asset, path in assets:
            results.append(fetch_asset(asset, args.cache_root or default_root(), archive_path=path))
    print(json.dumps(dict(scene=SCENE, status='draft', model_count=69, asset_sets=44,
                          generated_files=len(outputs), check=args.check, prepared_assets=results), indent=2))


if __name__ == '__main__':
    main()

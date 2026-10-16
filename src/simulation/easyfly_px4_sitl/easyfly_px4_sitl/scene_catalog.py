# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Offline scene inventory validation; no network or simulator.

Callers supply trusted package shares, external roots, cache policy and plugin dirs.
The launcher and the low-level SITL entrypoints share this implementation.
"""

from dataclasses import dataclass, field
import hashlib
import json
import math
import re
from pathlib import Path, PurePosixPath
import shlex
import xml.etree.ElementTree as ET

import jsonschema
import yaml


class CatalogError(ValueError):
    """Invalid catalog data or unresolved resource."""


class _UniqueLoader(yaml.SafeLoader):
    pass


def _mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node)
        if not isinstance(key, str) or key in result:
            raise CatalogError(f"duplicate or non-string YAML key: {key!r}")
        result[key] = loader.construct_object(value_node)
    return result


_UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)


def _read_yaml(path):
    if path.stat().st_size > 1024 * 1024:
        raise CatalogError(f"YAML too large: {path}")
    content = path.read_text(encoding="utf-8")
    # A small explicit data format; aliases/merges would conceal duplicate fields.
    if any(isinstance(e, (yaml.AliasEvent,)) or getattr(e, 'anchor', None)
           for e in yaml.parse(content)):
        raise CatalogError(f"YAML anchors/aliases are not supported: {path}")
    return yaml.load(content, Loader=_UniqueLoader)


def _finite(value):
    if isinstance(value, float) and not math.isfinite(value):
        raise CatalogError("non-finite number")
    if isinstance(value, dict):
        for item in value.values():
            _finite(item)
    elif isinstance(value, list):
        for item in value:
            _finite(item)


def _relative(path):
    if not isinstance(path, str) or not path or '\\' in path or '%' in path or ':' in path:
        raise CatalogError(f"invalid relative path: {path!r}")
    p = PurePosixPath(path)
    if p.is_absolute() or '..' in p.parts:
        raise CatalogError(f"path escapes root: {path}")
    return p


def _inside(root, path, trusted_sources=()):
    root = Path(root).resolve(strict=True)
    target = (root / _relative(path)).resolve(strict=True)
    if not any(target.is_relative_to(p) for p in (root, *trusted_sources)):
        raise CatalogError(f"symlink escapes trusted root: {path}")
    return target


def _digest(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


@dataclass
class ValidationReport:
    scene_id: str
    issues: list = field(default_factory=list)
    checked_files: set = field(default_factory=set)
    checked_plugins: set = field(default_factory=set)

    def add(self, scope, message):
        issue = {'scope': scope, 'message': str(message)}
        if issue not in self.issues:
            self.issues.append(issue)

    def passed(self, scope):
        return not any(i['scope'] == scope for i in self.issues)

    def as_dict(self):
        return dict(scene_id=self.scene_id,
                    resources_complete=self.passed('scene_resources') and self.passed('runtime_resources'),
                    scene_resources_complete=self.passed('scene_resources'),
                    runtime_resources_complete=self.passed('runtime_resources'),
                    licenses_complete=self.passed('licenses'),
                    registration_complete=self.passed('registration'),
                    # Offline validation never asserts runtime flight readiness.
                    runtime_verified=False, issues=self.issues,
                    checked_files=sorted(str(p) for p in self.checked_files),
                    checked_plugins=sorted(self.checked_plugins))


class SceneCatalog:
    """Load schema-v1 catalogs from explicitly trusted roots (including symlink installs)."""

    def __init__(self, package_shares, *, external_roots=None, plugin_dirs=(), trusted_sources=(),
                 px4_compatibility=(), asset_cache_root=None, asset_mode='packaged', protected_cache_roots=()):
        self.packages = {k: Path(v).resolve(strict=True) for k, v in package_shares.items()}
        self.external = {k: Path(v).resolve(strict=True) for k, v in (external_roots or {}).items()}
        self.plugin_dirs = tuple(Path(p).resolve(strict=True) for p in plugin_dirs)
        self.trusted_sources = tuple(Path(p).resolve(strict=True) for p in trusted_sources)
        self.px4_compatibility = frozenset(px4_compatibility)
        from .asset_cache import cache_root
        if asset_mode not in ('packaged', 'cache'):
            raise CatalogError('asset mode must be packaged or cache')
        self.asset_mode = asset_mode
        self.asset_cache_root = cache_root(asset_cache_root, (*self.packages.values(),
                                                            *self.trusted_sources, *protected_cache_roots))
        self._cached_models = {}
        worlds = self.packages['easyfly_gazebo_worlds']
        schemas = worlds / 'schemas'
        self.schemas = {}
        for kind in ('scene', 'asset-set'):
            schema = json.loads((schemas / f'{kind}-v1.schema.json').read_text())
            jsonschema.Draft7Validator.check_schema(schema)
            self.schemas[kind] = jsonschema.Draft7Validator(schema)
        self.scenes = self._load(worlds / 'scenes', 'scene', 'id')
        self.assets = self._load(self.packages['easyfly_gazebo_models'] / 'manifests',
                                 'asset-set', 'asset_set_id')

    def _load(self, directory, kind, identity):
        records = {}
        for path in sorted(directory.glob('*.yaml')):
            try:
                value = _read_yaml(path)
                self.schemas[kind].validate(value)
                _finite(value)
                name = value[identity]
                if name in records:
                    raise CatalogError(f"duplicate {identity}: {name}")
                records[name] = value
            except (ValueError, yaml.YAMLError, jsonschema.ValidationError) as error:
                raise CatalogError(f'{path}: {error}') from error
        return records

    def resolve(self, location):
        """Resolve a package or an explicitly configured external location."""
        key = 'package' if 'package' in location else 'external'
        roots = self.packages if key == 'package' else self.external
        name = location[key]
        if name not in roots:
            raise CatalogError(f'unconfigured {key} root: {name}')
        return _inside(roots[name], location['path'], self.trusted_sources if key == 'package' else ())

    def asset_root(self, asset_id):
        """Resolve the selected source without fetching or falling back to another source."""
        asset = self.assets[asset_id]
        if asset['storage'] == 'cached' or (self.asset_mode == 'cache' and 'archive' in asset):
            from .asset_cache import validate_cached
            if asset_id not in self._cached_models:
                self._cached_models[asset_id] = validate_cached(asset, self.asset_cache_root)
            return self._cached_models[asset_id]
        return self.resolve(asset['model_root'])

    def validate(self, scene_id):
        if scene_id not in self.scenes:
            raise CatalogError(f'unknown scene: {scene_id}')
        scene = self.scenes[scene_id]
        report = ValidationReport(scene_id)
        for feature in scene['verification'].get('required_px4_compatibility', []):
            if feature not in self.px4_compatibility:
                report.add('runtime_resources', f'required reviewed PX4 compatibility missing: {feature}')
        self._provenance(scene['provenance'], report, 'world')
        if scene['status'] == 'draft' or not scene['supported_profiles']:
            report.add('registration', 'draft scene; no newly verified profile compatibility')
        if scene['spawn']['position'] is None or scene['spawn']['yaw'] is None:
            report.add('registration', 'spawn not specified')
        if scene['navigation']['world_to_navigation'] is None:
            report.add('registration', 'world/odom alignment not measured')
        region = scene['navigation']['recommended_region']
        if region and any(a >= b for a, b in zip(region['min'], region['max'])):
            report.add('registration', 'invalid recommended region bounds')
        for loc in scene['recommended_missions'] + scene['map_products'] + scene['verification']['evidence_refs']:
            try:
                if not self.resolve(loc).is_file():
                    raise CatalogError('reference is not a file')
            except (OSError, ValueError) as error:
                report.add('registration', error)
        for scope, sets, entry, plugins, extra in [
            ('scene_resources', scene['asset_sets'], scene['world'], scene['required_plugins'], []),
            ('runtime_resources', list(dict.fromkeys(scene['asset_sets'] + scene['runtime']['asset_sets'])), scene['runtime']['entrypoint'],
             scene['runtime']['required_plugins'], scene['runtime']['additional_files']),
        ]:
            walker = _ResourceWalker(self, report, scope, sets)
            try:
                path = self.resolve(entry)
                if scope == 'scene_resources':
                    if path.suffix not in ('.world', '.sdf'):
                        raise CatalogError(f'world entry must be .world or .sdf: {path}')
                    try:
                        document = ET.parse(path).getroot()
                    except ET.ParseError as error:
                        raise CatalogError(f'invalid world XML: {path}: {error}') from error
                    if document.tag != 'sdf' or len(document.findall('world')) != 1:
                        raise CatalogError(f'world entry requires exactly one SDF world: {path}')
                if 'sha256' in entry and _digest(path) != entry['sha256']:
                    raise CatalogError(f'world fingerprint mismatch: {path}')
                walker.walk(path, path.parent)
            except (OSError, ValueError) as error:
                report.add(scope, error)
            for loc in extra:
                try:
                    path = self.resolve(loc)
                    walker.walk(path, path.parent)
                except (OSError, ValueError) as error:
                    report.add(scope, error)
            for plugin in plugins:
                walker.plugin(plugin)
        return report

    def _provenance(self, value, report, label):
        if value['status'] != 'recorded' or not value['declared_license'] or not value['evidence']:
            report.add('licenses', f'{label}: provenance/license unverified')
        for location in value['evidence']:
            try:
                if not self.resolve(location).is_file():
                    raise CatalogError('license evidence is not a file')
            except (OSError, ValueError) as error:
                report.add('licenses', f'{label}: {error}')


class _ResourceWalker:
    def __init__(self, catalog, report, scope, asset_sets):
        self.catalog, self.report, self.scope = catalog, report, scope
        self.models, self.visited = {}, set()
        roots = []
        for name in asset_sets:
            if name not in catalog.assets:
                report.add(scope, f'unknown asset set: {name}')
                continue
            asset = catalog.assets[name]
            catalog._provenance(asset['provenance'], report, name)
            try:
                root = catalog.asset_root(name)
                if root not in roots:
                    roots.append(root)
                for model in asset['models']:
                    catalog._provenance(model['provenance'], report, model['name'])
                    if model['name'] in self.models:
                        raise CatalogError(f'duplicate model name: {model["name"]}')
                    if _relative(model['name']).name != model['name']:
                        raise CatalogError(f'invalid model name: {model["name"]}')
                    installed = _inside(root, model['name'], catalog.trusted_sources)
                    sdf_path = _inside(installed, model['sdf'], catalog.trusted_sources)
                    # ament may symlink individual files rather than whole dirs.
                    # Bind relative model resources to the trusted SDF source dir.
                    directory = sdf_path.parents[len(_relative(model['sdf']).parts) - 1]
                    self.models[model['name']] = (directory, model['sdf'])
                inventory = set()
                for item in asset['files']:
                    if item['path'] in inventory:
                        raise CatalogError(f'duplicate inventory path: {item["path"]}')
                    inventory.add(item['path'])
                    path = _inside(root, item['path'], catalog.trusted_sources)
                    if not path.is_file() or _digest(path) != item['sha256']:
                        raise CatalogError(f'asset fingerprint mismatch: {path}')
                    report.checked_files.add(path)
                for directory, _ in [self.models[m['name']] for m in asset['models']]:
                    model_name = next(m['name'] for m in asset['models']
                                      if self.models[m['name']][0] == directory)
                    actual = {f'{model_name}/{f.relative_to(directory).as_posix()}'
                              for f in directory.rglob('*') if f.is_file()}
                    if not actual <= inventory:
                        raise CatalogError(f'unlocked model files: {sorted(actual - inventory)}')
            except (OSError, ValueError) as error:
                report.add(scope, f'{name}: {error}')
        exposed = {}
        for root in roots:
            for config in root.glob('*/model.config'):
                name, directory = config.parent.name, config.parent.resolve()
                if name in exposed and exposed[name] != directory:
                    report.add(scope, f'duplicate model name in exposed search roots: {name}')
                exposed[name] = directory

    def fail(self, error):
        self.report.add(self.scope, error)

    def plugin(self, filename):
        try:
            if _relative(filename).name != filename:
                raise CatalogError(f'plugin must be a basename: {filename}')
            if not any((root / filename).is_file() for root in self.catalog.plugin_dirs):
                raise CatalogError(f'plugin not found in configured dirs: {filename}')
            self.report.checked_plugins.add(filename)
        except (OSError, ValueError) as error:
            self.fail(error)

    def reference(self, uri, source, owner, material_names=None):
        try:
            if uri.startswith('model://'):
                suffix = str(_relative(uri[len('model://'):]))
                name, _, rest = suffix.partition('/')
                if name not in self.models:
                    raise CatalogError(f'{source.name}: undeclared/missing model {name}')
                directory, sdf = self.models[name]
                if rest:
                    self.walk(_inside(directory, rest), directory, material_names)
                else:
                    self.walk(_inside(directory, 'model.config'), directory)
                    self.walk(_inside(directory, sdf), directory)
            elif uri.startswith('file://media/'):
                self.walk(self.catalog.resolve({'external':'gazebo_share', 'path':uri[7:]}),
                          self.catalog.external['gazebo_share'], material_names)
            elif '://' in uri or uri.startswith('file:'):
                raise CatalogError(f'unsupported or remote URI: {uri}')
            else:
                # DAE/MTL relative paths may ascend within their owning model only.
                if '\\' in uri or '%' in uri or ':' in uri or Path(uri).is_absolute():
                    raise CatalogError(f'invalid resource URI: {uri}')
                target = (source.parent / uri).resolve()
                if not target.is_relative_to(owner):
                    raise CatalogError(f'relative reference escapes model: {uri}')
                candidates = {p.resolve() for p in (target, owner / 'materials/textures' / uri)
                              if p.is_file() and p.resolve().is_relative_to(owner)}
                if len(candidates) != 1:
                    raise CatalogError(f'{source.name}: missing/ambiguous relative resource: {uri}')
                self.walk(candidates.pop(), owner, material_names)
        except (OSError, ValueError) as error:
            self.fail(error)

    def walk(self, path, owner, material_names=None):
        path, owner = path.resolve(strict=True), owner.resolve(strict=True)
        if not path.is_file():
            raise CatalogError(f'not a resource file: {path}')
        visit_key = (path, tuple(material_names or ()))
        if visit_key in self.visited:
            return
        self.visited.add(visit_key)
        self.report.checked_files.add(path)
        suffix = path.suffix.lower()
        if suffix in ('.world', '.sdf', '.config', '.dae'):
            try:
                root = ET.parse(path).getroot()
            except ET.ParseError as error:
                raise CatalogError(f'invalid XML: {path}: {error}') from error
            script_uris = set()
            for node in root.iter():
                if node.tag.split('}')[-1] == 'script':
                    for child in node:
                        if child.tag == 'uri' and child.text:
                            script_uris.add(child)
                    # Keep the existing Gazebo builtin name-only convention;
                    # explicit file/directory URIs receive the closure audit.
                    if any(c.tag == 'uri' and c.text for c in node):
                        self.script(node, path, owner)
            for node in root.iter():
                tag = node.tag.split('}')[-1]
                if tag == 'uri' and node.text and node not in script_uris:
                    self.reference(node.text.strip(), path, owner)
                if tag == 'plugin':
                    self.plugin(node.attrib.get('filename', ''))
                if tag == 'image':
                    for child in node:
                        if child.tag.split('}')[-1] == 'init_from' and child.text:
                            self.reference(child.text.strip(), path, owner)
                if suffix == '.config' and tag == 'sdf' and node.text:
                    self.reference(node.text.strip(), path, owner)
        elif suffix in ('.obj', '.mtl'):
            for line in path.read_text().splitlines():
                tokens = shlex.split(line, comments=True)
                if not tokens:
                    continue
                if tokens[0] == 'mtllib':
                    for uri in tokens[1:]:
                        self.reference(uri, path, owner)
                elif tokens[0] in ('map_Ka','map_Kd','map_Ks','map_d','map_Bump','bump','disp','decal','norm'):
                    if len(tokens) != 2:
                        self.fail(f'unsupported MTL map syntax: {path}: {line}')
                    else:
                        self.reference(tokens[1], path, owner)
        elif suffix == '.material':
            self.material(path, owner, material_names)
        elif suffix == '.program':
            self.fail(f'OGRE program dependency audit pending: {path}')
        elif suffix not in ('.png', '.jpg', '.jpeg', '.stl', '.csv', '.dds', '.tga', '.bmp', '.gif', '.tif', '.tiff'):
            self.fail(f'unsupported resource format; dependency audit pending: {path}')

    def script(self, node, source, owner):
        """Resolve selected OGRE materials across explicit local search locations.

        SDF permits both script files and directories (including a separate
        texture directory). A directory is never accepted as a mesh/resource
        file. Every selected material must have exactly one definition across
        these locations and still passes the existing dependency audit.
        """
        names = [c.text.strip() for c in node if c.tag == 'name' and c.text]
        uris = [c.text.strip() for c in node if c.tag == 'uri' and c.text]
        if not names or len(set(names)) != len(names) or not uris:
            self.fail(f'OGRE script requires unique selected materials and local URIs: {source}')
            return
        candidates = {}
        for uri in uris:
            try:
                if uri.startswith('model://'):
                    suffix = str(_relative(uri[len('model://'):]))
                    name, _, rest = suffix.partition('/')
                    if name not in self.models:
                        raise CatalogError(f'{source.name}: undeclared/missing model {name}')
                    directory, _ = self.models[name]
                    target = _inside(directory, rest or '.')
                elif uri.startswith('file://media/'):
                    directory = self.catalog.external['gazebo_share'].resolve(strict=True)
                    target = self.catalog.resolve({'external':'gazebo_share', 'path':uri[7:]})
                elif '://' in uri or uri.startswith('file:'):
                    raise CatalogError(f'unsupported or remote URI: {uri}')
                else:
                    if '\\' in uri or '%' in uri or ':' in uri or Path(uri).is_absolute():
                        raise CatalogError(f'invalid resource URI: {uri}')
                    directory = owner
                    target = (source.parent / uri).resolve(strict=True)
                    if not target.is_relative_to(directory):
                        raise CatalogError(f'relative reference escapes model: {uri}')
                if target.is_dir():
                    files = sorted(target.glob('*.material'))
                elif target.is_file() and target.suffix.lower() == '.material':
                    files = [target]
                else:
                    raise CatalogError(f'unsupported OGRE script location: {uri}')
                for path in files:
                    path = path.resolve(strict=True)
                    if not path.is_relative_to(directory):
                        raise CatalogError(f'OGRE script symlink escapes model: {uri}')
                    candidates[path] = directory
            except (OSError, ValueError, KeyError) as error:
                self.fail(error)
        for name in names:
            matches = []
            for path in candidates:
                text = re.sub(r'/\*.*?\*/|//[^\n]*', '', path.read_text(), flags=re.S)
                if re.search(r'\bmaterial\s+' + re.escape(name) + r'\s*\{', text):
                    matches.append(path)
            if len(matches) != 1:
                self.fail(f'OGRE material absent, ambiguous or inherited/unsupported: {name} in {source}')
            else:
                path = matches[0]
                self.walk(path, candidates[path], [name])

    def material(self, path, owner, names):
        """Audit selected simple OGRE materials; fail closed on advanced syntax."""
        if not names:
            self.fail(f'OGRE script dependency audit pending (no selected material): {path}')
            return
        text = re.sub(r'/\*.*?\*/|//[^\n]*', '', path.read_text(), flags=re.S)
        for name in names:
            match = re.search(r'\bmaterial\s+' + re.escape(name) + r'\s*\{', text)
            if not match:
                self.fail(f'OGRE material absent or inherited/unsupported: {name} in {path}')
                continue
            if len(re.findall(r'\bmaterial\s+' + re.escape(name) + r'\s*\{', text)) != 1:
                self.fail(f'duplicate OGRE material: {name} in {path}')
                continue
            start, cursor, depth = match.end(), match.end(), 1
            while cursor < len(text) and depth:
                depth += (text[cursor] == '{') - (text[cursor] == '}')
                cursor += 1
            if depth:
                self.fail(f'unbalanced OGRE material: {path}: {name}')
                continue
            block = text[start:cursor - 1]
            if re.search(r'\b(?:\w*program\w*|texture_alias|anim_texture|cubic_texture|content_type)\b|\$|#', block):
                self.fail(f'advanced OGRE material dependency audit pending: {path}: {name}')
                continue
            for texture in re.finditer(r'^\s*texture\s+([^\n{}]+)', block, re.M):
                tokens = shlex.split(texture.group(1))
                if len(tokens) != 1:
                    self.fail(f'unsupported OGRE texture syntax: {path}: {name}')
                    continue
                self.reference(tokens[0], path, owner)

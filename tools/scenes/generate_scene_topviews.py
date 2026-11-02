#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Generate offline, native Gazebo top views of registered static worlds.

Uses an isolated master and a temporary camera-only world copy. Never launches
PX4, ROS control nodes, downloads, or changes the registered world files.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xml.etree.ElementTree as ET


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def asset_identity(catalog, scene):
    return {name: hashlib.sha256(json.dumps(catalog.assets[name], sort_keys=True,
                                          separators=(',', ':')).encode()).hexdigest()
            for name in scene['asset_sets']}


def port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=4)


def render(catalog, scene, args):
    world = catalog.resolve(scene['world'])
    if digest(world) != scene['world']['sha256']:
        raise ValueError('world fingerprint mismatch')
    source = ET.parse(world)
    if source.findall('.//plugin') or source.findall('.//sensor'):
        raise ValueError('preview only accepts camera-free static world inputs')
    roots = [catalog.asset_root(name) for name in scene['asset_sets']]
    ident = scene['id']
    work = args.work / ident
    work.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    env.update(GAZEBO_MASTER_URI=f'http://127.0.0.1:{port()}',
               GAZEBO_MODEL_DATABASE_URI='',
               GAZEBO_MODEL_PATH=':'.join(map(str, roots)),
               GAZEBO_LOG_PATH=str(work / 'gazebo-log'),
               GAZEBO_PLUGIN_PATH='', GAZEBO_IP='127.0.0.1',
               GAZEBO_RESOURCE_PATH='/usr/share/gazebo-11',
               DISPLAY=os.environ.get('DISPLAY', ':1'))
    with (work / 'bounds.log').open('w') as log:
        run = subprocess.run([str(args.bounds), str(world), str(work / 'bounds.json')],
                             env=env, stdout=log, stderr=subprocess.STDOUT,
                             timeout=args.timeout, start_new_session=True)
    if run.returncode:
        raise ValueError(f'native bounds exited {run.returncode}')
    measured = json.loads((work / 'bounds.json').read_text())
    lo_x, lo_y, lo_z, hi_x, hi_y, hi_z = measured['bounds']
    if not all(math.isfinite(v) for v in measured['bounds']):
        raise ValueError('nonfinite measured bounds')
    width, height = 1024, 768
    fov = math.pi / 3
    cut = args.cut_height if scene['category'].startswith('indoor/') else None
    distance = max((hi_x - lo_x) / (2 * math.tan(fov / 2)),
                   (hi_y - lo_y) / (2 * math.tan(fov / 2) * height / width)) * 1.12
    camera_z = max(distance + (cut or max(0., hi_z)), hi_z + 2.)
    camera = dict(x=(lo_x + hi_x) / 2, y=(lo_y + hi_y) / 2, z=camera_z,
                  roll=0., pitch=math.pi / 2, yaw=math.pi / 2,
                  horizontal_fov=fov, near=camera_z - cut if cut else .1,
                  far=camera_z - min(lo_z, 0.) + 50., width=width, height=height)
    node = source.find('world')
    if node is None:
        raise ValueError('no world element')
    scene_node = node.find('scene')
    if scene_node is None:
        scene_node = ET.SubElement(node, 'scene')
    for key, value in [('shadows', 'false'), ('ambient', '0.7 0.7 0.7 1'),
                       ('background', '0.87 0.9 0.94 1')]:
        child = scene_node.find(key)
        if child is None:
            child = ET.SubElement(scene_node, key)
        child.text = value
    images = work / 'frames'
    images.mkdir(exist_ok=True)
    for path in images.iterdir():
        path.unlink()
    xml = f'''<model name="easyfly_preview_camera"><static>true</static>
      <pose>{camera['x']} {camera['y']} {camera_z} 0 {math.pi/2} {math.pi/2}</pose>
      <link name="camera"><sensor name="topview" type="camera">
      <always_on>true</always_on><update_rate>1</update_rate><visualize>false</visualize>
      <camera><horizontal_fov>{fov}</horizontal_fov>
      <image><width>{width}</width><height>{height}</height><format>R8G8B8</format></image>
      <clip><near>{camera['near']}</near><far>{camera['far']}</far></clip>
      <save enabled="true"><path>{images}</path></save></camera></sensor></link></model>'''
    node.append(ET.fromstring(xml))
    temporary = work / 'preview.world'
    source.write(temporary, encoding='utf-8', xml_declaration=True)
    # Bound rendering rate independently of source-world flight physics settings.
    with (work / 'render.log').open('w') as log:
        process = subprocess.Popen([str(args.server), '--verbose', str(temporary)], env=env,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        chosen = None
        try:
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline and process.poll() is None:
                frames = sorted(p for p in images.iterdir() if p.suffix in ('.png', '.jpg'))
                if len(frames) >= 3:
                    chosen = frames[-2]
                    break
                time.sleep(.25)
        finally:
            stop(process)
    if chosen is None:
        raise ValueError('native camera produced no stable PNG')
    from PIL import Image, ImageStat
    image = Image.open(chosen).convert('RGB')
    if image.size != (width, height) or max(ImageStat.Stat(image).stddev) < 3:
        raise ValueError('native camera image is blank or has wrong dimensions')
    destination = args.output / f'{ident}.png'
    image.save(destination, optimize=True)
    caption = ('Gazebo 原生俯视剖切图（高于 %.1f 米的结构已裁切）' % cut if cut else
               'Gazebo 原生场景俯视图')
    return dict(scene_id=ident, world_sha256=scene['world']['sha256'],
                asset_identity=asset_identity(catalog, scene), file=destination.name,
                image_sha256=digest(destination), kind='native_topview_cutaway' if cut else
                'native_topview', caption=caption, cut_height_m=cut,
                bounds=measured['bounds'], geometry_links=measured['links'], camera=camera)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bounds', required=True, type=Path)
    parser.add_argument('--server', type=Path,
                        default=Path('install/easyfly_gazebo_plugins/lib/easyfly_gazebo_plugins/easyfly_gzserver'))
    parser.add_argument('--output', type=Path,
                        default=Path('src/simulation/easyfly_gazebo_worlds/previews'))
    parser.add_argument('--work', type=Path, required=True)
    parser.add_argument('--scene', action='append')
    parser.add_argument('--timeout', type=float, default=45.)
    parser.add_argument('--cut-height', type=float, default=2.5)
    args = parser.parse_args()
    args.bounds, args.server = args.bounds.resolve(), args.server.resolve()
    args.output, args.work = args.output.resolve(), args.work.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    args.work.mkdir(parents=True, exist_ok=True)
    from easyfly_px4_sitl.scene_environment import installed_catalog
    catalog, _ = installed_catalog()
    manifest_path = args.output / 'manifest.json'
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else dict(
        schema_version=1, generator='Gazebo Classic native camera', scenes={})
    failures = {}
    for ident, scene in sorted(catalog.scenes.items()):
        if scene['status'] != 'registered' or (args.scene and ident not in args.scene):
            continue
        print('RENDER', ident, flush=True)
        try:
            manifest['scenes'][ident] = render(catalog, scene, args)
            manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
            print('OK', ident, flush=True)
        except Exception as error:
            failures[ident] = str(error)
            print('FAILED', ident, str(error), flush=True)
    (args.work / 'failures.json').write_text(json.dumps(failures, indent=2) + '\n')
    return bool(failures)


if __name__ == '__main__':
    raise SystemExit(main())

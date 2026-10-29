# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Read-only GLX deployment preflight; this never certifies flight readiness."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess


def assess(output, kernel_version='', require_hardware=False):
    """Assess the actual current display, not the presence of a GPU device file."""
    fields = {}
    for line in output.splitlines():
        key, separator, value = line.partition(':')
        if separator:
            fields[key.strip()] = value.strip()
    renderer = fields.get('OpenGL renderer string', '')
    version = fields.get('OpenGL version string', '')
    software = any(name in renderer.lower() for name in
                   ('llvmpipe', 'softpipe', 'swrast', 'software rasterizer'))
    direct = fields.get('direct rendering') == 'Yes'
    accelerated = fields.get('Accelerated')
    hardware = bool(renderer and direct and not software and accelerated != 'no')
    errors = []
    if not renderer or not version:
        errors.append('GLX did not report an OpenGL renderer and version')
    if not direct:
        errors.append('Direct GLX rendering is unavailable')
    vendor_version = re.search(r'\bNVIDIA\s+([0-9.]+)', version)
    if vendor_version and kernel_version and vendor_version.group(1) != kernel_version:
        errors.append('NVIDIA OpenGL userspace and loaded kernel module versions differ')
    if require_hardware and not hardware:
        errors.append('This scene requires hardware rendering; software GLX is not qualified')
    return dict(passed=not errors, renderer=renderer, version=version,
                vendor=fields.get('OpenGL vendor string', ''), direct=direct,
                hardware_rendering=hardware, nvidia_kernel=kernel_version, errors=errors,
                flight_ready=False)


def main():
    """Print a bounded JSON report without changing drivers, environment or processes."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--require-hardware-rendering', action='store_true')
    parser.add_argument('--glxinfo', default='glxinfo', help='mesa-utils glxinfo executable')
    args = parser.parse_args()
    kernel = Path('/sys/module/nvidia/version')
    try:
        result = subprocess.run([args.glxinfo, '-B'], capture_output=True, text=True,
                                timeout=10, env=dict(os.environ, LC_ALL='C'))
        report = assess(result.stdout, kernel.read_text().strip() if kernel.exists() else '',
                        args.require_hardware_rendering)
        if result.returncode:
            report['errors'].append('glxinfo failed: ' + result.stderr.strip()[:1024])
            report['passed'] = False
    except (OSError, subprocess.TimeoutExpired) as error:
        report = dict(passed=False, flight_ready=False,
                      errors=[f'Cannot run glxinfo (mesa-utils): {error}'])
    report['display'] = os.environ.get('DISPLAY', '')
    report['lp_num_threads'] = os.environ.get('LP_NUM_THREADS', '1 (launcher default)')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['passed'] else 2

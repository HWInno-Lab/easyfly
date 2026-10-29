#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Build a genuine RGB-D atlas of the native natural-photo fixture."""
import importlib.util
from pathlib import Path
from orbslam3_natural_fixture import NaturalFixtureMixin
spec=importlib.util.spec_from_file_location('orb_natural_seed_fixture',Path(__file__).with_name('orbslam3_simulation.py'))
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)

class SeedProbe(NaturalFixtureMixin,base.Probe):
    pass

if __name__=='__main__':
    base.Probe=SeedProbe
    raise SystemExit(base.main())

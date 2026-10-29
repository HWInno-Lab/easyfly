#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Reconstruct the selected official ORB-SLAM3/Pangolin sources using zero-fuzz patches."""
import argparse
import importlib.util
import json
from pathlib import Path
spec=importlib.util.spec_from_file_location('pinned_source_verifier',Path(__file__).with_name('verify_fastlivo_source.py'))
verifier=importlib.util.module_from_spec(spec);spec.loader.exec_module(verifier)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,default=Path(__file__).resolve().parents[2]/'src/third_party/algorithms/localization/orb_slam3.provenance.json')
    a=p.parse_args();print(json.dumps(verifier.verify(a.manifest),indent=2))

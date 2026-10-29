# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Bound Mesa's renderer pool inside a shared flight simulation process tree."""
import os


def renderer_environment(environment=None):
    environment = os.environ if environment is None else environment
    if 'LP_NUM_THREADS' in environment:
        return {'LP_NUM_THREADS': environment['LP_NUM_THREADS']}
    # Mesa's pool shares CPU time with sensor/render work. The residential
    # regression met its unchanged freshness gates with one worker, while a
    # four-worker pool still left long pauses despite spare host CPUs.
    return {'LP_NUM_THREADS': '1'}

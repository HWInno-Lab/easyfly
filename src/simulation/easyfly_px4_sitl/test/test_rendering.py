# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
from easyfly_px4_sitl.rendering import renderer_environment


def test_shared_cpu_budget_and_explicit_overrides():
    assert renderer_environment({}) == {'LP_NUM_THREADS': '1'}
    for threads in ['0', '2', '16']:
        assert renderer_environment({'LP_NUM_THREADS': threads}) == {'LP_NUM_THREADS': threads}

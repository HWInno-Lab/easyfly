# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""A visible GPU must not hide software rendering or mismatched GLX libraries."""
from easyfly_px4_sitl.graphics import assess


def test_hardware_and_kernel_userspace_match():
    output = ('direct rendering: Yes\nOpenGL vendor string: NVIDIA Corporation\n'
              'OpenGL renderer string: Test GPU\nOpenGL version string: 4.6.0 NVIDIA 595.91.07')
    assert assess(output, '595.91.07', True)['passed']
    assert not assess(output, '595.99.02', True)['passed']
    assert not assess(output.replace('Yes', 'No'), '595.91.07', True)['passed']


def test_software_is_reported_even_with_a_loaded_nvidia_kernel():
    output = ('direct rendering: Yes\nOpenGL vendor string: Mesa\n'
              'OpenGL renderer string: llvmpipe (LLVM 15.0.7)\nOpenGL version string: 4.5 Mesa')
    assert assess(output, '595.91.07')['passed']
    assert not assess(output, '595.91.07')['hardware_rendering']
    assert not assess(output, '595.91.07', True)['passed']
    assert not assess('Error: unable to open display')['passed']
    assert not assess(output)['flight_ready']

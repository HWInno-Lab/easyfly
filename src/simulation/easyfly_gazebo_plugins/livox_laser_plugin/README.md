# Livox native ray geometry

The plugin preserves the configured scan, range bounds, measurement stamps and
aircraft geometry. Box contacts use a finite ray/box slab intersection in the
actual ODE box frame; other geometry continues to use native `dCollide`.

Every CustomMsg point has `offset_time=0`: one native ray update acquires the
entire cloud at `header.stamp == timebase`. CSV order specifies beam direction,
not an elapsed physics time. The former index-based 0–100 ms offsets made
FAST-LIO wait for future IMU and deskew points acquired simultaneously.
`test_livox_scan_timing` verifies ten actual native 20000-point scans and exit.

Gazebo Classic 11.10.2's bundled ODE `deps/opende/src/ray.cpp` uses different
parallel tests in the early rejection and interval calculation in
`dCollideRayBox`. A tiny component toward an outside slab can pass the first
check and then be skipped by the second. A native reproduction with a
0.47 × 0.47 × 0.11 box, ray start (0.2, 0.2, 0.13), and direction approximately
(1, 0, -1e-6) returned (0.235, 0.2, 0.13), above the actual top z=0.055.
A moving, no-flight model fixture reproduced the same phantom aircraft return.
In scene flights, such a point survived the unchanged 0.3 m preprocessing blind
zone and correctly triggered the unchanged planner's collision stop.

`ray_box_intersection.hpp` is an independent slab implementation using public
ODE geometry APIs. It evaluates every nonzero component, rejects an outside
exactly parallel slab, respects the finite native segment, and preserves the
exit surface for a segment starting inside a box. It handles translated and
rotated boxes without changing the system ODE library, collision masks, real
obstacles, point cloud noise, blind zone or planner inflation. It does not simply
ignore the aircraft's model or discard all close returns. The normal native
min-range offset is still added exactly once by `ray_geometry.hpp`.

`test_livox_ray_geometry` covers the native phantom case, a real shallow-angle
entry, segment limits, exact parallel rays, inside exits, transformed boxes,
external walls and invalid numerical inputs. Historical and corrected native
fixtures, real scene results, loaded library hashes and failures are recorded in
`schedule-v6.md`. Historical artifact directories were cleared at the user’s request;
reproduction requires a rebuilt environment and new evidence.

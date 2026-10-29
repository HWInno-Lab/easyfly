Recorded native GCOPTER request from the third FUEL room viewpoint in
`artifacts/v01-v16-20261025/sitl/v07-fuel_room_v1-cold-1-v2/`.
The original run exceeded the unchanged 0.20 m tracking guard and safely landed.
The request retains the measured start/velocity, goal, and all original map cells.
SHA256: `cbc33fd44d53b7afede833c85e6da053b0bd47b4913d656a6bb5b40d07971f61`.
The old optimizer reproduced its original polynomial byte-for-byte:
acceleration 0.2470399857 m/s², jerk 0.8769906946 m/s³.
This fixture checks the conservative position/velocity-controller profile while
preserving endpoints and continuous collision constraints; it is not a synthetic
flight result or a hardware performance specification.

`rolling_reverse.request.gz` is the exact original native 30 m rolling-navigation
postflight request (job 12) that failed GCOPTER's first numerical solve, with
measured residual velocity pointing away from the return goal. The worker must
re-solve within the existing eight-attempt / 2 s optimizer budget, preserve that
velocity and pass the unchanged collision and .15 m/s² / .30 m/s³ checks.

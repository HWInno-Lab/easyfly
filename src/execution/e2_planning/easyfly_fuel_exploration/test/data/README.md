# Recorded native flight snapshots

These are source-time inflated occupancy and measured state requests from E-04 native PX4/Gazebo flights, reused to regress the new native FUEL B-spline backend. They are neither generated obstacle truth nor evidence of a new flight. The dynamic-detour snapshot contains the physically inserted wall. The reverse snapshot includes measured velocity away from the destination.

- `initial.request.gz`: decompressed SHA-256 `83fceccbf1f7734878f1cac856eb9a4a48077878002ca875ff5c931e8e496db5`.
- `dynamic-detour.request.gz`: decompressed SHA-256 `f8de549be2804ae81b26c2d89ea05e1298d324b560d2302a143882af5117d9cd`.
- `reverse.request.gz`: decompressed SHA-256 `993b5a3a1de7aa8867f639d49d58f3b05c68f04fa44587faf129eb1a61c1916c`.

- `long-return.request.gz`: E-05 native 7-viewpoint flight return snapshot; decompressed SHA-256 `041f0b92e0b7f4e063d87e4196a0ff5605ca715d3790010c197ec25237ffa2f3`. The original overly coarse native timing retry only reached 0.728 m/s in flight; preserve it as a higher-speed regression.

- `long-return-timing.request.gz`: E-05 native 8-viewpoint flight return snapshot; decompressed SHA-256 `6ec373d362a6703eb337c179f16f38226a185da2ce3f3469bf053575f2643afe`. The first timing-step refinement reached 0.798 m/s, below the frozen 0.8 m/s flight threshold.

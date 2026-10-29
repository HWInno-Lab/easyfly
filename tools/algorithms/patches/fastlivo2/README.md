# FAST-LIVO2 / dependencies integration patches

The authoritative source URLs, exact commits, selected original source archives,
per-file hashes and ordered component patch hashes are in
`src/third_party/algorithms/localization/fast_livo2.provenance.json`.

- `fast-livo2-integration.patch`: ROS2 installed dependency linking, owned node
  startup, relative image publisher, bounded queues, source-stamped post-visual
  state/covariance, actual residual diagnostics, explicit instantaneous Gazebo
  beam interpretation, and disabled managed file / external vision output.
- `vikit-integration.patch`: build only required original camera/math routines,
  install/export C++17 dependencies, qualify Eigen matrices and modern Sophus
  include names. No prebuilt fork binaries are imported.
- `sophus-integration.patch`: package metadata for the pinned header library and
  explicit license installation.

`python3 tools/algorithms/verify_fastlivo_source.py` reconstructs selected inputs
with `patch --fuzz=0`, then compares every source byte. Source identity does not
substitute for numerical, ROS, replay or managed flight acceptance.

The engine retains its GPL-2.0 license. Sophus retains MIT. The Vikit fork's BSD
package metadata and absence of standalone full license terms are recorded as an
upstream evidence gap; per-file notices are retained and are not re-licensed.

# AWS warehouse asset import

Source: https://github.com/aws-robotics/aws-robomaker-small-warehouse-world

Revision: `ee0af733315e78432408c3cd98d378ecee5f767c` (ros2).
Pinned codeload archive SHA256:
`c536d4a027f42bdac1a15d751ad8fb37296e14871e5af909f01207a854002211`.

The official snapshot has a root MIT-0 LICENSE. No conflicting per-model
license or external model references were found in the selected dependency
closure. The original license is retained verbatim. This record applies to
the 13 imported models and the selected full-roof/no-roof warehouse worlds, not arbitrary
other AWS/Fuel repositories.

`origins.json` records every imported model/world path, upstream and local
SHA256, and local modifications. Only SDF `file://models/` URIs are changed to
`model://`; meshes, textures, collision geometry, poses and the world are
unchanged. The unused DeskC model and nine .DS_Store cache files are not imported. No upstream launch,
controller, executable, maps or nested Git repository is installed.

Reproduce the import with `tools/scenes/import_aws_warehouse.py` and the exact
local archive. The importer checks its fingerprint, refuses unsafe tar entries
and existing destinations, and never downloads or executes upstream code.

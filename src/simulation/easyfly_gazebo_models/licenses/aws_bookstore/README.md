# AWS bookstore source snapshot

Source: https://github.com/aws-robotics/aws-robomaker-bookstore-world

Revision: `f5a2aaacd5418f9ecfa30f79e788856417f17c13` (`ros2`).
Archive SHA256: `ae7f0649410c05974f125d7ad64ce5d1b2a43a815724fb7632bb1241277a9f54`.

The pinned official snapshot contains a root MIT-0 license and no additional
model LICENSE/NOTICE declarations. This record covers only the imported world
and its 26 model dependencies. Instances share their model files.
Original license bytes, per-file upstream/local hashes and changes are retained
in `LICENSE` and `origins.json`. The original world is retained here for audit.

The EasyFly world removes only static `BookE_01_009`, whose upstream altitude
is 343582.718718m; see `world-changes.diff`. Other world bytes and all collision
meshes are preserved. SDF `file://models/` paths become `model://`, retaining
original line endings. Spotlight SDF mesh paths use the exact lowercase
`aws_spotlight_01_*` filenames already present in upstream, enabling its
previously missing visual and collision meshes.

Only dependency models are imported. Unreferenced photos, documentation images, launch code,
controllers and unrelated models are excluded. Reproduce from the exact local
archive with `tools/scenes/import_aws_indoor.py`; importing refuses existing
destinations, hash mismatches and unsafe tar members, and executes no upstream code.

# AWS house source snapshot

Source: https://github.com/aws-robotics/aws-robomaker-small-house-world

Revision: `ff9631ca6d1db9c1ba656498151464b5ab74aafe` (`ros2`).
Archive SHA256: `e459bd9d7bdabdfc40f8afc6770ceb1d774316e5da94a1048f036baa7388b2d9`.

The pinned official snapshot contains a root MIT-0 license and no additional
model LICENSE/NOTICE declarations. This record covers only the imported world
and its 63 model dependencies. Instances share their model files.
Original license bytes, per-file upstream/local hashes and changes are retained
in `LICENSE` and `origins.json`. The original world is retained here for audit.

The world is byte-identical to upstream. Nineteen referenced portrait image
copies (18 source JPEGs) are relocated inside their owning models; COLLADA
image paths change accordingly. Image bytes and geometry arrays are preserved. All layout wrappers are static;
models with incomplete dynamic inertia are not enabled as dynamic objects.

Only dependency models are imported. Unreferenced photos, documentation images, launch code,
controllers and unrelated models are excluded. Reproduce from the exact local
archive with `tools/scenes/import_aws_indoor.py`; importing refuses existing
destinations, hash mismatches and unsafe tar members, and executes no upstream code.

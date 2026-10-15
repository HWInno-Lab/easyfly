# Curated runtime resource provenance

The new warehouse uses only the maintained EasyFly Iris model, a curated GPS
model, project-owned primitive lidar visualization, and installed Gazebo
built-ins. It does not expose the full PX4 model library or the old perception
model root. Existing scenes and their historical incomplete manifests remain
unchanged.

`ORIGINS.json` records source revisions, fingerprints and authors. PX4 source
revision is `54f0455ffcd755534539a7cf33a09a20bf71d29d`; its Classic submodule is
`70683dc759cbd3ebccfa78429b564f8b1fb5d149`. The three Iris mesh files are checked
byte-for-byte against that source. Original Iris authors are Lorenz Meier and
Thomas Gubler; GPS author is Nuno Marques. Package/model metadata and the PX4
BSD-3-Clause notice are retained. The Classic package explicitly declares BSD;
the Iris attribution is also corroborated by the asset-specific license record
at https://github.com/PegasusSimulator/PegasusSimulator/blob/main/docs/licenses/assets/iris-license.rst.
EasyFly changes to the maintained aircraft configuration use the project
Apache-2.0 license. The template-to-EasyFly diff records those existing changes;
S2 does not change aircraft physics, sensor extrinsics or noise settings.

The curated MID360 configuration uses the same ray, IMU, mass, inertia and
mounting configuration as the existing project model. Only its vendor mesh
visual is replaced by a self-authored cylinder, so the unknown vendor mesh is
excluded. Existing legacy meshes are not relabeled or deleted.

`GAZEBO-COPYRIGHT` is the installed gazebo-common 11.10.2+dfsg-1 notice. Its
Files:* Apache-2 declaration covers the selected built-in sun/ground_plane
files and gazebo.material; no matching exception applies to those paths.
The system resources remain external and fingerprinted in the manifests.

Reproduce local runtime preparation with `tools/scenes/prepare_runtime_assets.py`
using the recorded PX4 source. This reads pinned local files only, checks Iris
mesh identity, records notices and leaves the original model/aircraft files
unchanged.

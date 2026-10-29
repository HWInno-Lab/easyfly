# FAST-LIO source and license record

The retained root `LICENSE` is the upstream GNU GPL version 2 text. The previous
package metadata listed only BSD; it now records the root text and the BSD
notices present in individual files. This metadata correction does not replace
or relicense those files. Original copyright, conditions and disclaimers in
`src/laserMapping.cpp`, `include/IKFoM_toolkit` and other embedded code remain
authoritative for their respective files.

The current sources can be reconstructed from selected files of
`Ericsii/FAST_LIO_ROS2` commit `59b082f0d196764d6d3dc784ccb602c26b2de597`,
the exact ikd-Tree files from `hku-mars/ikd-Tree` commit
`e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`, and the recorded workspace patch.
Several ROS2 commits have identical selected inputs; this establishes a
reconstruction base, not a uniquely recovered historical checkout.

The ikd-Tree revision contains its README and two source files but no separate
license file. Its newer upstream license cannot be silently applied backwards.
That historical licensing gap remains recorded in the adjacent provenance
manifest. The IKFoM subtree matches the selected ROS2 base and retains its
original BSD notices, including the nested MTK notices.

The selected archive, patch and per-file hashes are delivered in
`../fast_lio.sources/`. They exclude generated logs, maps and unrelated local
repository history. Local integration, source-time, input-queue, shutdown and
map-publication changes are represented in the patch; source identity does not
by itself establish permission for every distribution scenario.

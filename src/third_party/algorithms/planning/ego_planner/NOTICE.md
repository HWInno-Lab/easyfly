# EGO source, authorship and lint scope

The project owner confirmed on 2026-10-28: “EGO ROS 2 移植版是我自己写的”.
This confirms authorship of their ROS 2 port changes. It does not transfer
ownership of original EGO algorithms or embedded third-party files.
The recovered local port is Gray-Jia/ego-planner-ros2 at
b757687b6dd6bdc50ab20d764fae665b4ce5861b (plus seven recorded local files).
Its exact source selection and EasyFly patch remain in ../ego_planner.sources/.
A new public license declaration for the owner's port is pending their choice;
this record does not invent one. Existing file notices remain in force.

Original EGO's GPLv3 text is restored verbatim from ZJU-FAST-Lab/ego-planner
commit 5e72241 (the license-adding commit), under licenses/. This is upstream
license evidence, not a claim that it resolves every embedded component or
port-file grant. quadrotor_msgs retains its upstream BSD package declaration;
FindEigen3.cmake retains its original three authors and two-clause BSD statement.
The workspace root Apache license does not replace these upstream conditions.

## Mechanical copyright checks

legacy-notice-scope.json lists the 23 inherited files whose missing headers
produced the old diagnostics. Each exists in the recovered original import.
The four package CMake helpers restrict the copyright-format exception to
those exact paths AND reviewed SHA-256 contents; changed contents fail CMake
configuration until explicitly reviewed. All other cpplint rules still run on
those C++ files in a separate scoped invocation (cpplint_legacy_notices when
a second invocation is needed). Other linters and functional tests
remain registered. Newly added files remain subject to normal header checks.
This is a documented check-scope change, not permission, a retroactive author
notice, or a declaration that every source file has a complete copyright header.
D-04 tracks actual permission gaps independently of the V-02 technical checks.

FindEigen3.cmake is not excepted: its original authors/years and BSD two-clause grant are preserved. The full
standard BSD two-clause text has been added to the first comment block so the
checker can recognize both the copyright and license.

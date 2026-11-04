# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_obstacle_geometry/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
include(CMakeFindDependencyMacro)
find_dependency(Eigen3)
find_dependency(PCL COMPONENTS common segmentation)

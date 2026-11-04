# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_kcf/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
include(CMakeFindDependencyMacro)
find_dependency(easyfly_single_tracking_api)
find_dependency(OpenCV 4 COMPONENTS core tracking)

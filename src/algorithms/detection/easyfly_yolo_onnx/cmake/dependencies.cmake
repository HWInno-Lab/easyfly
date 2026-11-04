# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_yolo_onnx/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
include(CMakeFindDependencyMacro)
find_dependency(easyfly_detection_api)
find_dependency(OpenCV COMPONENTS core imgproc)

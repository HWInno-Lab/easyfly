# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_msgs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Original source image header, including on invalidation. No refreshed detection timestamp.
std_msgs/Header header
uint32 image_width
uint32 image_height
EllipseObservation[] observations
# true + empty = processed image with no targets; false = unusable input/result.
bool valid
string reason

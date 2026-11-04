# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_msgs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Geometry remains useful when semantic_source is invalid; association IDs are aliases
# resolved ONLY inside semantic_source. Never treat geometry.target_id as a global key.
std_msgs/Header header
ClassifiedObstacleArray geometry
WorldTargetArrayV2 semantic_source

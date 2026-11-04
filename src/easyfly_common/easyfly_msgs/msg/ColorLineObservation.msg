# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_msgs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Image coordinates, pixels. Each observation describes one straight colored component.
float64 center_x
float64 center_y
float64 start_x
float64 start_y
float64 end_x
float64 end_y
# Undirected axis, image +x towards image +y, [-pi/2, pi/2).
float64 angle_rad
float64 length_px
float64 width_px
float64 area_px
# Center displacement from image horizontal midpoint, [-1,1]. Positive means right.
float64 offset_x_normalized
# Contour/rectangle fill ratio [0,1], not a calibrated probability.
float64 score

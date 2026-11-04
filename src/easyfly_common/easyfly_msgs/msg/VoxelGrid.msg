# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_msgs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Axis-aligned ENU grid. Cell order: x + size_x * (y + size_y * z).
# -1 = unknown, 0 = observed free, 100 = occupied after robot inflation.
# header.stamp is the actual last integrated scan; invalid maps use stamp zero.
std_msgs/Header header
string session_id
uint64 generation
uint64 revision
bool valid
float64 resolution
geometry_msgs/Point origin
uint32 size_x
uint32 size_y
uint32 size_z
float64 inflation_radius
int8[] data

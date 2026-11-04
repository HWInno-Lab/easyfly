# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_msgs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# ROS source times are preserved. This is software pairing, NOT hardware sync.
std_msgs/Header header
std_msgs/Header first_header
std_msgs/Header second_header
# second source time minus first source time, seconds.
float64 delta_seconds
bool pair_valid
string reason
uint64 paired_count
uint64 dropped_first
uint64 dropped_second
uint32 queued_first
uint32 queued_second

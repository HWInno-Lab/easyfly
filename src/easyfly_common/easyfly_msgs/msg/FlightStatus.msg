# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_msgs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Independent flight-status observation, not permission to command the vehicle.
# Original MAVROS State header; never replaced by aggregate publication time.
std_msgs/Header header
bool valid
bool connected
bool armed
uint8 flight_mode
string flight_mode_name
# Steady-clock age since the first receipt of this distinct source observation.
# Consumers must add their own time since receipt, and check source time too.
float64 receipt_age_s

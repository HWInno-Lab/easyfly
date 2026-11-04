# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_msgs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# EXT-02 versioned contract; V1 wire definitions remain unchanged.
std_msgs/Header header
std_msgs/Header observation_header
TrackKeyV2 key
ObservationRefV2 observation
int32 class_id
string class_name
bool confidence_valid
float32 confidence
float64 horizon_seconds
float64 observation_age_seconds
float64 position_x
float64 position_y
float64 position_z
float64 velocity_x
float64 velocity_y
float64 velocity_z
float64 variance_x
float64 variance_y
float64 variance_z
bool valid
string reason

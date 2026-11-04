# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_msgs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# EXT-02 versioned contract; V1 wire definitions remain unchanged.
uint8 UNKNOWN=0
uint8 PIXELS_PER_UPDATE=1
uint8 PIXELS_PER_SECOND=2
TrackKeyV2 key
PixelBoxV2 filtered_box
PixelBoxV2 observed_box
bool filtered_box_valid
bool observed_box_valid
bool observed_this_frame
bool confirmed
uint64 hits
uint64 age
uint64 missed_updates
int32 class_id
string class_name
bool confidence_valid
float32 confidence
bool image_velocity_valid
uint8 image_velocity_unit
float64 image_velocity_x
float64 image_velocity_y

# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_msgs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# EXT-02 versioned contract; V1 wire definitions remain unchanged.
uint8 VALID=0
uint8 NOT_READY=1
uint8 INVALID_INPUT=2
uint8 UNSUPPORTED_INPUT=3
uint8 RESOURCE_LIMIT=4
uint8 BACKEND_ERROR=5
uint8 STALE=6
std_msgs/Header header
string camera_id
uint32 image_width
uint32 image_height
unique_identifier_msgs/UUID producer_session_id
uint64 epoch
uint64 sequence
uint8 status
string reason

# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_msgs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Original image header; consumers must also check valid and producer session.
std_msgs/Header header
string camera_id
string session_id
string gallery_sha256        # Empty for discovery-only mode; explicit reference manifest otherwise
string model_sha256
bool valid
string reason
VehicleIdentity[] identities

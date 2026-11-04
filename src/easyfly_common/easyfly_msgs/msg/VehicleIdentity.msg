# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_msgs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Appearance identity observation; never substitutes a tracker ID or a flight target.
int32 observation_index
int32 source_track_id
string class_name
string identity_id
string state                 # new, matched, ambiguous, invalid_crop, capacity_exhausted
float32 cosine_distance      # cosine distance, NOT a confidence probability
float32 distance_margin
float32 bbox_center_x
float32 bbox_center_y
float32 bbox_width
float32 bbox_height

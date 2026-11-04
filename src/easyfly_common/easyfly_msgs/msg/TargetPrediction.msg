# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_msgs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Constant-velocity model output, NOT a new physical observation or flight permit.
# header.stamp is the prediction reference time, frame_id must be odom.
std_msgs/Header header
# Exact measurement used by this estimate; never restamped as a measurement.
std_msgs/Header observation_header
int32 track_id
string class_name
float32 confidence
float64 horizon_seconds
float64 observation_age_seconds
float64 position_x
float64 position_y
float64 position_z
float64 velocity_x
float64 velocity_y
float64 velocity_z
# Model-based diagonal position variances [m^2], NOT calibrated sensor covariance.
float64 variance_x
float64 variance_y
float64 variance_z
bool valid
string reason

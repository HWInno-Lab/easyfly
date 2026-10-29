# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
# Read-only relay of an actual owned child Action feedback, never a command.
# header.stamp is relay receipt time, NOT an image/measurement timestamp.
# The mission executor emits once per received SDK feedback, not per polling tick.
std_msgs/Header header
string mission_goal_id
string child_goal_id
string step_type
uint32 step_index
bool active
float64 feedback_age_sec
string phase
# PrecisionLand fields; meaningful only when step_type is precision_land.
bool target_valid
float64 horizontal_error_m
float64 height_above_target_m
# Explore fields; meaningful only when step_type is explore.
float64 coverage
uint32 completed_viewpoints
geometry_msgs/Point next_viewpoint

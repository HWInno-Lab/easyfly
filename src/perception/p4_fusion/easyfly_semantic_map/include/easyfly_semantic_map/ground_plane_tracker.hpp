// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_semantic_map/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//
//
// Ground-plane acceptance and tracking for semantic_map.
//
// The tracker deliberately works on a complete, normalized plane model rather
// than a single world-frame z value.  This keeps the policy independent of the
// absolute origin of the odometry frame and makes it possible to reject a
// single-frame RANSAC plane which is close to the vehicle (for example a sparse
// horizontal return at the aircraft height).
#pragma once
#include "easyfly_ground_segmentation/ground_plane_tracker.hpp"
namespace easyfly_semantic_map
{
using easyfly::ground::GroundPlaneModel;
using easyfly::ground::GroundSpatialSupport;
using easyfly::ground::GroundPlaneCandidate;
using easyfly::ground::GroundVehiclePose;
using easyfly::ground::GroundPlaneUpdate;
using easyfly::ground::GroundPlaneTracker;
}

// Copyright 2026 EasyFly Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
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

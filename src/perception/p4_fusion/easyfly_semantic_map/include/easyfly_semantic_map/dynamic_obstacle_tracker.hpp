// Copyright 2026 EasyFly Project
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
//
#ifndef EASYFLY_SEMANTIC_MAP__DYNAMIC_OBSTACLE_TRACKER_HPP_
#define EASYFLY_SEMANTIC_MAP__DYNAMIC_OBSTACLE_TRACKER_HPP_
#include "easyfly_obstacle_geometry/dynamic_obstacle_tracker.hpp"
// Source compatibility for existing semantic_map consumers.
namespace easyfly_semantic_map
{
using easyfly::obstacle_geometry::DynamicObstacleObservation;
using easyfly::obstacle_geometry::DynamicObstacleEstimate;
using easyfly::obstacle_geometry::DynamicObstaclePredictionPoint;
using easyfly::obstacle_geometry::DynamicObstaclePredictionEnvelope;
using easyfly::obstacle_geometry::DynamicObstacleTrackerStats;
using easyfly::obstacle_geometry::DynamicObstacleTrackerConfig;
using easyfly::obstacle_geometry::DynamicObstacleFrame;
using easyfly::obstacle_geometry::DynamicObstacleTracker;
using easyfly::obstacle_geometry::tryStableObstacleId;
using easyfly::obstacle_geometry::tryFiniteFloat;
using easyfly::obstacle_geometry::tryFiniteObstacleBox;
using easyfly::obstacle_geometry::allocateStableObstacleId;
using easyfly::obstacle_geometry::boundedCompletePredictionPointCount;
using easyfly::obstacle_geometry::dynamicObservationBoxIoU;
using easyfly::obstacle_geometry::dynamicObservationSizeCompatible;
using easyfly::obstacle_geometry::dynamicObservationDistance;
using easyfly::obstacle_geometry::dynamicObservationPlanarDistance;
using easyfly::obstacle_geometry::dynamicObservationPlanarIoU;
using easyfly::obstacle_geometry::dynamicObservationZCompatible;
using easyfly::obstacle_geometry::dynamicObservationAssociationCompatible;
using easyfly::obstacle_geometry::dynamicObservationWithinPredictionBounds;
using easyfly::obstacle_geometry::dynamicHungarianAssignment;
using easyfly::obstacle_geometry::matchCurrentDynamicObservations;
using easyfly::obstacle_geometry::assignUniqueObstacleIds;
}  // namespace easyfly_semantic_map
#endif  // EASYFLY_SEMANTIC_MAP__DYNAMIC_OBSTACLE_TRACKER_HPP_

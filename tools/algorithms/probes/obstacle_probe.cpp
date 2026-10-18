// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <iomanip>
#include <iostream>
#include <limits>
#include <type_traits>
#ifdef EASYFLY_NATIVE_LEGACY
#include "legacy_clustering.hpp"
namespace geometry = easyfly_semantic_map;
#else
#include "easyfly_obstacle_geometry/clustering.hpp"
namespace geometry = easyfly::obstacle_geometry;
#ifdef EASYFLY_CHECK_LEGACY
#include "easyfly_semantic_map/dynamic_obstacle_tracker.hpp"
#include "easyfly_semantic_map/ground_spatial_support.hpp"
static_assert(std::is_same_v<easyfly_semantic_map::DynamicObstacleTracker,
  geometry::DynamicObstacleTracker>);
static_assert(std::is_same_v<easyfly_semantic_map::DynamicObstacleObservation,
  geometry::DynamicObstacleObservation>);
#endif
#endif

void trace(geometry::DynamicObstacleTracker & tracker, double stamp,
  const std::vector<geometry::DynamicObstacleObservation> & observations)
{
  const auto frame = tracker.update(stamp, observations);
  std::cout << "F " << stamp << ' ' << frame.valid << ' ' << observations.size() << '\n';
  for (const auto & o : observations) {
    std::cout << "O " << o.center_x << ' ' << o.center_y << ' ' << o.center_z << ' '
              << o.size_x << ' ' << o.size_y << ' ' << o.size_z << '\n';
    const auto e = tracker.predictionEnvelope(o, .8);
    std::cout << "E " << e.valid << ' ' << e.half_x << ' ' << e.half_y << ' ' << e.uncertainty << '\n';
  }
  for (const auto & e : frame.estimates) {
    std::cout << "T " << e.valid << ' ' << e.stable_id_valid << ' ' << e.is_dynamic << ' '
              << e.track_id << ' ' << e.velocity_x << ' ' << e.velocity_y << ' ' << e.velocity_z << '\n';
  }
  const auto & s = frame.stats;
  std::cout << "S " << s.observations << ' ' << s.matched << ' ' << s.new_tracks << ' '
            << s.rejected_by_distance << ' ' << s.rejected_by_size << ' ' << s.rejected_by_ambiguity << ' '
            << s.rejected_by_direction << ' ' << s.rejected_by_deadband << ' ' << s.rejected_by_dt << ' '
            << s.rejected_by_speed << ' ' << s.rejected_by_bbox << ' ' << s.dormant_matches << ' '
            << s.confirmed_dynamic << ' ' << s.prediction_points << '\n';
  const auto matches = geometry::matchCurrentDynamicObservations(
    observations, observations, frame.estimates, 1.5, .1, .1);
  for (const auto match : matches) {std::cout << "M " << match << '\n';}
  const auto predicted = tracker.predict(observations, frame.estimates, 2., 1., 1.);
  std::cout << "P " << predicted.size() << '\n';
  if (predicted.size() % 9 != 0) {throw std::runtime_error("partial footprint");}
  for (const auto & p : predicted) {std::cout << p.x << ' ' << p.y << ' ' << p.z << '\n';}
}

int main()
{
  std::cout << std::setprecision(17);
  // Moving/static/ambiguous/oversized/dropout/reversal/jump/clock-reset traces.
  for (int scenario = 0; scenario < 6; ++scenario) {
    geometry::DynamicObstacleTrackerConfig cfg;
    cfg.max_prediction_points = scenario == 5 ? 35 : 256;
    geometry::DynamicObstacleTracker tracker(cfg);
    for (int i = 0; i < 100; ++i) {
      const double x = scenario == 1 ? .01 * (i % 2) : .08 * i;
      std::vector<geometry::DynamicObstacleObservation> observations = {
        {x, 0., 1., .6, .4, 1.2}, {8. - x, 2., .8, .5, .3, .7}};
      if (scenario == 2) {observations[1].center_y = 0.;}
      if (scenario == 3 && i > 25 && i < 45) {observations[0].size_x = 4.;}
      if (i == 15 || i == 16 || i == 60) {observations.clear();}
      if (i == 50) {tracker.reset();}
      if (scenario == 4 && i > 70) {observations[0].center_x = 20. - x;}
      if (i % 3 == 0) {std::reverse(observations.begin(), observations.end());}
      trace(tracker, 10. + .1 * (i > 80 ? i - 30 : i), observations);
    }
  }
  // Real PCL clustering: current input only, gates, ordering and both AABB precisions.
  geometry::DynamicObstacleTracker tracker;
  for (int frame = 0; frame < 80; ++frame) {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    if (frame % 17 != 0) {
      for (int object = 0; object < 3; ++object) {
        for (int x = 0; x < 4; ++x) {
          for (int z = 0; z < 4; ++z) {
            pcl::PointXYZI p;
            p.x = static_cast<float>(object * 3. + frame * .06 + x * .08);
            p.y = object * .7F;
            p.z = object == 2 ? .5F : .5F + z * .08F;
            p.intensity = 2.F;
            cloud->push_back(p);
          }
        }
      }
    }
    const auto clusters = geometry::clusterCloud(cloud);
    std::cout << "C " << frame << ' ' << clusters.size() << '\n';
    for (const auto & cluster : clusters) {
      const auto result = geometry::outputClusterGeometry(*cloud, cluster, frame % 11 == 0 ? .1 : 8.);
      std::cout << "G " << static_cast<int>(result.status);
      for (float b : result.bounds) {std::cout << ' ' << b;}
      if (result.status == geometry::ClusterGeometryStatus::kAccepted) {
        for (float f : result.fields) {std::cout << ' ' << f;}
      }
      std::cout << '\n';
    }
    trace(tracker, 100. + frame * .1, geometry::extractCurrentDynamicObservations(cloud));
  }
  return 0;
}

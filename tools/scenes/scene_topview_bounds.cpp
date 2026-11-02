// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Native static-world bounds for preview framing. No aircraft, ROS or flight goals.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include "easyfly_transport/compat.hpp"

int main(int argc, char ** argv)
{
  if (argc != 3 || !easyfly_transport::ValidateLibrary()) {return 2;}
  if (!gazebo::setupServer()) {return 3;}
  gazebo::physics::WorldPtr world;
  int result = 4;
  try {
    world = gazebo::loadWorld(argv[1]);
    if (world) {
      world->SetPaused(false); gazebo::runWorld(world, 1); world->SetPaused(true);
      double minx=1e100, miny=1e100, minz=1e100, maxx=-1e100, maxy=-1e100, maxz=-1e100;
      std::size_t count=0;
      std::function<void(const gazebo::physics::ModelPtr &)> visit;
      visit = [&](const gazebo::physics::ModelPtr & model) {
        for (const auto & link : model->GetLinks()) {
          const auto box = link->BoundingBox();
          const auto lo=box.Min(), hi=box.Max(), size=box.Size();
          // Infinite ground planes and large thin floor slabs do not define the scene extent.
          if (!lo.IsFinite() || !hi.IsFinite() || size.X() > 2000 || size.Y() > 2000 ||
              size.Z() < .15 || size.X() <= 0 || size.Y() <= 0) {continue;}
          minx=std::min(minx,lo.X()); miny=std::min(miny,lo.Y()); minz=std::min(minz,lo.Z());
          maxx=std::max(maxx,hi.X()); maxy=std::max(maxy,hi.Y()); maxz=std::max(maxz,hi.Z()); ++count;
        }
        for (const auto & nested : model->NestedModels()) {visit(nested);}
      };
      for (const auto & model : world->Models()) {visit(model);}
      if (count && minx < maxx && miny < maxy) {
        std::ofstream out(argv[2]); out << std::setprecision(17)
          << "{\"bounds\":[" << minx << ',' << miny << ',' << minz << ','
          << maxx << ',' << maxy << ',' << maxz << "],\"links\":" << count << "}\n";
        result = out.good() ? 0 : 5;
      }
    }
  } catch (const std::exception & error) {std::cerr << error.what() << '\n';}
  if (world) {world->SetPaused(false);}
  world.reset(); gazebo::shutdown(); easyfly_transport::ShutdownConnections();
  return easyfly_transport::Finish() ? result : 9;
}

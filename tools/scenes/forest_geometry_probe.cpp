// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Explicit native geometry audit, including individual links of static forests.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/RayShape.hh>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include "easyfly_transport/compat.hpp"

int main(int argc, char** argv)
{
    if (argc != 6 || !easyfly_transport::ValidateLibrary()) return 2;
    const double start_x = std::stod(argv[3]), y = std::stod(argv[4]);
    const double length = std::stod(argv[5]);
    if (!std::isfinite(start_x) || !std::isfinite(y) || !std::isfinite(length) || length <= 0) return 2;
    if (!gazebo::setupServer()) return 3;
    auto world = gazebo::loadWorld(argv[1]);
    if (!world) return 4;
    world->SetPaused(false);
    gazebo::runWorld(world, 1);
    world->SetPaused(true);
    auto ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        world->Physics()->CreateShape("ray", gazebo::physics::CollisionPtr()));
    if (!ray) return 5;
    std::ofstream rays(argv[2]), models(std::string(argv[2]) + ".models"),
        links(std::string(argv[2]) + ".links");
    rays << std::setprecision(17) << "x,y,z,dx,dy,dz,distance,entity\n";
    const std::string header = "name,static,x,y,z,minx,miny,minz,maxx,maxy,maxz\n";
    models << std::setprecision(17) << header;
    links << std::setprecision(17) << header;
    for (const auto& model : world->Models())
    {
        auto record = [&](std::ofstream& out, const std::string& name,
                          const ignition::math::Pose3d& pose,
                          const ignition::math::AxisAlignedBox& bounds)
        {
            out << name << ',' << model->IsStatic() << ',' << pose.Pos().X() << ',' << pose.Pos().Y() << ','
                << pose.Pos().Z() << ',' << bounds.Min().X() << ',' << bounds.Min().Y() << ','
                << bounds.Min().Z() << ',' << bounds.Max().X() << ',' << bounds.Max().Y() << ','
                << bounds.Max().Z() << '\n';
        };
        record(models, model->GetName(), model->WorldPose(), model->BoundingBox());
        for (const auto& link : model->GetLinks())
            record(links, link->GetScopedName(), link->WorldPose(), link->BoundingBox());
    }
    for (int step = 0; step <= 16; ++step)
    {
        const double x = start_x + length * step / 16.;
        auto cast = [&](double z, const ignition::math::Vector3d& direction)
        {
            ray->SetPoints({x, y, z}, ignition::math::Vector3d(x, y, z) + direction * 25.);
            double distance = 25.;
            std::string entity;
            ray->GetIntersection(distance, entity);
            rays << x << ',' << y << ',' << z << ',' << direction.X() << ',' << direction.Y() << ','
                 << direction.Z() << ',' << distance << ',' << entity << '\n';
        };
        for (int height = 1; height <= 40; ++height)
            for (int angle = 0; angle < 32; ++angle)
            {
                const double radians = 2. * 3.141592653589793 * angle / 32.;
                cast(.1 * height, {std::cos(radians), std::sin(radians), 0});
            }
        cast(.3, {0, 0, -1});
        cast(.1, {0, 0, 1});
    }
    if (!rays.good() || !models.good() || !links.good()) return 6;
    rays.close(); models.close(); links.close();
    ray.reset();
    world->SetPaused(false);
    world.reset();
    gazebo::shutdown();
    easyfly_transport::ShutdownConnections();
    return easyfly_transport::Finish() ? 0 : 7;
}

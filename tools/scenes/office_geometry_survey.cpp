// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Explicit native ODE floor/clearance survey. No ROS, vehicle or flight goals.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/RayShape.hh>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include "easyfly_transport/compat.hpp"

int Survey(const gazebo::physics::WorldPtr& world, char** argv)
{
    std::cerr << "survey: stepWorld\n";
    world->SetPaused(false);
    gazebo::runWorld(world, 1);
    world->SetPaused(true);
    auto model = world->ModelByName("willowgarage");
    if (!model || !model->IsStatic()) return 5;
    auto ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        world->Physics()->CreateShape("ray", gazebo::physics::CollisionPtr()));
    if (!ray) return 6;
    const auto bounds = model->BoundingBox();
    // Caller explicitly bounds the reviewed primary building. The unchanged
    // collision mesh also has a tiny distant line marker; retain it in physics
    // without expanding this indoor survey to the entire collision AABB.
    const double minx=std::stod(argv[3]), maxx=std::stod(argv[4]);
    const double miny=std::stod(argv[5]), maxy=std::stod(argv[6]);
    const double cells = (2*(maxx-minx)+1)*(2*(maxy-miny)+1);
    std::cerr << "survey: native bounds " << bounds << "; cells " << cells << '\n';
    if (!std::isfinite(minx) || !std::isfinite(maxx) || !std::isfinite(miny) ||
        !std::isfinite(maxy) || minx>=maxx || miny>=maxy ||
        !std::isfinite(cells) || cells <= 0 || cells > 20000) return 7;
    std::ofstream out(argv[2]);
    out << std::setprecision(17)
        << "x,y,floor_z,floor_entity,clearance_m,enclosure_hits,upward_distance_m,upward_entity\n";
    for (double x=minx; x<=maxx; x+=.5)
        for (double y=miny; y<=maxy; y+=.5)
        {
            double floor_distance=25.; std::string floor_entity;
            ray->SetPoints({x,y,.3}, {x,y,-1.});
            ray->GetIntersection(floor_distance,floor_entity);
            const double floor=.3-floor_distance;
            double minimum=1000.;
            int enclosure=0;
            // Coincident z=0 floors can report the builtin ground first. Do
            // not infer room membership from that tie: separately record all
            // 32 wall intersections at vehicle height for interior selection.
            if (!floor_entity.empty() && std::abs(floor)<.25)
                for (double height : {.3, 1., 1.5, 2.3})
                    for (int angle=0; angle<32; ++angle)
                    {
                        const double a=2.*3.141592653589793*angle/32.;
                        ray->SetPoints({x,y,floor+height}, {x+25.*std::cos(a),y+25.*std::sin(a),floor+height});
                        double distance=25.; std::string entity;
                        ray->GetIntersection(distance,entity);
                        minimum=std::min(minimum,distance);
                        if (height==1.5 && distance<=25. && entity.find("willowgarage::")==0)
                            ++enclosure;
                    }
            else minimum=-1.;
            double upward=25.; std::string upward_entity;
            ray->SetPoints({x,y,floor+.1}, {x,y,floor+25.1});
            ray->GetIntersection(upward,upward_entity);
            out << x << ',' << y << ',' << floor << ',' << floor_entity << ','
                << minimum << ',' << enclosure << ',' << upward << ',' << upward_entity << '\n';
        }
    if (!out.good()) return 8;
    return 0;
}

int main(int argc, char** argv)
{
    if (argc != 7 || !easyfly_transport::ValidateLibrary()) return 2;
    std::cerr << "survey: setupServer\n";
    if (!gazebo::setupServer()) return 3;
    std::cerr << "survey: loadWorld\n";
    auto world = gazebo::loadWorld(argv[1]);
    int result=4;
    try { if (world) result=Survey(world,argv); }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; result=2; }
    // Rejecting a survey must drain transport just as a successful audit does.
    if (world) world->SetPaused(false);
    world.reset(); gazebo::shutdown(); easyfly_transport::ShutdownConnections();
    return easyfly_transport::Finish() ? result : 9;
}

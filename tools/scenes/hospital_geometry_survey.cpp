// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Native ODE support/clearance audit only. No ROS, aircraft or flight goals.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/RayShape.hh>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include "easyfly_transport/compat.hpp"

int Survey(const gazebo::physics::WorldPtr& world, char** argv)
{
    world->SetPaused(false);
    gazebo::runWorld(world, 1);
    world->SetPaused(true);
    auto ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        world->Physics()->CreateShape("ray", gazebo::physics::CollisionPtr()));
    if (!ray) return 5;
    std::ofstream models(std::string(argv[2])+".models.csv"),
        links(std::string(argv[2])+".links.csv"),
        placements(std::string(argv[2])+".placements.csv"), out(argv[2]);
    const std::string header="name,static,x,y,z,minx,miny,minz,maxx,maxy,maxz\n";
    models << std::setprecision(17) << header;
    links << std::setprecision(17) << header;
    auto record=[](std::ofstream& stream, const std::string& name, bool is_static,
                   const ignition::math::Pose3d& pose, const ignition::math::AxisAlignedBox& box)
    {
        stream << name << ',' << is_static << ',' << pose.Pos().X() << ',' << pose.Pos().Y() << ','
            << pose.Pos().Z() << ',' << box.Min().X() << ',' << box.Min().Y() << ',' << box.Min().Z()
            << ',' << box.Max().X() << ',' << box.Max().Y() << ',' << box.Max().Z() << '\n';
    };
    std::function<void(const gazebo::physics::ModelPtr&)> visit;
    visit=[&](const gazebo::physics::ModelPtr& model)
    {
        record(models, model->GetScopedName(), model->IsStatic(), model->WorldPose(), model->BoundingBox());
        for (const auto& link : model->GetLinks())
            record(links, link->GetScopedName(), link->IsStatic(), link->WorldPose(), link->BoundingBox());
        for (const auto& nested : model->NestedModels()) visit(nested);
    };
    for (const auto& model : world->Models()) visit(model);
    auto cast=[&](double x, double y, double z, const ignition::math::Vector3d& delta)
    {
        ray->SetPoints({x,y,z}, ignition::math::Vector3d(x,y,z)+delta);
        double distance=1000.; std::string entity;
        ray->GetIntersection(distance,entity);
        return std::make_pair(distance,entity);
    };
    placements << std::setprecision(17) << "name,x,y,floor_z,floor_entity\n";
    const char* names[]={"StorageRack_1", "WhiteChipChair_1", "TrolleyBed_1"};
    const double xs[]={9.07005,-8.87142,4.85934}, ys[]={-10.3582,9.15707,-4.87229};
    for (int i=0; i<3; ++i)
    {
        auto floor=cast(xs[i],ys[i],.3,{0,0,-1.3});
        placements << names[i] << ',' << xs[i] << ',' << ys[i] << ',' << .3-floor.first << ',' << floor.second << '\n';
    }
    out << std::setprecision(17) << "x,y,floor_z,floor_entity,clearance_m,wall_hits,upward_distance_m,upward_entity\n";
    const double minx=std::stod(argv[3]), maxx=std::stod(argv[4]), miny=std::stod(argv[5]), maxy=std::stod(argv[6]);
    for (double x=minx; x<=maxx+.00001; x+=.5)
        for (double y=miny; y<=maxy+.00001; y+=.5)
        {
            auto floor=cast(x,y,.3,{0,0,-1.3});
            double z=.3-floor.first, minimum=1000.; int walls=0;
            if (!floor.second.empty() && std::abs(z)<.25)
                for (double height : {.3,1.,1.5,2.3})
                    for (int angle=0; angle<32; ++angle)
                    {
                        const double a=2.*3.141592653589793*angle/32.;
                        auto hit=cast(x,y,z+height,{25.*std::cos(a),25.*std::sin(a),0});
                        minimum=std::min(minimum,hit.first);
                        if (height==1.5 && hit.first<=25. &&
                            hit.second.find("aws_robomaker_hospital_floor_01_walls_1::")==0) ++walls;
                    }
            else minimum=-1.;
            auto upward=cast(x,y,z+.1,{0,0,25.});
            out << x << ',' << y << ',' << z << ',' << floor.second << ',' << minimum << ','
                << walls << ',' << upward.first << ',' << upward.second << '\n';
        }
    return out.good() && models.good() && links.good() && placements.good() ? 0 : 6;
}

int main(int argc, char** argv)
{
    if (argc!=7 || !easyfly_transport::ValidateLibrary()) return 2;
    try
    {
        const double minx=std::stod(argv[3]),maxx=std::stod(argv[4]),miny=std::stod(argv[5]),maxy=std::stod(argv[6]);
        const double cells=(std::floor(2*(maxx-minx))+1)*(std::floor(2*(maxy-miny))+1);
        if (!std::isfinite(minx)||!std::isfinite(maxx)||!std::isfinite(miny)||!std::isfinite(maxy)||
            minx>=maxx||miny>=maxy||!std::isfinite(cells)||cells<=0||cells>20000) return 7;
    }
    catch (const std::exception&) { return 2; }
    if (!gazebo::setupServer()) return 3;
    gazebo::physics::WorldPtr world;
    int result=4;
    try { world=gazebo::loadWorld(argv[1]); if (world) result=Survey(world,argv); }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; result=2; }
    if (world) world->SetPaused(false);
    world.reset(); gazebo::shutdown(); easyfly_transport::ShutdownConnections();
    return easyfly_transport::Finish() ? result : 9;
}

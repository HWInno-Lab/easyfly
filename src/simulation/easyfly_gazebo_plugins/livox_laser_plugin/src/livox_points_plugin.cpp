//
// Livox LiDAR Gazebo 仿真插件 (ROS2 移植版)
// 原版: Livox-SDK/livox_laser_simulation (ROS1, by lfc 2021)
// 移植说明见 livox_points_plugin.h 头部注释
//

#include "livox_laser_simulation/livox_points_plugin.h"

#include <ignition/math/Vector3.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/RayShape.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>

#include <limits>
#include <string>
#include <vector>

#include "livox_laser_simulation/csv_reader.hpp"
#include "livox_laser_simulation/livox_ode_multiray_shape.h"
#include "livox_laser_simulation/ray_geometry.hpp"

#ifndef SCAN_MODE_DIR
#define SCAN_MODE_DIR "/opt/easyfly/scan_mode"
#endif

namespace gazebo {

GZ_REGISTER_SENSOR_PLUGIN(LivoxPointsPlugin)

LivoxPointsPlugin::LivoxPointsPlugin() {}

LivoxPointsPlugin::~LivoxPointsPlugin() {}

void convertDataToRotateInfo(const std::vector<std::vector<double>> &datas, std::vector<AviaRotateInfo> &avia_infos) {
    avia_infos.reserve(datas.size());
    double deg_2_rad = M_PI / 180.0;
    for (size_t i = 0; i < datas.size(); ++i) {
        auto &data = datas[i];
        if (data.size() == 3) {
            avia_infos.emplace_back();
            avia_infos.back().time = data[0];
            avia_infos.back().azimuth = data[1] * deg_2_rad;
            avia_infos.back().zenith = data[2] * deg_2_rad - M_PI_2;
            avia_infos.back().line = i % 4;
        } else {
            RCLCPP_INFO(rclcpp::get_logger("livox_laser_simulation"), "data size is not 3!");
        }
    }
}

void LivoxPointsPlugin::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr sdf) {
    std::vector<std::vector<double>> datas;
    std::string file_name = sdf->Get<std::string>("csv_file_name");

    // ROS2 移植: 支持绝对路径; 相对名则从编译期 SCAN_MODE_DIR 解析
    if (file_name.find('/') == std::string::npos) {
        file_name = std::string(SCAN_MODE_DIR) + "/" + file_name;
    }

    RCLCPP_INFO(rclcpp::get_logger("livox_laser_simulation"), "load csv file name: %s", file_name.c_str());
    if (!CsvReader::ReadCsvFile(file_name, datas)) {
        RCLCPP_ERROR(rclcpp::get_logger("livox_laser_simulation"),
                     "cannot get csv file! %s will return!", file_name.c_str());
        return;
    }

    sdfPtr = sdf;
    auto rayElem = sdfPtr->GetElement("ray");
    auto scanElem = rayElem->GetElement("scan");
    auto rangeElem = rayElem->GetElement("range");

    auto curr_scan_topic = sdf->Get<std::string>("ros_topic");
    frameName = sdf->Get<std::string>("frameName");
    RCLCPP_INFO(rclcpp::get_logger("livox_laser_simulation"), "ros topic name: %s", curr_scan_topic.c_str());
    RCLCPP_INFO(rclcpp::get_logger("livox_laser_simulation"), "ros frame id: %s", frameName.c_str());

    raySensor = _parent;

    node = transport::NodePtr(new transport::Node());
    node->Init(raySensor->WorldName());
    scanPub = node->Advertise<msgs::LaserScanStamped>(_parent->Topic(), 50);
    aviaInfos.clear();
    convertDataToRotateInfo(datas, aviaInfos);
    RCLCPP_INFO(rclcpp::get_logger("livox_laser_simulation"), "scan info size: %zu", aviaInfos.size());
    maxPointSize = aviaInfos.size();

    RayPlugin::Load(_parent, sdfPtr);
    laserMsg.mutable_scan()->set_frame(_parent->ParentName());
    parentEntity = world->EntityByName(_parent->ParentName());
    auto physics = world->Physics();
    laserCollision = physics->CreateCollision("multiray", _parent->ParentName());
    laserCollision->SetName("ray_sensor_collision");
    laserCollision->SetRelativePose(_parent->Pose());
    laserCollision->SetInitialRelativePose(_parent->Pose());
    rayShape.reset(new gazebo::physics::LivoxOdeMultiRayShape(laserCollision));
    laserCollision->SetShape(rayShape);
    samplesStep = sdfPtr->Get<int>("samples");
    downSample = sdfPtr->Get<int>("downsample");
    if (downSample < 1) {
        downSample = 1;
    }
    RCLCPP_INFO(rclcpp::get_logger("livox_laser_simulation"), "sample: %ld", (long)samplesStep);
    RCLCPP_INFO(rclcpp::get_logger("livox_laser_simulation"), "downsample: %ld", (long)downSample);

    publishPointCloudType = sdfPtr->Get<int>("publish_pointcloud_type");
    RCLCPP_INFO(rclcpp::get_logger("livox_laser_simulation"), "publish_pointcloud_type: %d", (int)publishPointCloudType);

    // ROS2 移植: rclcpp 初始化 + 创建节点 (使用仿真时钟)
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    rclcpp::NodeOptions options;
    options.append_parameter_override("use_sim_time", true);
    rosNode = rclcpp::Node::make_shared("livox_laser_simulation", options);
    switch (publishPointCloudType) {
        case SENSOR_MSG_POINT_CLOUD2_POINTXYZ:
            rosPointPub2 = rosNode->create_publisher<sensor_msgs::msg::PointCloud2>(curr_scan_topic, 5);
            break;
        case livox_laser_simulation_CUSTOM_MSG:
            rosCustomMsgPub = rosNode->create_publisher<livox_ros_driver2::msg::CustomMsg>(curr_scan_topic, 5);
            break;
        default:
            RCLCPP_WARN(rclcpp::get_logger("livox_laser_simulation"),
                        "unsupported publish_pointcloud_type: %d", (int)publishPointCloudType);
            break;
    }

    visualize = sdfPtr->Get<bool>("visualize");

    rayShape->RayShapes().reserve(samplesStep / downSample);
    rayShape->Load(sdfPtr);
    rayShape->Init();
    minDist = rangeElem->Get<double>("min");
    maxDist = rangeElem->Get<double>("max");
    auto offset = laserCollision->RelativePose();
    ignition::math::Vector3d start_point, end_point;
    for (int j = 0; j < samplesStep; j += downSample) {
        int index = j % maxPointSize;
        auto &rotate_info = aviaInfos[index];
        ignition::math::Quaterniond ray;
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
        start_point = easyfly_livox::rayStart(offset.Pos(), axis, minDist);
        end_point = maxDist * axis + offset.Pos();
        rayShape->AddRay(start_point, end_point);
    }
}

void LivoxPointsPlugin::OnNewLaserScans() {
    if (rayShape) {
        std::vector<std::pair<int, AviaRotateInfo>> points_pair;
        InitializeRays(points_pair, rayShape);
        rayShape->Update();

        msgs::Set(laserMsg.mutable_time(), world->SimTime());

        switch (publishPointCloudType) {
            case SENSOR_MSG_POINT_CLOUD2_POINTXYZ:
                PublishPointCloud2XYZ(points_pair);
                break;
            case livox_laser_simulation_CUSTOM_MSG:
                PublishLivoxROSDriverCustomMsg(points_pair);
                break;
            default:
                break;
        }
    }
}

void LivoxPointsPlugin::InitializeRays(std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
                                       boost::shared_ptr<physics::LivoxOdeMultiRayShape> &ray_shape) {
    auto &rays = ray_shape->RayShapes();
    ignition::math::Vector3d start_point, end_point;
    ignition::math::Quaterniond ray;
    auto offset = laserCollision->RelativePose();
    int64_t end_index = currStartIndex + samplesStep;
    int ray_index = 0;
    auto ray_size = rays.size();
    points_pair.clear();
    points_pair.reserve(rays.size());
    for (int64_t k = currStartIndex; k < end_index; k += downSample) {
        auto index = k % maxPointSize;
        auto &rotate_info = aviaInfos[index];
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
        start_point = easyfly_livox::rayStart(offset.Pos(), axis, minDist);
        end_point = maxDist * axis + offset.Pos();
        if (ray_index < ray_size) {
            rays[ray_index]->SetPoints(start_point, end_point);
            points_pair.emplace_back(ray_index, rotate_info);
        }
        ray_index++;
    }
    currStartIndex += samplesStep;
    if (currStartIndex > maxPointSize) {
        currStartIndex -= maxPointSize;
    }
}

void LivoxPointsPlugin::InitializeScan(msgs::LaserScan *&scan) {
    // Store the latest laser scans into laserMsg
    msgs::Set(scan->mutable_world_pose(), raySensor->Pose() + parentEntity->WorldPose());
    scan->set_angle_min(AngleMin().Radian());
    scan->set_angle_max(AngleMax().Radian());
    scan->set_angle_step(AngleResolution());
    scan->set_count(RangeCount());

    scan->set_vertical_angle_min(VerticalAngleMin().Radian());
    scan->set_vertical_angle_max(VerticalAngleMax().Radian());
    scan->set_vertical_angle_step(VerticalAngleResolution());
    scan->set_vertical_count(VerticalRangeCount());

    scan->set_range_min(RangeMin());
    scan->set_range_max(RangeMax());

    scan->clear_ranges();
    scan->clear_intensities();

    // 填充 repeated field, 否则 set_ranges(index) 会越界触发 protobuf 断言崩溃
    unsigned int rangeCount = RangeCount();
    unsigned int verticalRangeCount = VerticalRangeCount();
    for (unsigned int j = 0; j < verticalRangeCount; ++j) {
        for (unsigned int i = 0; i < rangeCount; ++i) {
            scan->add_ranges(0);
            scan->add_intensities(0);
        }
    }
}

ignition::math::Angle LivoxPointsPlugin::AngleMin() const {
    if (rayShape)
        return rayShape->MinAngle();
    else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::AngleMax() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->MaxAngle().Radian());
    } else
        return -1;
}

double LivoxPointsPlugin::RangeMin() const {
    if (rayShape)
        return rayShape->GetMinRange();
    else
        return -1;
}

double LivoxPointsPlugin::RangeMax() const {
    if (rayShape)
        return rayShape->GetMaxRange();
    else
        return -1;
}

double LivoxPointsPlugin::AngleResolution() const {
    return (AngleMax() - AngleMin()).Radian() / (RangeCount() - 1);
}

int LivoxPointsPlugin::RayCount() const {
    if (rayShape)
        return rayShape->GetSampleCount();
    else
        return -1;
}

int LivoxPointsPlugin::RangeCount() const {
    if (rayShape)
        return rayShape->GetSampleCount() * rayShape->GetScanResolution();
    else
        return -1;
}

int LivoxPointsPlugin::VerticalRayCount() const {
    if (rayShape)
        return rayShape->GetVerticalSampleCount();
    else
        return -1;
}

int LivoxPointsPlugin::VerticalRangeCount() const {
    if (rayShape)
        return rayShape->GetVerticalSampleCount() * rayShape->GetVerticalScanResolution();
    else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::VerticalAngleMin() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->VerticalMinAngle().Radian());
    } else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::VerticalAngleMax() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->VerticalMaxAngle().Radian());
    } else
        return -1;
}

double LivoxPointsPlugin::VerticalAngleResolution() const {
    return (VerticalAngleMax() - VerticalAngleMin()).Radian() / (VerticalRangeCount() - 1);
}

void LivoxPointsPlugin::PublishPointCloud2XYZ(std::vector<std::pair<int, AviaRotateInfo>> &points_pair) {
    auto rayCount = RayCount();
    auto verticalRayCount = VerticalRayCount();
    auto angle_min = AngleMin().Radian();
    auto angle_incre = AngleResolution();
    auto verticle_min = VerticalAngleMin().Radian();
    auto verticle_incre = VerticalAngleResolution();

    msgs::LaserScan *scan = laserMsg.mutable_scan();
    InitializeScan(scan);

    // ROS2 移植: 手工构造 PointCloud2 (x,y,z,intensity), 不依赖 PCL
    sensor_msgs::msg::PointCloud2 cloud;
    // The ROS /clock subscription can lag or repeat while Gazebo produces
    // distinct scans. Use this scan's native simulator measurement time,
    // already recorded for the Gazebo LaserScanStamped message.
    cloud.header.stamp.sec = laserMsg.time().sec();
    cloud.header.stamp.nanosec = laserMsg.time().nsec();
    cloud.header.frame_id = frameName;
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = true;

    sensor_msgs::msg::PointField field;
    field.name = "x";
    field.offset = 0;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(field);
    field.name = "y";
    field.offset = 4;
    cloud.fields.push_back(field);
    field.name = "z";
    field.offset = 8;
    cloud.fields.push_back(field);
    field.name = "intensity";
    field.offset = 12;
    cloud.fields.push_back(field);
    const uint32_t point_step = 16;

    cloud.data.reserve(points_pair.size() * point_step);
    for (auto &pair : points_pair) {
        int verticle_index = roundf((pair.second.zenith - verticle_min) / verticle_incre);
        int horizon_index = roundf((pair.second.azimuth - angle_min) / angle_incre);
        if (verticle_index < 0 || horizon_index < 0) {
            continue;
        }
        if (verticle_index < verticalRayCount && horizon_index < rayCount) {
            auto index = (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
            auto range = easyfly_livox::sensorRange(
                rayShape->RayShapes()[pair.first]->GetLength(), minDist);
            auto intensity = rayShape->GetRetro(pair.first);
            if (range >= maxDist || range <= minDist || range <= 1e-5) {
                range = 0.0;
            }
            scan->set_ranges(index, range);
            scan->set_intensities(index, intensity);

            auto rotate_info = pair.second;
            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            auto point = range * axis;

            if (range > 1e-5) {
                float xyz[4] = {(float)point.X(), (float)point.Y(), (float)point.Z(), (float)intensity};
                const uint8_t *bytes = reinterpret_cast<const uint8_t *>(xyz);
                cloud.data.insert(cloud.data.end(), bytes, bytes + point_step);
            }
        }
    }
    cloud.width = cloud.data.size() / point_step;
    cloud.point_step = point_step;
    cloud.row_step = cloud.data.size();
    rosPointPub2->publish(cloud);

    if (scanPub && scanPub->HasConnections() && visualize) {
        scanPub->Publish(laserMsg);
    }
}

void LivoxPointsPlugin::PublishLivoxROSDriverCustomMsg(std::vector<std::pair<int, AviaRotateInfo>> &points_pair) {
    auto rayCount = RayCount();
    auto verticalRayCount = VerticalRayCount();
    auto angle_min = AngleMin().Radian();
    auto angle_incre = AngleResolution();
    auto verticle_min = VerticalAngleMin().Radian();
    auto verticle_incre = VerticalAngleResolution();

    msgs::LaserScan *scan = laserMsg.mutable_scan();
    InitializeScan(scan);

    livox_ros_driver2::msg::CustomMsg msg;
    msg.header.frame_id = frameName;

    msg.header.stamp.sec = laserMsg.time().sec();
    msg.header.stamp.nanosec = laserMsg.time().nsec();
    msg.timebase = static_cast<uint64_t>(laserMsg.time().sec()) * 1000000000ULL +
        static_cast<uint64_t>(laserMsg.time().nsec());

    msg.points.reserve(points_pair.size());
    for (int i = 0; i < points_pair.size(); ++i) {
        std::pair<int, AviaRotateInfo> &pair = points_pair[i];
        int verticle_index = roundf((pair.second.zenith - verticle_min) / verticle_incre);
        int horizon_index = roundf((pair.second.azimuth - angle_min) / angle_incre);
        if (verticle_index < 0 || horizon_index < 0) {
            continue;
        }
        if (verticle_index < verticalRayCount && horizon_index < rayCount) {
            auto index = (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
            auto range = easyfly_livox::sensorRange(
                rayShape->RayShapes()[pair.first]->GetLength(), minDist);
            auto intensity = rayShape->GetRetro(pair.first);
            if (range >= maxDist || range <= minDist || abs(range) <= 1e-5) {
                range = 0.0;
            }
            scan->set_ranges(index, range);
            scan->set_intensities(index, intensity);

            auto rotate_info = pair.second;
            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            auto point = range * axis;

            livox_ros_driver2::msg::CustomPoint pt;
            pt.x = point.X();
            pt.y = point.Y();
            pt.z = point.Z();
            pt.line = pair.second.line;
            pt.tag = 0x10;
            pt.reflectivity = 100;
            pt.offset_time = (uint32_t)(1e9 / 200000 * i);
            msg.points.push_back(pt);
        }
    }
    msg.point_num = msg.points.size();
    rosCustomMsgPub->publish(msg);

    if (scanPub && scanPub->HasConnections() && visualize) {
        scanPub->Publish(laserMsg);
    }
}

}  // namespace gazebo

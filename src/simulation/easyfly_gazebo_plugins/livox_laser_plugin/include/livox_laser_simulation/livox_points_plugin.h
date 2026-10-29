//
// Livox LiDAR Gazebo 仿真插件 (ROS2 移植版)
// 原版: Livox-SDK/livox_laser_simulation (ROS1, by lfc 2021)
// 移植: EasyFly 阶段三 M0 — ROS2 Humble 适配
//   - ros::NodeHandle      → rclcpp::Node (use_sim_time=true)
//   - livox_laser_simulation::CustomMsg → livox_ros_driver2::msg::CustomMsg
//   - 移除插件内 TF 广播 (TF 由 easyfly_coord_transform 统一管理)
//   - 保留 PointCloud2(XYZ) 与 CustomMsg 两种发布类型
//

#ifndef EASYFLY_LIVOX_LASER_PLUGIN_POINTS_PLUGIN_H
#define EASYFLY_LIVOX_LASER_PLUGIN_POINTS_PLUGIN_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <gazebo/plugins/RayPlugin.hh>

#include "livox_laser_simulation/livox_ode_multiray_shape.h"

namespace gazebo {

struct AviaRotateInfo {
    double time;
    double azimuth;
    double zenith;
    uint8_t line;
};

class LivoxPointsPlugin : public RayPlugin {
 public:
    LivoxPointsPlugin();
    virtual ~LivoxPointsPlugin();

    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

 private:
    ignition::math::Angle AngleMin() const;
    ignition::math::Angle AngleMax() const;
    double AngleResolution() const;
    double RangeMin() const;
    double RangeMax() const;
    int RayCount() const;
    int RangeCount() const;
    int VerticalRayCount() const;
    int VerticalRangeCount() const;
    ignition::math::Angle VerticalAngleMin() const;
    ignition::math::Angle VerticalAngleMax() const;
    double VerticalAngleResolution() const;

 protected:
    virtual void OnNewLaserScans();

 private:
    enum PointCloudType {
        SENSOR_MSG_POINT_CLOUD2_POINTXYZ = 1,
        livox_laser_simulation_CUSTOM_MSG = 3,
    };

    void InitializeRays(std::vector<std::pair<int, AviaRotateInfo>>& points_pair,
                        boost::shared_ptr<physics::LivoxOdeMultiRayShape>& ray_shape);

    void InitializeScan(msgs::LaserScan*& scan);

    void PublishPointCloud2XYZ(std::vector<std::pair<int, AviaRotateInfo>>& points_pair);
    void PublishLivoxROSDriverCustomMsg(std::vector<std::pair<int, AviaRotateInfo>>& points_pair);
    void PublishRayObservations(const std::vector<std::pair<int, AviaRotateInfo>>& points_pair);

    boost::shared_ptr<physics::LivoxOdeMultiRayShape> rayShape;
    gazebo::physics::CollisionPtr laserCollision;
    physics::EntityPtr parentEntity;
    transport::PublisherPtr scanPub;
    sdf::ElementPtr sdfPtr;
    msgs::LaserScanStamped laserMsg;
    transport::NodePtr node;
    gazebo::sensors::SensorPtr raySensor;
    std::vector<AviaRotateInfo> aviaInfos;

    rclcpp::Node::SharedPtr rosNode;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rosPointPub2;
    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr rosCustomMsgPub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rosRayPub;

    int64_t samplesStep = 0;
    int64_t currStartIndex = 0;
    int64_t maxPointSize = 1000;
    int64_t downSample = 1;
    uint16_t publishPointCloudType;
    bool visualize = false;
    std::string frameName = "livox_frame";

    double maxDist = 400.0;
    double minDist = 0.1;
};

}  // namespace gazebo

#endif  // EASYFLY_LIVOX_LASER_PLUGIN_POINTS_PLUGIN_H

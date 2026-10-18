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
// EasyFly P4: 障碍表征节点 (semantic_map, M4.0)
//
// 数据流:
//   /pointcloud_filtered (PointCloud2, base_link 系, 来自 easyfly_pointcloud_filter)
//     → TF 变换到 world 系 (odom) → 当前有效帧
//     → 地面候选: RANSAC 拟合水平主平面 + 可信平面跟踪 (SACMODEL_PERPENDICULAR_PLANE,
//       通过时间一致性、车辆相对高度与平滑更新拒绝退化平面; 写死世界高度裁剪带
//       已实测失效, 地面漏点会把聚类链成跨场景巨簇)
//     → PCL 欧式聚类 → 每簇轴对齐包围盒
//     → /obstacles        (easyfly_msgs/ObstacleInfoArray, world 系)
//     → /get_obstacles    (srv: 按距机体水平距离查询)
//     → /occupancy_grid   (nav_msgs/OccupancyGrid, 2D 投影 + 膨胀 + 持久化
//       衰减 — 聚类逐帧闪烁会使规划器拿到缺障碍的稀疏帧, 实测近乎撞人;
//       占据格保留 ~6 帧才释放, 规划/EGO 消费稳定)
//     → /obstacle_markers (RViz 立方体可视化)
//
// obstacle_type 仍由后续语义融合补充; 动态字段由有限资源的跨帧几何跟踪器
// 填写。当前帧观测若能保守匹配到当前输出簇, obstacle_id 使用 tracker 的稳定
// ID, 否则保留帧内回退索引。只有连续、无歧义且速度方向一致的观测才会标记
// is_dynamic, 否则 velocity 保持 0 并且不发布前向扫掠点。
//
// 非地面障碍不做跨帧点云混合：当前帧直接聚类，已确认动态目标只追加有限的
// 前向扫掠点。短期漏检记忆由 GridMap 自己的 trusted persistence 负责，避免
// semantic_map 同时维护滑窗与动态 ghost TTL 造成重复残留或误删静态物体。
#include <yaml-cpp/yaml.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <std_msgs/msg/header.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <easyfly_msgs/msg/obstacle_info_array.hpp>
#include <easyfly_msgs/msg/uav_state.hpp>
#include <easyfly_msgs/srv/get_obstacles.hpp>

#include "easyfly_ground_segmentation/ground_plane_tracker.hpp"
#include "easyfly_ground_segmentation/ground_candidate_selection.hpp"
#include "easyfly_ground_segmentation/ground_spatial_support.hpp"
#include "easyfly_ground_segmentation/ground_plane_transform.hpp"
#include "easyfly_ground_segmentation/ground_fitting.hpp"
#include "easyfly_semantic_map/grid_inflation.hpp"
#include "easyfly_semantic_map/observed_grid.hpp"
#include "easyfly_semantic_map/map_validity.hpp"
#include "easyfly_semantic_map/dynamic_obstacle_tracker.hpp"
#include "easyfly_obstacle_geometry/clustering.hpp"

namespace easyfly_semantic_map
{
using easyfly::ground::isGroundFitSample;
using easyfly::ground::groundSpatialSupport;
using easyfly::ground::GroundPlaneModel;
using easyfly::ground::GroundSpatialSupport;
using easyfly::ground::GroundPlaneCandidate;
using easyfly::ground::GroundVehiclePose;
using easyfly::ground::GroundPlaneUpdate;
using easyfly::ground::GroundPlaneTracker;
using easyfly::ground::groundPlaneInWorld;
using easyfly::ground::groundBasisContinuous;
using easyfly::ground::groundBasisStampFresh;


using easyfly_msgs::msg::ObstacleInfo;
using easyfly_msgs::msg::ObstacleInfoArray;

// Resource limits are deliberately finite even when a YAML file is malformed
// or an upstream publisher sends an unexpectedly large cloud.  The configured
// limits below are validated against these hard caps before any PCL conversion
// or current-frame/prediction cloud allocation can grow proportional storage.
constexpr int kHardMaxInputPoints = 500000;
constexpr int kHardMaxNongroundPoints = 1000000;
constexpr double kMinimumNongroundVoxelSize = 0.02;
// Keep the semantic producer's total output (current cloud plus any
// prediction points) within GridMap's trusted-cloud input limit.
constexpr int kDownstreamTrustedCloudPointLimit = 200000;

class SemanticMapNode : public rclcpp::Node
{
public:
  explicit SemanticMapNode(const rclcpp::NodeOptions & options)
  : Node("easyfly_semantic_map", options)
  {
    loadParams();
    grid_footprint_mode_ = declare_parameter<std::string>("grid_footprint_mode", "cluster_boxes");
    if (grid_footprint_mode_ != "cluster_boxes" && grid_footprint_mode_ != "observed_cells") {
      throw std::invalid_argument("grid_footprint_mode must be cluster_boxes or observed_cells");
    }
    grid_height_limit_m_ = declare_parameter<double>("grid_height_limit_m", 0.0);
    if (!std::isfinite(grid_height_limit_m_) || grid_height_limit_m_ < 0.0 ||
      (grid_height_limit_m_ > 0.0 && grid_footprint_mode_ != "observed_cells"))
    {
      throw std::invalid_argument(
              "grid height limit requires finite nonnegative value and observed_cells");
    }
    const auto ground_frame = declare_parameter<std::string>("ground_model_frame", "");
    if (!ground_frame.empty()) {params_.ground_model_frame = ground_frame;}

    DynamicObstacleTrackerConfig dynamic_config;
    dynamic_config.association_gate = params_.dynamic_association_gate;
    dynamic_config.association_ambiguity_distance =
      params_.dynamic_association_ambiguity_distance;
    dynamic_config.association_cost_margin = params_.dynamic_association_cost_margin;
    dynamic_config.association_z_gate = params_.dynamic_association_z_gate;
    dynamic_config.association_xy_size_ratio = params_.dynamic_association_xy_size_ratio;
    dynamic_config.association_z_size_ratio = params_.dynamic_association_z_size_ratio;
    dynamic_config.min_dt = params_.dynamic_min_dt;
    dynamic_config.max_dt = params_.dynamic_max_dt;
    dynamic_config.speed_deadband = params_.dynamic_speed_deadband;
    dynamic_config.max_speed = params_.dynamic_max_speed;
    dynamic_config.min_direction_cosine = params_.dynamic_min_direction_cosine;
    dynamic_config.min_confirmations = static_cast<std::size_t>(
      std::max(1, params_.dynamic_min_confirmations));
    dynamic_config.velocity_ema_alpha = params_.dynamic_velocity_ema_alpha;
    dynamic_config.min_displacement = params_.dynamic_min_displacement;
    dynamic_config.dormant_max_misses = static_cast<std::size_t>(
      std::max(0, params_.dynamic_dormant_max_misses));
    dynamic_config.prediction_horizon = params_.dynamic_prediction_horizon;
    dynamic_config.prediction_steps = static_cast<std::size_t>(
      std::max(1, params_.dynamic_prediction_steps));
    dynamic_config.max_prediction_points = static_cast<std::size_t>(
      std::max(0, params_.dynamic_max_prediction_points));
    dynamic_config.prediction_min_half_x = params_.dynamic_prediction_min_half_x;
    dynamic_config.prediction_min_half_y = params_.dynamic_prediction_min_half_y;
    dynamic_config.prediction_uncertainty_growth =
      params_.dynamic_prediction_uncertainty_growth;
    dynamic_config.prediction_max_uncertainty = params_.dynamic_prediction_max_uncertainty;
    dynamic_config.max_dynamic_bbox_size_x = params_.dynamic_max_bbox_size_x;
    dynamic_config.max_dynamic_bbox_size_y = params_.dynamic_max_bbox_size_y;
    dynamic_config.max_dynamic_bbox_size_z = params_.dynamic_max_bbox_size_z;
    dynamic_tracker_ = DynamicObstacleTracker(dynamic_config);

    GroundPlaneTracker::Params ground_params;
    ground_params.max_tilt_rad = params_.plane_max_tilt_rad;
    ground_params.min_inliers = static_cast<std::size_t>(
      std::max(1, params_.plane_min_inliers));
    ground_params.min_inlier_ratio = params_.plane_min_inlier_ratio;
    ground_params.spatial_support_enabled = params_.ground_spatial_support_enabled;
    ground_params.min_ground_clearance = params_.ground_min_clearance;
    ground_params.max_ground_distance_below_vehicle =
      params_.ground_max_distance_below_vehicle;
    ground_params.init_consecutive_frames = params_.ground_init_consecutive_frames;
    ground_params.init_state_timeout = params_.ground_init_state_timeout;
    ground_params.init_max_z_difference = params_.ground_init_max_z_difference;
    ground_params.init_max_normal_angle_rad = params_.ground_init_max_normal_angle_rad;
    ground_params.max_innovation_z = params_.ground_max_innovation_z;
    ground_params.max_innovation_angle_rad = params_.ground_max_innovation_angle_rad;
    ground_params.smoothing_alpha = params_.ground_smoothing_alpha;
    ground_tracker_ = GroundPlaneTracker(ground_params);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // Keep the last map state for late subscribers.  This is important
    // during reset: a planner that discovers the publisher after the
    // reset must receive invalid/not-ready, not an old valid map.
    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    obstacles_pub_ = create_publisher<ObstacleInfoArray>(params_.output_topic, map_qos);
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(params_.grid_topic, map_qos);
    // EGO consumes only this trusted ground-removed cloud.  There is no
    // raw-cloud fallback while GroundPlaneTracker is not ready.
    nonground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      params_.nonground_topic, rclcpp::QoS(10));
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      params_.marker_topic, 10);
    state_sub_ = create_subscription<easyfly_msgs::msg::UAVState>(
      params_.uav_state_topic, 10,
      [this](easyfly_msgs::msg::UAVState::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(state_mtx_);
        latest_state_ = *msg;
        state_received_ = true;
        state_arrival_ = std::chrono::steady_clock::now();
      });

    // Use a wall timer only to schedule the watchdog; elapsed time is
    // measured with the node's ROS clock so simulation time pauses and
    // resets are handled consistently with the published cloud stamps.
    input_watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&SemanticMapNode::checkInputWatchdog, this));

    // Publish the explicit invalid/not-ready protocol immediately.  A
    // consumer that was already running must not retain a previous map
    // while this node is rebuilding after a restart.
    invalidateMap();

    // 服务独立回调组 (响应内做 TF 查询, 不阻塞点云回调)
    srv_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_ = create_service<easyfly_msgs::srv::GetObstacles>(
      params_.service_name,
      std::bind(
        &SemanticMapNode::onGetObstacles, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, srv_group_);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      params_.input_topic, 10,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {onCloud(msg);});

    RCLCPP_INFO(
      get_logger(),
      "semantic_map 启动: %s → %s (world=%s, 聚类容差 %.2fm, min %d 点, "
      "栅格 %.0fx%.0fm@%.2fm, non-ground=%s)",
      params_.input_topic.c_str(), params_.output_topic.c_str(),
      params_.world_frame.c_str(), params_.cluster_tolerance,
      params_.cluster_min_points, params_.grid_width, params_.grid_height,
      params_.grid_resolution, params_.nonground_topic.c_str());
    const auto & effective_dynamic_config = dynamic_tracker_.config();
    RCLCPP_INFO(
      get_logger(),
      "动态障碍跟踪: xy_gate=%.2fm z_gate=%.2fm xy_size_ratio<=%.1f "
      "z_size_ratio<=%.1f ambiguity_distance=%.2fm cost_margin=%.2f "
      "deadband=%.2fm speed<=%.2fm/s confirm=%d displacement>=%.2fm "
      "ema=%.2f dormant_misses<=%d cos>=%.2f horizon=%.2fs steps=%d "
      "footprint=9 min_half=(%.2f,%.2f)m uncertainty_growth=%.2fm/s "
      "uncertainty_max=%.2fm dynamic_bbox<=(%.2f,%.2f,%.2f)m "
      "prediction_points<=%d (current frame only)",
      effective_dynamic_config.association_gate,
      effective_dynamic_config.association_z_gate,
      effective_dynamic_config.association_xy_size_ratio,
      effective_dynamic_config.association_z_size_ratio,
      effective_dynamic_config.association_ambiguity_distance,
      effective_dynamic_config.association_cost_margin,
      effective_dynamic_config.speed_deadband, effective_dynamic_config.max_speed,
      static_cast<int>(effective_dynamic_config.min_confirmations),
      effective_dynamic_config.min_displacement, effective_dynamic_config.velocity_ema_alpha,
      static_cast<int>(effective_dynamic_config.dormant_max_misses),
      effective_dynamic_config.min_direction_cosine,
      effective_dynamic_config.prediction_horizon,
      static_cast<int>(effective_dynamic_config.prediction_steps),
      effective_dynamic_config.prediction_min_half_x,
      effective_dynamic_config.prediction_min_half_y,
      effective_dynamic_config.prediction_uncertainty_growth,
      effective_dynamic_config.prediction_max_uncertainty,
      effective_dynamic_config.max_dynamic_bbox_size_x,
      effective_dynamic_config.max_dynamic_bbox_size_y,
      effective_dynamic_config.max_dynamic_bbox_size_z,
      static_cast<int>(effective_dynamic_config.max_prediction_points));
  }

private:
  struct NodeParams
  {
    std::string input_topic{"/pointcloud_filtered"};
    std::string uav_state_topic{"/uav_state"};
    std::string output_topic{"/obstacles"};
    std::string nonground_topic{"/pointcloud_nonground"};
    std::string grid_topic{"/occupancy_grid"};
    std::string marker_topic{"/obstacle_markers"};
    std::string service_name{"/get_obstacles"};
    std::string world_frame{"odom"};
    std::string base_frame{"base_link"};
    std::string ground_model_frame;  // empty preserves the legacy world-frame model
    double plane_distance_threshold{0.12};      // 地面平面厚度 [m] (RANSAC 内点距离)
    double plane_max_tilt_rad{0.15};
    int plane_min_inliers{50};
    double plane_min_inlier_ratio{0.15};
    bool ground_spatial_support_enabled{false};
    // Direct landed SITL traces are 0.053-0.054m; retain about 0.02m of
    // measured margin while rejecting zero/near-body-height planes.
    double ground_min_clearance{0.03};
    double ground_max_distance_below_vehicle{8.0};
    int ground_init_consecutive_frames{3};
    double ground_init_max_z_difference{0.25};
    double ground_init_max_normal_angle_rad{0.12};
    double ground_max_innovation_z{0.45};
    double ground_max_innovation_angle_rad{0.15};
    double ground_smoothing_alpha{0.25};
    double ground_init_state_timeout{1.0};
    double tf_guard_jump_distance{1.0};
    int tf_guard_init_consecutive_frames{3};
    int tf_guard_reject_limit{10};
    double obstacle_z_max{4.0};            // 关心高度上限 [m, 离拟合地面]
    double cluster_tolerance{0.4};       // 欧式聚类容差 [m]
    int cluster_min_points{5};
    int cluster_max_points{100000};
    double nonground_voxel_size{0.10};      // EGO 输入下采样叶尺寸 [m]
    double input_timeout{1.0};            // 输入到达超时 (node ROS time) [s]
    double cloud_stamp_timeout{1.0};      // 点云时间戳最大允许旧度 [s]
    double cloud_stamp_future_tolerance{0.2};      // 点云未来时间容差 [s]
    double clock_jump_forward_limit{10.0};      // ROS 时钟异常前跳门限 [s]
    int max_input_points{kDownstreamTrustedCloudPointLimit};
    // 当前非地面+预测点总上限; must equal GridMap's 200000 limit.
    int max_nonground_points{kDownstreamTrustedCloudPointLimit};
    double max_obstacle_size{8.0};       // 单簇包围盒上界 [m] (超过视为噪声链/地面残留)
    double inflate_radius{kDefaultGridInflationRadius};      // 栅格膨胀半径 [m]
    bool verbose{false};                 // 逐帧统计日志 (调参用)
    int grid_persist_frames{20};         // 占据保持帧数 (≈3s @7Hz; 漏检不立即可穿越)
    double grid_resolution{0.2};         // 栅格分辨率 [m/cell]
    double grid_width{100.0};            // 栅格宽 [m] (以 world 原点为中心)
    double grid_height{100.0};
    // 动态障碍跟踪/预测参数; 统一由 ROS-free helper 做硬上限归一化。
    double dynamic_association_gate{1.5};
    double dynamic_association_ambiguity_distance{0.10};
    double dynamic_association_cost_margin{0.10};
    double dynamic_association_z_gate{1.0};
    double dynamic_association_xy_size_ratio{6.0};
    double dynamic_association_z_size_ratio{8.0};
    double dynamic_min_dt{0.03};
    double dynamic_max_dt{1.5};
    double dynamic_speed_deadband{0.20};
    double dynamic_max_speed{5.0};
    double dynamic_min_direction_cosine{0.60};
    int dynamic_min_confirmations{2};
    double dynamic_velocity_ema_alpha{0.50};
    double dynamic_min_displacement{0.15};
    int dynamic_dormant_max_misses{2};
    double dynamic_prediction_horizon{1.5};
    int dynamic_prediction_steps{8};
    int dynamic_max_prediction_points{256};
    double dynamic_prediction_min_half_x{0.20};
    double dynamic_prediction_min_half_y{0.20};
    double dynamic_prediction_uncertainty_growth{0.10};
    double dynamic_prediction_max_uncertainty{0.50};
    double dynamic_max_bbox_size_x{2.5};
    double dynamic_max_bbox_size_y{2.5};
    double dynamic_max_bbox_size_z{2.5};
  };

  void loadParams()
  {
    declare_parameter<std::string>("config_file", "");
    std::string cfg = get_parameter("config_file").as_string();
    if (cfg.empty()) {
      cfg = ament_index_cpp::get_package_share_directory("easyfly_semantic_map") +
        "/config/semantic_map.yaml";
    }
    YAML::Node root = YAML::LoadFile(cfg);
    const auto & p = root["semantic_map"];
    params_.input_topic = p["input_topic"].as<std::string>(params_.input_topic);
    params_.uav_state_topic =
      p["uav_state_topic"].as<std::string>(params_.uav_state_topic);
    params_.output_topic = p["output_topic"].as<std::string>(params_.output_topic);
    params_.nonground_topic =
      p["nonground_topic"].as<std::string>(params_.nonground_topic);
    params_.grid_topic = p["grid_topic"].as<std::string>(params_.grid_topic);
    params_.marker_topic = p["marker_topic"].as<std::string>(params_.marker_topic);
    params_.service_name = p["service_name"].as<std::string>(params_.service_name);
    params_.world_frame = p["world_frame"].as<std::string>(params_.world_frame);
    params_.base_frame = p["base_frame"].as<std::string>(params_.base_frame);
    params_.ground_model_frame =
      p["ground_model_frame"].as<std::string>(params_.ground_model_frame);
    params_.plane_distance_threshold =
      p["plane_distance_threshold"].as<double>(params_.plane_distance_threshold);
    params_.plane_max_tilt_rad =
      p["plane_max_tilt_rad"].as<double>(params_.plane_max_tilt_rad);
    params_.plane_min_inliers = p["plane_min_inliers"].as<int>(params_.plane_min_inliers);
    params_.plane_min_inlier_ratio =
      p["plane_min_inlier_ratio"].as<double>(params_.plane_min_inlier_ratio);
    params_.ground_spatial_support_enabled =
      p["ground_spatial_support_enabled"].as<bool>(params_.ground_spatial_support_enabled);
    params_.ground_min_clearance =
      p["ground_min_clearance"].as<double>(params_.ground_min_clearance);
    params_.ground_max_distance_below_vehicle =
      p["ground_max_distance_below_vehicle"].as<double>(
      params_.ground_max_distance_below_vehicle);
    params_.ground_init_consecutive_frames =
      p["ground_init_consecutive_frames"].as<int>(params_.ground_init_consecutive_frames);
    params_.ground_init_max_z_difference =
      p["ground_init_max_z_difference"].as<double>(params_.ground_init_max_z_difference);
    params_.ground_init_max_normal_angle_rad =
      p["ground_init_max_normal_angle_rad"].as<double>(
      params_.ground_init_max_normal_angle_rad);
    params_.ground_max_innovation_z =
      p["ground_max_innovation_z"].as<double>(params_.ground_max_innovation_z);
    params_.ground_max_innovation_angle_rad =
      p["ground_max_innovation_angle_rad"].as<double>(
      params_.ground_max_innovation_angle_rad);
    params_.ground_smoothing_alpha =
      p["ground_smoothing_alpha"].as<double>(params_.ground_smoothing_alpha);
    params_.ground_init_state_timeout =
      p["ground_init_state_timeout"].as<double>(params_.ground_init_state_timeout);
    params_.tf_guard_jump_distance =
      p["tf_guard_jump_distance"].as<double>(params_.tf_guard_jump_distance);
    params_.tf_guard_init_consecutive_frames =
      p["tf_guard_init_consecutive_frames"].as<int>(params_.tf_guard_init_consecutive_frames);
    params_.tf_guard_reject_limit =
      p["tf_guard_reject_limit"].as<int>(params_.tf_guard_reject_limit);
    params_.obstacle_z_max = p["obstacle_z_max"].as<double>(params_.obstacle_z_max);
    params_.cluster_tolerance = p["cluster_tolerance"].as<double>(params_.cluster_tolerance);
    params_.cluster_min_points = p["cluster_min_points"].as<int>(params_.cluster_min_points);
    params_.cluster_max_points = p["cluster_max_points"].as<int>(params_.cluster_max_points);
    params_.nonground_voxel_size =
      p["nonground_voxel_size"].as<double>(params_.nonground_voxel_size);
    params_.input_timeout = p["input_timeout"].as<double>(params_.input_timeout);
    params_.cloud_stamp_timeout =
      p["cloud_stamp_timeout"].as<double>(params_.cloud_stamp_timeout);
    params_.cloud_stamp_future_tolerance = p["cloud_stamp_future_tolerance"].as<double>(
      params_.cloud_stamp_future_tolerance);
    params_.clock_jump_forward_limit =
      p["clock_jump_forward_limit"].as<double>(params_.clock_jump_forward_limit);
    params_.max_input_points =
      p["max_input_points"].as<int>(params_.max_input_points);
    params_.max_nonground_points =
      p["max_nonground_points"].as<int>(params_.max_nonground_points);
    params_.max_obstacle_size = p["max_obstacle_size"].as<double>(params_.max_obstacle_size);
    params_.inflate_radius = p["inflate_radius"].as<double>(params_.inflate_radius);
    params_.grid_persist_frames =
      p["grid_persist_frames"].as<int>(params_.grid_persist_frames);
    params_.verbose = p["verbose"].as<bool>(params_.verbose);
    params_.grid_resolution = p["grid_resolution"].as<double>(params_.grid_resolution);
    params_.grid_width = p["grid_width"].as<double>(params_.grid_width);
    params_.grid_height = p["grid_height"].as<double>(params_.grid_height);
    params_.dynamic_association_gate = p["dynamic_association_gate"].as<double>(
      params_.dynamic_association_gate);
    params_.dynamic_association_ambiguity_distance =
      p["dynamic_association_ambiguity_distance"].as<double>(
      params_.dynamic_association_ambiguity_distance);
    params_.dynamic_association_cost_margin =
      p["dynamic_association_cost_margin"].as<double>(
      params_.dynamic_association_cost_margin);
    params_.dynamic_association_z_gate = p["dynamic_association_z_gate"].as<double>(
      params_.dynamic_association_z_gate);
    params_.dynamic_association_xy_size_ratio =
      p["dynamic_association_xy_size_ratio"].as<double>(
      params_.dynamic_association_xy_size_ratio);
    params_.dynamic_association_z_size_ratio =
      p["dynamic_association_z_size_ratio"].as<double>(
      params_.dynamic_association_z_size_ratio);
    params_.dynamic_min_dt = p["dynamic_min_dt"].as<double>(params_.dynamic_min_dt);
    params_.dynamic_max_dt = p["dynamic_max_dt"].as<double>(params_.dynamic_max_dt);
    params_.dynamic_speed_deadband = p["dynamic_speed_deadband"].as<double>(
      params_.dynamic_speed_deadband);
    params_.dynamic_max_speed = p["dynamic_max_speed"].as<double>(
      params_.dynamic_max_speed);
    params_.dynamic_min_direction_cosine = p["dynamic_min_direction_cosine"].as<double>(
      params_.dynamic_min_direction_cosine);
    params_.dynamic_min_confirmations = p["dynamic_min_confirmations"].as<int>(
      params_.dynamic_min_confirmations);
    params_.dynamic_velocity_ema_alpha = p["dynamic_velocity_ema_alpha"].as<double>(
      params_.dynamic_velocity_ema_alpha);
    params_.dynamic_min_displacement = p["dynamic_min_displacement"].as<double>(
      params_.dynamic_min_displacement);
    params_.dynamic_dormant_max_misses = p["dynamic_dormant_max_misses"].as<int>(
      params_.dynamic_dormant_max_misses);
    params_.dynamic_prediction_horizon = p["dynamic_prediction_horizon"].as<double>(
      params_.dynamic_prediction_horizon);
    params_.dynamic_prediction_steps = p["dynamic_prediction_steps"].as<int>(
      params_.dynamic_prediction_steps);
    params_.dynamic_max_prediction_points = p["dynamic_max_prediction_points"].as<int>(
      params_.dynamic_max_prediction_points);
    params_.dynamic_prediction_min_half_x = p["dynamic_prediction_min_half_x"].as<double>(
      params_.dynamic_prediction_min_half_x);
    params_.dynamic_prediction_min_half_y = p["dynamic_prediction_min_half_y"].as<double>(
      params_.dynamic_prediction_min_half_y);
    params_.dynamic_prediction_uncertainty_growth =
      p["dynamic_prediction_uncertainty_growth"].as<double>(
      params_.dynamic_prediction_uncertainty_growth);
    params_.dynamic_prediction_max_uncertainty =
      p["dynamic_prediction_max_uncertainty"].as<double>(
      params_.dynamic_prediction_max_uncertainty);
    params_.dynamic_max_bbox_size_x = p["dynamic_max_bbox_size_x"].as<double>(
      params_.dynamic_max_bbox_size_x);
    params_.dynamic_max_bbox_size_y = p["dynamic_max_bbox_size_y"].as<double>(
      params_.dynamic_max_bbox_size_y);
    params_.dynamic_max_bbox_size_z = p["dynamic_max_bbox_size_z"].as<double>(
      params_.dynamic_max_bbox_size_z);
    const double configured_inflate_radius = params_.inflate_radius;
    params_.inflate_radius = normalizeGridInflationRadius(configured_inflate_radius);
    if (!std::isfinite(configured_inflate_radius) || configured_inflate_radius < 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "inflate_radius=%.3f 无效，使用安全默认值 %.2fm",
        configured_inflate_radius, params_.inflate_radius);
    } else if (configured_inflate_radius < kMinimumGridInflationRadius) {
      RCLCPP_WARN(
        get_logger(),
        "inflate_radius=%.3f 低于安全下限 %.2fm，提升到下限",
        configured_inflate_radius, kMinimumGridInflationRadius);
    }
    const float configured_voxel_size =
      static_cast<float>(params_.nonground_voxel_size);
    if (!std::isfinite(params_.nonground_voxel_size) ||
      !std::isfinite(configured_voxel_size) ||
      params_.nonground_voxel_size < kMinimumNongroundVoxelSize ||
      params_.nonground_voxel_size > 1.0 || configured_voxel_size <= 0.0f)
    {
      RCLCPP_WARN(
        get_logger(),
        "nonground_voxel_size=%.9g (float %.9g) 无效，使用安全默认值 0.10m",
        params_.nonground_voxel_size, configured_voxel_size);
      params_.nonground_voxel_size = 0.10;
    }
    if (!std::isfinite(params_.input_timeout) ||
      params_.input_timeout <= 0.0 || params_.input_timeout > 60.0)
    {
      RCLCPP_WARN(
        get_logger(),
        "input_timeout=%.3f 无效，使用安全默认值 1.00s",
        params_.input_timeout);
      params_.input_timeout = 1.0;
    }
    if (!std::isfinite(params_.cloud_stamp_timeout) ||
      params_.cloud_stamp_timeout <= 0.0 || params_.cloud_stamp_timeout > 60.0)
    {
      RCLCPP_WARN(
        get_logger(),
        "cloud_stamp_timeout=%.3f 无效，使用安全默认值 1.00s",
        params_.cloud_stamp_timeout);
      params_.cloud_stamp_timeout = 1.0;
    }
    if (!std::isfinite(params_.cloud_stamp_future_tolerance) ||
      params_.cloud_stamp_future_tolerance < 0.0 ||
      params_.cloud_stamp_future_tolerance > 60.0)
    {
      RCLCPP_WARN(
        get_logger(),
        "cloud_stamp_future_tolerance=%.3f 无效，使用安全默认值 0.20s",
        params_.cloud_stamp_future_tolerance);
      params_.cloud_stamp_future_tolerance = 0.2;
    }
    if (!std::isfinite(params_.clock_jump_forward_limit) ||
      params_.clock_jump_forward_limit <= 0.0 ||
      params_.clock_jump_forward_limit > 3600.0)
    {
      RCLCPP_WARN(
        get_logger(),
        "clock_jump_forward_limit=%.3f 无效，使用安全默认值 10.00s",
        params_.clock_jump_forward_limit);
      params_.clock_jump_forward_limit = 10.0;
    }
    const int configured_input_points = params_.max_input_points;
    params_.max_input_points = std::clamp(
      params_.max_input_points, 1, kDownstreamTrustedCloudPointLimit);
    if (configured_input_points != params_.max_input_points) {
      RCLCPP_WARN(
        get_logger(),
        "max_input_points=%d 超出 [1,%d]，使用 %d",
        configured_input_points, kDownstreamTrustedCloudPointLimit,
        params_.max_input_points);
    }
    const int configured_nonground_points = params_.max_nonground_points;
    params_.max_nonground_points = std::clamp(
      params_.max_nonground_points, 1, kDownstreamTrustedCloudPointLimit);
    if (configured_nonground_points != params_.max_nonground_points) {
      RCLCPP_WARN(
        get_logger(),
        "max_nonground_points=%d 超出 [1,%d]，使用 %d",
        configured_nonground_points, kDownstreamTrustedCloudPointLimit,
        params_.max_nonground_points);
    }
    if (params_.max_nonground_points < params_.max_input_points) {
      RCLCPP_WARN(
        get_logger(),
        "max_nonground_points=%d 小于 max_input_points=%d，提升到单帧上限",
        params_.max_nonground_points, params_.max_input_points);
      params_.max_nonground_points = params_.max_input_points;
    }
    const auto configured_ambiguity_distance =
      params_.dynamic_association_ambiguity_distance;
    if (!std::isfinite(configured_ambiguity_distance) ||
      configured_ambiguity_distance < 0.0 || configured_ambiguity_distance > 20.0)
    {
      RCLCPP_WARN(
        get_logger(),
        "dynamic_association_ambiguity_distance=%.3f 无效，使用安全默认值 0.10m",
        configured_ambiguity_distance);
      params_.dynamic_association_ambiguity_distance = 0.10;
    }
    const auto configured_cost_margin = params_.dynamic_association_cost_margin;
    if (!std::isfinite(configured_cost_margin) ||
      configured_cost_margin < 0.0 || configured_cost_margin > 1.0)
    {
      RCLCPP_WARN(
        get_logger(),
        "dynamic_association_cost_margin=%.3f 无效，使用安全默认值 0.10",
        configured_cost_margin);
      params_.dynamic_association_cost_margin = 0.10;
    }
  }

  /// 与 pixel_to_3d 同款双路容错 TF 查询
  bool lookupTf(
    const std::string & target, const std::string & source,
    const rclcpp::Time & stamp, Eigen::Isometry3d & out)
  {
    try {
      auto tf = tf_buffer_->lookupTransform(
        target, source, stamp,
        rclcpp::Duration::from_seconds(0.05));
      out = tf2::transformToEigen(tf);
      return true;
    } catch (const tf2::TransformException &) {
      try {
        auto tf = tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
        out = tf2::transformToEigen(tf);
        return true;
      } catch (const tf2::TransformException & e2) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "TF %s←%s 不可用: %s", target.c_str(), source.c_str(), e2.what());
        return false;
      }
    }
  }

  GroundVehiclePose groundVehiclePose(
    const double x, const double y, const double z)
  {
    GroundVehiclePose pose;
    pose.x = x;
    pose.y = y;
    pose.z = z;
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (!state_received_) {return pose;}
    pose.connected = latest_state_.connected;
    pose.pose_valid = latest_state_.pose_valid;
    pose.armed = latest_state_.armed;
    pose.state_age_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - state_arrival_).count();
    return pose;
  }

  enum class ClockSampleStatus
  {
    kNotStarted,
    kUsable,
    kJumped,
  };

  ClockSampleStatus sampleRosClock(int64_t & now_ns)
  {
    now_ns = this->now().nanoseconds();
    bool jumped = false;
    {
      std::lock_guard<std::mutex> lk(input_mtx_);
      const bool had_positive_sample =
        clock_sample_valid_ && last_clock_sample_ns_ > 0;
      if (now_ns <= 0) {
        // Sim time commonly remains at zero until /clock starts.  Do
        // not emit a warning for that startup state.  A transition
        // from a positive sample to zero is a real reset, however.
        jumped = had_positive_sample;
        clock_sample_valid_ = false;
        last_clock_sample_ns_ = now_ns;
        input_seen_ = false;
        last_input_ros_ns_ = 0;
      } else if (!clock_sample_valid_ || last_clock_sample_ns_ <= 0) {
        last_clock_sample_ns_ = now_ns;
        clock_sample_valid_ = true;
      } else {
        const int64_t delta_ns = now_ns - last_clock_sample_ns_;
        const int64_t jump_limit_ns = static_cast<int64_t>(
          params_.clock_jump_forward_limit * 1e9);
        jumped = delta_ns<0 || delta_ns> jump_limit_ns;
        last_clock_sample_ns_ = now_ns;
        clock_sample_valid_ = true;
        if (jumped) {
          input_seen_ = false;
          last_input_ros_ns_ = 0;
        }
      }
    }
    if (jumped) {return ClockSampleStatus::kJumped;}
    return now_ns > 0 ? ClockSampleStatus::kUsable : ClockSampleStatus::kNotStarted;
  }

  void noteInputArrival(const int64_t now_ns)
  {
    std::lock_guard<std::mutex> lk(input_mtx_);
    input_seen_ = true;
    last_input_ros_ns_ = now_ns;
  }

  void checkInputWatchdog()
  {
    int64_t now_ns = 0;
    const auto clock_status = sampleRosClock(now_ns);
    if (clock_status == ClockSampleStatus::kJumped) {
      invalidateMap("ROS 时钟重置/回退/异常前跳，清除 semantic_map");
      return;
    }
    // A simulated clock may not have started yet.  In that state there is
    // no meaningful age to compare; the node already published its
    // initial invalid state in the constructor.
    if (clock_status != ClockSampleStatus::kUsable) {return;}

    int64_t last_input_ns = 0;
    bool input_seen = false;
    {
      std::lock_guard<std::mutex> lk(input_mtx_);
      input_seen = input_seen_;
      last_input_ns = last_input_ros_ns_;
    }
    if (!input_seen || last_input_ns <= 0) {return;}

    const int64_t timeout_ns = static_cast<int64_t>(
      params_.input_timeout * 1e9);
    if (now_ns - last_input_ns > timeout_ns) {
      invalidateMap("pointcloud 输入超时，清除 semantic_map");
    }
  }

  bool validateCloudStamp(
    const sensor_msgs::msg::PointCloud2 & msg, const int64_t now_ns,
    rclcpp::Time & stamp) const
  {
    if (zeroStamp(msg.header.stamp) || now_ns <= 0) {return false;}
    try {
      stamp = rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type());
    } catch (const std::exception &) {
      return false;
    }
    const int64_t stamp_ns = stamp.nanoseconds();
    if (stamp_ns <= 0) {return false;}
    const double age_s = static_cast<double>(now_ns) / 1e9 -
      static_cast<double>(stamp_ns) / 1e9;
    return std::isfinite(age_s) && age_s <= params_.cloud_stamp_timeout &&
           age_s >= -params_.cloud_stamp_future_tolerance;
  }

  void publishEmptyNonground(const rclcpp::Time & stamp)
  {
    // Keep the frame and stamp in the same ROS clock domain as valid
    // output.  GridMap uses width==0 as an immediate stale-cloud reset,
    // while the header lets diagnostics identify the reset transition.
    sensor_msgs::msg::PointCloud2 empty;
    empty.header.stamp = stamp;
    empty.header.frame_id = params_.world_frame;
    empty.height = 1;
    empty.width = 0;
    empty.row_step = 0;
    empty.is_dense = true;
    nonground_pub_->publish(empty);
  }

  void publishEmptyNonground()
  {
    publishEmptyNonground(this->now());
  }

  void publishInvalidMap()
  {
    {
      std::lock_guard<std::mutex> lk(latest_mtx_);
      latest_ = makeInvalidObstacles(params_.world_frame);
      map_ready_ = false;
    }
    obstacles_pub_->publish(makeInvalidObstacles(params_.world_frame));
    grid_pub_->publish(
      makeInvalidGrid(
        params_.grid_width, params_.grid_height, params_.grid_resolution,
        params_.world_frame));

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = params_.world_frame;
    clear.ns = "obstacles";
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    marker_pub_->publish(markers);
  }

  void publishReadyEmptyMap(
    const std_msgs::msg::Header & header,
    const pcl::PointCloud<pcl::PointXYZI> * nonground = nullptr,
    const GroundPlaneModel * ground = nullptr)
  {
    // A trusted, complete window with no usable obstacle cluster is a
    // valid empty obstacle observation, not an invalid/reset map.  Keep
    // service state ready with the current non-zero stamp and retain the
    // accumulated grid evidence.  If the trusted nonground window is
    // non-empty, still send it to EGO: an empty cluster result must not
    // masquerade as an invalid width==0 cloud and erase GridMap's static
    // obstacle persistence.
    ObstacleInfoArray empty_obstacles;
    empty_obstacles.header = header;
    {
      std::lock_guard<std::mutex> lk(latest_mtx_);
      latest_ = empty_obstacles;
      map_ready_ = true;
      invalid_published_ = false;
    }
    obstacles_pub_->publish(empty_obstacles);
    publishGrid(header, {}, nonground, ground);
    publishMarkers(header, {});
    const rclcpp::Time stamp(header.stamp, get_clock()->get_clock_type());
    if (nonground != nullptr && !nonground->empty()) {
      sensor_msgs::msg::PointCloud2 nonground_msg;
      pcl::toROSMsg(*nonground, nonground_msg);
      nonground_msg.header.stamp = header.stamp;
      nonground_msg.header.frame_id = params_.world_frame;
      nonground_pub_->publish(nonground_msg);
    } else {
      publishEmptyNonground(stamp);
    }
  }

  void invalidateMap(const char * reason = nullptr)
  {
    grid_persist_.clear();
    grid_miss_.clear();
    resetObstacleTracking();

    bool publish_transition = false;
    {
      std::lock_guard<std::mutex> lk(latest_mtx_);
      publish_transition = map_ready_ || !invalid_published_;
      if (publish_transition) {
        invalid_published_ = true;
        map_ready_ = false;
      }
    }
    // Invalid input can arrive continuously (for example, an empty cloud
    // during sensor startup).  Clear local buffers every time, but emit
    // the invalid protocol and warning only on the valid→invalid (or
    // initial) transition to avoid flooding downstream consumers.
    if (!publish_transition) {return;}
    if (reason != nullptr) {RCLCPP_WARN(get_logger(), "%s", reason);}
    publishInvalidMap();
    publishEmptyNonground();
  }

  void resetObstacleTracking()
  {
    dynamic_tracker_.reset();
    last_obstacle_stamp_ns_ = 0;
  }

  easyfly::obstacle_geometry::ClusteringParams clusteringParams() const
  {
    return {params_.cluster_tolerance, params_.cluster_min_points,
      params_.cluster_max_points, params_.max_obstacle_size};
  }

  std::vector<DynamicObstacleObservation> extractCurrentDynamicObservations(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud) const
  {
    return easyfly::obstacle_geometry::extractCurrentDynamicObservations(cloud, clusteringParams());
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    int64_t now_ns = 0;
    const auto clock_status = sampleRosClock(now_ns);
    if (clock_status == ClockSampleStatus::kJumped) {
      invalidateMap("ROS 时钟重置/回退/异常前跳，清除 semantic_map");
      return;
    }
    if (!msg) {
      if (clock_status == ClockSampleStatus::kUsable) {
        invalidateMap("收到空 PointCloud2 指针，清除 semantic_map");
      }
      return;
    }
    // Do not process or log sensor data while sim time is still at zero.
    // The constructor has already published the initial invalid state;
    // the first positive /clock sample establishes a new baseline.
    if (clock_status != ClockSampleStatus::kUsable) {return;}
    noteInputArrival(now_ns);
    if (msg->width == 0 || msg->height == 0) {
      invalidateMap("收到空点云，清除 semantic_map");
      return;
    }
    const std::uint64_t raw_point_count =
      static_cast<std::uint64_t>(msg->width) *
      static_cast<std::uint64_t>(msg->height);
    if (raw_point_count > static_cast<std::uint64_t>(params_.max_input_points)) {
      invalidateMap("输入点云超过单帧资源上限，清除 semantic_map");
      return;
    }

    // Validate the message stamp before PCL conversion, TF lookup,
    // RANSAC, ground-model update, or sliding-window insertion.  A stamp
    // from another clock epoch must never contaminate trusted state.
    rclcpp::Time stamp;
    if (!validateCloudStamp(*msg, now_ns, stamp)) {
      invalidateMap("点云时间戳为零/过旧/未来或不可转换，清除 semantic_map");
      return;
    }
    const int64_t stamp_ns = stamp.nanoseconds();
    if (last_obstacle_stamp_ns_ > 0 && stamp_ns <= last_obstacle_stamp_ns_) {
      // Duplicate/backward cloud stamps indicate a sensor/ROS-clock
      // epoch change.  Do not reuse old tracker IDs or publish this
      // frame as if it were current; invalidate and await a fresh epoch.
      invalidateMap("点云时间戳重复/回退，清除动态跟踪与 semantic_map");
      return;
    }
    last_obstacle_stamp_ns_ = stamp_ns;

    // → PCL (input frame) → world 系 → 可信地面平面分类 → 当前帧障碍
    // 地面候选仍逐帧由 RANSAC 提取；只有 GroundPlaneTracker 自身保留模型历史。
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) {
      invalidateMap("转换后点云为空，清除 semantic_map");
      return;
    }
    if (cloud->size() > static_cast<std::size_t>(params_.max_input_points)) {
      invalidateMap("转换后点云超过单帧资源上限，清除 semantic_map");
      return;
    }
    for (const auto & point : cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z))
      {
        invalidateMap("输入点云包含非有限坐标，清除 semantic_map");
        return;
      }
    }

    Eigen::Isometry3d T;
    if (!lookupTf(params_.world_frame, msg->header.frame_id, stamp, T)) {
      invalidateMap("输入点云 TF 不可用，清除 semantic_map");
      return;
    }
    // The filtered cloud is normally already in odom, so T can be the
    // identity.  The vehicle pose is still needed independently: it is
    // the physical reference for the TF jump guard and for rejecting a
    // RANSAC plane which has moved up to the aircraft height.
    Eigen::Isometry3d vehicle_T;
    if (!lookupTf(params_.world_frame, params_.base_frame, stamp, vehicle_T)) {
      invalidateMap("车辆位姿 TF 不可用，清除 semantic_map");
      return;
    }
    // TF 跳变守卫: mavros local_position 存在启动毛刺 (实测首帧 (0.98,-2.52),
    // 与 M6 z=-10 尖峰同族) — 毛刺帧会把整个场景平移复制进累积窗口, 产出
    // 幻影簇。水平帧间平移 >1m 物理上不可能 (7Hz 下 2m/s 飞行 Δ0.3m), 丢帧。
    // z 轴不参与此守卫: 起飞/降落本身就是合法的大尺度 z 运动。
    const Eigen::Vector3d tt = vehicle_T.translation();
    const auto horizontalDistance = [](const Eigen::Vector3d & first,
        const Eigen::Vector3d & second) {
        return std::hypot(first.x() - second.x(), first.y() - second.y());
      };
    if (!tf_init_) {
      // 基线须连续 3 帧稳定才建立 (毛刺值若被当作基线, 之后所有正常帧
      // 都会被误判跳变 → 永久丢帧); 稳定前不产出
      if (!pend_init_ || horizontalDistance(tt, pend_t_) >
        params_.tf_guard_jump_distance / 2.0)
      {
        pend_t_ = tt;
        pend_cnt_ = 1;
        pend_init_ = true;
      } else if (++pend_cnt_ >= params_.tf_guard_init_consecutive_frames) {
        last_tf_t_ = tt;
        tf_init_ = true;
      }
      return;
    }
    if (horizontalDistance(tt, last_tf_t_) > params_.tf_guard_jump_distance) {
      // EKF 启动期是"持续错误后收敛"而非孤立尖峰: 固定基线会永久卡死,
      // 连续拒绝 10 帧 (≈1.4s) 视为真值永久移动, 重置基线
      if (++rej_cnt_ >= params_.tf_guard_reject_limit) {
        RCLCPP_WARN(
          get_logger(), "TF 位置持续漂移, 基线重置为 (%.2f,%.2f)",
          tt.x(), tt.y());
        last_tf_t_ = tt;
        rej_cnt_ = 0;
        ground_tracker_.reset();
        invalidateMap();
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "TF 位置跳变 (%.2f,%.2f→%.2f,%.2f), 丢帧",
          last_tf_t_.x(), last_tf_t_.y(), tt.x(), tt.y());
        invalidateMap("TF 位置跳变，清除 semantic_map");
      }
      return;
    }
    rej_cnt_ = 0;
    last_tf_t_ = tt;
    pcl::transformPointCloud(*cloud, *cloud, T.matrix());
    // A trusted model can still classify a sparse frame.  Only an
    // uninitialized tracker requires enough points to fit a new candidate.
    if (cloud->size() < 100 && !ground_tracker_.initialized()) {
      invalidateMap("可信地面模型未就绪，清除 semantic_map");
      return;
    }

    GroundPlaneUpdate plane_update = GroundPlaneUpdate::kRejected;
    double candidate_ground_z = std::numeric_limits<double>::quiet_NaN();
    const GroundVehiclePose vehicle_pose = groundVehiclePose(
      vehicle_T.translation().x(), vehicle_T.translation().y(),
      vehicle_T.translation().z());
    if (!vehicle_pose.connected || !vehicle_pose.pose_valid ||
      !std::isfinite(vehicle_pose.x) || !std::isfinite(vehicle_pose.y) ||
      !std::isfinite(vehicle_pose.z) ||
      !std::isfinite(vehicle_pose.state_age_s) ||
      vehicle_pose.state_age_s > params_.ground_init_state_timeout)
    {
      invalidateMap("UAV 状态未连接/位姿无效/过期，清除 semantic_map");
      return;
    }
    Eigen::Isometry3d world_to_ground = Eigen::Isometry3d::Identity();
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud = cloud;
    GroundVehiclePose ground_vehicle = vehicle_pose;
    if (!params_.ground_model_frame.empty() && params_.ground_model_frame != params_.world_frame) {
      try {
        // Query a fresh dynamic basis. No timeless/stale TF fallback: a map
        // reset or missing SLAM alignment invalidates the navigation cloud.
        auto tf = tf_buffer_->lookupTransform(
          params_.ground_model_frame, params_.world_frame, tf2::TimePointZero);
        if (!groundBasisStampFresh(
            static_cast<double>(now_ns) / 1e9, rclcpp::Time(tf.header.stamp).seconds(),
            params_.ground_init_state_timeout))
        {
          invalidateMap("地面模型坐标变换过期，清除 semantic_map");
          return;
        }
        world_to_ground = tf2::transformToEigen(tf);
      } catch (const tf2::TransformException &) {
        invalidateMap("地面模型坐标变换不可用，清除 semantic_map");
        return;
      }
      if (ground_basis_valid_ && !groundBasisContinuous(last_ground_basis_, world_to_ground)) {
        ground_tracker_.reset();  // armed cannot reinitialize after a frame jump
        last_ground_basis_ = world_to_ground;
        invalidateMap("地面模型坐标变换跳变，重置可信模型");
        return;
      }
      if (!world_to_ground.matrix().allFinite()) {
        invalidateMap("地面模型坐标变换非有限，清除 semantic_map");
        return;
      }
      ground_basis_valid_ = true;
      last_ground_basis_ = world_to_ground;
      ground_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::transformPointCloud(*cloud, *ground_cloud, world_to_ground.matrix());
      const Eigen::Vector3d position = world_to_ground *
        Eigen::Vector3d(vehicle_pose.x, vehicle_pose.y, vehicle_pose.z);
      ground_vehicle.x = position.x(); ground_vehicle.y = position.y();
      ground_vehicle.z = position.z();
    }
    const auto fit = easyfly::ground::fitGroundCandidate(
      *ground_cloud, ground_vehicle,
      {params_.plane_distance_threshold, params_.plane_min_inliers,
       params_.ground_min_clearance, params_.ground_spatial_support_enabled,
       params_.plane_min_inlier_ratio});
    if (fit.has_candidate) {
      const auto & candidate = fit.candidate;
      candidate_ground_z = candidate.model.zAt(ground_vehicle.x, ground_vehicle.y);
      plane_update = ground_tracker_.update(candidate, ground_vehicle);
      if (plane_update == GroundPlaneUpdate::kInitialized) {
        RCLCPP_INFO(
          get_logger(),
          "ground_initialized frame=%s method=%s global_inliers=%zu/%zu spatial_cells=%zu/%zu "
          "spatial_points=%zu clearance=%.3fm",
          (params_.ground_model_frame.empty() ?
          params_.world_frame : params_.ground_model_frame).c_str(),
          candidate.spatial_support.qualified() ? "spatial_support" : "global_ratio",
          candidate.inlier_count, candidate.sample_count, candidate.spatial_support.cells,
          candidate.spatial_support.eligible_cells, candidate.spatial_support.points,
          ground_vehicle.z - candidate_ground_z);
      }
    }
    if (!ground_tracker_.initialized()) {
      if (std::isfinite(candidate_ground_z) && std::isfinite(ground_vehicle.z)) {
        const double candidate_clearance = ground_vehicle.z - candidate_ground_z;
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(), 5000,
          "可信地面平面尚未初始化, 候选净空 %.3fm (要求 >= %.3fm), "
          "inliers=%zu/%zu (要求 >= %.3f), state_age=%.3fs "
          "connected=%d pose_valid=%d armed=%d, 丢弃当前帧",
          candidate_clearance, params_.ground_min_clearance,
          fit.inlier_count,
          cloud->size(), params_.plane_min_inlier_ratio,
          ground_vehicle.state_age_s, ground_vehicle.connected, ground_vehicle.pose_valid,
          ground_vehicle.armed);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "可信地面平面尚未初始化, 候选净空不可用, 丢弃当前帧");
      }
      invalidateMap("可信地面模型尚未初始化，清除 semantic_map");
      return;
    }
    if (fit.inlier_count == 0 || fit.coefficient_count < 4) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "当前帧未得到有效地面候选, 使用历史可信平面分类");
    }
    const bool candidate_rejected = plane_update == GroundPlaneUpdate::kRejected &&
      std::isfinite(candidate_ground_z);
    if (plane_update == GroundPlaneUpdate::kFrozen) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "armed 或 UAV 状态无效/过期，保持可信地面模型，不融合当前 RANSAC 候选");
    } else if (candidate_rejected) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "地面候选偏离可信模型, 使用历史平面分类当前帧 (候选 z=%.2f, 当前 z=%.2f)",
        candidate_ground_z,
        ground_tracker_.model().zAt(ground_vehicle.x, ground_vehicle.y));
    }

    const GroundPlaneModel classification_ground =
      groundPlaneInWorld(ground_tracker_.model(), world_to_ground);
    if (!classification_ground.valid()) {
      invalidateMap("地面模型不能变换到导航坐标系");
      return;
    }
    if (grid_height_limit_m_ > 0.0) {
      const double height = vehicle_pose.z -
        classification_ground.zAt(vehicle_pose.x, vehicle_pose.y);
      // Preserve the existing 0.4m local-planner vertical reserve. Outside
      // this declared envelope, never offer a falsely free global layer.
      if (!std::isfinite(height) || height > grid_height_limit_m_ - 0.4) {
        invalidateMap("UAV outside bounded grid height envelope");
        return;
      }
    }
    // Classify against the trusted model, never against the rejected
    // candidate's inlier indices.  This is what prevents a sparse plane
    // at the aircraft height from deleting all low obstacles.
    pcl::PointCloud<pcl::PointXYZI>::Ptr obs(
      new pcl::PointCloud<pcl::PointXYZI>);
    obs->reserve(cloud->size());
    std::size_t above_count = 0;
    for (const auto & p : cloud->points) {
      const double ground_z = classification_ground.zAt(p.x, p.y);
      const double height = static_cast<double>(p.z) - ground_z;
      if (std::abs(classification_ground.signedDistance(p.x, p.y, p.z)) >
        params_.plane_distance_threshold)
      {
        ++above_count;
        if (height > 0.05 && height < params_.obstacle_z_max) {
          obs->push_back(p);
        }
      }
    }
    if (params_.verbose) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "TF world←%s: (%.2f,%.2f,%.2f) "
        "vehicle=(%.2f,%.2f,%.2f) state=%d/%d armed=%d age=%.2f "
        "ground_plane_z=%.2f 点数 %zu→%zu→%zu",
        msg->header.frame_id.c_str(),
        T.translation().x(), T.translation().y(), T.translation().z(),
        vehicle_pose.x, vehicle_pose.y, vehicle_pose.z,
        vehicle_pose.connected ? 1 : 0, vehicle_pose.pose_valid ? 1 : 0,
        vehicle_pose.armed ? 1 : 0, vehicle_pose.state_age_s,
        classification_ground.zAt(vehicle_pose.x, vehicle_pose.y),
        cloud->size(), above_count, obs->size());
    }

    // Voxelize the current valid frame.  No non-ground point is carried
    // into this callback from an earlier frame; GridMap owns the only
    // short-term persistence layer downstream.
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_nonground(
      new pcl::PointCloud<pcl::PointXYZI>);
    if (!obs->empty()) {
      pcl::VoxelGrid<pcl::PointXYZI> voxel;
      voxel.setInputCloud(obs);
      const float leaf = static_cast<float>(params_.nonground_voxel_size);
      voxel.setLeafSize(leaf, leaf, leaf);
      voxel.filter(*current_nonground);
    }
    if (current_nonground->size() > static_cast<std::size_t>(params_.max_nonground_points)) {
      invalidateMap("当前非地面点云超过资源上限，清除 semantic_map");
      return;
    }

    // Dynamic velocity evidence is intentionally derived from clusters in
    // this callback's frame only.  Empty/clusterless valid frames reset the
    // tracker, so stale velocity and IDs cannot cross a gap or timestamp
    // epoch.
    const auto current_observations = extractCurrentDynamicObservations(current_nonground);
    auto current_dynamic_frame = dynamic_tracker_.update(
      stamp.seconds(), current_observations);
    const auto current_predicted_points = dynamic_tracker_.predict(
      current_observations, current_dynamic_frame.estimates,
      vehicle_pose.x, vehicle_pose.y, vehicle_pose.z);
    current_dynamic_frame.stats.prediction_points = current_predicted_points.size();
    if (!current_dynamic_frame.valid) {
      invalidateMap("当前帧动态跟踪无效，清除 semantic_map");
      return;
    }
    // EGO receives this current trusted frame plus only confirmed forward
    // occupancy.  There is no historical nonground fallback.
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged(current_nonground);
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_nonground(
      new pcl::PointCloud<pcl::PointXYZI>(*current_nonground));

    // A valid but too-small current frame is a ready empty observation;
    // publish it immediately rather than waiting for or mixing history.
    if (merged->size() < static_cast<size_t>(params_.cluster_min_points)) {
      std_msgs::msg::Header empty_header;
      empty_header.stamp = msg->header.stamp;
      empty_header.frame_id = params_.world_frame;
      publishReadyEmptyMap(empty_header, merged_nonground.get(), &classification_ground);
      return;
    }

    // 欧式聚类 (当前有效帧)
    const auto clusters = easyfly::obstacle_geometry::clusterCloud(merged, clusteringParams());

    if (params_.verbose) {
      std::string stat = "current=" + std::to_string(merged->size()) +
        " nonground=" + std::to_string(merged_nonground->size()) +
        " predicted=" + std::to_string(current_predicted_points.size()) + " 簇:";
      for (const auto & c : clusters) {
        const auto b = easyfly::obstacle_geometry::clusterBounds(*merged, c);
        const float x1 = b[0], y1 = b[1], z1 = b[2], x2 = b[3], y2 = b[4], z2 = b[5];
        stat += " [" + std::to_string(c.indices.size()) + "pt c=(" +
          std::to_string((x1 + x2) * 0.5).substr(0, 6) + "," +
          std::to_string((y1 + y2) * 0.5).substr(0, 6) + ") s=(" +
          std::to_string(x2 - x1).substr(0, 4) + "," +
          std::to_string(y2 - y1).substr(0, 4) + "," +
          std::to_string(z2 - z1).substr(0, 4) + ") z" +
          std::to_string(z1).substr(0, 5) + "~" +
          std::to_string(z2).substr(0, 5) + "]";
      }
      RCLCPP_INFO(get_logger(), "%s", stat.c_str());
    }

    // 簇 → ObstacleInfo (轴对齐包围盒)
    ObstacleInfoArray arr;
    arr.header.stamp = msg->header.stamp;
    arr.header.frame_id = params_.world_frame;
    // x1,y1,z1,x2,y2,z2 for semantic labels and the legacy grid.
    std::vector<std::array<double, 6>> boxes;
    std::vector<DynamicObstacleObservation> observations;
    std::vector<std::int32_t> fallback_ids;
    boxes.reserve(clusters.size());
    observations.reserve(clusters.size());
    fallback_ids.reserve(clusters.size());
    for (const auto & c : clusters) {
      const auto geometry = easyfly::obstacle_geometry::outputClusterGeometry(
        *merged, c, params_.max_obstacle_size);
      using easyfly::obstacle_geometry::ClusterGeometryStatus;
      if (geometry.status == ClusterGeometryStatus::kInvalidGeometry) {
        invalidateMap("ObstacleInfo 包框坐标/尺寸非法，清除 semantic_map");
        return;
      }
      if (geometry.status == ClusterGeometryStatus::kInvalidFloatBox) {
        invalidateMap("ObstacleInfo 包框 double→float 非有限/负尺寸，清除 semantic_map");
        return;
      }
      if (geometry.status != ClusterGeometryStatus::kAccepted) {continue;}
      const auto & b = geometry.bounds;
      const float x1 = b[0], y1 = b[1], z1 = b[2], x2 = b[3], y2 = b[4], z2 = b[5];
      const auto & f = geometry.fields;
      const float center_x = f[0], center_y = f[1], center_z = f[2];
      const float size_x = f[3], size_y = f[4], size_z = f[5];

      ObstacleInfo o;
      o.header = arr.header;
      o.obstacle_id = static_cast<int32_t>(arr.obstacles.size());
      o.obstacle_type = "unknown";
      o.center_x = center_x;
      o.center_y = center_y;
      o.center_z = center_z;
      o.size_x = size_x;
      o.size_y = size_y;
      o.size_z = size_z;
      o.is_dynamic = false;
      o.velocity_x = 0.0F;
      o.velocity_y = 0.0F;
      o.velocity_z = 0.0F;
      arr.obstacles.push_back(o);
      fallback_ids.push_back(o.obstacle_id);
      boxes.push_back({x1, y1, z1, x2, y2, z2});
      observations.push_back(
        {
          o.center_x, o.center_y, o.center_z,
          o.size_x, o.size_y, o.size_z});
    }

    // Apply current-frame tracker results only after a one-to-one,
    // finite-distance match.  The output boxes are current-frame boxes;
    // no historical cluster center can become the dynamic track's
    // position or ID.
    const auto output_matches = matchCurrentDynamicObservations(
      observations, current_observations, current_dynamic_frame.estimates,
      params_.dynamic_association_gate,
      params_.dynamic_association_ambiguity_distance,
      params_.dynamic_association_cost_margin);
    std::vector<std::int32_t> stable_ids(arr.obstacles.size(), 0);
    std::vector<bool> stable_id_valid(arr.obstacles.size(), false);
    for (std::size_t output_index = 0; output_index < arr.obstacles.size(); ++output_index) {
      const int current_index = output_matches[output_index];
      if (current_index < 0 ||
        static_cast<std::size_t>(current_index) >= current_observations.size())
      {
        continue;
      }
      const auto & estimate = current_dynamic_frame.estimates[
        static_cast<std::size_t>(current_index)];
      std::int32_t stable_id = 0;
      if (estimate.valid && estimate.stable_id_valid &&
        tryStableObstacleId(estimate.track_id, stable_id))
      {
        stable_ids[output_index] = stable_id;
        stable_id_valid[output_index] = true;
      }
      if (!estimate.is_dynamic) {
        continue;
      }
      float velocity_x = 0.0F;
      float velocity_y = 0.0F;
      float velocity_z = 0.0F;
      if (tryFiniteFloat(estimate.velocity_x, velocity_x) &&
        tryFiniteFloat(estimate.velocity_y, velocity_y) &&
        tryFiniteFloat(estimate.velocity_z, velocity_z))
      {
        arr.obstacles[output_index].is_dynamic = true;
        arr.obstacles[output_index].velocity_x = velocity_x;
        arr.obstacles[output_index].velocity_y = velocity_y;
        arr.obstacles[output_index].velocity_z = velocity_z;
      } else {
        // A finite double can still overflow when converted to the
        // float message field. Keep this obstacle static rather than
        // publishing an invalid velocity or dynamic claim.
        stable_id_valid[output_index] = false;
      }
    }
    const auto unique_ids = assignUniqueObstacleIds(
      fallback_ids, stable_ids, stable_id_valid);
    if (unique_ids.size() != arr.obstacles.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ObstacleInfo ID 分配失败，丢弃当前动态字段并保持保守回退 ID");
      for (std::size_t i = 0; i < arr.obstacles.size(); ++i) {
        arr.obstacles[i].obstacle_id = static_cast<std::int32_t>(i);
        arr.obstacles[i].is_dynamic = false;
        arr.obstacles[i].velocity_x = 0.0F;
        arr.obstacles[i].velocity_y = 0.0F;
        arr.obstacles[i].velocity_z = 0.0F;
      }
    } else {
      for (std::size_t i = 0; i < arr.obstacles.size(); ++i) {
        arr.obstacles[i].obstacle_id = unique_ids[i];
      }
    }

    // Add only a bounded, confirmed forward sweep.  If the trusted cloud
    // is already at its configured memory budget, keep the current static
    // evidence and omit the optional prediction rather than truncating a
    // sweep into an ambiguous partial allocation.
    std::size_t published_prediction_points = 0;
    if (!current_predicted_points.empty()) {
      const auto current_size = merged_nonground->size();
      const auto point_limit = static_cast<std::size_t>(params_.max_nonground_points);
      const auto append_count = boundedCompletePredictionPointCount(
        current_size, point_limit, current_predicted_points.size(),
        DynamicObstacleTracker::kPredictionFootprintPoints);
      if (append_count == current_predicted_points.size()) {
        std::vector<pcl::PointXYZI> converted_prediction;
        converted_prediction.reserve(current_predicted_points.size());
        bool finite_prediction = true;
        for (const auto & predicted : current_predicted_points) {
          pcl::PointXYZI point;
          if (!tryFiniteFloat(predicted.x, point.x) ||
            !tryFiniteFloat(predicted.y, point.y) ||
            !tryFiniteFloat(predicted.z, point.z))
          {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "动态障碍预测点 double→float 非有限，跳过整批前向扫掠");
            finite_prediction = false;
            break;
          }
          point.intensity = 0.0F;
          converted_prediction.push_back(point);
        }
        if (finite_prediction && converted_prediction.size() == current_predicted_points.size()) {
          for (const auto & point : converted_prediction) {
            merged_nonground->push_back(point);
          }
          published_prediction_points = converted_prediction.size();
        }
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "动态障碍预测点超出 trusted cloud 预算，保留当前观测并跳过前向扫掠");
      }
    }
    current_dynamic_frame.stats.prediction_points = published_prediction_points;
    if (params_.verbose) {
      const auto & s = current_dynamic_frame.stats;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "动态障碍统计: observations=%zu matched=%zu new=%zu "
        "rejected_by_distance=%zu rejected_by_size=%zu "
        "rejected_by_ambiguity=%zu rejected_by_direction=%zu "
        "rejected_by_deadband=%zu rejected_by_dt=%zu rejected_by_speed=%zu "
        "rejected_by_bbox=%zu dormant=%zu confirmed_dynamic=%zu "
        "prediction_points=%zu",
        s.observations, s.matched, s.new_tracks, s.rejected_by_distance,
        s.rejected_by_size, s.rejected_by_ambiguity, s.rejected_by_direction,
        s.rejected_by_deadband, s.rejected_by_dt, s.rejected_by_speed,
        s.rejected_by_bbox, s.dormant_matches, s.confirmed_dynamic,
        s.prediction_points);
      std::string dynamic_detail;
      std::size_t detail_count = 0;
      for (std::size_t i = 0; i < current_observations.size() && detail_count < 8; ++i) {
        const auto & estimate = current_dynamic_frame.estimates[i];
        if (!estimate.is_dynamic) {
          continue;
        }
        const auto & observation = current_observations[i];
        const auto now_envelope = dynamic_tracker_.predictionEnvelope(observation, 0.0);
        const auto end_envelope = dynamic_tracker_.predictionEnvelope(
          observation, dynamic_tracker_.config().prediction_horizon);
        dynamic_detail += " id=" + std::to_string(estimate.track_id) +
          " c=(" + std::to_string(observation.center_x) + "," +
          std::to_string(observation.center_y) + "," +
          std::to_string(observation.center_z) + ") v=(" +
          std::to_string(estimate.velocity_x) + "," +
          std::to_string(estimate.velocity_y) + ") bbox=(" +
          std::to_string(observation.size_x) + "," +
          std::to_string(observation.size_y) + "," +
          std::to_string(observation.size_z) + ") env=(" +
          std::to_string(now_envelope.half_x) + "," +
          std::to_string(now_envelope.half_y) + ")->(" +
          std::to_string(end_envelope.half_x) + "," +
          std::to_string(end_envelope.half_y) + ")";
        ++detail_count;
      }
      if (detail_count > 0) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "动态障碍预测包络(最多8条):%s", dynamic_detail.c_str());
      }
    }

    if (arr.obstacles.empty()) {
      publishReadyEmptyMap(arr.header, merged_nonground.get(), &classification_ground);
      return;
    }

    sensor_msgs::msg::PointCloud2 nonground_msg;
    pcl::toROSMsg(*merged_nonground, nonground_msg);
    nonground_msg.header.stamp = msg->header.stamp;
    nonground_msg.header.frame_id = params_.world_frame;
    nonground_pub_->publish(nonground_msg);

    {
      std::lock_guard<std::mutex> lk(latest_mtx_);
      latest_ = arr;
      map_ready_ = true;
      invalid_published_ = false;
    }
    obstacles_pub_->publish(arr);
    if (grid_footprint_mode_ == "observed_cells") {
      // Keep full compact dynamic envelopes, including each complete predicted footprint.
      // Environmental cluster AABBs remain semantic labels, not filled room interiors.
      std::vector<std::array<double, 6>> dynamic_boxes;
      for (std::size_t i = 0; i < boxes.size(); ++i) {
        if (arr.obstacles[i].is_dynamic) {dynamic_boxes.push_back(boxes[i]);}
      }
      constexpr auto footprint = DynamicObstacleTracker::kPredictionFootprintPoints;
      for (std::size_t i = 0; i < published_prediction_points; i += footprint) {
        const auto start = merged_nonground->size() - published_prediction_points + i;
        const auto & p = merged_nonground->points[start];
        std::array<double, 6> b{p.x, p.y, p.z, p.x, p.y, p.z};
        for (std::size_t j = 1; j < footprint; ++j) {
          const auto & v = merged_nonground->points[start + j];
          b[0] = std::min(b[0], static_cast<double>(v.x)); b[1] = std::min(
            b[1],
            static_cast<double>(v.y));
          b[3] = std::max(b[3], static_cast<double>(v.x)); b[4] = std::max(
            b[4],
            static_cast<double>(v.y));
        }
        dynamic_boxes.push_back(b);
      }
      publishGrid(arr.header, dynamic_boxes, merged_nonground.get(), &classification_ground);
    } else {
      publishGrid(arr.header, boxes);
    }
    publishMarkers(arr.header, boxes);
  }

  // ---- 2D 占据栅格: 簇足迹 (膨胀后) 标记为占据 + 持久化衰减 ----
  // 本帧有簇 → 该格 100; 本帧无簇 → 每帧衰减 20 (≈1s 后归零)。
  // 聚类逐帧闪烁 (8↔25 簇波动), 直接发布单帧会让规划器偶尔拿到缺障碍
  // 的栅格 (实测 A* 直线穿过 person, 最近 0.14m)。
  void publishGrid(
    const std_msgs::msg::Header & header,
    const std::vector<std::array<double, 6>> & boxes,
    const pcl::PointCloud<pcl::PointXYZI> * nonground = nullptr,
    const GroundPlaneModel * ground = nullptr)
  {
    if (grid_height_limit_m_ > 0.0 &&
      (nonground == nullptr || ground == nullptr || !ground->valid()))
    {
      invalidateMap("bounded grid requires trusted current ground and cloud");
      return;
    }
    nav_msgs::msg::OccupancyGrid grid;
    grid.header = header;
    grid.info.resolution = params_.grid_resolution;
    grid.info.width = static_cast<uint32_t>(params_.grid_width / params_.grid_resolution);
    grid.info.height = static_cast<uint32_t>(params_.grid_height / params_.grid_resolution);
    grid.info.origin.position.x = -params_.grid_width / 2.0;
    grid.info.origin.position.y = -params_.grid_height / 2.0;
    grid.info.origin.orientation.w = 1.0;
    const size_t n = static_cast<size_t>(grid.info.width) * grid.info.height;
    grid.data.assign(n, 0);

    const double inf = params_.inflate_radius;
    if (grid_footprint_mode_ == "observed_cells" && nonground != nullptr &&
      !rasterizeObservedFootprints(
        *nonground, grid.info.width, grid.info.height,
        params_.grid_resolution, grid.info.origin.position.x, grid.info.origin.position.y,
        inf + params_.nonground_voxel_size * 0.5, grid.data,
        ground, grid_height_limit_m_, params_.nonground_voxel_size * 0.5))
    {
      invalidateMap("observed occupancy geometry invalid");
      return;
    }
    for (const auto & b : boxes) {
      const int c1 = std::max(
        0, static_cast<int>(std::floor(
          (b[0] - inf -
          grid.info.origin.position.x) / params_.grid_resolution)));
      const int c2 = std::min(
        static_cast<int>(grid.info.width) - 1,
        static_cast<int>(std::floor(
          (b[3] + inf -
          grid.info.origin.position.x) / params_.grid_resolution)));
      const int r1 = std::max(
        0, static_cast<int>(std::floor(
          (b[1] - inf -
          grid.info.origin.position.y) / params_.grid_resolution)));
      const int r2 = std::min(
        static_cast<int>(grid.info.height) - 1,
        static_cast<int>(std::floor(
          (b[4] + inf -
          grid.info.origin.position.y) / params_.grid_resolution)));
      for (int r = r1; r <= r2; ++r) {
        for (int c = c1; c <= c2; ++c) {
          grid.data[r * grid.info.width + c] = 100;
        }
      }
    }

    // 持久化融合 (两段式): 最近 persist_frames 帧内见过 → 100, 超时 → 0。
    // 不用数值衰减: 衰减值会在 A* 占据阈值 (>=50) 边缘抖动 — 测试目标格
    // 采样时 55(占据)、规划执行时 48(空闲), 同一障碍时有时无。
    // 历史尺寸不符则重置。
    if (grid_persist_.size() != n || grid_miss_.size() != n) {
      grid_persist_.assign(n, 0);
      grid_miss_.assign(n, 0);
    }
    for (size_t i = 0; i < n; ++i) {
      if (grid.data[i] == 100) {
        grid_persist_[i] = 100;
        grid_miss_[i] = 0;
      } else if (grid_persist_[i] > 0) {
        if (++grid_miss_[i] > static_cast<uint32_t>(
            std::max(0, params_.grid_persist_frames)))
        {
          grid_persist_[i] = 0;
          grid_miss_[i] = 0;
        }
        grid.data[i] = grid_persist_[i];
      }
    }
    grid_pub_->publish(grid);
  }

  void publishMarkers(
    const std_msgs::msg::Header & header,
    const std::vector<std::array<double, 6>> & boxes)
  {
    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker del;
    del.header = header;
    del.ns = "obstacles";
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);

    for (size_t i = 0; i < boxes.size(); ++i) {
      const auto & b = boxes[i];
      visualization_msgs::msg::Marker m;
      m.header = header;
      m.ns = "obstacles";
      m.id = static_cast<int32_t>(i);
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.pose.position.x = (b[0] + b[3]) / 2.0;
      m.pose.position.y = (b[1] + b[4]) / 2.0;
      m.pose.position.z = (b[2] + b[5]) / 2.0;
      m.pose.orientation.w = 1.0;
      m.scale.x = std::max(0.1, b[3] - b[0]);
      m.scale.y = std::max(0.1, b[4] - b[1]);
      m.scale.z = std::max(0.1, b[5] - b[2]);
      m.color.r = 1.0f; m.color.g = 0.55f; m.color.b = 0.0f; m.color.a = 0.45f;
      m.lifetime = rclcpp::Duration::from_seconds(1.5);
      ma.markers.push_back(m);
    }
    marker_pub_->publish(ma);
  }

  // ---- /get_obstacles: 按"距机体水平距离"过滤 (range_limit<=0 返回全部) ----
  void onGetObstacles(
    const std::shared_ptr<easyfly_msgs::srv::GetObstacles::Request> req,
    std::shared_ptr<easyfly_msgs::srv::GetObstacles::Response> res)
  {
    res->obstacles = ObstacleInfoArray{};
    ObstacleInfoArray latest;
    bool map_ready = false;
    // Keep the snapshot lock through response construction.  A TF reset
    // cannot clear the map between this readiness check and the reply.
    std::unique_lock<std::mutex> latest_lock(latest_mtx_);
    latest = latest_;
    map_ready = map_ready_;
    if (!map_ready ||
      (latest.header.stamp.sec == 0 && latest.header.stamp.nanosec == 0))
    {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "/get_obstacles 暂无就绪地图，返回空结果");
      return;
    }

    double drone_x = 0.0, drone_y = 0.0;
    if (req->range_limit > 0.0f) {
      Eigen::Isometry3d T;
      if (lookupTf(params_.world_frame, params_.base_frame, this->now(), T)) {
        drone_x = T.translation().x();
        drone_y = T.translation().y();
      }
    }
    for (const auto & o : latest.obstacles) {
      if (req->range_limit > 0.0f) {
        const double dx = o.center_x - drone_x, dy = o.center_y - drone_y;
        if (std::sqrt(dx * dx + dy * dy) > req->range_limit) {continue;}
      }
      res->obstacles.obstacles.push_back(o);
    }
    res->obstacles.header = latest.header;
    res->obstacles.header.frame_id = params_.world_frame;
  }

  NodeParams params_;
  bool tf_init_{false};
  bool pend_init_{false};
  int pend_cnt_{0};
  int rej_cnt_{0};
  Eigen::Vector3d pend_t_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_tf_t_{Eigen::Vector3d::Zero()};    // TF 跳变守卫
  GroundPlaneTracker ground_tracker_;
  Eigen::Isometry3d last_ground_basis_{Eigen::Isometry3d::Identity()};
  bool ground_basis_valid_{false};
  DynamicObstacleTracker dynamic_tracker_;
  int64_t last_obstacle_stamp_ns_{0};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex latest_mtx_;
  ObstacleInfoArray latest_;
  bool map_ready_{false};
  bool invalid_published_{false};
  std::string grid_footprint_mode_;
  double grid_height_limit_m_{0.0};
  std::vector<int8_t> grid_persist_;    // 栅格持久化状态 (两段式)
  std::vector<uint32_t> grid_miss_;     // 连续未观测帧数

  std::mutex input_mtx_;
  bool input_seen_{false};
  int64_t last_input_ros_ns_{0};
  bool clock_sample_valid_{false};
  int64_t last_clock_sample_ns_{0};

  std::mutex state_mtx_;
  easyfly_msgs::msg::UAVState latest_state_;
  bool state_received_{false};
  std::chrono::steady_clock::time_point state_arrival_{};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<easyfly_msgs::msg::UAVState>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr input_watchdog_timer_;
  rclcpp::Publisher<ObstacleInfoArray>::SharedPtr obstacles_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<easyfly_msgs::srv::GetObstacles>::SharedPtr srv_;
  rclcpp::CallbackGroup::SharedPtr srv_group_;
};

}  // namespace easyfly_semantic_map

RCLCPP_COMPONENTS_REGISTER_NODE(easyfly_semantic_map::SemanticMapNode)

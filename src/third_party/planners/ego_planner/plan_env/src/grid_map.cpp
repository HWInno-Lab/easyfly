#include "plan_env/grid_map.h"
#include "plan_env/inflation_union.h"
#include "rclcpp/logging.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

namespace
{
constexpr double kDefaultOdomFreshnessTimeout = 1.0;
constexpr double kMaxOdomFreshnessTimeout = 1.0;
constexpr double kDefaultOdomClockJumpLimit = 10.0;
constexpr double kMaxOdomClockJumpLimit = 60.0;
constexpr int kDefaultTrustedCloudPointLimit = 200000;
constexpr int kMaxTrustedCloudPointLimit = 1000000;

std::int64_t secondsToNanoseconds(const double seconds)
{
  if (!std::isfinite(seconds) || seconds < 0.0) {
    return 0;
  }
  const long double nanoseconds = static_cast<long double>(seconds) * 1.0e9L;
  if (nanoseconds >= static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
    return std::numeric_limits<std::int64_t>::max();
  }
  return static_cast<std::int64_t>(nanoseconds);
}
}

void GridMap::initMap(std::shared_ptr<rclcpp::Node> &nh)
{
  node_ = nh;

  /* get parameter */
  double x_size, y_size, z_size;
  node_->declare_parameter<double>("grid_map/resolution",-1.0);
  node_->declare_parameter<double>("grid_map/map_size_x",-1.0);
  node_->declare_parameter<double>("grid_map/map_size_y",-1.0);
  node_->declare_parameter<double>("grid_map/map_size_z",-1.0);
  node_->declare_parameter<double>("grid_map/local_update_range_x",-1.0);
  node_->declare_parameter<double>("grid_map/local_update_range_y",-1.0);
  node_->declare_parameter<double>("grid_map/local_update_range_z",-1.0);
  node_->declare_parameter<double>("grid_map/obstacles_inflation",-1.0);
  node_->declare_parameter<double>("grid_map/fx",-1.0);
  node_->declare_parameter<double>("grid_map/fy",-1.0);
  node_->declare_parameter<double>("grid_map/cx",-1.0);
  node_->declare_parameter<double>("grid_map/cy",-1.0);
  node_->declare_parameter<bool>("grid_map/use_depth_filter",true);
  node_->declare_parameter<double>("grid_map/depth_filter_tolerance",-1.0);
  node_->declare_parameter<double>("grid_map/depth_filter_maxdist",-1.0);
  node_->declare_parameter<double>("grid_map/depth_filter_mindist",-1.0);
  node_->declare_parameter<int>("grid_map/depth_filter_margin",-1);
  node_->declare_parameter<double>("grid_map/k_depth_scaling_factor",-1.0);
  node_->declare_parameter<int>("grid_map/skip_pixel",-1);
  node_->declare_parameter<double>("grid_map/p_hit",0.70);
  node_->declare_parameter<double>("grid_map/p_miss",0.35);
  node_->declare_parameter<double>("grid_map/p_min",0.12);
  node_->declare_parameter<double>("grid_map/p_max",0.97);
  node_->declare_parameter<double>("grid_map/p_occ",0.80);
  node_->declare_parameter<double>("grid_map/min_ray_length",-0.1);
  node_->declare_parameter<double>("grid_map/max_ray_length",-0.1);
  node_->declare_parameter<double>("grid_map/visualization_truncate_height",-0.1);
  // -1.0 is the explicit disabled sentinel; positive values are hard
  // continuous collision ceilings.  Keep declaration and helper semantics in
  // the same disabled range so a bare launch cannot reject every sample.
  node_->declare_parameter<double>("grid_map/virtual_ceil_height",-1.0);
  node_->declare_parameter<double>("grid_map/virtual_ceil_yp",-0.1);
  node_->declare_parameter<double>("grid_map/virtual_ceil_yn",-0.1);
  node_->declare_parameter<bool>("grid_map/show_occ_time",false);
  node_->declare_parameter<int>("grid_map/pose_type",1);
  node_->declare_parameter<std::string>("grid_map/frame_id",string("world"));
  node_->declare_parameter<int>("grid_map/local_map_margin",1);
  node_->declare_parameter<double>("grid_map/ground_height",1.0);
  node_->declare_parameter<double>("grid_map/odom_depth_timeout",1.0);
  node_->declare_parameter<double>("grid_map/trusted_cloud_timeout",1.0);
  node_->declare_parameter<double>("grid_map/trusted_cloud_future_tolerance",0.2);
  node_->declare_parameter<double>("grid_map/odom_freshness_timeout",1.0);
  node_->declare_parameter<double>("grid_map/odom_clock_jump_limit",10.0);
  node_->declare_parameter<int>("grid_map/max_trusted_cloud_points",200000);
  node_->declare_parameter<double>("grid_map/trusted_cloud_vertical_inflation",0.40);
  node_->declare_parameter<int>("grid_map/trusted_cloud_persistence_frames",20);

  node_->get_parameter("grid_map/resolution",mp_.resolution_);
  node_->get_parameter("grid_map/map_size_x",x_size);
  node_->get_parameter("grid_map/map_size_y",y_size);
  node_->get_parameter("grid_map/map_size_z",z_size);
  node_->get_parameter("grid_map/local_update_range_x",mp_.local_update_range_(0));
  node_->get_parameter("grid_map/local_update_range_y",mp_.local_update_range_(1));
  node_->get_parameter("grid_map/local_update_range_z",mp_.local_update_range_(2));
  node_->get_parameter("grid_map/obstacles_inflation",mp_.obstacles_inflation_);
  node_->get_parameter("grid_map/fx",mp_.fx_);
  node_->get_parameter("grid_map/fy",mp_.fy_);
  node_->get_parameter("grid_map/cx",mp_.cx_);
  node_->get_parameter("grid_map/cy",mp_.cy_);
  node_->get_parameter("grid_map/use_depth_filter",mp_.use_depth_filter_);
  node_->get_parameter("grid_map/depth_filter_tolerance",mp_.depth_filter_tolerance_);
  node_->get_parameter("grid_map/depth_filter_maxdist",mp_.depth_filter_maxdist_);
  node_->get_parameter("grid_map/depth_filter_mindist",mp_.depth_filter_mindist_);
  node_->get_parameter("grid_map/depth_filter_margin",mp_.depth_filter_margin_);
  node_->get_parameter("grid_map/k_depth_scaling_factor",mp_.k_depth_scaling_factor_);
  node_->get_parameter("grid_map/skip_pixel",mp_.skip_pixel_);
  node_->get_parameter("grid_map/p_hit",mp_.p_hit_);
  node_->get_parameter("grid_map/p_miss",mp_.p_miss_);
  node_->get_parameter("grid_map/p_min",mp_.p_min_);
  node_->get_parameter("grid_map/p_max",mp_.p_max_);
  node_->get_parameter("grid_map/p_occ",mp_.p_occ_);
  node_->get_parameter("grid_map/min_ray_length",mp_.min_ray_length_);
  node_->get_parameter("grid_map/max_ray_length",mp_.max_ray_length_);
  node_->get_parameter("grid_map/visualization_truncate_height",mp_.visualization_truncate_height_);
  node_->get_parameter("grid_map/virtual_ceil_height",mp_.virtual_ceil_height_);
  node_->get_parameter("grid_map/virtual_ceil_yp",mp_.virtual_ceil_yp_);
  node_->get_parameter("grid_map/virtual_ceil_yn",mp_.virtual_ceil_yn_);
  node_->get_parameter("grid_map/show_occ_time",mp_.show_occ_time_);
  node_->get_parameter("grid_map/pose_type",mp_.pose_type_);
  node_->get_parameter("grid_map/frame_id",mp_.frame_id_);
  node_->get_parameter("grid_map/local_map_margin",mp_.local_map_margin_);
  node_->get_parameter("grid_map/ground_height",mp_.ground_height_);
  node_->get_parameter("grid_map/odom_depth_timeout",mp_.odom_depth_timeout_);
  node_->get_parameter("grid_map/trusted_cloud_timeout",mp_.trusted_cloud_timeout_);
  node_->get_parameter(
      "grid_map/trusted_cloud_future_tolerance", mp_.trusted_cloud_future_tolerance_);
  node_->get_parameter("grid_map/odom_freshness_timeout", mp_.odom_freshness_timeout_);
  node_->get_parameter("grid_map/odom_clock_jump_limit", mp_.odom_clock_jump_limit_);
  int configured_max_trusted_cloud_points = kDefaultTrustedCloudPointLimit;
  node_->get_parameter(
      "grid_map/max_trusted_cloud_points", configured_max_trusted_cloud_points);
  node_->get_parameter(
      "grid_map/trusted_cloud_vertical_inflation", mp_.trusted_cloud_vertical_inflation_);
  int configured_persistence_frames = 20;
  node_->get_parameter(
      "grid_map/trusted_cloud_persistence_frames", configured_persistence_frames);
  if (configured_persistence_frames <= 0 || configured_persistence_frames >
      static_cast<int>(std::numeric_limits<std::uint16_t>::max())) {
    RCLCPP_WARN(
        node_->get_logger(),
        "grid_map/trusted_cloud_persistence_frames=%d 无效，使用 20 帧",
        configured_persistence_frames);
    configured_persistence_frames = 20;
  }
  mp_.trusted_cloud_persistence_frames_ =
      static_cast<std::size_t>(configured_persistence_frames);
  if (!std::isfinite(mp_.trusted_cloud_timeout_) || mp_.trusted_cloud_timeout_ <= 0.0) {
    mp_.trusted_cloud_timeout_ = 1.0;
  }
  if (!std::isfinite(mp_.trusted_cloud_future_tolerance_) ||
      mp_.trusted_cloud_future_tolerance_ < 0.0) {
    mp_.trusted_cloud_future_tolerance_ = 0.2;
  }
  if (!std::isfinite(mp_.odom_freshness_timeout_) ||
      mp_.odom_freshness_timeout_ <= 0.0 ||
      mp_.odom_freshness_timeout_ > kMaxOdomFreshnessTimeout) {
    RCLCPP_WARN(
        node_->get_logger(),
        "grid_map/odom_freshness_timeout=%.17g 无效或超过 %.1fs，使用 %.1fs",
        mp_.odom_freshness_timeout_, kMaxOdomFreshnessTimeout,
        kDefaultOdomFreshnessTimeout);
    mp_.odom_freshness_timeout_ = kDefaultOdomFreshnessTimeout;
  }
  if (!std::isfinite(mp_.odom_clock_jump_limit_) ||
      mp_.odom_clock_jump_limit_ <= mp_.odom_freshness_timeout_ ||
      mp_.odom_clock_jump_limit_ > kMaxOdomClockJumpLimit) {
    RCLCPP_WARN(
        node_->get_logger(),
        "grid_map/odom_clock_jump_limit=%.17g 无效，使用 %.1fs",
        mp_.odom_clock_jump_limit_, kDefaultOdomClockJumpLimit);
    mp_.odom_clock_jump_limit_ = kDefaultOdomClockJumpLimit;
  }
  if (configured_max_trusted_cloud_points <= 0 ||
      configured_max_trusted_cloud_points > kMaxTrustedCloudPointLimit) {
    RCLCPP_WARN(
        node_->get_logger(),
        "grid_map/max_trusted_cloud_points=%d 无效，使用 %d",
        configured_max_trusted_cloud_points, kDefaultTrustedCloudPointLimit);
    configured_max_trusted_cloud_points = kDefaultTrustedCloudPointLimit;
  }
  mp_.max_trusted_cloud_points_ =
      static_cast<std::size_t>(configured_max_trusted_cloud_points);
  if (!std::isfinite(mp_.resolution_) || mp_.resolution_ <= 0.0 ||
      !std::isfinite(1.0 / mp_.resolution_)) {
    RCLCPP_FATAL(
        node_->get_logger(),
        "grid_map/resolution=%.17g 无效，必须为有限正值且倒数有限",
        mp_.resolution_);
    throw std::invalid_argument("grid_map/resolution must be finite and positive");
  }
  if (!std::isfinite(x_size) || !std::isfinite(y_size) || !std::isfinite(z_size) ||
      x_size <= 0.0 || y_size <= 0.0 || z_size <= 0.0) {
    RCLCPP_FATAL(
        node_->get_logger(),
        "grid_map/map_size 必须为有限正值: %.17g %.17g %.17g",
        x_size, y_size, z_size);
    throw std::invalid_argument("grid_map/map_size must be finite and positive");
  }

  if (mp_.virtual_ceil_height_ - mp_.ground_height_ > z_size)
  {
    mp_.virtual_ceil_height_ = mp_.ground_height_ + z_size;
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  // ground_height_ is the lower planning boundary.  The semantic input is
  // already ground-filtered, but EGO still needs a bounded flight volume so
  // optimization cannot route below the physical floor to bypass an obstacle.
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  for (int i = 0; i < 3; ++i) {
    const long double voxel_count = std::ceil(
        static_cast<long double>(mp_.map_size_(i)) /
        static_cast<long double>(mp_.resolution_));
    if (!std::isfinite(voxel_count) || voxel_count < 1.0L ||
        voxel_count > static_cast<long double>(std::numeric_limits<int>::max())) {
      RCLCPP_FATAL(
          node_->get_logger(),
          "grid_map/map_size[%d]/resolution 产生无效体素数: %.17Lg",
          i, voxel_count);
      throw std::invalid_argument("grid_map voxel count is out of range");
    }
    mp_.map_voxel_num_(i) = static_cast<int>(voxel_count);
  }

  plan_env::InflationKernelConfig inflation_kernel;
  const double configured_inflation = mp_.obstacles_inflation_;
  if (!plan_env::computeInflationKernel(
          mp_.resolution_, configured_inflation,
          mp_.map_voxel_num_(0), mp_.map_voxel_num_(1), mp_.map_voxel_num_(2),
          inflation_kernel)) {
    RCLCPP_WARN(
        node_->get_logger(),
        "grid_map/obstacles_inflation=%.17g 或对应膨胀核超出安全范围，使用 0.80m",
        configured_inflation);
    if (!plan_env::computeInflationKernel(
            mp_.resolution_, 0.80,
            mp_.map_voxel_num_(0), mp_.map_voxel_num_(1), mp_.map_voxel_num_(2),
            inflation_kernel)) {
      RCLCPP_FATAL(
          node_->get_logger(),
          "默认 obstacles_inflation=0.80m 在当前 resolution/map 尺寸下也无法安全表示");
      throw std::invalid_argument("grid_map inflation kernel is out of range");
    }
    mp_.obstacles_inflation_ = 0.80;
  }
  mp_.inflation_step_ = inflation_kernel.step;
  mp_.inflation_kernel_elements_ = inflation_kernel.kernel_elements;

  const double configured_vertical_inflation = mp_.trusted_cloud_vertical_inflation_;
  if (!plan_env::computeTrustedCloudVerticalStep(
          mp_.resolution_, configured_vertical_inflation,
          mp_.map_voxel_num_(2), mp_.trusted_cloud_vertical_step_)) {
    RCLCPP_WARN(
        node_->get_logger(),
        "grid_map/trusted_cloud_vertical_inflation=%.17g 无效或超出安全范围，使用 0.40m",
        configured_vertical_inflation);
    mp_.trusted_cloud_vertical_inflation_ = 0.40;
    if (!plan_env::computeTrustedCloudVerticalStep(
            mp_.resolution_, mp_.trusted_cloud_vertical_inflation_,
            mp_.map_voxel_num_(2), mp_.trusted_cloud_vertical_step_)) {
      RCLCPP_FATAL(
          node_->get_logger(),
          "默认 trusted_cloud_vertical_inflation=0.40m 在当前 resolution/map 尺寸下无法安全表示");
      throw std::invalid_argument("trusted cloud vertical inflation is out of range");
    }
  }

  RCLCPP_INFO(
      node_->get_logger(),
      "GridMap trusted cloud: frame=%s inflation=%.2fm resolution=%.2fm "
      "inflation_step=%d kernel=%zu vertical_inflation=%.2fm vertical_step=%d "
      "planning_floor=%.2fm map_top=%.2fm virtual_ceil=%.2fm "
      "fresh_timeout=%.2fs odom_timeout=%.2fs odom_jump=%.2fs max_points=%zu "
      "persistence=%zu frames",
      mp_.frame_id_.c_str(), mp_.obstacles_inflation_, mp_.resolution_,
      mp_.inflation_step_, mp_.inflation_kernel_elements_,
      mp_.trusted_cloud_vertical_inflation_, mp_.trusted_cloud_vertical_step_,
      mp_.ground_height_, mp_.map_origin_(2) + mp_.map_size_(2),
      mp_.virtual_ceil_height_,
      mp_.trusted_cloud_timeout_, mp_.odom_freshness_timeout_,
      mp_.odom_clock_jump_limit_, mp_.max_trusted_cloud_points_,
      mp_.trusted_cloud_persistence_frames_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  // initialize data buffers

  std::size_t buffer_size = 1;
  for (int i = 0; i < 3; ++i) {
    const std::size_t dim = static_cast<std::size_t>(mp_.map_voxel_num_(i));
    if (dim == 0 || buffer_size > std::numeric_limits<std::size_t>::max() / dim) {
      throw std::invalid_argument("grid_map buffer size overflow");
    }
    buffer_size *= dim;
  }
  // toAddress() intentionally returns int for compatibility with the
  // original EGO planner.  Do not allocate a larger map that cannot be
  // addressed safely by the existing API.
  if (buffer_size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    RCLCPP_FATAL(node_->get_logger(), "grid_map buffer too large: %zu cells", buffer_size);
    throw std::invalid_argument("grid_map buffer exceeds addressable size");
  }

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
  md_.trusted_cloud_age_ = vector<std::uint16_t>(buffer_size, 0);

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  md_.raycast_num_ = 0;

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;

  md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  /* init callback */

  // depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::Image>(node_, "grid_map/depth"));
  depth_sub_.subscribe(node_.get(), "grid_map/depth");

  extrinsic_sub_ = 
      node_->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/extrinsic", 10, 
          std::bind(&GridMap::extrinsicCallback, this, std::placeholders::_1));

  if (mp_.pose_type_ == POSE_STAMPED)
  {
    pose_sub_.subscribe(node_.get(), "grid_map/pose");
    sync_image_pose_ = std::make_shared<message_filters::Synchronizer<SyncPolicyImagePose>>(
        SyncPolicyImagePose(100), depth_sub_, pose_sub_);
    sync_image_pose_->registerCallback(
        std::bind(&GridMap::depthPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
  }
  else if (mp_.pose_type_ == ODOMETRY)
  {
    odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>();
    odom_sub_->subscribe(node_.get(), "grid_map/odom");
    sync_image_odom_ = std::make_shared<message_filters::Synchronizer<SyncPolicyImageOdom>>(
        SyncPolicyImageOdom(100), depth_sub_, *odom_sub_);
    sync_image_odom_->registerCallback(
        std::bind(&GridMap::depthOdomCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

  // Live control inputs are latest-only, not a reliable replay queue. Every
  // accepted sample still passes the original source/odom freshness checks.
  const auto latest_sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
  indep_cloud_sub_ =
      node_->create_subscription<sensor_msgs::msg::PointCloud2>("grid_map/cloud", latest_sensor_qos,
          std::bind(&GridMap::cloudCallback, this, std::placeholders::_1));

  indep_odom_sub_ =
      node_->create_subscription<nav_msgs::msg::Odometry>("grid_map/odom", latest_sensor_qos,
          std::bind(&GridMap::odomCallback, this, std::placeholders::_1));

  occ_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), 
      std::bind(&GridMap::updateOccupancyCallback, this));

  vis_timer_ = node_->create_wall_timer(std::chrono::milliseconds(110), 
      std::bind(&GridMap::visCallback, this));

  map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy_inflate", 10);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.last_odom_sample_ = {};
  md_.odom_clock_valid_ = false;
  md_.image_cnt_ = 0;
  const auto clock_type = node_->get_clock()->get_clock_type();
  md_.last_occ_update_time_ = rclcpp::Time(0, 0, clock_type);
  md_.last_trusted_cloud_time_ = rclcpp::Time(0, 0, clock_type);

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;

  md_.flag_depth_odom_timeout_ = false;
  md_.flag_use_depth_fusion = false;

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
}

void GridMap::resetBuffer()
{
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
        plan_env::clearTrustedCloudVoxel(
          md_.occupancy_buffer_inflate_, md_.trusted_cloud_age_,
          static_cast<std::size_t>(toAddress(x, y, z)));
      }
}

void GridMap::clearTrustedCloudState()
{
  // /pointcloud_nonground is an obstacle-only stream.  When its producer or
  // odometry becomes invalid, clear the complete trusted layer rather than a
  // camera-relative window: otherwise an out-of-map/NaN camera pose can leave
  // stale occupancy alive and a later valid pose can appear to resurrect it.
  plan_env::clearTrustedCloudStorage(
    md_.occupancy_buffer_inflate_, md_.trusted_cloud_age_);
  md_.has_cloud_ = false;
  const auto clock_type = node_->get_clock()->get_clock_type();
  md_.last_trusted_cloud_time_ = rclcpp::Time(0, 0, clock_type);
  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void GridMap::clearOdomState()
{
  clearTrustedCloudState();
  md_.has_odom_ = false;
  md_.camera_pos_ = Eigen::Vector3d::Constant(
    std::numeric_limits<double>::quiet_NaN());
  md_.last_odom_sample_ = {};
  md_.odom_clock_valid_ = false;
}

bool GridMap::checkOdomFreshness()
{
  if (!node_ || !md_.has_odom_ || !md_.odom_clock_valid_) {
    clearOdomState();
    return false;
  }

  const auto now_ns = node_->get_clock()->now().nanoseconds();
  const auto result = plan_env::evaluateOdomFreshness(
    md_.last_odom_sample_, now_ns,
    secondsToNanoseconds(mp_.odom_freshness_timeout_),
    secondsToNanoseconds(mp_.trusted_cloud_future_tolerance_),
    secondsToNanoseconds(mp_.odom_clock_jump_limit_));
  if (result != plan_env::OdomFreshnessResult::kFresh) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "GridMap odom freshness lost: reason=%d now=%lld source=%lld receipt=%lld",
      static_cast<int>(result), static_cast<long long>(now_ns),
      static_cast<long long>(md_.last_odom_sample_.stamp_ns),
      static_cast<long long>(md_.last_odom_sample_.receive_ns));
    clearOdomState();
    return false;
  }
  return true;
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::projectDepthImage()
{
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;
  int skip_pix = mp_.skip_pixel_;
  if (rows <= 0 || cols <= 0 || skip_pix <= 0 ||
      md_.depth_image_.type() != CV_16UC1 ||
      (mp_.use_depth_filter_ && mp_.depth_filter_margin_ < 0)) {
    return;
  }
  // The input dimensions and sampling stride need not divide 640x480.
  // A ceiling count bounds either branch without writing beyond vector storage.
  const size_t sampled_rows = (static_cast<size_t>(rows) + skip_pix - 1) / skip_pix;
  const size_t sampled_cols = (static_cast<size_t>(cols) + skip_pix - 1) / skip_pix;
  md_.proj_points_.resize(sampled_rows * sampled_cols);

  double depth;

  Eigen::Matrix3d camera_r = md_.camera_r_m_;

  if (!mp_.use_depth_filter_)
  {
    for (int v = 0; v < rows; v+=skip_pix)
    {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u+=skip_pix)
      {

        Eigen::Vector3d proj_pt;
        depth = row_ptr[u] / mp_.k_depth_scaling_factor_;
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + md_.camera_pos_;

        if (u == 320 && v == 240)
          std::cout << "depth: " << depth << std::endl;
        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  }
  /* use depth filter */
  else
  {

    if (!md_.has_first_depth_)
      md_.has_first_depth_ = true;
    else
    {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_r_m_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
      {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v);

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_)
        {

          const uint16_t pixel = row_ptr[u];
          depth = pixel * inv_factor;

          // filter depth
          // depth += rand_noise_(eng_);
          // if (depth > 0.01) depth += rand_noise2_(eng_);

          if (pixel == 0)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }
          else if (depth < mp_.depth_filter_mindist_)
          {
            continue;
          }
          else if (depth > mp_.depth_filter_maxdist_)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }

          // project to world frame
          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;

          pt_world = camera_r * pt_cur + md_.camera_pos_;
          // if (!isInMap(pt_world)) {
          //   pt_world = closetPointInMap(pt_world, md_.camera_pos_);
          // }

          md_.proj_points_[md_.proj_points_cnt++] = pt_world;

          // check consistency with last image, disabled...
          if (false)
          {
            pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
            double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows)
            {
              if (fabs(md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                       pt_reproj.z()) < mp_.depth_filter_tolerance_)
              {
                md_.proj_points_[md_.proj_points_cnt++] = pt_world;
              }
            }
            else
            {
              md_.proj_points_[md_.proj_points_cnt++] = pt_world;
            }
          }
        }
      }
    }
  }

  /* maintain camera pose for consistency check */

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_r_m_ = md_.camera_r_m_;
  md_.last_depth_image_ = md_.depth_image_;
}

void GridMap::raycastProcess()
{
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0)
    return;

  rclcpp::Time t1, t2;

  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i];

    // set flag for projected point

    if (!isInMap(pt_w))
    {
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);

      length = (pt_w - md_.camera_pos_).norm();
      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);
    }
    else
    {
      length = (pt_w - md_.camera_pos_).norm();

      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      }
      else
      {
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between camera center and point

    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.camera_pos_).norm();

      // if (length < mp_.min_ray_length_) break;

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX)
      {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          break;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  md_.local_updated_ = true;

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      continue;
    }
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                    idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
    }

    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
  }
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

void GridMap::clearAndInflateLocalMap()
{
  /*clear outside local*/
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  Eigen::Vector3i min_cut = md_.local_bound_min_ -
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);

  // clear data outside the local range

  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    {

      for (int z = min_cut_m(2); z < min_cut(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    {

      for (int y = min_cut_m(1); y < min_cut(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    {

      for (int x = min_cut_m(0); x < min_cut(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  // inflate occupied voxels to compensate robot size

  const int inf_step = mp_.inflation_step_;
  // int inf_step_z = 1;
  vector<Eigen::Vector3i> inf_pts(mp_.inflation_kernel_elements_);
  Eigen::Vector3i inf_pt;

  // clear outdated data
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate obstacles
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_)
        {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          for (int k = 0; k < (int)inf_pts.size(); ++k)
          {
            inf_pt = inf_pts[k];
            int idx_inf = toAddress(inf_pt);
            if (idx_inf < 0 ||
                idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2))
            {
              continue;
            }
            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
        }
      }

  // add virtual ceiling to limit flight height
  if (mp_.virtual_ceil_height_ > -0.5) {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_) - 1;
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y) {
        plan_env::markTrustedCloudVoxel(
          md_.occupancy_buffer_inflate_, md_.trusted_cloud_age_,
          static_cast<std::size_t>(toAddress(x, y, ceil_id)));
      }
  }
}

void GridMap::visCallback()
{

  publishMapInflate(true);
  publishMap();
}

void GridMap::updateOccupancyCallback()
{
  if (md_.last_occ_update_time_.seconds() < 1.0 ) md_.last_occ_update_time_ = node_->get_clock()->now();
  
  if (!md_.occ_need_update_)
  {
    if ( md_.flag_use_depth_fusion && (node_->get_clock()->now() - md_.last_occ_update_time_).seconds() > mp_.odom_depth_timeout_ )
    {
      // ROS_ERROR("odom or depth lost! ros::Time::now()=%f, md_.last_occ_update_time_=%f, mp_.odom_depth_timeout_=%f", 
      //   node_->get_clock()->now().seconds(), md_.last_occ_update_time_.seconds(), mp_.odom_depth_timeout_);

      RCLCPP_ERROR(rclcpp::get_logger("GridMap"),"odom or depth lost! ros::Time::now()=%f, md_.last_occ_update_time_=%f, mp_.odom_depth_timeout_=%f", 
        node_->get_clock()->now().seconds(), md_.last_occ_update_time_.seconds(), mp_.odom_depth_timeout_);

      md_.flag_depth_odom_timeout_ = true;
    }
    return;
  }
  md_.last_occ_update_time_ = node_->get_clock()->now();

  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  projectDepthImage();
  // t2 = ros::Time::now();
  raycastProcess();
  // t3 = ros::Time::now();

  if (md_.local_updated_)
    clearAndInflateLocalMap();

  // t4 = ros::Time::now();

  // cout << setprecision(7);
  // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" << (t4-t3).toSec() << endl;;

  // md_.fuse_time_ += (t2 - t1).toSec();
  // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  // if (mp_.show_occ_time_)
  //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
  //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
}

void GridMap::depthPoseCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img,
                                const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose)
{
  
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  md_.camera_r_m_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                       pose->pose.orientation.y, pose->pose.orientation.z)
                        .toRotationMatrix();
  if (isInMap(md_.camera_pos_))
  {
    md_.has_odom_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  }
  else
  {
    md_.occ_need_update_ = false;
  }

  md_.flag_use_depth_fusion = true;
}

void GridMap::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  const auto invalidateOdom = [this]() { clearOdomState(); };

  if (!node_ || !odom || odom->header.frame_id != mp_.frame_id_) {
    invalidateOdom();
    return;
  }

  const auto clock_type = node_->get_clock()->get_clock_type();
  const rclcpp::Time stamp(odom->header.stamp, clock_type);
  const rclcpp::Time now = node_->get_clock()->now();
  const auto now_ns = now.nanoseconds();
  const auto & position = odom->pose.pose.position;
  const auto & orientation = odom->pose.pose.orientation;
  const bool finite_pose = std::isfinite(position.x) &&
    std::isfinite(position.y) && std::isfinite(position.z) &&
    std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
    std::isfinite(orientation.z) && std::isfinite(orientation.w);
  const plan_env::OdomFreshnessSample current_sample{
    stamp.nanoseconds(), now_ns};
  const auto current_result = plan_env::evaluateOdomFreshness(
    current_sample, now_ns,
    secondsToNanoseconds(mp_.odom_freshness_timeout_),
    secondsToNanoseconds(mp_.trusted_cloud_future_tolerance_),
    secondsToNanoseconds(mp_.odom_clock_jump_limit_));
  if (!finite_pose || current_result != plan_env::OdomFreshnessResult::kFresh) {
    invalidateOdom();
    return;
  }

  Eigen::Vector3d position_eigen(position.x, position.y, position.z);
  if (!isInMap(position_eigen)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "GridMap odom outside map; clearing trusted cloud: "
      "pos=(%.3f, %.3f, %.3f), bounds x=(%.3f, %.3f) "
      "y=(%.3f, %.3f) z=(%.3f, %.3f), frame=%s",
      position.x, position.y, position.z,
      mp_.map_min_boundary_(0), mp_.map_max_boundary_(0),
      mp_.map_min_boundary_(1), mp_.map_max_boundary_(1),
      mp_.map_min_boundary_(2), mp_.map_max_boundary_(2),
      mp_.frame_id_.c_str());
    invalidateOdom();
    return;
  }

  // A stopped stream or clock reset is cleared before accepting a new sample,
  // so the first valid odometry/cloud pair in the new epoch always starts
  // from an empty trusted layer.
  if (md_.odom_clock_valid_) {
    const auto previous_result = plan_env::evaluateOdomFreshness(
      md_.last_odom_sample_, now_ns,
      secondsToNanoseconds(mp_.odom_freshness_timeout_),
      secondsToNanoseconds(mp_.trusted_cloud_future_tolerance_),
      secondsToNanoseconds(mp_.odom_clock_jump_limit_));
    if (previous_result != plan_env::OdomFreshnessResult::kFresh) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        "fresh odom resets expired local reference: reason=%d now=%lld previous_source=%lld "
        "previous_receipt=%lld current_source=%lld",
        static_cast<int>(previous_result), static_cast<long long>(now_ns),
        static_cast<long long>(md_.last_odom_sample_.stamp_ns),
        static_cast<long long>(md_.last_odom_sample_.receive_ns),
        static_cast<long long>(current_sample.stamp_ns));
      clearOdomState();
    }
  }

  md_.last_odom_sample_ = current_sample;
  md_.odom_clock_valid_ = true;
  if (md_.has_first_depth_) {
    // Depth fusion normally owns camera_pos_ once its first frame arrives.
    // After freshness invalidation it is deliberately NaN, however, so the
    // first valid odom in the new epoch must restore a usable trusted-cloud
    // pose without taking ownership away from an already-valid depth pose.
    if (!md_.has_odom_ || !md_.camera_pos_.allFinite()) {
      md_.camera_pos_ = position_eigen;
    }
    md_.has_odom_ = true;
    return;
  }

  md_.camera_pos_ = position_eigen;

  md_.has_odom_ = true;
}

void GridMap::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr img)
{
  const auto processing_started = std::chrono::steady_clock::now();

  md_.has_cloud_ = false;
  const auto clock_type = node_->get_clock()->get_clock_type();
  md_.last_trusted_cloud_time_ = rclcpp::Time(0, 0, clock_type);

  const auto clearLocalCloud = [this]() {clearTrustedCloudState();};

  if (!img || img->width == 0 || img->height == 0 ||
      img->header.frame_id != mp_.frame_id_) {
    clearLocalCloud();
    return;
  }

  // A cloud cannot revive a stopped odometry stream.  This check is before
  // pcl::fromROSMsg so an otherwise valid-looking cloud never rebuilds the
  // trusted layer after an odom timeout or ROS-clock reset.
  if (!checkOdomFreshness()) {
    return;
  }

  const auto width = static_cast<std::uint64_t>(img->width);
  const auto height = static_cast<std::uint64_t>(img->height);
  if (height == 0 || width > std::numeric_limits<std::uint64_t>::max() / height) {
    clearLocalCloud();
    return;
  }
  const auto raw_point_count = width * height;
  if (mp_.max_trusted_cloud_points_ == 0 ||
      raw_point_count > static_cast<std::uint64_t>(mp_.max_trusted_cloud_points_)) {
    clearLocalCloud();
    return;
  }

  // Message stamps must be interpreted in exactly the same clock domain as
  // the owning node.  Constructing the stamp with the default clock type can
  // throw when use_sim_time and system-time messages are mixed.
  const rclcpp::Time msg_stamp(img->header.stamp, clock_type);
  const rclcpp::Time now = node_->get_clock()->now();
  const double age = (now - msg_stamp).seconds();
  if (msg_stamp.nanoseconds() <= 0 || !std::isfinite(age) ||
      age < -mp_.trusted_cloud_future_tolerance_ ||
      age > mp_.trusted_cloud_timeout_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "trusted cloud source age rejected: age=%.6f now=%.6f source=%.6f",
      age, now.seconds(), msg_stamp.seconds());
    clearLocalCloud();
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  if (latest_cloud.points.empty() ||
      latest_cloud.points.size() > mp_.max_trusted_cloud_points_) {
    clearLocalCloud();
    return;
  }
  for (const auto & point : latest_cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      clearLocalCloud();
      return;
    }
  }

  if (!md_.has_odom_)
  {
    clearTrustedCloudState();
    std::cout << "no odom!" << std::endl;
    return;
  }

  if (!md_.camera_pos_.allFinite())
  {
    clearLocalCloud();
    return;
  }

  // This stream contains trusted obstacle surfaces, not a complete depth
  // image.  Clearing the local inflated map for every accepted frame would
  // therefore turn a short occlusion/sparse return window into a false free
  // corridor.  Age old trusted voxels instead; invalid or explicitly empty
  // frames still take the fail-closed clearLocalCloud path above.
  if (!plan_env::ageTrustedCloudVoxels(
      md_.occupancy_buffer_inflate_, md_.trusted_cloud_age_,
      mp_.trusted_cloud_persistence_frames_)) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "trusted-cloud persistence storage/configuration invalid; clearing local map");
    clearLocalCloud();
    return;
  }

  // The topic is semantic_map's trusted, ground-removed output.  Record the
  // arrival in the owning node's ROS clock only after all basic validity
  // checks and an odometry reference are available.  The FSM separately
  // rejects this state once trusted_cloud_timeout_ has elapsed.
  md_.has_cloud_ = true;
  md_.last_trusted_cloud_time_ = node_->get_clock()->now();

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d;
  std::vector<plan_env::InflationBox> inflation_boxes;
  inflation_boxes.reserve(latest_cloud.size());

  const int inf_step = mp_.inflation_step_;
  const int inf_step_z = mp_.trusted_cloud_vertical_step_;

  double max_x, max_y, max_z, min_x, min_y, min_z;
  std::size_t local_point_count = 0;

  min_x = mp_.map_max_boundary_(0);
  min_y = mp_.map_max_boundary_(1);
  min_z = mp_.map_max_boundary_(2);

  max_x = mp_.map_min_boundary_(0);
  max_y = mp_.map_min_boundary_(1);
  max_z = mp_.map_min_boundary_(2);

  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

    /* point inside update range */
    Eigen::Vector3d devi = p3d - md_.camera_pos_;

    if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) &&
        fabs(devi(2)) < mp_.local_update_range_(2))
    {

      ++local_point_count;

      // Endpoints retain the original physical kernel extent. All old
      // per-offset samples lie inside this inclusive box, including floating
      // point boundary cases. Unioning fills any rounding holes conservatively.
      const Eigen::Vector3d extent(inf_step * mp_.resolution_,
        inf_step * mp_.resolution_, inf_step_z * mp_.resolution_);
      const Eigen::Vector3d lower = p3d - extent;
      const Eigen::Vector3d upper = p3d + extent;
      min_x = min(min_x, lower.x()); min_y = min(min_y, lower.y()); min_z = min(min_z, lower.z());
      max_x = max(max_x, upper.x()); max_y = max(max_y, upper.y()); max_z = max(max_z, upper.z());
      Eigen::Vector3i low_index, high_index;
      posToIndex(lower, low_index);
      posToIndex(upper, high_index);
      // Match the original integer isInMap contract, including a partially
      // occupied last voxel when map size is not a resolution multiple.
      if ((high_index.array() < 0).any() ||
          (low_index.array() >= mp_.map_voxel_num_.array()).any()) {continue;}
      boundIndex(low_index);
      boundIndex(high_index);
      inflation_boxes.push_back({{low_index.x(), low_index.y(), low_index.z()},
        {high_index.x(), high_index.y(), high_index.z()}});
    }
  }

  if (!plan_env::visitInflationUnion(inflation_boxes, [this](int x, int y, int z) {
      plan_env::markTrustedCloudVoxel(md_.occupancy_buffer_inflate_, md_.trusted_cloud_age_,
        static_cast<std::size_t>(toAddress(x, y, z)));
    })) {
    RCLCPP_ERROR(node_->get_logger(), "trusted-cloud inflation union exceeded safe work budget");
    clearLocalCloud();
    return;
  }

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));

  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  // add virtual ceiling to limit flight height
  if (mp_.virtual_ceil_height_ > -0.5) {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_) - 1;
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y) {
        plan_env::markTrustedCloudVoxel(
          md_.occupancy_buffer_inflate_, md_.trusted_cloud_age_,
          static_cast<std::size_t>(toAddress(x, y, ceil_id)));
      }
  }

  const double processing_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - processing_started).count();
  RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "trusted cloud accepted: points=%zu local=%zu xyz=[%.2f,%.2f]x[%.2f,%.2f]x[%.2f,%.2f] "
      "camera=(%.2f,%.2f,%.2f) persistence=%zu wall_ms=%.3f source_age=%.6f",
      latest_cloud.points.size(), local_point_count,
      min_x, max_x, min_y, max_y, min_z, max_z,
      md_.camera_pos_(0), md_.camera_pos_(1), md_.camera_pos_(2),
      mp_.trusted_cloud_persistence_frames_, processing_ms,
      (node_->get_clock()->now() - msg_stamp).seconds());
  if (processing_ms > 250.0) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "trusted cloud processing slow: points=%zu wall_ms=%.3f source_age=%.6f ready_age=%.6f",
      latest_cloud.points.size(), processing_ms,
      (node_->get_clock()->now() - msg_stamp).seconds(),
      (node_->get_clock()->now() - md_.last_trusted_cloud_time_).seconds());
  }
}

bool GridMap::hasFreshTrustedCloud()
{
  if (!checkOdomFreshness()) {
    return false;
  }
  if (!md_.has_cloud_ || !node_ ||
      !std::isfinite(mp_.trusted_cloud_timeout_) ||
      mp_.trusted_cloud_timeout_ <= 0.0 ||
      md_.last_trusted_cloud_time_.nanoseconds() <= 0) {
    return false;
  }
  const double age =
      (node_->get_clock()->now() - md_.last_trusted_cloud_time_).seconds();
  if (!std::isfinite(age) || age < 0.0 || age > mp_.trusted_cloud_timeout_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "trusted cloud ready age expired: age=%.6f now=%.6f accepted=%.6f",
      age, node_->get_clock()->now().seconds(), md_.last_trusted_cloud_time_.seconds());
  }
  return std::isfinite(age) && age >= 0.0 && age <= mp_.trusted_cloud_timeout_;
}

bool GridMap::hasFreshTrustedCloud() const
{
  return const_cast<GridMap *>(this)->hasFreshTrustedCloud();
}

void GridMap::publishMap()
{

  if (map_pub_->get_subscription_count() <= 0)
    return;
  
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_->publish(cloud_msg);
}

void GridMap::publishMapInflate(bool all_info)
{

  if (map_inf_pub_->get_subscription_count() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  if (all_info)
  {
    int lmm = mp_.local_map_margin_;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_->publish(cloud_msg);

  // ROS_INFO("pub map");
}

bool GridMap::odomValid() { return checkOdomFreshness(); }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

void GridMap::extrinsicCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  Eigen::Quaterniond cam2body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                     odom->pose.pose.orientation.x,
                                                     odom->pose.pose.orientation.y,
                                                     odom->pose.pose.orientation.z);
  Eigen::Matrix3d cam2body_r_m = cam2body_q.toRotationMatrix();
  md_.cam2body_.block<3, 3>(0, 0) = cam2body_r_m;
  md_.cam2body_(0, 3) = odom->pose.pose.position.x;
  md_.cam2body_(1, 3) = odom->pose.pose.position.y;
  md_.cam2body_(2, 3) = odom->pose.pose.position.z;
  md_.cam2body_(3, 3) = 1.0;
}

void GridMap::depthOdomCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img,
                                const nav_msgs::msg::Odometry::ConstSharedPtr& odom)
{
  
  /* get pose */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
  md_.camera_pos_(0) = cam_T(0, 3);
  md_.camera_pos_(1) = cam_T(1, 3);
  md_.camera_pos_(2) = cam_T(2, 3);
  md_.camera_r_m_ = cam_T.block<3, 3>(0, 0);

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  md_.occ_need_update_ = true;
  md_.flag_use_depth_fusion = true;
}

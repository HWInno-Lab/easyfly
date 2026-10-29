// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <System.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <deque>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <easyfly_msgs/msg/localization_status.hpp>
#include <easyfly_msgs/msg/uav_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <random>
#include <regex>
#include <sstream>
#include "easyfly_orbslam3_integration/pose_continuity.hpp"
#include "easyfly_orbslam3_integration/inertial_covariance.hpp"
#include "easyfly_orbslam3_integration/atlas_manifest.hpp"

using Status = easyfly_msgs::msg::LocalizationStatus;
using Image = sensor_msgs::msg::Image;
using Info = sensor_msgs::msg::CameraInfo;
using Imu = sensor_msgs::msg::Imu;
using Clock = std::chrono::steady_clock;
namespace fs = std::filesystem;
static volatile std::sig_atomic_t stop_requested = 0;
static void request_stop(int) {stop_requested = 1;}

static int64_t stamp(const builtin_interfaces::msg::Time & t)
{
  if (t.sec < 0 || t.nanosec >= 1000000000u) {throw std::runtime_error("malformed_source_stamp");}
  return int64_t(t.sec) * 1000000000 + t.nanosec;
}
static double steady()
{
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

class RGBDNode : public rclcpp::Node
{
public:
  RGBDNode()
  : Node("orbslam3_rgbd")
  {
    if (!get_parameter("use_sim_time").as_bool()) {
      throw std::runtime_error("calibrated Gazebo time required");
    }
    rcl_interfaces::msg::ParameterDescriptor fixed;fixed.read_only = true;
    auto text = [&](const char * name, const std::string & value) {
        return declare_parameter<std::string>(name, value, fixed);
      };
    const auto mode = text("sensor_mode", "rgbd");
    if (mode != "rgbd" && mode != "rgbd_imu") {throw std::runtime_error("invalid sensor_mode");}
    inertial_ = mode == "rgbd_imu";
    require_admission_ = declare_parameter<bool>("require_navigation_admission", false, fixed);
    if (require_admission_ && !inertial_) {
      throw std::runtime_error("explicit activation is only for inertial navigation");
    }
    navigation_admitted_ = !require_admission_;
    imu_topic_ = text("imu_topic", "/livox/imu");
    rgb_topic_ = text("rgb_topic", "/camera/color/image_raw");
    depth_topic_ = text("depth_topic", "/camera/aligned_depth_to_color/depth/image_raw");
    const auto valid_topic = [](const std::string & t) {
        return t.size() <= 256 &&
               std::regex_match(t, std::regex("/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*"));
      };
    if (!valid_topic(rgb_topic_) || !valid_topic(depth_topic_) || rgb_topic_ == depth_topic_ ||
      rgb_topic_.rfind("/orbslam3/", 0) == 0 || depth_topic_.rfind("/orbslam3/", 0) == 0)
    {
      throw std::runtime_error("invalid or feedback sensor topics");
    }
    if (inertial_ && (!valid_topic(imu_topic_) || imu_topic_ == rgb_topic_ ||
      imu_topic_ == depth_topic_ || imu_topic_.rfind("/orbslam3/", 0) == 0))
    {throw std::runtime_error("invalid or feedback IMU topic");}
    runtime_ = fs::path(text("runtime_directory", ""));
    if (runtime_.empty() || !runtime_.is_absolute() || !fs::is_directory(runtime_)) {
      throw std::runtime_error("fresh absolute runtime_directory required");
    }
    auto templ = text("settings_template", "");auto vocabulary = text("vocabulary", "");
    auto save = text("save_atlas", "");auto load = text("load_atlas", "");
    if (!save.empty() &&
      (save == load || fs::exists(save + ".osa") || !fs::path(save).is_absolute() ||
      !fs::is_directory(fs::path(save).parent_path())))
    {
      throw std::runtime_error("atlas output must be new and absolute");
    }
    if (!load.empty() && (!fs::path(load).is_absolute() || !fs::is_regular_file(load + ".osa"))) {
      throw std::runtime_error("atlas input missing");
    }

    loaded_ = !load.empty();
    save_atlas_ = save;
    if (inertial_ && !save.empty()) {
      native_save_atlas_ = (runtime_ / "native-save-atlas").string();
      if (save_atlas_ == native_save_atlas_ || fs::exists(native_save_atlas_ + ".osa")) {
        throw std::runtime_error("private inertial atlas staging path already exists");
      }
      save = native_save_atlas_;
    }
    std::ifstream input(templ);if (!input || !fs::is_regular_file(vocabulary)) {
      throw std::runtime_error("settings or vocabulary missing");
    }
    cv::FileStorage calibration_file(templ, cv::FileStorage::READ);
    if (!calibration_file.isOpened() || std::string(calibration_file["Camera.type"]) != "PinHole") {
      throw std::runtime_error("unsupported estimator calibration template");
    }
    for (const auto & parameter: std::map<std::string, double>{
      {"Camera1.fx", 337.3570508552852}, {"Camera1.fy", 337.3570508552852},
      {"Camera1.cx", 320.5}, {"Camera1.cy", 240.5}, {"Camera.width", 640},
      {"Camera.height", 480}, {"Camera.RGB", 0}, {"RGBD.DepthMapFactor", 1},
      {"Camera1.k1", 0}, {"Camera1.k2", 0}, {"Camera1.p1", 0}, {"Camera1.p2", 0},
      {"Camera1.k3", 0}})
    {
      const auto value = calibration_file[parameter.first];
      if (value.empty() || !std::isfinite(double(value)) ||
        std::abs(double(value) - parameter.second) > 1e-6)
      {
        throw std::runtime_error("estimator/input calibration mismatch");
      }
    }
    if (inertial_) {
      cv::Mat extrinsic;calibration_file["IMU.T_b_c1"] >> extrinsic;
      const cv::Mat expected = (cv::Mat_<float>(4, 4) <<
        0, 0, 1, .29350, -1, 0, 0, .02329, 0, -1, 0, .09588, 0, 0, 0, 1);
      if (extrinsic.type() != CV_32F || extrinsic.size() != expected.size() ||
        !cv::checkRange(extrinsic) || cv::norm(extrinsic - expected, cv::NORM_INF) > 1e-6)
      {throw std::runtime_error("IMU/camera extrinsic mismatch");}
      for (const auto & parameter: std::map<std::string, double>{
        {"IMU.Gravity", 9.8}, {"IMU.Frequency", 200.}, {"IMU.NoiseGyro", 1.2020815280171309e-05},
        {"IMU.NoiseAcc", .001202081528017131}, {"IMU.GyroWalk", 1e-6},
        {"IMU.AccWalk", 1e-4}, {"IMU.InsertKFsWhenLost", 0}})
      {
        const auto value = calibration_file[parameter.first];
        if (value.empty() || !std::isfinite(double(value)) ||
          std::abs(double(value) - parameter.second) > 1e-10)
        {throw std::runtime_error("native IMU calibration mismatch");}
      }
      // Initialization policy is not a physical calibration value. The
      // upstream acceleration-excitation check (0) must remain selectable.
      const auto fast_init = calibration_file["IMU.fastInit"];
      if (fast_init.empty() || !std::isfinite(double(fast_init)) ||
        (double(fast_init) != 0. && double(fast_init) != 1.))
      {throw std::runtime_error("IMU.fastInit must be 0 or 1");}
      if (std::abs(ORB_SLAM3::IMU::GravityMagnitude() - 9.8) > 1e-6) {
        throw std::runtime_error("loaded ORB core gravity differs from Gazebo calibration");
      }
    }
    if (!calibration_file["System.SaveAtlasToFile"].empty() ||
      !calibration_file["System.LoadAtlasFromFile"].empty())
    {
      throw std::runtime_error("atlas lifecycle must use explicit launch arguments");
    }
    if (inertial_) {
      settings_hash_ = easyfly_orbslam3::fileSha256(templ, 512 * 1024);
      vocabulary_hash_ = easyfly_orbslam3::fileSha256(vocabulary, 256 * 1024 * 1024);
      if (!save_atlas_.empty() && fs::exists(save_atlas_ + ".easyfly.yaml")) {
        throw std::runtime_error("inertial atlas manifest output must be new");
      }
      if (loaded_) {
        easyfly_orbslam3::verifyInertialAtlas(load, settings_hash_, vocabulary_hash_);
        const auto original_hash =
          easyfly_orbslam3::fileSha256(load + ".osa", 1024ULL * 1024 * 1024);
        const auto snapshot = (runtime_ / "loaded-atlas").string();
        fs::copy_file(load + ".osa", snapshot + ".osa", fs::copy_options::none);
        if (original_hash !=
          easyfly_orbslam3::fileSha256(snapshot + ".osa", 1024ULL * 1024 * 1024))
        {
          throw std::runtime_error("atlas changed during owned snapshot acquisition");
        }
        load = snapshot;
      }
    }
    const fs::path settings = runtime_ / "runtime-settings.yaml";
    if (fs::exists(settings)) {throw std::runtime_error("runtime directory already used");}
    std::ofstream output(settings);output << input.rdbuf();
    if (!save.empty()) {output << "\nSystem.SaveAtlasToFile: " << std::quoted(save) << "\n";}
    if (!load.empty()) {output << "\nSystem.LoadAtlasFromFile: " << std::quoted(load) << "\n";}
    output.close();
    engine_ = std::make_unique<ORB_SLAM3::System>(
      vocabulary,
      settings.string(), inertial_ ? ORB_SLAM3::System::IMU_RGBD : ORB_SLAM3::System::RGBD, false);
    try {
      if (loaded_ && inertial_) {engine_->UseLoadedAtlasForInertialMapping();} else if (loaded_) {
        engine_->UseLoadedAtlasForLocalization();
      }
    } catch (...) {engine_->Shutdown();throw;}

    auto initial = engine_->GetRuntimeSnapshot();loaded_map_ = initial.map_id;
    loaded_keyframes_ = initial.keyframes;
    if (loaded_ && inertial_) {
      loaded_map_ = initial.loaded_reference_map;
      loaded_keyframes_ = initial.loaded_reference_keyframes;
    }
    std::random_device rnd;std::ostringstream id;id << std::hex << rnd() << rnd() << rnd() << rnd();
    session_ = id.str();
    auto status_qos = rclcpp::QoS(1).transient_local();
    status_ = create_publisher<Status>("/localization/orbslam3/status", status_qos);
    pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/orbslam3/pose", 16);
    raw_ = create_publisher<geometry_msgs::msg::PoseStamped>("/orbslam3/raw/body_pose", 32);
    diagnostic_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/orbslam3/raw/visual_tracking", 32);
    map_diagnostic_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/orbslam3/raw/map_status", 32);
    information_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/orbslam3/raw/inertial_information", 32);
    atlas_events_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/orbslam3/raw/atlas_events", 32);
    forward_rgb_ = create_publisher<Image>("/orbslam3/input/rgb", 8);
    forward_depth_ = create_publisher<Image>("/orbslam3/input/depth", 8);
    if (inertial_) {
      forward_imu_ = create_publisher<Imu>("/orbslam3/input/imu", 256);
      imu_ = create_subscription<Imu>(
        imu_topic_, rclcpp::SensorDataQoS().keep_last(1000),
        [this](Imu::ConstSharedPtr m) {imu(m);});
    }
    rgb_ =
      create_subscription<Image>(
      rgb_topic_, rclcpp::SensorDataQoS().keep_last(32),
      [this](Image::ConstSharedPtr m) {image(m, true);});
    depth_ =
      create_subscription<Image>(
      depth_topic_, rclcpp::SensorDataQoS().keep_last(32),
      [this](Image::ConstSharedPtr m) {image(m, false);});
    rgb_info_ =
      create_subscription<Info>(
      "/camera/color/camera_info", rclcpp::SensorDataQoS(),
      [this](Info::ConstSharedPtr m) {calibration(*m, true);});
    depth_info_ = create_subscription<Info>(
      "/camera/aligned_depth_to_color/depth/camera_info",
      rclcpp::SensorDataQoS(), [this](Info::ConstSharedPtr m) {calibration(*m, false);});
    if (require_admission_) {
      vehicle_ = create_subscription<easyfly_msgs::msg::UAVState>(
        "/uav_state", 16,
        [this](easyfly_msgs::msg::UAVState::ConstSharedPtr message) {
          vehicle_state_ = message;vehicle_receipt_ = steady();
        });
      activation_ = create_service<std_srvs::srv::Trigger>(
        "/localization/orbslam3/activate",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
          const auto current = engine_->GetRuntimeSnapshot();
          if (!failure_.empty() || !current.inertial_ready || !have_snapshot_ ||
          current.epoch != candidate_epoch_ || !fresh(
            last_candidate_source_,
            candidate_receipt_) ||
          stable_ < 5 || (loaded_ && (current.merges == 0 || current.reused_keyframes < 10)))
          {
            response->message = "fresh completed native initialization/map merge and conditional covariance required";return;
          }
          if (!navigation_admitted_) {
            if (!vehicle_state_ || vehicle_state_->header.frame_id != "odom" ||
            !vehicle_state_->connected || vehicle_state_->armed || !vehicle_state_->pose_valid ||
            !vehicle_state_->velocity_valid ||
            !fresh(stamp(vehicle_state_->header.stamp), vehicle_receipt_) ||
            get_publishers_info_by_topic("/uav_state").size() != 1 ||
            !std::isfinite(vehicle_state_->velocity_x) ||
            !std::isfinite(vehicle_state_->velocity_y) ||
            !std::isfinite(vehicle_state_->velocity_z) ||
            std::hypot(
              vehicle_state_->velocity_x, vehicle_state_->velocity_y,
              vehicle_state_->velocity_z) > .15)
            {
              response->message = "land and disarm after inertial initialization; fresh unique settled vehicle state required";return;
            }
            activation_epoch_ = current.epoch;navigation_admitted_ = true;
          }
          response->success = true;response->message = "inertial navigation pose epoch explicitly activated; no flight command issued";
        });
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {report();});
  }
  void finish()
  {
    if (finished_ || !engine_) {return;} finished_ = true;
    engine_->Shutdown();
    auto s = engine_->GetRuntimeSnapshot();
    if (inertial_ && !save_atlas_.empty()) {
      easyfly_orbslam3::publishNewInertialAtlas(native_save_atlas_ + ".osa", save_atlas_ + ".osa");
      easyfly_orbslam3::saveInertialManifest(
        save_atlas_, settings_hash_, vocabulary_hash_,
        s.inertial_map_initialized, s.keyframes);
    }
    if (s.keyframes) {engine_->SaveTrajectoryTUM((runtime_ / "camera-trajectory.txt").string());}
    std::ofstream f(runtime_ / "atlas-final.json");
    f << "{\"workers_joined\":true,\"loaded\":" << (loaded_ ? "true" : "false")
      << ",\"loaded_map_id\":" << loaded_map_ << ",\"loaded_keyframes\":" << loaded_keyframes_
      << ",\"map_id\":" << s.map_id << ",\"keyframes\":" << s.keyframes << ",\"map_points\":" <<
      s.map_points
      << ",\"loops\":" << s.loops << ",\"relocalizations\":" << s.relocalizations <<
      ",\"epoch\":" << s.epoch << ",\"merges\":" << s.merges
      << ",\"reused_keyframes\":" << s.reused_keyframes
      << ",\"inertial_map_initialized\":" << (s.inertial_map_initialized ? "true" : "false") <<
      "}\n";
  }

private:
  void fail(const std::string & why)
  {
    if (failure_.empty()) {
      failure_ = why;++generation_;rgbs_.clear();depths_.clear();imus_.clear();RCLCPP_ERROR(
        get_logger(), "latched reset: %s", why.c_str());
    }
  }
  void update_clock()
  {
    const int64_t n = now().nanoseconds();if (clock_ > 0 && n < clock_) {
      fail("clock_rollback");
    }
    clock_ = n;
  }
  bool fresh(int64_t source, double receipt) const
  {
    return clock_ > 0 && source > 0 && clock_ - source >= -200000000 &&
           clock_ - source <= 500000000 && steady() - receipt <= .5;
  }
  void calibration(const Info & m, bool rgb)
  {
    const std::string frame = rgb ? "camera_color_optical_frame" : "camera_depth_optical_frame";
    const double k[9] = {337.3570508552852, 0, 320.5, 0, 337.3570508552852, 240.5, 0, 0, 1};
    bool valid = m.width == 640 && m.height == 480 && m.header.frame_id == frame;
    for (int i = 0; i < 9; ++i) {
      valid = valid && std::isfinite(m.k[i]) && std::abs(m.k[i] - k[i]) < 1e-6;
    }
    for (double d:m.d) {valid = valid && std::isfinite(d) && std::abs(d) < 1e-9;}
    if (!valid) {fail("camera_calibration_mismatch");return;}
    (rgb ? rgb_calib_ : depth_calib_) = true;
  }
  void image(Image::ConstSharedPtr m, bool rgb)
  {
    update_clock();if (!failure_.empty()) {return;}
    try {
      const int64_t source = stamp(m->header.stamp);
      auto & previous = rgb ? rgb_source_ : depth_source_;
      auto & receipt = rgb ? rgb_receipt_ : depth_receipt_;
      if (source <= previous) {fail("image_rollback_or_duplicate");return;}
      previous = source;receipt = steady();
      const bool layout = m->width == 640 && m->height == 480 && !m->is_bigendian &&
        m->header.frame_id == (rgb ? "camera_color_optical_frame" : "camera_depth_optical_frame") &&
        (rgb ? (m->encoding == "rgb8" || m->encoding == "bgr8") : m->encoding == "32FC1") &&
        m->step == m->width * (rgb ? 3u : 4u) && m->data.size() == size_t(m->step) * m->height;
      if (!layout) {fail("invalid_image_layout");return;}
      if (!fresh(source, receipt)) {++rejected_;return;}
      auto & buffer = rgb ? rgbs_ : depths_;buffer[source] = {m, receipt};
      while (buffer.size() > 32) {buffer.erase(buffer.begin());++rejected_;}
      pair();
    } catch (const std::exception & e) {fail(e.what());}
  }
  void imu(Imu::ConstSharedPtr m)
  {
    update_clock();if (!failure_.empty()) {return;}
    try {
      const auto source = stamp(m->header.stamp);
      if (source <= imu_source_) {++imu_rejected_;fail("imu_rollback_or_duplicate");return;}
      if (imu_source_ && source - imu_source_ > 20000000) {
        ++imu_rejected_;fail("imu_source_gap");return;
      }
      const auto & a = m->linear_acceleration;const auto & w = m->angular_velocity;
      if (m->header.frame_id != "livox_imu_frame" ||
        !std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z) ||
        !std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.z))
      {++imu_rejected_;fail("invalid_imu_frame_or_values");return;}
      imu_source_ = source;imu_receipt_ = steady();
      if (!fresh(source, imu_receipt_)) {++imu_rejected_;return;}
      if (queued_imu_source_ && source - queued_imu_source_ > 20000000) {
        ++imu_rejected_;fail("imu_admission_gap");return;
      }
      queued_imu_source_ = source;
      if (imus_.size() >= 2000) {++imu_rejected_;fail("imu_queue_overflow");return;}
      imus_.push_back(m);pair();
    } catch (const std::exception & e) {fail(e.what());}
  }
  void pair()
  {
    if (!rgb_calib_ || !depth_calib_ || !failure_.empty()) {return;}
    while (!rgbs_.empty() && !depths_.empty() && failure_.empty()) {
      auto a = rgbs_.begin(), b = depths_.begin();
      if (std::abs(a->first - b->first) > 1000000) {
        if (a->first < b->first) {
          rgbs_.erase(a);
        } else {
          depths_.erase(b);
        } ++rejected_;continue;
      }
      if (inertial_ && (imus_.empty() || stamp(imus_.back()->header.stamp) < a->first)) {return;}
      const auto rgb = a->second, depth = b->second;rgbs_.erase(a);depths_.erase(b);
      const auto source = stamp(rgb.msg->header.stamp);
      if (!fresh(source, rgb.receipt) || !fresh(stamp(depth.msg->header.stamp), depth.receipt)) {
        ++rejected_;continue;
      }
      std::vector<ORB_SLAM3::IMU::Point> measurements;
      if (inertial_) {
        if (!fresh(imu_source_, imu_receipt_)) {++rejected_;continue;}
        while (!imus_.empty()) {
          const auto m = imus_.front();imus_.pop_front();
          const auto t = stamp(m->header.stamp);
          const auto & a = m->linear_acceleration;const auto & w = m->angular_velocity;
          measurements.emplace_back(a.x, a.y, a.z, w.x, w.y, w.z, t / 1e9);
          forward_imu_->publish(*m);++imu_forwarded_;
          // Include the original first boundary sample once. The engine retains
          // its boundary internally for its own standard preintegration.
          if (t >= source) {break;}
        }
      }
      forward_rgb_->publish(*rgb.msg);forward_depth_->publish(*depth.msg);++forwarded_;
      auto color = cv_bridge::toCvCopy(rgb.msg, "bgr8")->image;
      auto range = cv_bridge::toCvCopy(depth.msg, "32FC1")->image;
      // Gazebo +/-infinity denotes missing depth. Mask only those invalid values internally.
      for (int y = 0; y < range.rows; ++y) {
        float * row = range.ptr<float>(y);for (int x = 0; x < range.cols; ++x) {
          if (!std::isfinite(row[x]) || row[x] <= 0) {
            row[x] = 0;
          }
        }
      }
      auto before = engine_->GetRuntimeSnapshot();
      const auto Tcw = engine_->TrackRGBD(color, range, source / 1e9, measurements);
      snapshot_ = engine_->GetRuntimeSnapshot();have_snapshot_ = true;
      if (require_admission_ && navigation_admitted_ && snapshot_.epoch != activation_epoch_) {
        fail("atlas_correction_after_navigation_activation");
      }
      if (pose_stamp_ &&
        (snapshot_.epoch != accepted_epoch_ || snapshot_.map_id != accepted_map_))
      {
        fail("atlas_correction_or_map_change");
      }
      if (pose_stamp_ && before.epoch != snapshot_.epoch) {
        fail("atlas_correction_during_tracking");
      }
      diagnostics(rgb.msg->header.stamp);
      if (snapshot_.state != 2 || snapshot_.inliers < 30 || !Tcw.matrix().allFinite() ||
        (inertial_ && (!snapshot_.inertial_ready ||
        (loaded_ && (snapshot_.merges == 0 || snapshot_.reused_keyframes < 10)))))
      {
        stable_ = 0;candidate_source_ = 0;continue;
      }
      // Fixed optical-to-FLU axes and physical base/camera extrinsic, not truth alignment.
      Eigen::Matrix3f Rbc;Rbc << 0, 0, 1, -1, 0, 0, 0, -1, 0;
      // Inertial initialization aligns the map to gravity (Z-up).
      Sophus::SE3f normalized(Eigen::Quaternionf(
          inertial_ ? Eigen::Matrix3f::Identity().eval() :
          Rbc), Eigen::Vector3f::Zero());
      Sophus::SE3f camera_to_base(Eigen::Quaternionf(Rbc.transpose()),
        Eigen::Vector3f(0, .22f, -.2825f));
      const auto body = normalized * Tcw.inverse() * camera_to_base;
      geometry_msgs::msg::PoseStamped raw;raw.header = rgb.msg->header;
      raw.header.frame_id = "orbslam3_map";
      raw.pose.position.x = body.translation().x();raw.pose.position.y = body.translation().y();
      raw.pose.position.z = body.translation().z();
      const auto q = body.unit_quaternion();raw.pose.orientation.x = q.x();
      raw.pose.orientation.y = q.y();raw.pose.orientation.z = q.z();
      raw.pose.orientation.w = q.w();raw_->publish(raw);
      easyfly_orbslam3::Covariance6 conditional;
      if (inertial_) {
        easyfly_orbslam3::Information15 information;Eigen::Matrix3d Rwb;
        const bool have_information = engine_->GetConditionalInertialInformation(information, Rwb);
        const bool covariance_valid = have_information &&
          easyfly_orbslam3::conditionalBaseCovariance(
          information, Rwb, Eigen::Vector3d(.011, .02329, -.12412), conditional);
        if (have_information) {
          diagnostic_msgs::msg::DiagnosticArray trace;trace.header = raw.header;
          diagnostic_msgs::msg::DiagnosticStatus entry;
          entry.name = "conditional_inertial_information";
          auto matrix = [&](const char * name, const auto & value) {
              std::ostringstream stream;stream << std::setprecision(17);
              for (int r = 0; r < value.rows(); ++r) {
                for (int c = 0; c < value.cols(); ++c) {
                  if (r || c) {stream << ',';} stream << value(r, c);
                }
              }
              diagnostic_msgs::msg::KeyValue field;field.key = name;field.value = stream.str();
              entry.values.push_back(field);
            };
          matrix("information_15x15", information);matrix("imu_rotation_3x3", Rwb);
          entry.message =
            covariance_valid ? "conditional on native map/calibration" :
            "unobservable_or_invalid_information";
          trace.status.push_back(entry);information_->publish(trace);
        }
        if (!covariance_valid)
        {stable_ = 0;candidate_source_ = 0;continue;}
        if (!candidate_source_ || candidate_epoch_ != snapshot_.epoch) {
          candidate_source_ = source;candidate_epoch_ = snapshot_.epoch;stable_ = 0;
        }
        if (source - candidate_source_ < 1000000000LL) {continue;}
      }
      ++stable_;
      if (inertial_) {last_candidate_source_ = source;candidate_receipt_ = steady();}
      if (!navigation_admitted_) {report();continue;}
      if (!failure_.empty() || stable_ < 5 ||
        !fresh(source, rgb.receipt) || !fresh(stamp(depth.msg->header.stamp), depth.receipt) ||
        (inertial_ && !fresh(imu_source_, imu_receipt_)))
      {
        continue;
      }
      geometry_msgs::msg::PoseWithCovarianceStamped msg;msg.header = raw.header;
      if (!continuity_.accept(
          source,
          {raw.pose.position.x, raw.pose.position.y, raw.pose.position.z},
          {q.x(), q.y(), q.z(), q.w()}))
      {
        fail("unannounced_pose_discontinuity");
        report();
        continue;
      }
      msg.pose.pose = raw.pose;
      // Inertial information is conditional on fixed map/calibration; RGB-D
      // has no native covariance. No empirical constant is substituted.
      if (inertial_) {
        for (int r = 0; r < 6; ++r) {
          for (int c = 0; c < 6; ++c) {msg.pose.covariance[6 * r + c] = conditional(r, c);}
        }
      } else {msg.pose.covariance[0] = -1.;}
      pose_->publish(msg);++public_pose_count_;pose_stamp_ = source;
      pose_receipt_ = steady();
      accepted_epoch_ = snapshot_.epoch;accepted_map_ = snapshot_.map_id;
      report();
    }
  }
  void diagnostics(const builtin_interfaces::msg::Time & source)
  {
    diagnostic_msgs::msg::DiagnosticArray array;array.header.stamp = source;
    diagnostic_msgs::msg::DiagnosticStatus s;s.name = "orbslam3/visual_tracking";
    s.hardware_id = "gazebo_rgbd";
    auto field = [&](const std::string & k, auto v) {
        diagnostic_msgs::msg::KeyValue p;p.key = k;p.value = std::to_string(v);
        s.values.push_back(p);
      };
    field("tracked_features", snapshot_.inliers);field("initialized", snapshot_.state == 2 ? 1 : 0);
    field("inertial_ready", snapshot_.inertial_ready ? 1 : 0);
    field("tracking_state", snapshot_.state);
    field("map_id", snapshot_.map_id);field("keyframes", snapshot_.keyframes);field(
      "map_points",
      snapshot_.map_points);
    field("correction_epoch", snapshot_.epoch);field("loop_closures", snapshot_.loops);field(
      "map_merges", snapshot_.merges);field("relocalizations", snapshot_.relocalizations);
    field("reused_keyframes", snapshot_.reused_keyframes);
    field("native_inertial_map_initialized", snapshot_.inertial_map_initialized ? 1 : 0);
    field("conditional_inertial_covariance", inertial_ ? 1 : 0);
    field("loaded_atlas", loaded_ ? 1 : 0);field("loaded_map_id", loaded_map_);field(
      "loaded_keyframes", loaded_keyframes_);
    array.status.push_back(s);
    array.status[0].name = "orbslam3/map_lifecycle";map_diagnostic_->publish(array);
    array.status[0].name = "orbslam3/visual_tracking";array.status[0].values.resize(2);
    diagnostic_->publish(array);
  }
  void report()
  {
    update_clock();
    if (steady() - graph_receipt_ > .2) {
      graph_receipt_ = steady();
      if (inertial_ && count_publishers(imu_topic_) > 1) {fail("multiple_imu_publishers");}
      for (const auto & topic:
        {rgb_topic_, depth_topic_, std::string("/camera/color/camera_info"),
          std::string("/camera/aligned_depth_to_color/depth/camera_info")})
      {
        if (count_publishers(topic) > 1) {fail("multiple_sensor_publishers");}}
    }
    if (have_snapshot_) {
      const auto current = engine_->GetRuntimeSnapshot();
      if (pose_stamp_ && (current.epoch != accepted_epoch_ || current.map_id != accepted_map_)) {
        fail("atlas_correction_or_map_change");
      }
      if (current.epoch != event_epoch_ || current.loops != event_loops_ ||
        current.merges != event_merges_ || current.map_id != event_map_)
      {
        diagnostic_msgs::msg::DiagnosticArray event;event.header.stamp = now();
        diagnostic_msgs::msg::DiagnosticStatus entry;entry.name = "orbslam3/atlas_event";
        auto add = [&](const std::string & key, uint64_t value) {
            diagnostic_msgs::msg::KeyValue v;v.key = key;v.value = std::to_string(value);
            entry.values.push_back(v);
          };
        add("correction_epoch", current.epoch);add("loop_closures", current.loops);add(
          "map_merges",
          current.merges);
        add("relocalizations", current.relocalizations);add("map_id", current.map_id);add(
          "public_pose_count", public_pose_count_);
        add("last_public_pose_stamp_ns", pose_stamp_);
        add("reset_latched", failure_.empty() ? 0 : 1);
        event.status.push_back(entry);atlas_events_->publish(event);
        event_epoch_ = current.epoch;event_loops_ = current.loops;event_merges_ = current.merges;
        event_map_ = current.map_id;
      }
    }
    Status s;s.header.stamp = now();s.header.frame_id = "orbslam3_map";s.backend_id = "orbslam3";
    s.session_id = session_;s.generation = generation_;s.tracked_frame = "base_link";
    s.state = Status::INITIALIZING;s.reason = "waiting_for_calibrated_rgbd_tracking";
    if (!failure_.empty()) {
      s.state = Status::RESET_REQUIRED;s.reason = failure_;
    } else if (!rgb_calib_ || !depth_calib_ || !rgb_source_ || !depth_source_) {
      s.state = Status::WAITING_INPUT;s.reason = "waiting_for_calibrated_rgbd";
    } else if (!fresh(rgb_source_, rgb_receipt_) || !fresh(depth_source_, depth_receipt_)) {
      s.state = Status::STALE;s.reason = "rgbd_input_stale";rgbs_.clear();depths_.clear();
    } else if (inertial_ && !imu_source_) {
      s.state = Status::WAITING_INPUT;s.reason = "waiting_for_native_imu";
    } else if (inertial_ && !fresh(imu_source_, imu_receipt_)) {
      s.state = Status::STALE;s.reason = "imu_input_stale";imus_.clear();
    } else if (inertial_ && (!have_snapshot_ || !snapshot_.inertial_ready)) {
      s.reason = "waiting_for_completed_inertial_initialization";
    } else if (inertial_ && loaded_ &&
      (snapshot_.merges == 0 || snapshot_.reused_keyframes < 10))
    {
      s.reason = "waiting_for_native_inertial_old_map_merge";
    } else if (require_admission_ && !navigation_admitted_ && stable_ >= 5 &&
      fresh(last_candidate_source_, candidate_receipt_))
    {
      s.reason = "ready_for_explicit_inertial_navigation_activation";
    } else if (pose_stamp_ && have_snapshot_ && snapshot_.state == 2 && snapshot_.inliers >= 30 &&
      fresh(pose_stamp_, pose_receipt_))
    {
      s.state = Status::ESTIMATING;s.reason = "actual_rgbd_map_inliers";
    } else if (pose_stamp_) {s.state = Status::STALE;s.reason = "tracking_lost_or_pose_stale";}
    s.pose_valid = s.state == Status::ESTIMATING;s.velocity_valid = false;
    s.covariance_available = inertial_;s.tracking_quality_available = have_snapshot_;
    s.pose_stamp = rclcpp::Time(pose_stamp_);s.camera_stamp = rclcpp::Time(rgb_source_);
    s.camera_forwarded = forwarded_;s.camera_rejected = rejected_;
    s.imu_stamp = rclcpp::Time(imu_source_);s.imu_forwarded = imu_forwarded_;
    s.imu_rejected = imu_rejected_;
    s.pose_rejected = rejected_;s.last_rejection = failure_;status_->publish(s);
  }
  struct Sample {Image::ConstSharedPtr msg;double receipt;};
  std::map<int64_t, Sample> rgbs_, depths_;
  std::deque<Imu::ConstSharedPtr> imus_;
  rclcpp::Subscription<Imu>::SharedPtr imu_;
  rclcpp::Publisher<Imu>::SharedPtr forward_imu_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr information_;
  bool inertial_ = false, require_admission_ = false, navigation_admitted_ = false;
  uint64_t activation_epoch_ = 0;
  int64_t last_candidate_source_ = 0;double candidate_receipt_ = 0., vehicle_receipt_ = 0.;
  easyfly_msgs::msg::UAVState::ConstSharedPtr vehicle_state_;
  rclcpp::Subscription<easyfly_msgs::msg::UAVState>::SharedPtr vehicle_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr activation_;
  std::string save_atlas_, native_save_atlas_, settings_hash_, vocabulary_hash_;
  int64_t candidate_source_ = 0;uint64_t candidate_epoch_ = 0;
  easyfly_orbslam3::PoseContinuity continuity_;
  int64_t imu_source_ = 0, queued_imu_source_ = 0;double imu_receipt_ = 0;
  uint64_t imu_forwarded_ = 0, imu_rejected_ = 0;
  std::unique_ptr<ORB_SLAM3::System> engine_;
  ORB_SLAM3::System::RuntimeSnapshot snapshot_{};
  rclcpp::Subscription<Image>::SharedPtr rgb_, depth_;
  rclcpp::Subscription<Info>::SharedPtr rgb_info_, depth_info_;
  rclcpp::Publisher<Status>::SharedPtr status_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr raw_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_, map_diagnostic_,
    atlas_events_;
  rclcpp::Publisher<Image>::SharedPtr forward_rgb_, forward_depth_;
  rclcpp::TimerBase::SharedPtr timer_;
  fs::path runtime_;std::string imu_topic_, rgb_topic_, depth_topic_, session_, failure_;
  bool rgb_calib_ = false, depth_calib_ = false, loaded_ = false, finished_ = false,
    have_snapshot_ = false;
  int64_t clock_ = 0, rgb_source_ = 0, depth_source_ = 0, pose_stamp_ = 0;
  double rgb_receipt_ = 0, depth_receipt_ = 0, pose_receipt_ = 0, graph_receipt_ = 0;
  uint64_t generation_ = 0, forwarded_ = 0, rejected_ = 0, accepted_epoch_ = 0, accepted_map_ = 0,
    loaded_map_ = 0, loaded_keyframes_ = 0, public_pose_count_ = 0,
    event_epoch_ = 0, event_loops_ = 0, event_merges_ = 0, event_map_ = 0;
  unsigned stable_ = 0;
};

int main(int argc, char ** argv)
{
  // Keep the ROS context valid until callbacks stop and atlas workers join.
  // A signal handler may only set this flag; all shutdown work stays here.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);std::signal(SIGTERM, request_stop);
  std::shared_ptr<RGBDNode> node;
  int result = 0;
  try {
    node = std::make_shared<RGBDNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (!stop_requested && rclcpp::ok()) {
      executor.spin_once(std::chrono::milliseconds(50));
    }
    executor.remove_node(node);
  } catch (const std::exception & e) {
    std::cerr << "ORB-SLAM3 integration failure: " << e.what() << std::endl;result = 1;
  }
  try {if (node) {node->finish();}} catch (const std::exception & e) {
    std::cerr << "ORB-SLAM3 finalization failure: " << e.what() << std::endl;result = 1;
  }
  node.reset();
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return result;
}

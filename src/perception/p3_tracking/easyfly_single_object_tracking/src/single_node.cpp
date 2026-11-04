// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_single_object_tracking/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
#include "easyfly_perception_contracts/backend_status.hpp"
#include <easyfly_perception_contracts/contracts.hpp>
#include <easyfly_msgs/msg/single_track_state_v2.hpp>
#include <easyfly_msgs/srv/manage_single_track_v2.hpp>
namespace pc = easyfly::perception;
// Copyright 2026 EasyFly Project
//

#include <condition_variable>
#include <deque>
#include <algorithm>
#include <thread>
#include "safety.hpp"
#include "easyfly_single_object_tracking/backend_factory.hpp"
#include "easyfly_single_tracking_api/roi.hpp"
#include <opencv2/imgproc.hpp>
#include "easyfly_backend_registry/configuration_yaml.hpp"
namespace ef_single = easyfly_single_object_tracking;
#include <easyfly_msgs/msg/single_track_state.hpp>
#include <easyfly_msgs/msg/track_info_array.hpp>
#include <easyfly_msgs/srv/get_capabilities.hpp>
#include <easyfly_msgs/srv/manage_single_track.hpp>
#include <sensor_msgs/msg/image.hpp>
class SingleNode : public rclcpp::Node
{
public:
  SingleNode()
  : Node("single_object_tracking")
  {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.read_only = true;
    backend_config_.backend = declare_parameter<std::string>("tracker_backend", "csrt", d);
    const auto parameters = declare_parameter<std::string>("backend_parameters", "{}", d);
    backend_config_.parameters = easyfly::backends::parameters_from_yaml(
      YAML::Load(parameters), "single_tracking", backend_config_.backend);
    // Resolve unknown backend/schema failures before advertising services or starting a worker.
    backend_config_ = ef_single::tracker_factory().resolve(backend_config_);
    ef_single::make_tracker(backend_config_);

    const auto version = declare_parameter("perception_version", 1, d);
    if (version != 1 && version != 2) {throw std::invalid_argument("perception_version must be 1 or 2");}
    v2_ = version == 2;
    legacy_ = declare_parameter("publish_legacy_compatibility", !v2_, d);
    camera_ = declare_parameter("camera_id", "front", d);
    native_pub_ = create_publisher<easyfly_msgs::msg::SingleTrackStateV2>("/perception/v2/single_track/state", 10);
    frame_ = declare_parameter("optical_frame", "camera_color_optical_frame", d);

    age_ = declare_parameter("max_age_seconds", .5, d);

    if (frame_.empty() || !std::isfinite(age_) || age_ < .05 || age_ > 5) {
      throw std::invalid_argument("invalid tracking configuration");
    }
    backend_status_ = std::make_unique<pc::BackendStatus>(*this, "single_tracking", backend_config_.backend, age_);
    backend_status_->created(backend_config_.backend, easyfly::backends::parameters_json(backend_config_.parameters));
    state_.track_id = -1;
    state_.phase = "idle";
    state_.reason = "explicit_roi_required";

    pub_ = create_publisher<easyfly_msgs::msg::SingleTrackState>("/single_track/state", 10);

    tracks_ = create_publisher<easyfly_msgs::msg::TrackInfoArray>("/single_tracks", 10);

    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::Image::ConstSharedPtr m, const rclcpp::MessageInfo & info) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (v2_) {
          const auto source = pc::writer(*this, "/camera/image_raw", info);
          if (source.empty()) {invalidate("ambiguous_image_writer"); latest_.reset(); return;}
          if (!image_writer_.empty() && source != image_writer_) {
            producer_.reset(); watermark_ = 0; latest_.reset(); invalidate("image_source_changed");
          }
          image_writer_ = source;
        }
        double receipt = ef_safety::steady();

        const int channels = m->encoding ==
        "mono8" ? 1 : (m->encoding == "bgr8" || m->encoding == "rgb8") ? 3 : 0;

        if (!good_clock() ||
        !ef_safety::fresh(m->header, now().nanoseconds(), receipt, age_, frame_) ||
        ef_safety::stamp(m->header) <= watermark_ || !channels || m->width < 16 || m->height < 16 ||
        uint64_t(m->width) * m->height > 2073600 || m->step != m->width * channels ||
        m->data.size() != uint64_t(m->step) * m->height)
        {
          invalidate("image_invalid");
          latest_.reset();
          return;
        }
        watermark_ = ef_safety::stamp(
          m->header);
        latest_ = m;
        receipt_ = receipt;
        history_.push_back({m, receipt}); history_bytes_ += m->data.size();
        while (history_.size() > 16 || history_bytes_ > 64U * 1024 * 1024 ||
        (!history_.empty() && receipt - history_.front().receipt > age_))
        {
          history_bytes_ -= history_.front().image->data.size(); history_.pop_front();
        }
        backend_status_->input(true, "", ef_safety::stamp(m->header));
        pending_ = true;
        cv_.notify_one();
      });

    manage_ = create_service<easyfly_msgs::srv::ManageSingleTrack>(
      "/single_track/manage",
      [this](easyfly_msgs::srv::ManageSingleTrack::Request::SharedPtr r,
      easyfly_msgs::srv::ManageSingleTrack::Response::SharedPtr response) {
        if (v2_ && r->operation != "query") {
          response->message = "use_native_v2_service"; return;
        }
        handle_manage(r, response);

      });

    native_manage_ = create_service<easyfly_msgs::srv::ManageSingleTrackV2>("/single_track/v2/manage",
      [this](easyfly_msgs::srv::ManageSingleTrackV2::Request::SharedPtr r,
        easyfly_msgs::srv::ManageSingleTrackV2::Response::SharedPtr response) {
        if (!v2_) {response->message = "native_v2_mode_required"; return;}
        auto request = std::make_shared<easyfly_msgs::srv::ManageSingleTrack::Request>();
        request->operation = r->operation; request->image_header = r->image_header;
        request->roi_x = r->roi_x; request->roi_y = r->roi_y;
        request->roi_width = r->roi_width; request->roi_height = r->roi_height; request->class_name = r->class_name;
        auto result = std::make_shared<easyfly_msgs::srv::ManageSingleTrack::Response>();
        handle_manage(request, result);
        std::lock_guard<std::mutex> lock(mutex_);
        response->accepted = result->accepted; response->message = result->message; response->state = native_state_;
      });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this] {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!good_clock() ||
        (latest_ &&
        !ef_safety::fresh(latest_->header, now().nanoseconds(), receipt_, age_, frame_)) ||
        (state_.valid &&
        !ef_safety::fresh(state_.header, now().nanoseconds(), state_receipt_, age_, frame_)))
        {
          invalidate("expired_or_clock_invalid");
          latest_.reset();
        }
      });

    capabilities_ = create_service<easyfly_msgs::srv::GetCapabilities>(
      "/platform/single_tracking/capabilities",
      [this](easyfly_msgs::srv::GetCapabilities::Request::SharedPtr,
      easyfly_msgs::srv::GetCapabilities::Response::SharedPtr r) {
        std::lock_guard<std::mutex> lock(
          mutex_);
        r->header.stamp = now();
        r->names = {backend_config_.backend + "_explicit_roi"};
        r->implemented = {true};

        r->ready =
        {good_clock() && state_.valid && latest_ &&
          ef_safety::fresh(
            state_.header,
            now().nanoseconds(), state_receipt_, age_, frame_)};

        r->details = {"Explicit ROI only; independent ID namespace; no 3D claim. "
          "Calibrated confidence is valid only when supplied by the selected backend."};
      });

    worker_ = std::thread(
      [this] {
        run();
      });
  }
  ~SingleNode() override
  {
    {std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      cv_.notify_one();}
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  std::unique_ptr<pc::BackendStatus> backend_status_;
  easyfly::backends::BackendConfig backend_config_;
  bool calibrated_confidence_valid_{false};
  double calibrated_confidence_value_{0};
  void handle_manage(easyfly_msgs::srv::ManageSingleTrack::Request::SharedPtr r,
    easyfly_msgs::srv::ManageSingleTrack::Response::SharedPtr response)
  {
        std::lock_guard<std::mutex> lock(
          mutex_);
        response->accepted = false;

        if (r->operation == "query") {
          response->accepted = true;
        } else if (r->operation == "reset") {
          ++generation_; if (v2_) {producer_.reset();}
          initialize_ = false;
          init_image_.reset();
          state_.track_id = -1;
          state_.phase = "idle";
          state_.valid = false;
          state_.reason = "explicit_reset";
          publish();
          response->accepted = true;
        } else if (r->operation == "initialize") {
          const bool clock_ok = good_clock();
          const auto found = std::find_if(
            history_.begin(), history_.end(), [&](const auto & frame) {
              return frame.image->header == r->image_header;
            });
          const auto selected = found == history_.end() ? nullptr : found->image;
          const double selected_receipt = found == history_.end() ? 0 : found->receipt;
          if (!clock_ok || !selected ||
          !ef_safety::fresh(
            selected->header, now().nanoseconds(), selected_receipt, age_,
            frame_) ||
          !easyfly::single_tracking::roi(
            r->roi_x, r->roi_y, r->roi_width,
            r->roi_height) || r->class_name.size() > 128 || next_id_ == INT32_MAX)
          {
            response->message = "invalid_or_stale_displayed_roi";
          } else {
            auto box =
            easyfly::single_tracking::pixels(
              r->roi_x, r->roi_y, r->roi_width, r->roi_height, selected->width,
              selected->height);

            if (box.width < 8 || box.height < 8) {
              response->message = "roi_too_small";
            } else {
              ++generation_; if (v2_) {producer_.reset();}
              width_ = selected->width; height_ = selected->height;
              box_ = box;
              initialize_ = true;
              pending_ = true;
              init_image_ = selected;
              init_receipt_ = selected_receipt;
              state_.header = selected->header;

              state_.track_id = next_id_++;
              state_.phase = state_.phase == "lost" ? "relocalizing" : "initializing";
              state_.valid = false;

              state_.reason = backend_config_.backend + "_initialization_pending";
              class_ = r->class_name;
              response->accepted = true;
              publish();
              cv_.notify_one();
            }
          }
        } else {
          response->message = "unsupported_operation";
        }
        if (response->message.empty()) {
          response->message = response->accepted ? "accepted_not_tracking_success" : "rejected";
        }
        response->state = state_;
  }
  bool good_clock()
  {
    const auto t = now().nanoseconds();
    if (t < clock_.last) {
      watermark_ = 0;
      latest_.reset();
      invalidate("clock_regression");
    }
    return clock_.check(t, age_);
  }
  void invalidate(const std::string & reason)
  {
    if (backend_status_) backend_status_->input(false, reason);
    ++generation_;
    history_.clear(); history_bytes_ = 0; init_image_.reset();
    initialize_ = false;
    pending_ = false;
    state_.valid = false;
    calibrated_confidence_valid_ = false;

    if (state_.track_id >= 0) {
      state_.phase = "lost";
    }
    state_.reason = reason;
    publish();
  }
  void publish()
  {
    easyfly_msgs::msg::TrackInfoArray output;
    output.header = state_.header;
    if (state_.valid) {
      output.tracks.push_back(state_.track);
    }
    try {
      if (legacy_) {pub_->publish(state_); tracks_->publish(output);}
      if (v2_) {
        native_state_ = easyfly_msgs::msg::SingleTrackStateV2();
        native_state_.context = producer_.next(state_.header, camera_, width_, height_,
          state_.valid ? pc::Context::VALID : pc::Context::NOT_READY, state_.reason);
        native_state_.valid = state_.valid; native_state_.phase = state_.phase; native_state_.reason = state_.reason;
        auto & t = native_state_.track;
        t.key.tracker_session_id = producer_.session(); t.key.tracker_epoch = producer_.epoch();
        t.key.raw_id = state_.track_id; t.key.legacy_id_valid = state_.track_id >= 0;
        t.key.legacy_raw_id = t.key.legacy_stable_id = std::max(0, state_.track_id);
        t.class_name = class_; t.class_id = -1; t.confidence_valid = state_.valid && calibrated_confidence_valid_;
        t.confidence = t.confidence_valid ? calibrated_confidence_value_ : 0;
        t.filtered_box.x1 = box_.x; t.filtered_box.y1 = box_.y;
        t.filtered_box.x2 = box_.x + box_.width; t.filtered_box.y2 = box_.y + box_.height;
        t.observed_box = t.filtered_box; t.filtered_box_valid = t.observed_box_valid = state_.valid;
        t.observed_this_frame = t.confirmed = state_.valid;
        native_pub_->publish(native_state_);
      }
    } catch (const std::exception &) {}}
  void run()
  {
    std::unique_ptr<easyfly::single_tracking::Tracker> tracker;
    uint64_t tracker_generation = 0;

    for (;
      ;
    )
    {
      sensor_msgs::msg::Image::ConstSharedPtr image;
      uint64_t generation;
      bool init;
      cv::Rect box;

      double receipt;

      {std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(
          lock, [this] {
            return stopping_ || pending_;
          });
        if (stopping_) {
          return;
        }
        init = initialize_;
        initialize_ = false;
        pending_ = false;
        generation = generation_;

        image = init ? init_image_ : latest_;
        receipt = init ? init_receipt_ : receipt_;
        box = box_;}
      if (!image) {
        continue;
      }
      bool ok = false;
      std::optional<double> confidence;

      try {
        int channels = image->encoding == "mono8" ? 1 : 3;

        cv::Mat pixels(image->height, image->width, channels == 1 ? CV_8UC1 : CV_8UC3,
          const_cast<uint8_t *>(image->data.data()));

        cv::Mat color;

        if (channels == 1) {
          cv::cvtColor(pixels, color, cv::COLOR_GRAY2BGR);
        } else if (image->encoding == "rgb8") {
          cv::cvtColor(pixels, color, cv::COLOR_RGB2BGR);
        } else {
          color = pixels;
        }
        if (init) {
          tracker = ef_single::make_tracker(backend_config_);
          ok = tracker->initialize(color, box);

          tracker_generation = generation;
        } else if (tracker && tracker_generation == generation) {
          const auto result = tracker->update(color);
          ok = ef_single::valid_result(result, image->width, image->height);
          box = result.box;
          confidence = result.calibrated_confidence;
        }
      } catch (const std::exception &) {
        ok = false;
      }
      std::lock_guard<std::mutex> lock(mutex_);

      if (stopping_ || generation != generation_) {
        continue;
      }
      if (init && !ok) {
        invalidate(backend_config_.backend + "_initialization_failed");
        continue;
      }
      if (!tracker || tracker_generation != generation) {
        continue;
      }
      if (!good_clock() || !ok ||
        !ef_safety::fresh(image->header, now().nanoseconds(), receipt, age_, frame_) ||
        box.x < 0 || box.y < 0 || box.width <= 0 || box.height <= 0 ||
        box.x + box.width > static_cast<int>(image->width) ||
        box.y + box.height > static_cast<int>(image->height))
      {
        tracker.reset();
        invalidate(backend_config_.backend + "_lost_or_expired");
        continue;
      }
      box_ = box;
      width_ = image->width; height_ = image->height;
      state_.header = image->header;
      state_.phase = "tracking";

      state_.reason = backend_config_.backend + (confidence ? "_tracking" : "_uncalibrated_score");
      calibrated_confidence_valid_ = confidence.has_value();
      calibrated_confidence_value_ = confidence.value_or(0);
      state_.valid = true;
      state_receipt_ = receipt;

      state_.track.header = image->header;
      state_.track.track_id = state_.track_id;

      state_.track.class_name = class_;
      state_.track.confidence = confidence.value_or(0);

      state_.track.bbox_center_x = static_cast<float>(box.x + box.width * .5) / image->width;

      state_.track.bbox_center_y = static_cast<float>(box.y + box.height * .5) / image->height;

      state_.track.bbox_width = static_cast<float>(box.width) / image->width;

      state_.track.bbox_height = static_cast<float>(box.height) / image->height;
      publish();
    }
  }
  bool v2_{false}, legacy_{true}; std::string camera_, image_writer_;
  uint32_t width_{0}, height_{0}; pc::Producer producer_;
  easyfly_msgs::msg::SingleTrackStateV2 native_state_;
  rclcpp::Publisher<easyfly_msgs::msg::SingleTrackStateV2>::SharedPtr native_pub_;
  rclcpp::Service<easyfly_msgs::srv::ManageSingleTrackV2>::SharedPtr native_manage_;
  std::string frame_, class_;
  double age_, receipt_{0}, init_receipt_{0}, state_receipt_{0};

  int32_t next_id_{1};
  int64_t watermark_{0};

  uint64_t generation_{0};
  bool pending_{false}, initialize_{false}, stopping_{false};

  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_;

  cv::Rect box_;
  ef_safety::ClockGate clock_;

  sensor_msgs::msg::Image::ConstSharedPtr latest_, init_image_;
  struct RetainedFrame {sensor_msgs::msg::Image::ConstSharedPtr image; double receipt;};
  std::deque<RetainedFrame> history_;
  size_t history_bytes_{0};

  easyfly_msgs::msg::SingleTrackState state_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

  rclcpp::Publisher<easyfly_msgs::msg::SingleTrackState>::SharedPtr pub_;

  rclcpp::Publisher<easyfly_msgs::msg::TrackInfoArray>::SharedPtr tracks_;

  rclcpp::Service<easyfly_msgs::srv::ManageSingleTrack>::SharedPtr manage_;

  rclcpp::Service<easyfly_msgs::srv::GetCapabilities>::SharedPtr capabilities_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SingleNode>());
  } catch (const std::exception & e) {
    fprintf(stderr, "%s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
}

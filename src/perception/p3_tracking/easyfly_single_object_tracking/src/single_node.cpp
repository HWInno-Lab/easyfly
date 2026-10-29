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

#include <condition_variable>
#include <deque>
#include <algorithm>
#include <thread>
#include "safety.hpp"
#include "easyfly_csrt/track_core.hpp"
namespace ef_single = easyfly::csrt;
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

    frame_ = declare_parameter("optical_frame", "camera_color_optical_frame", d);

    age_ = declare_parameter("max_age_seconds", .5, d);

    if (frame_.empty() || !std::isfinite(age_) || age_ < .05 || age_ > 5) {
      throw std::invalid_argument("invalid tracking configuration");
    }
    state_.track_id = -1;
    state_.phase = "idle";
    state_.reason = "explicit_roi_required";

    pub_ = create_publisher<easyfly_msgs::msg::SingleTrackState>("/single_track/state", 10);

    tracks_ = create_publisher<easyfly_msgs::msg::TrackInfoArray>("/single_tracks", 10);

    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
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
        pending_ = true;
        cv_.notify_one();
      });

    manage_ = create_service<easyfly_msgs::srv::ManageSingleTrack>(
      "/single_track/manage",
      [this](easyfly_msgs::srv::ManageSingleTrack::Request::SharedPtr r,
      easyfly_msgs::srv::ManageSingleTrack::Response::SharedPtr response) {
        std::lock_guard<std::mutex> lock(
          mutex_);
        response->accepted = false;

        if (r->operation == "query") {
          response->accepted = true;
        } else if (r->operation == "reset") {
          ++generation_;
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
          !ef_single::roi(
            r->roi_x, r->roi_y, r->roi_width,
            r->roi_height) || r->class_name.size() > 128 || next_id_ == INT32_MAX)
          {
            response->message = "invalid_or_stale_displayed_roi";
          } else {
            auto box =
            ef_single::pixels(
              r->roi_x, r->roi_y, r->roi_width, r->roi_height, selected->width,
              selected->height);

            if (box.width < 8 || box.height < 8) {
              response->message = "roi_too_small";
            } else {
              ++generation_;
              box_ = box;
              initialize_ = true;
              pending_ = true;
              init_image_ = selected;
              init_receipt_ = selected_receipt;
              state_.header = selected->header;

              state_.track_id = next_id_++;
              state_.phase = state_.phase == "lost" ? "relocalizing" : "initializing";
              state_.valid = false;

              state_.reason = "csrt_initialization_pending";
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
        r->names = {"csrt_explicit_roi"};
        r->implemented = {true};

        r->ready =
        {good_clock() && state_.valid && latest_ &&
          ef_safety::fresh(
            state_.header,
            now().nanoseconds(), state_receipt_, age_, frame_)};

        r->details = {"Explicit ROI only; independent ID namespace, "
          "no 3D or calibrated confidence claim."};
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
    ++generation_;
    history_.clear(); history_bytes_ = 0; init_image_.reset();
    initialize_ = false;
    pending_ = false;
    state_.valid = false;

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
      pub_->publish(state_);
      tracks_->publish(output);
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
          tracker = ef_single::make_tracker("csrt");
          ok = tracker->initialize(color, box);

          tracker_generation = generation;
        } else if (tracker && tracker_generation == generation) {
          const auto result = tracker->update(color);
          ok = result.ok;
          box = result.box;
        }
      } catch (const cv::Exception &) {
        ok = false;
      }
      std::lock_guard<std::mutex> lock(mutex_);

      if (stopping_ || generation != generation_) {
        continue;
      }
      if (init && !ok) {
        invalidate("csrt_initialization_failed");
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
        invalidate("csrt_lost_or_expired");
        continue;
      }
      box_ = box;
      state_.header = image->header;
      state_.phase = "tracking";

      state_.reason = "csrt_uncalibrated_score";
      state_.valid = true;
      state_receipt_ = receipt;

      state_.track.header = image->header;
      state_.track.track_id = state_.track_id;

      state_.track.class_name = class_;
      state_.track.confidence = 0;

      state_.track.bbox_center_x = static_cast<float>(box.x + box.width * .5) / image->width;

      state_.track.bbox_center_y = static_cast<float>(box.y + box.height * .5) / image->height;

      state_.track.bbox_width = static_cast<float>(box.width) / image->width;

      state_.track.bbox_height = static_cast<float>(box.height) / image->height;
      publish();
    }
  }
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

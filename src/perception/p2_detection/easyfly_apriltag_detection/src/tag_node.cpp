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
#include <thread>
#include "safety.hpp"
#include "easyfly_tag_pose/tag_core.hpp"
namespace ef_tag = easyfly::tag_pose;
#include <easyfly_msgs/msg/tag_observation_array.hpp>
#include <easyfly_msgs/srv/get_capabilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
class TagNode : public rclcpp::Node
{
public:
  TagNode()
  : Node("apriltag_detection")
  {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.read_only = true;

    frame_ = declare_parameter("optical_frame", "camera_color_optical_frame", d);

    size_ = declare_parameter("tag_size_m", 0., d);

    age_ = declare_parameter("max_age_seconds", .5, d);

    error_ = declare_parameter("max_reprojection_error_px", 2., d);

    ambiguity_ = declare_parameter("ambiguity_margin_px", .2, d);
    const auto threads = declare_parameter("opencv_threads", 2, d);

    if (frame_.empty() || !std::isfinite(size_) || size_ < 0 || size_ > 10 ||
      !std::isfinite(age_) || age_ < .05 || age_ > 5 ||
      !std::isfinite(error_) || error_ <= 0 || error_ > 20 || !std::isfinite(ambiguity_) ||
      ambiguity_ <= 0 || ambiguity_ > 5 || threads < 1 || threads > 8)
    {
      throw std::invalid_argument("invalid tag parameters");
    }
    // Budget only this standalone detector process, never other OpenCV nodes.
    cv::setNumThreads(threads);
    pub_ = create_publisher<easyfly_msgs::msg::TagObservationArray>("/tag_observations", 10);

    info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        const double receipt = ef_safety::steady();

        const bool valid = good_clock() && valid_info(m, receipt);

        const bool changed = calibration_ && !same_model(*calibration_, *m);

        calibration_ = m;
        info_receipt_ = receipt;

        if (!valid || changed) {
          ++generation_;
          invalidate(valid ? "calibration_model_changed" : "calibration_invalid");
        }
      });

    image_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(
          mutex_);
        double receipt = ef_safety::steady();
        const auto time = ef_safety::stamp(m->header);
        const bool clock_valid = good_clock();
        if (clock_valid && duplicate_ && duplicate_generation_ == generation_ &&
        valid_info(calibration_, info_receipt_) &&
        ef_safety::fresh(last_header_, now().nanoseconds(), image_receipt_, age_, frame_) &&
        same_image(*duplicate_, *m))
        {
          // Preserve the original source and receipt lifetime, including an in-flight result.
          return;
        }
        last_header_ = m->header;

        if (!clock_valid ||
        !ef_safety::fresh(
          m->header, now().nanoseconds(), receipt, age_,
          frame_) || time <= watermark_ || !bounded_image(*m))
        {
          ++generation_;
          pending_.reset();
          invalidate("image_time_or_frame_invalid");
          return;
        }
        watermark_ = time;
        pending_ = m;
        image_receipt_ = receipt;
        duplicate_ = valid_info(calibration_, info_receipt_) && size_ > 0 &&
        calibration_->width == m->width && calibration_->height == m->height ? m : nullptr;
        duplicate_generation_ = generation_;
        cv_.notify_one();
      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this] {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!good_clock() ||
        !ef_safety::fresh(last_header_, now().nanoseconds(), image_receipt_, age_, frame_))
        {
          ++generation_;
          pending_.reset();
          invalidate("expired_or_clock_invalid");
        } else if (last_valid_ && !result_fresh()) {
          invalidate("result_calibration_or_image_expired");
        }
      });

    capabilities_ = create_service<easyfly_msgs::srv::GetCapabilities>(
      "/platform/apriltag/capabilities",
      [this](easyfly_msgs::srv::GetCapabilities::Request::SharedPtr,
      easyfly_msgs::srv::GetCapabilities::Response::SharedPtr r) {
        std::lock_guard<std::mutex> lock(
          mutex_);
        r->header.stamp = now();
        r->names = {"tag36h11_pose"};
        r->implemented = {true};

        r->ready =
        {good_clock() && last_valid_ && result_fresh() && valid_info(
            calibration_,
            info_receipt_)};

        r->details = {"Explicit metric size and fresh calibration required; "
          "perception only, ambiguous pose rejected."};
      });

    worker_ = std::thread(
      [this] {
        run();
      });
  }
  ~TagNode() override
  {
    {std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      cv_.notify_one();}
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  static bool bounded_image(const sensor_msgs::msg::Image & image)
  {
    const int channels = image.encoding == "mono8" ? 1 :
      (image.encoding == "bgr8" || image.encoding == "rgb8") ? 3 : 0;
    return channels && image.width >= 16 && image.height >= 16 &&
           uint64_t(image.width) * image.height <= 2073600 &&
           image.step == uint64_t(image.width) * channels &&
           image.data.size() == uint64_t(image.step) * image.height;
  }
  static bool same_image(
    const sensor_msgs::msg::Image & a, const sensor_msgs::msg::Image & b)
  {
    // Compare all raw fields and bytes, never a hash or decoded grayscale approximation.
    return a.header == b.header && a.width == b.width && a.height == b.height &&
           a.encoding == b.encoding && a.is_bigendian == b.is_bigendian &&
           a.step == b.step && a.data == b.data;
  }
  static bool same_model(
    const sensor_msgs::msg::CameraInfo & a,
    const sensor_msgs::msg::CameraInfo & b)
  {
    return a.width == b.width && a.height == b.height && a.header.frame_id == b.header.frame_id &&
           a.distortion_model == b.distortion_model && a.k == b.k && a.d == b.d;
  }
  bool valid_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info, double receipt) const
  {
    return info && info->width >= 16 && info->height >= 16 &&
           uint64_t(info->width) * info->height <= 2073600 &&
           ef_safety::fresh(info->header, now().nanoseconds(), receipt, age_, frame_) &&
           ((info->distortion_model == "plumb_bob" && (info->d.empty() || info->d.size() == 5)) ||
           (info->distortion_model == "rational_polynomial" && info->d.size() == 8)) &&
           std::all_of(
      info->k.begin(), info->k.end(), [](double x) {
        return std::isfinite(x);
      }) &&
           std::all_of(
      info->d.begin(), info->d.end(), [](double x) {
        return std::isfinite(x);
      }) &&
           info->k[0] > 0 && info->k[4] > 0 && info->k[1] == 0 && info->k[3] == 0 &&
           info->k[2] >= 0 && info->k[2] <= info->width && info->k[5] >= 0 &&
           info->k[5] <= info->height &&
           info->k[6] == 0 && info->k[7] == 0 && info->k[8] == 1;
  }
  bool result_fresh() const
  {
    return valid_info(result_info_, result_info_receipt_) &&
           ef_safety::fresh(result_header_, now().nanoseconds(), result_receipt_, age_, frame_);
  }
  bool good_clock()
  {
    const auto t = now().nanoseconds();
    if (t < clock_.last) {
      watermark_ = 0;
      ++generation_;
      pending_.reset();
      last_valid_ = false;
    }
    const bool valid = clock_.check(t, age_);
    if (!valid) {
      duplicate_.reset();
    }
    return valid;
  }
  void invalidate(const std::string & reason)
  {
    duplicate_.reset();
    easyfly_msgs::msg::TagObservationArray output;
    output.header = last_header_;
    output.valid = false;

    output.reason = reason;
    last_valid_ = false;
    try {
      pub_->publish(output);
    } catch (const std::exception &) {
    }
  }
  void run()
  {
    for (;
      ;
    )
    {
      sensor_msgs::msg::Image::ConstSharedPtr image;

      sensor_msgs::msg::CameraInfo::ConstSharedPtr info;

      uint64_t generation;
      double receipt, info_receipt;

      {std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(
          lock, [this] {
            return stopping_ || pending_;
          });
        if (stopping_) {
          return;
        }
        image = pending_;
        pending_.reset();
        info = calibration_;
        generation = generation_;

        receipt = image_receipt_;
        info_receipt = info_receipt_;}
      easyfly_msgs::msg::TagObservationArray output;
      output.header = image->header;

      output.reason = "invalid_image_or_calibration";

      try {
        const int channels = image->encoding ==
          "mono8" ? 1 : (image->encoding == "bgr8" || image->encoding == "rgb8") ? 3 : 0;

        bool calibrated =
          valid_info(
          info,
          info_receipt) && info->width == image->width && info->height == image->height;

        const bool image_eligible =
          calibrated && channels && image->width >= 16 && image->height >= 16 &&
          uint64_t(image->width) * image->height <= 2073600 &&
          image->step == image->width * channels &&
          image->data.size() == uint64_t(image->step) * image->height;
        if (size_ <= 0) {
          output.reason = "tag_size_not_configured";
        } else if (image_eligible) {
          cv::Mat pixels(image->height, image->width, channels == 1 ? CV_8UC1 : CV_8UC3,
            const_cast<uint8_t *>(image->data.data()));

          cv::Mat gray;
          if (channels == 1) {
            gray = pixels;
          } else {
            cv::cvtColor(
              pixels, gray,
              image->encoding == "rgb8" ? cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY);
          }
          const auto detections = ef_tag::detect(gray);
          const auto & ids = detections.ids;
          const auto & corners = detections.corners;

          if (ids.size() > 64) {
            output.reason = "too_many_tags";
          } else {
            cv::Mat K(3, 3, CV_64F, const_cast<double *>(info->k.data()));
            cv::Mat D(info->d, true);

            output.valid = true;
            output.reason = ids.empty() ? "no_tags" : "detected";

            for (size_t i = 0;
              i < ids.size();
              ++i)
            {
              auto pose = ef_tag::estimate(corners[i], K, D, size_, error_, ambiguity_);

              easyfly_msgs::msg::TagObservation observation;
              observation.header = image->header;

              observation.tag_id = ids[i];

              observation.tag_size_m = size_;
              observation.reprojection_error_px = pose.error;

              observation.valid = pose.valid;
              observation.reason = pose.reason;

              if (pose.valid) {
                observation.pose.position.x = pose.t[0];
                observation.pose.position.y = pose.t[1];

                observation.pose.position.z = pose.t[2];

                observation.pose.orientation.x = pose.q[0];

                observation.pose.orientation.y = pose.q[1];

                observation.pose.orientation.z = pose.q[2];

                observation.pose.orientation.w = pose.q[3];
                observation.range_m = cv::norm(pose.t);
              }
              output.observations.push_back(observation);
            }
          }
        }
      } catch (const cv::Exception & e) {
        output.valid = false;
        output.observations.clear();
        output.reason = "opencv_error";
      }
      {std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_ || generation != generation_ ||
          ef_safety::stamp(image->header) != watermark_)
        {
          continue;
        }
        if (!good_clock() ||
          !ef_safety::fresh(image->header, now().nanoseconds(), receipt, age_, frame_))
        {
          invalidate("worker_result_expired");
          continue;
        }
        if (output.valid &&
          (!valid_info(info, info_receipt) || !valid_info(calibration_, info_receipt_)))
        {
          invalidate("worker_calibration_expired");
          continue;
        }
        last_valid_ = output.valid;
        if (!output.valid) {
          duplicate_.reset();
        }
        result_info_ = info;
        result_info_receipt_ = info_receipt;

        result_header_ = image->header;
        result_receipt_ = receipt;

        try {
          pub_->publish(output);
        } catch (const std::exception &) {}}
    }
  }
  std::mutex mutex_;
  std::condition_variable cv_;
  bool stopping_{false}, last_valid_{false};

  std::thread worker_;

  ef_safety::ClockGate clock_;
  uint64_t generation_{0};
  int64_t watermark_{0};

  std_msgs::msg::Header last_header_;

  std::string frame_;
  double size_, age_, error_, ambiguity_, image_receipt_{0}, info_receipt_{0};

  sensor_msgs::msg::Image::ConstSharedPtr pending_;
  sensor_msgs::msg::Image::ConstSharedPtr duplicate_;
  uint64_t duplicate_generation_{0};

  sensor_msgs::msg::CameraInfo::ConstSharedPtr calibration_;

  sensor_msgs::msg::CameraInfo::ConstSharedPtr result_info_;

  double result_info_receipt_{0}, result_receipt_{0};
  std_msgs::msg::Header result_header_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_;

  rclcpp::Publisher<easyfly_msgs::msg::TagObservationArray>::SharedPtr pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<easyfly_msgs::srv::GetCapabilities>::SharedPtr capabilities_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TagNode>());
  } catch (const std::exception & e) {
    fprintf(stderr, "%s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
}

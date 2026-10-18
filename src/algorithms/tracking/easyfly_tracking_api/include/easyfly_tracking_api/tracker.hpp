// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace easyfly::tracking
{
enum class Status {ok, invalid_input, unsupported_input, resource_limit, backend_error};
enum class VelocityUnit {pixels_per_frame, pixels_per_second};
enum class PixelFormat {mono8, rgb8, bgr8};

struct SourceTime
{
  std::uint64_t epoch{0};
  std::int64_t stamp_ns{0};  // Source time only; never a steady-clock reading.
};

struct Box
{
  double x1{0}, y1{0}, x2{0}, y2{0};  // Original-image pixels, xyxy; no clipping here.
};

struct Detection
{
  Box box;
  double confidence{0};  // Upstream score, not a calibrated probability.
  std::int32_t class_id{-1};  // -1 = unknown.
  std::string class_name;
  std::vector<float> appearance;  // Optional precomputed embedding, same detection.
};

// Borrowed read-only storage, valid until update() returns. No asynchronous retention.
struct ImageView
{
  const std::uint8_t * data{nullptr};
  std::size_t byte_size{0}, row_stride{0};
  std::uint32_t width{0}, height{0};
  PixelFormat format{PixelFormat::bgr8};
};

struct Frame
{
  SourceTime source;
  std::uint32_t image_width{0}, image_height{0};
  std::vector<Detection> detections;
  std::optional<ImageView> image;
  // A declaration by the detection pipeline, not something an adapter may assume.
  bool includes_low_confidence_detections{false};
};

struct Requirements
{
  bool image{false};
  std::size_t appearance_dimension{0};  // 0 = embedding not required.
  bool low_confidence_detections{false};
};

struct Limits
{
  std::size_t max_detections{1024};
  std::size_t max_appearance_dimension{4096};
  std::size_t max_image_bytes{64 * 1024 * 1024};
};

struct Capabilities
{
  std::string backend;
  Requirements inputs;
  VelocityUnit velocity_unit{VelocityUnit::pixels_per_frame};
};

struct Track
{
  std::int64_t raw_id{-1};  // Scoped by result.source.epoch; never a consolidated ID.
  Box box, last_observed_box;
  double confidence{0};
  std::int32_t class_id{-1};
  std::string class_name;
  double vx{0}, vy{0};
  std::uint64_t hits{0}, age{0}, missed_updates{0};
  bool confirmed{false};
};

struct UpdateResult
{
  Status status{Status::backend_error};  // A default result cannot claim success.
  SourceTime source;
  VelocityUnit velocity_unit{VelocityUnit::pixels_per_frame};
  std::vector<Track> tracks;  // Owning output. ok + empty means a valid empty result.
  std::string detail;
};

inline bool valid_box(const Box & box)
{
  return std::isfinite(box.x1) && std::isfinite(box.y1) &&
         std::isfinite(box.x2) && std::isfinite(box.y2) &&
         box.x2 > box.x1 && box.y2 > box.y1 &&
         std::isfinite(box.x2 - box.x1) && std::isfinite(box.y2 - box.y1);
}

// Structural/numerical checks only. Backend state/order checks are separate.
inline Status validate_frame(
  const Frame & frame, const Requirements & needs = {}, const Limits & limits = {})
{
  if (frame.source.stamp_ns <= 0 || frame.image_width == 0 || frame.image_height == 0) {
    return Status::invalid_input;
  }
  if (frame.detections.size() > limits.max_detections ||
    needs.appearance_dimension > limits.max_appearance_dimension)
  {
    return Status::resource_limit;
  }
  if ((needs.image && !frame.image) ||
    (needs.low_confidence_detections && !frame.includes_low_confidence_detections))
  {
    return Status::unsupported_input;
  }
  if (frame.image) {
    const auto & image = *frame.image;
    std::size_t channels = 0;
    switch (image.format) {
      case PixelFormat::mono8: channels = 1; break;
      case PixelFormat::rgb8:
      case PixelFormat::bgr8: channels = 3; break;
      default: return Status::invalid_input;
    }
    if (!image.data || image.width != frame.image_width || image.height != frame.image_height ||
      image.width > std::numeric_limits<std::size_t>::max() / channels ||
      image.row_stride < static_cast<std::size_t>(image.width) * channels ||
      image.row_stride > std::numeric_limits<std::size_t>::max() / image.height)
    {
      return Status::invalid_input;
    }
    const auto required = image.row_stride * image.height;
    if (image.byte_size < required) {return Status::invalid_input;}
    if (image.byte_size > limits.max_image_bytes) {return Status::resource_limit;}
  }
  for (const auto & detection : frame.detections) {
    if (!valid_box(detection.box) || !std::isfinite(detection.confidence) ||
      detection.confidence < 0 || detection.confidence > 1 || detection.class_id < -1)
    {
      return Status::invalid_input;
    }
    if (detection.appearance.size() > limits.max_appearance_dimension) {
      return Status::resource_limit;
    }
    if (needs.appearance_dimension && detection.appearance.size() != needs.appearance_dimension) {
      return Status::unsupported_input;
    }
    for (const auto value : detection.appearance) {
      if (!std::isfinite(value)) {return Status::invalid_input;}
    }
  }
  return Status::ok;
}

// Optional adapter helper: only commit a timestamp after a successful update.
class FrameSequence
{
public:
  void reset(std::uint64_t epoch) {epoch_ = epoch; last_stamp_ = 0;}
  Status check(SourceTime source) const
  {
    return source.epoch == epoch_ && source.stamp_ns > last_stamp_ ?
           Status::ok : Status::invalid_input;
  }
  bool commit(SourceTime source)
  {
    if (check(source) != Status::ok) {return false;}
    last_stamp_ = source.stamp_ns;
    return true;
  }
private:
  std::uint64_t epoch_{0};
  std::int64_t last_stamp_{0};
};

class Tracker
{
public:
  virtual ~Tracker() = default;
  virtual Capabilities capabilities() const = 0;
  // One caller per instance. Reject unsupported/invalid input without partial tracks.
  // Echo source; keep raw IDs and native velocity unit. Never retain borrowed image data.
  virtual UpdateResult update(const Frame & frame) = 0;
  // Clear ALL algorithm history and bind the next update to this epoch.
  // ID reuse after reset is safe only when the caller also clears ID-fusion/world history.
  virtual void reset(std::uint64_t epoch) = 0;
};
}  // namespace easyfly::tracking

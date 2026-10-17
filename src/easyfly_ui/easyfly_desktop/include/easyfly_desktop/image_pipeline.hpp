// Copyright 2026 EasyFly Project
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

#ifndef EASYFLY_DESKTOP__IMAGE_PIPELINE_HPP_
#define EASYFLY_DESKTOP__IMAGE_PIPELINE_HPP_
#include <QImage>

#include <cstdint>
#include <memory>
#include <string>

#include "easyfly_desktop/source.hpp"
namespace easyfly_desktop
{
struct ImageBinding
{
  uint64_t session{0}, epoch{0};
  int camera{0};
  std::string topic;
  bool operator==(const ImageBinding & other) const;
};
struct ConvertedFrame
{
  ImageBinding binding;
  uint64_t sequence{0};
  int64_t stamp_ns{0};
  double first_received{0};
  QImage image;
  std::string error;
  std::string source_frame;
};
struct ImageQueueStats {std::size_t pending{0}; bool active{false}, stopped{false};
  uint64_t dropped{0};};
class ImagePipeline
{
public:
  using Converter = std::function<QImage(const sensor_msgs::msg::Image &)>;
  ImagePipeline();
  explicit ImagePipeline(Converter converter);
  ~ImagePipeline();
  ImagePipeline(const ImagePipeline &) = delete;
  ImagePipeline & operator=(const ImagePipeline &) = delete;
  void bind(const ImageBinding & binding);
  void clear();
  void submit(sensor_msgs::msg::Image::ConstSharedPtr image, double first_received);
  std::shared_ptr<const ConvertedFrame> latest() const;
  ImageQueueStats stats() const;
  void stop();  // Nonblocking; release after stats().stopped, or outside the GUI thread.
  static QImage convert(const sensor_msgs::msg::Image & image);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_desktop
#endif  // EASYFLY_DESKTOP__IMAGE_PIPELINE_HPP_

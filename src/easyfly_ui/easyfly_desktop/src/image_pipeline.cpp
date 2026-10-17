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

#include <condition_variable>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

#include "easyfly_desktop/image_pipeline.hpp"
namespace easyfly_desktop
{
bool ImageBinding::operator==(const ImageBinding & b) const
{
  return session == b.session && epoch == b.epoch && camera == b.camera && topic == b.topic;
}
struct ImagePipeline::Impl
{
  struct Job {ImageBinding binding; uint64_t generation, sequence;
    sensor_msgs::msg::Image::ConstSharedPtr image; double received;};
  mutable std::mutex mutex;
  std::condition_variable wake;
  ImageBinding binding;
  bool bound{false}, closing{false};
  uint64_t generation{0}, sequence{0};
  std::optional<Job> pending;
  std::shared_ptr<const ConvertedFrame> output;
  ImageQueueStats state;
  Converter convert;
  std::thread worker;
  explicit Impl(Converter callback)
  : convert(std::move(callback)), worker([this] {run();}) {}
  ~Impl() {stop(); if (worker.joinable()) {worker.join();}}
  void stop()
  {
    std::lock_guard<std::mutex> lock(mutex);
    closing = true; ++generation; pending.reset(); output.reset();
    wake.notify_all();
  }
  void run()
  {
    std::unique_lock<std::mutex> lock(mutex);
    while (!closing) {
      wake.wait(lock, [this] {return closing || pending.has_value();});
      if (closing) {break;}
      const auto job = std::move(*pending); pending.reset(); state.active = true;
      lock.unlock();
      auto result = std::make_shared<ConvertedFrame>();
      result->binding = job.binding; result->sequence = job.sequence;
      result->stamp_ns = bridge::detail::stamp_ns(job.image->header.stamp);
      result->first_received = job.received;
      result->source_frame = job.image->header.frame_id;
      try {result->image = convert(*job.image);} catch (const std::exception & e) {
        result->error = std::string(e.what()).substr(0, 2048);
      }
      lock.lock(); state.active = false;
      if (!closing && job.generation == generation && job.sequence == sequence) {
        output = result;
      } else {++state.dropped;}
    }
    state.stopped = true;
  }
};
ImagePipeline::ImagePipeline()
: ImagePipeline(ImagePipeline::convert) {}
ImagePipeline::ImagePipeline(Converter converter)
: impl_(std::make_unique<Impl>(std::move(converter))) {}
ImagePipeline::~ImagePipeline() = default;
void ImagePipeline::bind(const ImageBinding & binding)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->closing || (impl_->bound && impl_->binding == binding)) {return;}
  impl_->binding = binding; impl_->bound = true; ++impl_->generation;
  impl_->pending.reset(); impl_->output.reset();
}
void ImagePipeline::clear()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->bound) {return;}
  impl_->bound = false; ++impl_->generation; impl_->pending.reset(); impl_->output.reset();
}
void ImagePipeline::submit(sensor_msgs::msg::Image::ConstSharedPtr image, double received)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->bound || impl_->closing || !image) {return;}
  if (impl_->pending) {++impl_->state.dropped;}
  impl_->pending =
    Impl::Job{impl_->binding, impl_->generation, ++impl_->sequence, std::move(image), received};
  impl_->wake.notify_all();
}
std::shared_ptr<const ConvertedFrame> ImagePipeline::latest() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex); return impl_->output;
}
ImageQueueStats ImagePipeline::stats() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  auto result = impl_->state; result.pending = impl_->pending ? 1 : 0; return result;
}
void ImagePipeline::stop() {impl_->stop();}
QImage ImagePipeline::convert(const sensor_msgs::msg::Image & image)
{
  if (bridge::detail::validate_image(image) != bridge::Quality::ValidAtObservation) {
    throw std::invalid_argument("图像编码、尺寸、步长或字节布局无效/超预算");
  }
  const auto format = image.encoding == "mono8" ? QImage::Format_Grayscale8 : QImage::Format_RGB888;
  const QImage view(image.data.data(), static_cast<int>(image.width),
    static_cast<int>(image.height),
    static_cast<int>(image.step), format);
  QImage owned = image.encoding == "bgr8" ? view.rgbSwapped() : view.copy();
  if (owned.isNull()) {throw std::runtime_error("图像转换内存分配失败");}
  return owned;
}
}  // namespace easyfly_desktop

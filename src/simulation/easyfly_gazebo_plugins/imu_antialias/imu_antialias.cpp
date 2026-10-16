// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <memory>
#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <google/protobuf/descriptor.h>
#include "easyfly_imu/window.hpp"
#include "easyfly_imu/message_type.hpp"
#include "easyfly_imu/accel_fifo.hpp"

namespace easyfly_imu
{
// Original native noisy IMU remains the sole producer. No protobuf definitions
// are duplicated: the validated PX4 plugin has already registered this schema.
class Antialias : public gazebo::ModelPlugin
{
public:
  ~Antialias() override
  {
    subscriber_.reset();
    // Fini clears callbacks under Node's incoming mutex after active dispatch.
    if (node_) {node_->Fini();}
    publisher_.reset(); node_.reset();
  }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    auto physics = model->GetWorld()->Physics();
    if (std::abs(physics->GetMaxStepSize() - .001) > 1e-12 ||
      std::abs(physics->GetRealTimeUpdateRate() - 1000.) > 1e-6)
    {
      gzthrow("EasyFly IMU antialias requires reviewed 1000Hz physics / 250Hz MAVLink");
    }
    descriptor_ = google::protobuf::DescriptorPool::generated_pool()->FindMessageTypeByName(
      "sensor_msgs.msgs.Imu");
    if (!descriptor_ || descriptor_->field_count() != 8) {
      gzthrow("EasyFly IMU antialias requires original PX4 Imu schema loaded first");
    }
    auto require = [this](const std::string & name, int number,
        google::protobuf::FieldDescriptor::Type type) {
        auto field = descriptor_->FindFieldByName(name);
        if (!field || field->number() != number || field->type() != type || field->is_repeated()) {
          gzthrow("EasyFly IMU schema mismatch: " + name);
        }
        return field;
      };
    sequence_ = require("seq", 8, google::protobuf::FieldDescriptor::TYPE_INT64);
    time_ = require("time_usec", 7, google::protobuf::FieldDescriptor::TYPE_INT64);
    fields_[0] = require("linear_acceleration", 5, google::protobuf::FieldDescriptor::TYPE_MESSAGE);
    fields_[1] = require("angular_velocity", 3, google::protobuf::FieldDescriptor::TYPE_MESSAGE);
    for (auto field : fields_) {
      if (field->message_type()->full_name() != "gazebo.msgs.Vector3d") {
        gzthrow("EasyFly IMU vector schema mismatch");
      }
    }
    prototype_ = NativePrototype(descriptor_);
    if (!prototype_) {gzthrow("EasyFly IMU prototype unavailable");}
    node_.reset(new gazebo::transport::Node());
    node_->Init(sdf->HasElement("robotNamespace") ? sdf->Get<std::string>("robotNamespace") : "");
    const auto input = sdf->Get<std::string>("inputTopic");
    const auto output = sdf->Get<std::string>("outputTopic");
    if (input.empty() || output.empty() || input == output) {gzthrow("Invalid IMU relay topics");}
    const auto prefix = "~/" + model->GetName();
    publisher_ = node_->Advertise(prefix + output, descriptor_->full_name(), 10);
    subscriber_ = node_->Subscribe(prefix + input, &Antialias::Receive, this);
    gzmsg << "EasyFly IMU antialias: native 1000Hz, window=4, accel FIFO saturation, source seq/time preserved\n";
  }
private:
  void Receive(const std::string & serialized)
  {
    if (failed_) {return;}
    std::unique_ptr<google::protobuf::Message> message(prototype_->New());
    if (!message->ParseFromString(serialized) || !message->IsInitialized()) {
      Fail("invalid native protobuf"); return;
    }
    const auto reflection = message->GetReflection();
    Window::Values values;
    const char * axes[] = {"x", "y", "z"};
    for (int v = 0; v < 2; ++v) {
      const auto & vector = reflection->GetMessage(*message, fields_[v]);
      for (int i = 0; i < 3; ++i) {
        values[3*v+i] = vector.GetReflection()->GetDouble(
          vector, vector.GetDescriptor()->FindFieldByName(axes[i]));
      }
    }
    auto mean = window_.Push(reflection->GetInt64(*message, sequence_),
      reflection->GetInt64(*message, time_), values);
    if (!mean) {Fail("nonfinite, skipped, repeated or non-1000Hz native sample"); return;}
    for (double value : *mean) {
      if (!std::isfinite(value)) {Fail("nonfinite accumulated sample"); return;}
    }
    for (int v = 0; v < 2; ++v) {
      auto * vector = reflection->MutableMessage(message.get(), fields_[v]);
      for (int i = 0; i < 3; ++i) {
        vector->GetReflection()->SetDouble(vector,
          vector->GetDescriptor()->FindFieldByName(axes[i]), v == 0 ? AccelForFifo((*mean)[i]) : (*mean)[3*v+i]);
      }
    }
    // Copying the parsed message preserves actual source stamps, sequence,
    // orientation and covariance. Channels are averaged; acceleration is then
    // bounded to the reviewed downstream FIFO representation, never wrapped.
    publisher_->Publish(*message);
  }
  void Fail(const char * reason)
  {
    failed_ = true;
    gzerr << "EasyFly IMU antialias halted: " << reason << "; no synthetic samples\n";
  }
  gazebo::transport::NodePtr node_;
  gazebo::transport::PublisherPtr publisher_;
  gazebo::transport::SubscriberPtr subscriber_;
  const google::protobuf::Descriptor * descriptor_{nullptr};
  const google::protobuf::Message * prototype_{nullptr};
  const google::protobuf::FieldDescriptor * sequence_{nullptr}, * time_{nullptr};
  const google::protobuf::FieldDescriptor * fields_[2]{};
  Window window_;
  bool failed_{false};
};
GZ_REGISTER_MODEL_PLUGIN(Antialias)
}  // namespace easyfly_imu

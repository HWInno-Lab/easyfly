// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <google/protobuf/message.h>
namespace easyfly_imu
{
inline const google::protobuf::Message * NativePrototype(
  const google::protobuf::Descriptor * descriptor)
{
  // Gazebo local typed subscribers dynamic_cast MessagePtr. A DynamicMessage
  // with the same schema serializes correctly but is NOT the native C++ type.
  return descriptor ? google::protobuf::MessageFactory::generated_factory()->GetPrototype(descriptor) : nullptr;
}
}  // namespace easyfly_imu

// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#pragma once
#include "easyfly_backend_registry/configuration.hpp"
#include "easyfly_object_detection/detector_base.hpp"
namespace easyfly_object_detection {
using DetectorFactory = easyfly::backends::Factory<easyfly::detection::DetectorBase>;
const DetectorFactory & detector_factory();
inline std::unique_ptr<easyfly::detection::DetectorBase> createBackend(
  const easyfly::backends::BackendConfig & config, const DetectorFactory & factory = detector_factory()) {
  auto result = factory.create(config);
  if (result->params().backend != config.backend) throw std::runtime_error("detector creator identity mismatch");
  return result;
}
// Compatibility for existing typed callers. No implementation header is exposed.
easyfly::backends::BackendConfig detector_config(const easyfly::detection::DetectorParams & params);
inline std::unique_ptr<easyfly::detection::DetectorBase> createBackend(const easyfly::detection::DetectorParams & params) {
  return createBackend(detector_config(params));
}
}  // namespace easyfly_object_detection

// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "easyfly_backend_registry/configuration_yaml.hpp"
namespace b = easyfly::backends;
TEST(ConfigurationYaml, RejectDuplicateUnknownQuotedAndNonFiniteFields) {
  for (const auto & text : {"{min_hits: 1, min_hits: 2}", "{min_hits: true}", "{min_hits: 1.5}",
      "{min_hits: '3'}", "{vendor: 1}", "{iou_threshold: nan}", "{iou_threshold: inf}"}) {
    EXPECT_THROW(b::parameters_from_yaml(YAML::Load(text), "multi_tracking", "sort"), std::exception) << text;
  }
  auto values = b::parameters_from_yaml(YAML::Load("{min_hits: 4, iou_threshold: .2}"), "multi_tracking", "sort");
  EXPECT_EQ(b::parameter<int>(values, "min_hits"), 4);
  EXPECT_DOUBLE_EQ(b::parameter<double>(values, "iou_threshold"), .2);
}

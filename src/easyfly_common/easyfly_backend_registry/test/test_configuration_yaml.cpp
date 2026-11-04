// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_backend_registry/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
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

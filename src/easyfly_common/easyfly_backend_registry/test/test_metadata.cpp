// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_backend_registry/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
#include <gtest/gtest.h>
#include "easyfly_backend_registry/metadata.hpp"
TEST(Registry, RealAndUnknown) {
  EXPECT_NO_THROW(easyfly::backends::require_backend("multi_tracking", "sort"));
  EXPECT_THROW(easyfly::backends::require_backend("multi_tracking", "missing"), std::invalid_argument);
}

namespace b = easyfly::backends;
TEST(Registry, CppSchemaRejectsTypesUnknownsBoundsAndVersions) {
  EXPECT_THROW(b::resolve_parameters("multi_tracking", "sort", {{"min_hits", true}}), std::invalid_argument);
  EXPECT_THROW(b::resolve_parameters("multi_tracking", "sort", {{"min_hits", 1.5}}), std::invalid_argument);
  EXPECT_THROW(b::resolve_parameters("multi_tracking", "sort", {{"vendor", 1}}), std::invalid_argument);
  EXPECT_THROW(b::resolve_parameters("detection", "onnx", {{"max_detections", 101}}), std::invalid_argument);
  EXPECT_THROW(b::resolve_parameters("grid_planning", "astar", {}), std::invalid_argument);
  EXPECT_THROW(b::resolve_parameters("single_tracking", "csrt", {{"threshold", .3}}), std::invalid_argument);
  auto p = b::resolve_parameters("multi_tracking", "sort", {});
  EXPECT_DOUBLE_EQ(b::parameter<double>(p, "iou_threshold"), .15);
  EXPECT_EQ(b::parameter<int>(p, "max_lost_frames"), 45);
  EXPECT_THROW(b::parameter<int>(b::Parameters{{"x", std::int64_t{1LL << 40}}}, "x"), std::invalid_argument);
}
TEST(Registry, CreatorTablesAreLocalAndFailExplicitly) {
  b::Factory<int> f;
  f.add("fixture", [](const b::Parameters & p) {return std::make_unique<int>(b::parameter<int>(p, "value"));});
  EXPECT_EQ(*f.create({"fixture", {{"value", 7}}}), 7);
  EXPECT_THROW(f.create({"missing", {}}), std::invalid_argument);
  EXPECT_THROW(f.create({"fixture", {}, 2}), std::invalid_argument);
  EXPECT_THROW(f.add("fixture", [](const auto &) {return std::make_unique<int>();}), std::invalid_argument);
  f.add("null", [](const auto &) -> std::unique_ptr<int> {return nullptr;});
  EXPECT_THROW(f.create({"null", {}}), std::runtime_error);
  b::Factory<int> isolated;
  EXPECT_THROW(isolated.create({"fixture", {}}), std::invalid_argument);
}

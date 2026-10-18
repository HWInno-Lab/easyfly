// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <limits>
#include "easyfly_grid_planner_api/planner.hpp"

namespace gp = easyfly::grid_planning;

namespace
{
gp::GridView grid(const std::int8_t * cells)
{
  return {2, 2, .2, -1, -1, cells, 4, 50, gp::UnknownPolicy::free};
}
}  // namespace

TEST(GridContract, ChecksShapeStorageAndCellBudget)
{
  const std::int8_t cells[4]{};
  auto value = grid(cells);
  EXPECT_EQ(gp::validate_layout(value, 4), gp::Status::ok);
  EXPECT_EQ(gp::validate_layout(value, 3), gp::Status::resource_limit);
  value.cell_count = 3;
  EXPECT_EQ(gp::validate_layout(value, 4), gp::Status::invalid_input);
  value.cell_count = 4;
  value.cells = nullptr;
  EXPECT_EQ(gp::validate_layout(value, 4), gp::Status::invalid_input);
}

TEST(GridContract, DetectsDimensionOverflowWithoutReadingStorage)
{
  const std::int8_t sentinel = 0;
  auto value = grid(&sentinel);
  value.width = std::numeric_limits<std::size_t>::max();
  EXPECT_EQ(gp::validate_layout(value, value.width), gp::Status::invalid_input);
}

TEST(GridContract, RejectsInvalidGeometryAndUnrepresentableExtents)
{
  const std::int8_t cells[4]{};
  auto value = grid(cells);
  value.resolution = 0;
  EXPECT_EQ(gp::validate_layout(value, 4), gp::Status::invalid_input);
  value.resolution = std::numeric_limits<double>::infinity();
  EXPECT_EQ(gp::validate_layout(value, 4), gp::Status::invalid_input);
  value.resolution = .2;
  value.origin_x = std::numeric_limits<double>::max();
  EXPECT_EQ(gp::validate_layout(value, 4), gp::Status::invalid_input);
  EXPECT_FALSE(gp::valid_point({std::numeric_limits<double>::quiet_NaN(), 0}));
}

TEST(GridContract, UnknownPolicyAndThresholdAreExplicit)
{
  EXPECT_FALSE(gp::occupied_value(-1, 50, gp::UnknownPolicy::free));
  EXPECT_TRUE(gp::occupied_value(-1, 50, gp::UnknownPolicy::occupied));
  EXPECT_FALSE(gp::occupied_value(49, 50, gp::UnknownPolicy::free));
  EXPECT_TRUE(gp::occupied_value(50, 50, gp::UnknownPolicy::free));
  EXPECT_TRUE(gp::occupied_value(-2, 50, gp::UnknownPolicy::free));
  EXPECT_TRUE(gp::occupied_value(101, 50, gp::UnknownPolicy::free));
  EXPECT_TRUE(gp::occupied_value(0, 0, gp::UnknownPolicy::free));
}

TEST(GridContract, RejectsInvalidPolicyAndThresholdBeforeSearch)
{
  const std::int8_t cells[4]{};
  auto value = grid(cells);
  value.unknown_policy = static_cast<gp::UnknownPolicy>(99);
  EXPECT_EQ(gp::validate_layout(value, 4), gp::Status::invalid_input);
  value.unknown_policy = gp::UnknownPolicy::free;
  value.occupied_threshold = 0;
  EXPECT_EQ(gp::validate_layout(value, 4), gp::Status::invalid_input);
}

TEST(GridContract, RejectsRequestedButUnsupportedControl)
{
  gp::SearchControl control;
  gp::Capabilities basic{"basic", false, false, false};
  EXPECT_EQ(gp::validate_control(control, basic), gp::Status::ok);
  control.expansion_limit = 0;
  EXPECT_EQ(gp::validate_control(control, basic), gp::Status::unsupported_request);
  control.expansion_limit.reset();
  control.deadline = gp::SearchControl::Clock::time_point{};
  EXPECT_EQ(gp::validate_control(control, basic), gp::Status::unsupported_request);
  control.deadline.reset();
  const std::atomic_bool canceled{false};
  control.cancel_requested = &canceled;
  EXPECT_EQ(gp::validate_control(control, basic), gp::Status::unsupported_request);
}

TEST(GridContract, CancellationDeadlineAndBudgetHaveDeterministicPrecedence)
{
  const auto now = gp::SearchControl::Clock::time_point{} + std::chrono::seconds(10);
  std::atomic_bool cancel{true};
  gp::SearchControl control;
  control.cancel_requested = &cancel;
  control.deadline = now;
  control.expansion_limit = 0;
  EXPECT_EQ(gp::check_control(control, 0, now), gp::Status::canceled);
  cancel.store(false);
  EXPECT_EQ(gp::check_control(control, 0, now), gp::Status::timed_out);
  control.deadline = now + std::chrono::seconds(1);
  EXPECT_EQ(gp::check_control(control, 0, now), gp::Status::budget_exhausted);
  control.expansion_limit = 2;
  EXPECT_EQ(gp::check_control(control, 1, now), gp::Status::ok);
  EXPECT_EQ(gp::check_control(control, 2, now), gp::Status::budget_exhausted);
}

TEST(GridContract, AbsentControlIsExplicitlyUnlimitedAndResultsDefaultToFailure)
{
  EXPECT_EQ(gp::check_control({}, std::numeric_limits<std::size_t>::max(), {}), gp::Status::ok);
  EXPECT_EQ(gp::PathResult{}.status, gp::Status::backend_error);
}

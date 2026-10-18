// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace easyfly::grid_planning
{
enum class Status
{
  ok, invalid_input, unsupported_request, resource_limit, out_of_bounds,
  start_in_obstacle, goal_in_obstacle, no_path, canceled, timed_out,
  budget_exhausted, backend_error
};
enum class UnknownPolicy {free, occupied};

struct Point
{
  double x{0}, y{0};  // Metres, in the same caller-resolved frame as the grid.
};

struct MapIdentity
{
  std::uint64_t epoch{0}, revision{0};
  std::int64_t source_stamp_ns{0};  // Source time; no freshness decisions in the API.
};

// Axis-aligned, row-major int8 occupancy. Borrowed until plan() returns.
// Legal values are -1 (unknown) and 0..100. Geometry/byte ownership stays with caller.
struct GridView
{
  std::size_t width{0}, height{0};
  double resolution{0}, origin_x{0}, origin_y{0};
  const std::int8_t * cells{nullptr};
  std::size_t cell_count{0};
  std::int8_t occupied_threshold{50};
  UnknownPolicy unknown_policy{UnknownPolicy::free};  // Current A* interpretation.
};

struct Request
{
  GridView grid;
  MapIdentity map;
  Point start, goal;
};

struct SearchControl
{
  using Clock = std::chrono::steady_clock;
  // nullopt = not requested; an engaged zero limit allows no expansions.
  std::optional<std::size_t> expansion_limit;
  std::optional<Clock::time_point> deadline;
  // Borrowed atomic, must outlive plan(); true means cancellation requested.
  const std::atomic_bool * cancel_requested{nullptr};
};

struct Capabilities
{
  std::string backend;
  bool cancellation{false}, deadline{false}, expansion_limit{false};
};

struct PathResult
{
  Status status{Status::backend_error};
  MapIdentity map;
  std::vector<Point> waypoints;  // Owning XY path, not an executable 3D trajectory.
  std::size_t expansions{0};
  std::string detail;
};

// O(1) layout checks; does not dereference arbitrary cell storage or scan the grid.
// max_cells is supplied by the consumer, not an implicit global allocation budget.
inline Status validate_layout(const GridView & grid, std::size_t max_cells)
{
  if (grid.width == 0 || grid.height == 0 || !grid.cells ||
    grid.width > std::numeric_limits<std::size_t>::max() / grid.height ||
    !std::isfinite(grid.resolution) || grid.resolution <= 0 ||
    !std::isfinite(grid.origin_x) || !std::isfinite(grid.origin_y) ||
    grid.occupied_threshold < 1 || grid.occupied_threshold > 100 ||
    (grid.unknown_policy != UnknownPolicy::free && grid.unknown_policy != UnknownPolicy::occupied))
  {
    return Status::invalid_input;
  }
  const auto count = grid.width * grid.height;
  if (grid.cell_count != count) {return Status::invalid_input;}
  if (count > max_cells) {return Status::resource_limit;}
  const double end_x = grid.origin_x + grid.resolution * grid.width;
  const double end_y = grid.origin_y + grid.resolution * grid.height;
  if (!std::isfinite(end_x) || !std::isfinite(end_y) ||
    end_x <= grid.origin_x || end_y <= grid.origin_y)
  {
    return Status::invalid_input;
  }
  return Status::ok;
}

// Invalid stored cell values fail closed. Bounds checks must precede array access.
inline bool occupied_value(std::int8_t cell, std::int8_t threshold, UnknownPolicy unknown)
{
  if (cell < -1 || cell > 100 || threshold < 1 || threshold > 100 ||
    (unknown != UnknownPolicy::free && unknown != UnknownPolicy::occupied))
  {
    return true;
  }
  return cell == -1 ? unknown == UnknownPolicy::occupied : cell >= threshold;
}

inline bool valid_point(Point point) {return std::isfinite(point.x) && std::isfinite(point.y);}

inline Status validate_control(const SearchControl & control, const Capabilities & capabilities)
{
  return ((control.cancel_requested && !capabilities.cancellation) ||
         (control.deadline && !capabilities.deadline) ||
         (control.expansion_limit && !capabilities.expansion_limit)) ?
         Status::unsupported_request : Status::ok;
}

// Explicit now makes tests deterministic; caller polls at bounded checkpoints in
// validation/search/pruning. No thread interruption or hard real-time guarantee.
// Precedence: cancellation, deadline, expansion budget.
inline Status check_control(
  const SearchControl & control, std::size_t expansions, SearchControl::Clock::time_point now)
{
  if (control.cancel_requested && control.cancel_requested->load(std::memory_order_relaxed)) {
    return Status::canceled;
  }
  if (control.deadline && now >= *control.deadline) {return Status::timed_out;}
  if (control.expansion_limit && expansions >= *control.expansion_limit) {
    return Status::budget_exhausted;
  }
  return Status::ok;
}

class GridPlanner
{
public:
  virtual ~GridPlanner() = default;
  virtual Capabilities capabilities() const = 0;
  // Serial, synchronous use. Echo map identity; reject unsupported controls.
  // Failure has no executable partial path. Success includes exact requested
  // start/goal, even for a same-cell request. Validate layout before cell access.
  virtual PathResult plan(const Request & request, const SearchControl & control) = 0;
};
}  // namespace easyfly::grid_planning

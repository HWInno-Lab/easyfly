// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
// Test-only backends: no data association, obstacle search or flight execution.
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include "easyfly_grid_planner_api/planner.hpp"
#include "easyfly_tracking_api/tracker.hpp"

namespace tr = easyfly::tracking;
namespace gp = easyfly::grid_planning;

void require(bool condition, const char * reason)
{
  if (!condition) {throw std::runtime_error(reason);}
}

class EchoTracker final : public tr::Tracker
{
public:
  tr::Capabilities capabilities() const override
  {
    return {"test_echo", {}, tr::VelocityUnit::pixels_per_frame};
  }
  void reset(std::uint64_t epoch) override {sequence_.reset(epoch);}
  tr::UpdateResult update(const tr::Frame & frame) override
  {
    tr::UpdateResult out;
    out.source = frame.source;
    out.status = tr::validate_frame(frame);
    if (out.status == tr::Status::ok) {out.status = sequence_.check(frame.source);}
    if (out.status != tr::Status::ok) {return out;}
    if (!frame.detections.empty()) {
      tr::Track track;
      track.raw_id = 7;
      track.box = track.last_observed_box = frame.detections.front().box;
      out.tracks.push_back(track);
    }
    sequence_.commit(frame.source);
    return out;
  }
private:
  tr::FrameSequence sequence_;
};

class EmptyTracker final : public tr::Tracker
{
public:
  tr::Capabilities capabilities() const override
  {
    return {"test_empty", {}, tr::VelocityUnit::pixels_per_second};
  }
  void reset(std::uint64_t epoch) override {sequence_.reset(epoch);}
  tr::UpdateResult update(const tr::Frame & frame) override
  {
    tr::UpdateResult out;
    out.source = frame.source;
    out.velocity_unit = capabilities().velocity_unit;
    out.status = tr::validate_frame(frame);
    if (out.status == tr::Status::ok) {out.status = sequence_.check(frame.source);}
    if (out.status == tr::Status::ok) {sequence_.commit(frame.source);}
    return out;
  }
private:
  tr::FrameSequence sequence_;
};

class DirectPlanner final : public gp::GridPlanner
{
public:
  gp::Capabilities capabilities() const override {return {"test_direct", true, true, true};}
  gp::PathResult plan(const gp::Request & request, const gp::SearchControl & control) override
  {
    gp::PathResult out;
    out.map = request.map;
    out.status = gp::validate_control(control, capabilities());
    if (out.status != gp::Status::ok) {return out;}
    out.status = gp::validate_layout(request.grid, 4);
    if (out.status != gp::Status::ok) {return out;}
    if (!gp::valid_point(request.start) || !gp::valid_point(request.goal)) {
      out.status = gp::Status::invalid_input;
      return out;
    }
    out.status = gp::check_control(control, 0, gp::SearchControl::Clock::now());
    if (out.status == gp::Status::ok) {out.waypoints = {request.start, request.goal};}
    return out;
  }
};

class NoPathPlanner final : public gp::GridPlanner
{
public:
  gp::Capabilities capabilities() const override {return {"test_no_path", false, false, false};}
  gp::PathResult plan(const gp::Request & request, const gp::SearchControl & control) override
  {
    gp::PathResult out;
    out.map = request.map;
    out.status = gp::validate_control(control, capabilities());
    if (out.status != gp::Status::ok) {return out;}
    out.status = gp::validate_layout(request.grid, 4);
    if (out.status == gp::Status::ok) {out.status = gp::Status::no_path;}
    return out;
  }
};

void tracking()
{
  tr::Frame frame;
  frame.source = {9, 100};
  frame.image_width = frame.image_height = 10;
  frame.detections.push_back({{1, 1, 3, 3}, .8, 0, "person", {}});
  std::unique_ptr<tr::Tracker> backend = std::make_unique<EchoTracker>();
  backend->reset(9);
  const auto echo = backend->update(frame);
  require(echo.status == tr::Status::ok && echo.tracks.size() == 1, "echo result");
  require(echo.source.epoch == 9 && echo.source.stamp_ns == 100, "source identity");
  require(echo.velocity_unit == tr::VelocityUnit::pixels_per_frame, "frame velocity unit");
  require(backend->update(frame).status == tr::Status::invalid_input, "reject replay");
  backend->reset(10);
  frame.source = {10, 1};
  require(backend->update(frame).status == tr::Status::ok, "new epoch accepts small stamp");
  backend = std::make_unique<EmptyTracker>();
  backend->reset(10);
  const auto empty = backend->update(frame);
  require(empty.status == tr::Status::ok && empty.tracks.empty(), "valid empty result");
  require(empty.velocity_unit == tr::VelocityUnit::pixels_per_second, "second velocity unit");
}

gp::Request request(const std::int8_t * cells)
{
  return {{2, 2, 1, 0, 0, cells, 4, 50, gp::UnknownPolicy::free},
    {3, 17, 100}, {.1, .1}, {.7, .7}};
}

void planning()
{
  const std::int8_t cells[4]{};
  const auto input = request(cells);
  std::unique_ptr<gp::GridPlanner> backend = std::make_unique<DirectPlanner>();
  const auto path = backend->plan(input, {});
  require(path.status == gp::Status::ok && path.waypoints.size() == 2, "two endpoints");
  require(path.waypoints.front().x == input.start.x &&
    path.waypoints.back().x == input.goal.x, "exact same-cell endpoints");
  require(path.map.epoch == 3 && path.map.revision == 17 &&
    path.map.source_stamp_ns == 100, "map identity");
  backend = std::make_unique<NoPathPlanner>();
  const auto failure = backend->plan(input, {});
  require(failure.status == gp::Status::no_path && failure.waypoints.empty(), "no path is failure");
}

void controls()
{
  const std::int8_t cells[4]{};
  const auto input = request(cells);
  DirectPlanner backend;
  std::atomic_bool canceled{true};
  gp::SearchControl control;
  control.cancel_requested = &canceled;
  auto out = backend.plan(input, control);
  require(out.status == gp::Status::canceled && out.waypoints.empty(), "canceled path");
  canceled.store(false);
  control.deadline = gp::SearchControl::Clock::time_point{};
  require(backend.plan(input, control).status == gp::Status::timed_out, "deadline");
  control.deadline.reset();
  control.expansion_limit = 0;
  require(backend.plan(input, control).status == gp::Status::budget_exhausted, "zero budget");
  NoPathPlanner unsupported;
  require(unsupported.plan(input, control).status == gp::Status::unsupported_request,
    "unsupported control must not be ignored");
}

int main(int argc, char ** argv)
{
  try {
    require(argc == 2, "expected tracking, planning or controls");
    const std::string test = argv[1];
    if (test == "tracking") {tracking();}
    else if (test == "planning") {planning();}
    else if (test == "controls") {controls();}
    else {throw std::runtime_error("unknown test");}
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

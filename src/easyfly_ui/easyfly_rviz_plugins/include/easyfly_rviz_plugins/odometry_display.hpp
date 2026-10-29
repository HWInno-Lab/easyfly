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
#ifndef EASYFLY_RVIZ_PLUGINS__ODOMETRY_DISPLAY_HPP_
#define EASYFLY_RVIZ_PLUGINS__ODOMETRY_DISPLAY_HPP_
#include <memory>
#include <vector>
#include <rviz_default_plugins/displays/odometry/odometry_display.hpp>
namespace easyfly_rviz_plugins
{
// RViz 11.2.26 allocates a parent node per visual, outside the visual's ownership.
// Preserve its subscriptions, properties and rendering; retire only empty parents.
class OdometryDisplay : public rviz_default_plugins::displays::OdometryDisplay
{
public:
  using rviz_default_plugins::displays::OdometryDisplay::OdometryDisplay;
  ~OdometryDisplay() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

private:
  struct NodeLifetime;
  using ObservedNodes = std::vector<std::unique_ptr<NodeLifetime>>;
  ObservedNodes owned_nodes() const;
  void remove_retired_parents(const ObservedNodes & owned);
};
}  // namespace easyfly_rviz_plugins
#endif  // EASYFLY_RVIZ_PLUGINS__ODOMETRY_DISPLAY_HPP_

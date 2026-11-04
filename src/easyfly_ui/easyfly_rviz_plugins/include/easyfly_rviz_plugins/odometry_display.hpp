// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_rviz_plugins/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
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

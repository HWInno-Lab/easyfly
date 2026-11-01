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
#ifndef EASYFLY_DESKTOP__GEOGRAPHIC_VIEW_HPP_
#define EASYFLY_DESKTOP__GEOGRAPHIC_VIEW_HPP_
#include <QWidget>
#include <memory>
#include <optional>
#include <functional>
#include "easyfly_desktop/geography.hpp"
#include "easyfly_desktop/source.hpp"
namespace easyfly_desktop
{
struct OverviewTaskOverlay;
class GeographicView : public QWidget
{
public:
  explicit GeographicView(QWidget * parent = nullptr, QString cache_directory = {});
  ~GeographicView() override;
  void snapshot(std::shared_ptr<const bridge::UiSnapshot> value, double now);
  void set_datum(const geo::Datum & datum);
  void import_map(const QString & path);
  bool gps_available() const;
  void set_compact(bool compact);
  void set_task_overlay(const OverviewTaskOverlay & overlay);
  std::optional<geo::Datum> datum() const;
  void clear_datum();
  // world (default), offline (imported MBTiles), osm (explicit online opt-in).
  QString map_source() const;
  void set_map_source(const QString & source);
  QString offline_map_path() const;
  // Called only on explicit edits or connection-scoped datum invalidation.
  std::function<void()> configuration_changed;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_desktop
#endif  // EASYFLY_DESKTOP__GEOGRAPHIC_VIEW_HPP_

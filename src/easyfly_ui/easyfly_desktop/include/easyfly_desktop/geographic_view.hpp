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
#include "easyfly_desktop/geography.hpp"
#include "easyfly_desktop/source.hpp"
namespace easyfly_desktop
{
class GeographicView : public QWidget
{
public:
  explicit GeographicView(QWidget * parent = nullptr, QString cache_directory = {});
  ~GeographicView() override;
  void snapshot(std::shared_ptr<const bridge::UiSnapshot> value, double now);
  void set_datum(const geo::Datum & datum);
  void import_map(const QString & path);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_desktop
#endif  // EASYFLY_DESKTOP__GEOGRAPHIC_VIEW_HPP_

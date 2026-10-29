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
#ifndef RVIZ_LAYERS_HPP_
#define RVIZ_LAYERS_HPP_
#ifdef EASYFLY_ENABLE_RVIZ_EMBED
#include "easyfly_desktop/rviz_view.hpp"
#include <rviz_common/display.hpp>
namespace easyfly_desktop
{
enum class SpatialKind {Actual, Planned, Cloud, Map, Obstacles, Landing, Predictions, Classified};
class SpatialDiagnostics
{
public:
  virtual ~SpatialDiagnostics() = default;
  virtual RvizLayerSnapshot layer_snapshot() const = 0;
};
rviz_common::Display * aircraft_display(const QString & topic);
rviz_common::Display * spatial_display(SpatialKind kind, const QString & topic);
}  // namespace easyfly_desktop
#endif
#endif  // RVIZ_LAYERS_HPP_

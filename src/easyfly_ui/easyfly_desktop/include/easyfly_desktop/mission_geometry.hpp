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
#ifndef EASYFLY_DESKTOP__MISSION_GEOMETRY_HPP_
#define EASYFLY_DESKTOP__MISSION_GEOMETRY_HPP_
#include <QPointF>
#include <QRectF>
#include <QWidget>
#include <cstddef>
#include <functional>
#include <string>
#include <vector>
#include <easyfly_mission_format/mission_format.hpp>
namespace easyfly_desktop
{
struct DraftPoint {size_t step; double x, y, z; std::string type;};
struct DraftArea
{
  size_t step; double x, y, width, height, z;
  std::string type{"search"}; double max_z{0}; bool forbidden{false};
};
struct DraftLink {size_t from, to;};  // Indices in points, never a planner route.
struct DraftGeometry
{
  bool accepted{false};
  std::string error;
  std::vector<DraftPoint> points;
  std::vector<DraftArea> areas;
  std::vector<DraftLink> links;
  std::vector<size_t> unlocated;
};
// Takes a shared-validator plan. Defensive rendering bounds are not flight admission.
DraftGeometry draft_geometry(const easyfly_mission_fsm::MissionValidation & plan);
class MissionGeometryCanvas : public QWidget
{
  Q_OBJECT

public:
  explicit MissionGeometryCanvas(QWidget * parent = nullptr);
  void plan(const easyfly_mission_fsm::MissionValidation & plan);
  void invalidate(const QString & reason);
  void select_step(int step);
  void selection_handler(std::function<void(size_t)> handler);
  const DraftGeometry & geometry() const;
  void fit();
  void zoom(double factor);
  double zoom_level() const;
  QPointF world_to_screen(double x, double y) const;
  QPointF screen_to_world(const QPointF & point) const;
  QSize sizeHint() const override;

protected:
  void paintEvent(QPaintEvent *) override;
  void wheelEvent(QWheelEvent *) override;
  void mousePressEvent(QMouseEvent *) override;
  void mouseMoveEvent(QMouseEvent *) override;
  void mouseReleaseEvent(QMouseEvent *) override;

private:
  QRectF plot() const;
  double scale() const;
  DraftGeometry data_;
  QString reason_{QStringLiteral("等待当前 v1 草稿校验")};
  int selected_{-1};
  double center_x_{0}, center_y_{0}, span_x_{2}, span_y_{2}, zoom_{1};
  bool panning_{false};
  QPointF last_mouse_;
  std::function<void(size_t)> selected_handler_;
};
}  // namespace easyfly_desktop
#endif  // EASYFLY_DESKTOP__MISSION_GEOMETRY_HPP_

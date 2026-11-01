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
#ifndef WIDGETS_HPP_
#define WIDGETS_HPP_
#include <QAbstractTableModel>
#include <QImage>
#include <QIcon>
#include <QPixmap>
#include <QToolButton>
#include <QStringList>
#include <QVector>
#include <QWidget>
#include <QTimer>
#include <memory>
#include "obstacle_display.hpp"
#include "easyfly_desktop/source.hpp"
#include "easyfly_desktop/presentation.hpp"
#include "easyfly_desktop/camera_overlay.hpp"
#include "easyfly_desktop/overview_task.hpp"
namespace easyfly_desktop
{
QPixmap brand_pixmap();
QIcon dashboard_icon(const QString & name);
class FlightInstrumentCanvas final : public QWidget
{
public:
  explicit FlightInstrumentCanvas(QWidget * parent = nullptr);
  void snapshot(const bridge::UiSnapshot & snapshot, double now);
  void set_compact(bool compact);

protected:
  void paintEvent(QPaintEvent *) override;

private:
  FlightInstruments instruments_;
  bool compact_{false};
};
class FlightInstrumentOverlay final : public QWidget
{
public:
  explicit FlightInstrumentOverlay(QWidget * parent = nullptr);
  FlightInstrumentCanvas * canvas() const;
  void attach(QWidget * canvas);

protected:
  bool eventFilter(QObject * watched, QEvent * event) override;

private:
  void position_overlay();
  FlightInstrumentCanvas * canvas_{nullptr};
  QToolButton * toggle_{nullptr};
  bool requested_expanded_{true}, user_toggled_{false};
};
class TextTable final : public QAbstractTableModel
{
public:
  TextTable(QStringList headers, QObject * parent);
  int rowCount(const QModelIndex & parent = QModelIndex()) const override;
  int columnCount(const QModelIndex & parent = QModelIndex()) const override;
  QVariant data(const QModelIndex & index, int role) const override;
  QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
  void replace(QVector<QStringList> rows);

private:
  QStringList headers_;
  QVector<QStringList> rows_;
};
class CameraCanvas final : public QWidget
{
public:
  explicit CameraCanvas(QWidget * parent = nullptr);
  void frame(QImage image, const QString & caption);
  void set_frozen(bool frozen);
  void overlay(CameraOverlay overlay);

protected:
  void paintEvent(QPaintEvent *) override;

private:
  QImage image_;
  QString caption_;
  bool frozen_{false};
  CameraOverlay overlay_;
};
class MapCanvas final : public QWidget
{
public:
  explicit MapCanvas(QWidget * parent = nullptr);
  void snapshot(std::shared_ptr<const bridge::UiSnapshot> snapshot, double now);
  void set_task_overlay(const OverviewTaskOverlay & task);
  void set_grid_enabled(bool enabled);
  void set_obstacle_overlay(bool enabled);
  void zoom(double factor);
  void fit();
  void center_on_vehicle();
  void set_follow(bool follow);
  bool following() const;

protected:
  void paintEvent(QPaintEvent *) override;
  void wheelEvent(QWheelEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;

private:
  void update_local_layers();
  OverviewTaskOverlay task_;
  bool grid_enabled_{false}, obstacle_overlay_{true};
  std::shared_ptr<const nav_msgs::msg::OccupancyGrid> grid_source_;
  QImage grid_image_;
  QRect grid_occupied_bounds_;
  QString grid_status_;
  ObstacleDisplay obstacles_;
  QTimer * animation_timer_{nullptr};
  double snapshot_received_steady_{0};
  bool follow_{true}, user_view_{false}, panning_{false};
  double center_x_{0}, center_y_{0}, zoom_{1}, last_scale_{1}, last_radius_{5};
  double fitted_radius_{5};
  uint64_t fit_session_{0}, fit_epoch_{0};
  QPoint last_mouse_;
  std::shared_ptr<const bridge::UiSnapshot> snapshot_;
  double now_{0};
};
}  // namespace easyfly_desktop
#endif  // WIDGETS_HPP_

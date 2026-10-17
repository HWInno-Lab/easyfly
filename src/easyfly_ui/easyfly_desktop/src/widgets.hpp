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
#include <QStringList>
#include <QVector>
#include <QWidget>

#include <memory>

#include "easyfly_desktop/source.hpp"
#include "easyfly_desktop/presentation.hpp"
#include "easyfly_desktop/camera_overlay.hpp"
namespace easyfly_desktop
{
QPixmap brand_pixmap();
QIcon dashboard_icon(const QString & name);
class FlightInstrumentCanvas final : public QWidget
{
public:
  explicit FlightInstrumentCanvas(QWidget * parent = nullptr);
  void snapshot(const bridge::UiSnapshot & snapshot, double now);

protected:
  void paintEvent(QPaintEvent *) override;

private:
  FlightInstruments instruments_;
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
  void overlay(CameraOverlay overlay);

protected:
  void paintEvent(QPaintEvent *) override;

private:
  QImage image_;
  QString caption_;
  CameraOverlay overlay_;
};
class MapCanvas final : public QWidget
{
public:
  explicit MapCanvas(QWidget * parent = nullptr);
  void snapshot(std::shared_ptr<const bridge::UiSnapshot> snapshot, double now);

protected:
  void paintEvent(QPaintEvent *) override;

private:
  std::shared_ptr<const bridge::UiSnapshot> snapshot_;
  double now_{0};
};
}  // namespace easyfly_desktop
#endif  // WIDGETS_HPP_

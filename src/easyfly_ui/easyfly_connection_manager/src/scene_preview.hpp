// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#pragma once

#include <QGraphicsView>
#include <QJsonArray>
#include <QJsonObject>

class QGraphicsPixmapItem;
class QGraphicsScene;

namespace easyfly_connection_manager
{
class ScenePreview : public QGraphicsView
{
  Q_OBJECT
public:
  explicit ScenePreview(QWidget * parent = nullptr);

  bool setPreview(const QJsonArray & shapes, const QJsonObject & preview);
  void zoomIn();
  void zoomOut();
  void fitToWindow();
  double zoomFactor() const;
  bool hasPreview() const;

signals:
  void viewChanged();

protected:
  void resizeEvent(QResizeEvent * event) override;
  void wheelEvent(QWheelEvent * event) override;
  void keyPressEvent(QKeyEvent * event) override;
  void mouseDoubleClickEvent(QMouseEvent * event) override;
  void drawForeground(QPainter * painter, const QRectF & rect) override;

private:
  double fitScale() const;
  void zoomBy(double multiplier, const QPoint & anchor);
  void applyFit();

  QGraphicsScene * scene_;
  QGraphicsPixmapItem * image_;
  QByteArray identity_;
  bool native_image_{false};
  bool automatic_fit_{true};
};
}  // namespace easyfly_connection_manager

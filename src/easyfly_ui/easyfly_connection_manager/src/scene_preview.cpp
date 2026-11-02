// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include "scene_preview.hpp"

#include <QBuffer>
#include <QCryptographicHash>
#include <QFile>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QImageReader>
#include <QJsonDocument>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QResizeEvent>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

namespace easyfly_connection_manager
{
namespace
{
QPixmap verifiedImage(const QJsonObject & preview)
{
  constexpr qint64 maximum = 8 * 1024 * 1024;
  QFile file(preview["path"].toString());
  if (file.size() <= 32 || file.size() > maximum || !file.open(QIODevice::ReadOnly)) {return {};}
  auto bytes = file.read(maximum + 1);
  if (bytes.size() > maximum ||
    QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex() !=
    preview["sha256"].toString().toLatin1()) {return {};}
  QBuffer buffer(&bytes); buffer.open(QIODevice::ReadOnly);
  QImageReader reader(&buffer, "PNG"); const auto size = reader.size();
  if (!size.isValid() || size.width() > 4096 || size.height() > 4096) {return {};}
  return QPixmap::fromImage(reader.read());
}

QPixmap geometryImage(const QJsonArray & shapes)
{
  if (shapes.isEmpty()) {return {};}
  constexpr int width = 1200, height = 800;
  double extent_x = 1., extent_y = 1.;
  const auto valid = [](const QJsonObject & shape) {
      for (const auto * key : {"x", "y", "width", "height", "yaw"}) {
        if (!shape[key].isDouble() || !std::isfinite(shape[key].toDouble())) {return false;}
      }
      return shape["width"].toDouble() > 0 && shape["height"].toDouble() > 0;
    };
  for (int i = 0; i < std::min(shapes.size(), 128); ++i) {
    const auto shape = shapes[i].toObject(); if (!valid(shape)) {continue;}
    const auto yaw = shape["yaw"].toDouble();
    const auto w = shape["width"].toDouble() / 2, h = shape["height"].toDouble() / 2;
    extent_x = std::max(extent_x, std::abs(shape["x"].toDouble()) +
      std::abs(std::cos(yaw)) * w + std::abs(std::sin(yaw)) * h);
    extent_y = std::max(extent_y, std::abs(shape["y"].toDouble()) +
      std::abs(std::sin(yaw)) * w + std::abs(std::cos(yaw)) * h);
  }
  QPixmap image(width, height); image.fill(QColor("#edf3fc")); QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing);
  const auto scale = std::min((width - 40.) / (2 * extent_x), (height - 40.) / (2 * extent_y));
  painter.translate(width / 2., height / 2.); painter.scale(scale, -scale);
  for (int i = 0; i < std::min(shapes.size(), 128); ++i) {
    const auto shape = shapes[i].toObject(); if (!valid(shape)) {continue;}
    painter.save(); painter.translate(shape["x"].toDouble(), shape["y"].toDouble());
    painter.rotate(shape["yaw"].toDouble() * 180 / 3.141592653589793);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(shape["ground"].toBool() ? "#dbe4f1" : "#527bb5"));
    painter.drawRect(QRectF(-shape["width"].toDouble() / 2,
      -shape["height"].toDouble() / 2, shape["width"].toDouble(), shape["height"].toDouble()));
    painter.restore();
  }
  return image;
}
}  // namespace

ScenePreview::ScenePreview(QWidget * parent)
: QGraphicsView(parent), scene_(new QGraphicsScene(this)), image_(new QGraphicsPixmapItem)
{
  setObjectName("scenePreview"); setAccessibleName(QStringLiteral("所选场景的俯视预览"));
  setMinimumSize(240, 220); setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Expanding);
  setScene(scene_); scene_->addItem(image_);
  image_->setTransformationMode(Qt::SmoothTransformation);
  setBackgroundBrush(QColor("#edf3fc")); setFrameShape(QFrame::NoFrame);
  setRenderHint(QPainter::SmoothPixmapTransform); setDragMode(QGraphicsView::ScrollHandDrag);
  setTransformationAnchor(QGraphicsView::NoAnchor); setResizeAnchor(QGraphicsView::NoAnchor);
  setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setFocusPolicy(Qt::StrongFocus);
  setToolTip(QStringLiteral("滚轮缩放；按住左键拖动；双击或按 0 适应窗口；+ / − 缩放"));
}

bool ScenePreview::setPreview(const QJsonArray & shapes, const QJsonObject & preview)
{
  const auto identity = QJsonDocument(shapes).toJson(QJsonDocument::Compact) +
    QJsonDocument(preview).toJson(QJsonDocument::Compact);
  if (identity == identity_) {return native_image_;}
  identity_ = identity;
  auto pixmap = verifiedImage(preview); native_image_ = !pixmap.isNull();
  if (pixmap.isNull()) {pixmap = geometryImage(shapes);}
  image_->setPixmap(pixmap);
  const auto rect = image_->boundingRect();
  // A modest virtual margin permits cursor-anchored zoom and dragging even
  // when one dimension of the fitted image is narrower than the viewport.
  const auto margin = std::max(rect.width(), rect.height()) * 4.;
  setSceneRect(rect.isEmpty() ? QRectF(-1., -1., 2., 2.) :
    rect.adjusted(-margin, -margin, margin, margin));
  fitToWindow(); viewport()->update(); return native_image_;
}

double ScenePreview::fitScale() const
{
  const auto rect = image_->boundingRect();
  if (rect.isEmpty() || viewport()->width() <= 2 || viewport()->height() <= 2) {return 1.;}
  return std::max(.000001, std::min(std::max(1, viewport()->width() - 16) / rect.width(),
    std::max(1, viewport()->height() - 16) / rect.height()));
}

double ScenePreview::zoomFactor() const
{
  return image_->pixmap().isNull() ? 1. : transform().m11() / fitScale();
}

bool ScenePreview::hasPreview() const {return !image_->pixmap().isNull();}

void ScenePreview::applyFit()
{
  const auto scale = fitScale(); setTransform(QTransform::fromScale(scale, scale));
  centerOn(image_->boundingRect().center()); emit viewChanged();
}

void ScenePreview::fitToWindow()
{
  automatic_fit_ = true; applyFit();
}

void ScenePreview::zoomBy(double multiplier, const QPoint & anchor)
{
  if (image_->pixmap().isNull() || !std::isfinite(multiplier) || multiplier <= 0) {return;}
  const auto fit = fitScale();
  const auto current = transform().m11();
  // Resizing a manually zoomed view may move its relative factor beyond the
  // usual range. A subsequent zoom action must still move in its named direction.
  const auto next = multiplier > 1. ?
    std::min(current * multiplier, std::max(current, fit * 8.)) :
    std::max(current * multiplier, std::min(current, fit * .25));
  if (std::abs(next - current) < 1e-12) {return;}
  const auto point = mapToScene(anchor);
  const QPointF viewport_center(viewport()->width() / 2., viewport()->height() / 2.);
  automatic_fit_ = false; setTransform(QTransform::fromScale(next, next));
  centerOn(point + (viewport_center - QPointF(anchor)) / next); emit viewChanged();
}

void ScenePreview::zoomIn() {zoomBy(1.25, viewport()->rect().center());}
void ScenePreview::zoomOut() {zoomBy(1. / 1.25, viewport()->rect().center());}

void ScenePreview::resizeEvent(QResizeEvent * event)
{
  // Qt has already resized the viewport when the view receives this event.
  // Map the old viewport's center before its scrollbar geometry is updated.
  const auto old_size = event->oldSize();
  const QPoint old_center((old_size.width() - 1) / 2, (old_size.height() - 1) / 2);
  const auto center = mapToScene(old_center);
  QGraphicsView::resizeEvent(event);
  if (automatic_fit_) {applyFit();} else {centerOn(center); emit viewChanged();}
}

void ScenePreview::wheelEvent(QWheelEvent * event)
{
  const auto delta = !event->angleDelta().isNull() ? event->angleDelta().y() / 120. :
    event->pixelDelta().y() / 40.;
  if (delta != 0) {zoomBy(std::pow(1.2, delta), event->position().toPoint());}
  event->accept();
}

void ScenePreview::keyPressEvent(QKeyEvent * event)
{
  if (event->key() == Qt::Key_Plus || event->key() == Qt::Key_Equal) {zoomIn();} else if (
    event->key() == Qt::Key_Minus) {zoomOut();} else if (event->key() == Qt::Key_0) {
    fitToWindow();
  } else {QGraphicsView::keyPressEvent(event); return;}
  event->accept();
}

void ScenePreview::mouseDoubleClickEvent(QMouseEvent * event)
{
  if (event->button() == Qt::LeftButton) {fitToWindow(); event->accept(); return;}
  QGraphicsView::mouseDoubleClickEvent(event);
}

void ScenePreview::drawForeground(QPainter * painter, const QRectF & rect)
{
  QGraphicsView::drawForeground(painter, rect);
  if (!image_->pixmap().isNull()) {return;}
  painter->save(); painter->resetTransform(); painter->setPen(palette().color(QPalette::Text));
  painter->drawText(viewport()->rect().adjusted(12, 12, -12, -12),
    Qt::AlignCenter | Qt::TextWordWrap, QStringLiteral("此场景暂无俯视预览图")); painter->restore();
}
}  // namespace easyfly_connection_manager

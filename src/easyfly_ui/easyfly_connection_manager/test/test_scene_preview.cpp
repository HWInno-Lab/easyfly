// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <QApplication>
#include <QCryptographicHash>
#include <QFile>
#include <QImage>
#include <QMouseEvent>
#include <QScrollArea>
#include <QScrollBar>
#include <QTemporaryDir>
#include <QTest>
#include <QVBoxLayout>
#include <QWheelEvent>

#include "../src/scene_preview.hpp"

namespace cm = easyfly_connection_manager;
namespace
{
QJsonObject previewImage(const QString & directory, const QString & name, QRgb color)
{
  const auto path = directory + '/' + name + ".png";
  QImage image(1000, 600, QImage::Format_RGB32); image.fill(color);
  if (!image.save(path)) {return {};}
  QFile file(path); if (!file.open(QIODevice::ReadOnly)) {return {};}
  return {{"path", path}, {"sha256", QString::fromLatin1(
      QCryptographicHash::hash(file.readAll(), QCryptographicHash::Sha256).toHex())}};
}

void drag(cm::ScenePreview & view, const QPoint & from, const QPoint & to)
{
  QTest::mousePress(view.viewport(), Qt::LeftButton, Qt::NoModifier, from);
  QMouseEvent move(QEvent::MouseMove, to, view.viewport()->mapToGlobal(to), Qt::NoButton,
    Qt::LeftButton, Qt::NoModifier);
  QApplication::sendEvent(view.viewport(), &move);
  QTest::mouseRelease(view.viewport(), Qt::LeftButton, Qt::NoModifier, to);
  QApplication::processEvents();
}
}  // namespace

TEST(ScenePreview, PollingPreservesZoomAndPanningWhileSceneChangesResetTheView)
{
  QTemporaryDir directory; ASSERT_TRUE(directory.isValid());
  auto first = previewImage(directory.path(), "first", qRgb(40, 100, 180));
  auto second = previewImage(directory.path(), "second", qRgb(170, 90, 30));
  cm::ScenePreview view; view.resize(600, 420); view.show(); QApplication::processEvents();
  ASSERT_TRUE(view.setPreview({}, first)); EXPECT_TRUE(view.hasPreview());
  EXPECT_NEAR(view.zoomFactor(), 1., 1e-6);
  for (int i = 0; i < 5; ++i) {view.zoomIn();}
  const auto center_before_drag = view.mapToScene(view.viewport()->rect().center());
  drag(view, QPoint(300, 200), QPoint(390, 250));
  const auto center = view.mapToScene(view.viewport()->rect().center());
  EXPECT_GT(QLineF(center_before_drag, center).length(), 20.);
  const auto zoom = view.zoomFactor();
  ASSERT_TRUE(view.setPreview({}, first));
  EXPECT_NEAR(view.zoomFactor(), zoom, 1e-6);
  EXPECT_LT(QLineF(view.mapToScene(view.viewport()->rect().center()), center).length(), 1.);
  ASSERT_TRUE(view.setPreview({}, second));
  EXPECT_NEAR(view.zoomFactor(), 1., 1e-6);
  EXPECT_LT(QLineF(view.mapToScene(view.viewport()->rect().center()), QPointF(500, 300)).length(), 3.);
}

TEST(ScenePreview, CursorAnchoredWheelZoomDoesNotScrollItsOuterForm)
{
  QTemporaryDir directory; ASSERT_TRUE(directory.isValid());
  QScrollArea outer; outer.resize(700, 420); outer.setWidgetResizable(true);
  auto * body = new QWidget; auto * layout = new QVBoxLayout(body);
  auto * view = new cm::ScenePreview; layout->addWidget(view); body->setMinimumHeight(700);
  outer.setWidget(body); outer.show(); QApplication::processEvents();
  ASSERT_TRUE(view->setPreview({}, previewImage(directory.path(), "wheel", qRgb(90, 120, 150))));
  outer.verticalScrollBar()->setValue(80);
  const auto outer_position = outer.verticalScrollBar()->value();
  const QPoint anchor(150, 130); const auto before = view->mapToScene(anchor);
  QWheelEvent wheel(anchor, view->viewport()->mapToGlobal(anchor), {}, QPoint(0, 120),
    Qt::NoButton, Qt::NoModifier, Qt::NoScrollPhase, false);
  wheel.setAccepted(false); QApplication::sendEvent(view->viewport(), &wheel);
  EXPECT_TRUE(wheel.isAccepted()); EXPECT_GT(view->zoomFactor(), 1.);
  EXPECT_EQ(outer.verticalScrollBar()->value(), outer_position);
  EXPECT_LT(QLineF(before, view->mapToScene(anchor)).length() * view->transform().m11(), 2.);
  for (int i = 0; i < 40; ++i) {view->zoomIn();}
  EXPECT_NEAR(view->zoomFactor(), 8., 1e-6);
  for (int i = 0; i < 70; ++i) {view->zoomOut();}
  EXPECT_NEAR(view->zoomFactor(), .25, 1e-6);
}

TEST(ScenePreview, ResizeKeepsManualScaleAndFitModeContinuesAdapting)
{
  QTemporaryDir directory; ASSERT_TRUE(directory.isValid());
  cm::ScenePreview view; view.resize(600, 400); view.show(); QApplication::processEvents();
  ASSERT_TRUE(view.setPreview({}, previewImage(directory.path(), "resize", qRgb(70, 110, 150))));
  view.resize(900, 650); QApplication::processEvents();
  EXPECT_NEAR(view.zoomFactor(), 1., 1e-6);
  view.zoomIn(); view.zoomIn();
  const auto scale = view.transform().m11();
  const auto center = view.mapToScene(view.viewport()->rect().center());
  view.resize(700, 500); QApplication::processEvents();
  EXPECT_NEAR(view.transform().m11(), scale, 1e-6);
  EXPECT_LT(QLineF(center, view.mapToScene(view.viewport()->rect().center())).length(), 3.);
  view.fitToWindow(); EXPECT_NEAR(view.zoomFactor(), 1., 1e-6);
  view.resize(650, 450); QApplication::processEvents(); EXPECT_NEAR(view.zoomFactor(), 1., 1e-6);
  for (int i = 0; i < 30; ++i) {view.zoomIn();}
  view.resize(300, 230); QApplication::processEvents();
  const auto resized_scale = view.transform().m11();
  view.zoomIn(); EXPECT_GE(view.transform().m11(), resized_scale);
  view.zoomOut(); EXPECT_LT(view.transform().m11(), resized_scale);
  EXPECT_FALSE(view.setPreview({}, {})); EXPECT_FALSE(view.hasPreview());
  view.zoomIn(); EXPECT_NEAR(view.zoomFactor(), 1., 1e-6);
}

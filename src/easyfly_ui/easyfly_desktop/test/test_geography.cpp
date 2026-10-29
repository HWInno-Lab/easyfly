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
#include <gtest/gtest.h>
#include <sqlite3.h>
#include <QBuffer>
#include <QDir>
#include <QFile>
#include <QImage>
#include <QLabel>
#include <QPushButton>
#include <QTemporaryDir>
#include <QTest>
#include <cmath>
#include <limits>
#include "easyfly_desktop/geography.hpp"
#include "easyfly_desktop/geographic_view.hpp"
#include "support.hpp"
namespace geo = easyfly_desktop::geo;
namespace ui = easyfly_desktop;
namespace
{
void fixture(const QString & path, bool indexed = true, int tile_size = 256)
{
  sqlite3 * db = nullptr; ASSERT_EQ(sqlite3_open(QFile::encodeName(path), &db), SQLITE_OK);
  ASSERT_EQ(
    sqlite3_exec(
      db,
      "CREATE TABLE metadata(name TEXT,value TEXT);"
      "INSERT INTO metadata VALUES ('name','Test authored raster'),"
      "('attribution','EasyFly test fixture'),"
      "('minzoom','1'),('maxzoom','1'),('format','png'),('scheme','tms');"
      "CREATE TABLE tiles(zoom_level INTEGER,tile_column INTEGER,tile_row INTEGER,tile_data BLOB);",
      nullptr, nullptr, nullptr), SQLITE_OK);
  if (indexed) {
    ASSERT_EQ(
      sqlite3_exec(
        db,
        "CREATE UNIQUE INDEX tile_index ON tiles(zoom_level,tile_column,tile_row)", nullptr,
        nullptr, nullptr), SQLITE_OK);
  }
  for (int row = 0; row < 2; ++row) {
    QImage image(tile_size, tile_size, QImage::Format_RGB32); image.fill(
      row == 0 ? Qt::red : Qt::blue);
    QByteArray bytes; QBuffer buffer(&bytes); buffer.open(QIODevice::WriteOnly); ASSERT_TRUE(
      image.save(
        &buffer,
        "PNG"));
    sqlite3_stmt * statement = nullptr;
    ASSERT_EQ(
      sqlite3_prepare_v2(
        db, "INSERT INTO tiles VALUES(1,1,?,?)", -1, &statement,
        nullptr), SQLITE_OK);
    sqlite3_bind_int(statement, 1, row); sqlite3_bind_blob(
      statement, 2,
      bytes.constData(), bytes.size(), SQLITE_TRANSIENT);
    ASSERT_EQ(sqlite3_step(statement), SQLITE_DONE); sqlite3_finalize(statement);
  }
  sqlite3_close(db);
}
geo::Datum datum()
{return {{0, 0, 0}, {0, 0, 0}, 0, "odom", "test surveyed corresponding point"};}
}  // namespace
TEST(Geography, AnalyticEquatorialAxesAndAltitudeAreWgs84Ellipsoid)
{
  const auto d = datum(); const auto east = geo::to_geodetic(d, {100, 0, 0});
  constexpr double a = 6378137., pi = 3.14159265358979323846;
  EXPECT_NEAR(east.latitude, 0, 1e-12);
  EXPECT_NEAR(east.longitude, std::atan2(100., a) * 180 / pi, 1e-10);
  EXPECT_NEAR(east.ellipsoid_height, std::hypot(a, 100.) - a, 1e-8);
  const auto up = geo::to_geodetic(d, {0, 0, 100});
  EXPECT_NEAR(up.ellipsoid_height, 100, 1e-8);
  auto rotated = d; rotated.local_origin = {4, 5, 6}; rotated.yaw_rad = pi / 2;
  const auto north = geo::to_geodetic(rotated, {104, 5, 6});
  EXPECT_GT(north.latitude, 0); EXPECT_NEAR(north.longitude, 0, 1e-10);
  const auto back = geo::to_local(rotated, north);
  EXPECT_NEAR(back[0], 104, 1e-7); EXPECT_NEAR(back[1], 5, 1e-7); EXPECT_NEAR(back[2], 6, 1e-7);
}
TEST(Geography, DateLinePolarProjectionAndBoundedExplicitDatum)
{
  auto d = datum(); d.origin = {70, 179.999, 42};
  const auto p = geo::to_geodetic(d, {1000, 100, 12}); EXPECT_LT(p.longitude, 0);
  const auto back = geo::to_local(d, p);
  EXPECT_NEAR(back[0], 1000, 1e-6); EXPECT_NEAR(back[1], 100, 1e-6); EXPECT_NEAR(back[2], 12, 1e-6);
  EXPECT_THROW(geo::to_geodetic(d, {5001, 0, 0}), std::runtime_error);
  d.evidence.clear(); EXPECT_FALSE(geo::valid(d));
  EXPECT_THROW(geo::to_geodetic(d, {0, 0, 0}), std::runtime_error);
  EXPECT_THROW(geo::mercator(90, 0), std::runtime_error);
  EXPECT_THROW(geo::mercator(std::numeric_limits<double>::quiet_NaN(), 0), std::runtime_error);
  EXPECT_NEAR(geo::mercator(0, 0).x(), .5, 1e-12);
  EXPECT_NEAR(geo::mercator(0, 0).y(), .5, 1e-12);
  const auto ll = geo::unmercator(geo::mercator(47.397742, 8.545594));
  EXPECT_NEAR(ll.latitude, 47.397742, 1e-10); EXPECT_NEAR(ll.longitude, 8.545594, 1e-10);
}
TEST(Geography, IndexedTmsLookupLosslessCacheAndColdRead)
{
  QTemporaryDir temp; ASSERT_TRUE(temp.isValid()); const auto source = temp.filePath(
    "source.mbtiles"); fixture(source);
  const auto cached = geo::Tiles::cache(source, temp.filePath("cache"));
  EXPECT_EQ(cached, geo::Tiles::cache(source, temp.filePath("cache")));
  QFile original(source), copy(cached); ASSERT_TRUE(original.open(QIODevice::ReadOnly));
  ASSERT_TRUE(copy.open(QIODevice::ReadOnly));
  EXPECT_EQ(original.readAll(), copy.readAll());
  geo::Tiles tiles(cached);
  EXPECT_EQ(tiles.tile(1, 1, 0).pixelColor(0, 0), QColor(Qt::blue));
  EXPECT_EQ(tiles.tile(1, 1, 1).pixelColor(0, 0), QColor(Qt::red));
  EXPECT_TRUE(tiles.tile(1, 0, 0).isNull()); EXPECT_TRUE(tiles.tile(20, 0, 0).isNull());
  EXPECT_EQ(tiles.metadata().attribution, "EasyFly test fixture");
}
TEST(Geography, RefusesUnsupportedSchemaImagesAndAlteredCache)
{
  QTemporaryDir temp; ASSERT_TRUE(temp.isValid());
  fixture(temp.filePath("unindexed.mbtiles"), false);
  EXPECT_THROW(geo::Tiles(temp.filePath("unindexed.mbtiles")), std::runtime_error);
  fixture(temp.filePath("oversize.mbtiles"), true, 2048);
  geo::Tiles oversized(temp.filePath("oversize.mbtiles")); EXPECT_THROW(
    oversized.tile(
      1, 1,
      0),
    std::runtime_error);
  fixture(temp.filePath("valid.mbtiles"));
  const auto cache = geo::Tiles::cache(temp.filePath("valid.mbtiles"), temp.filePath("cache"));
  QFile file(cache); ASSERT_TRUE(file.open(QIODevice::Append)); file.write("changed"); file.close();
  EXPECT_THROW(
    geo::Tiles::cache(
      temp.filePath("valid.mbtiles"), temp.filePath(
        "cache")), std::runtime_error);
  EXPECT_THROW(geo::land_polygons("{}"), std::runtime_error);
}
TEST(GeographicView, OriginalLandRendersAndGpsDoesNotInventLocalDatum)
{
  desktop_test::application(); QTemporaryDir temp;
  ui::GeographicView view(nullptr, temp.filePath("cache")); view.resize(900, 620); view.show();
  auto s = desktop_test::sample(); s->state.gps_valid = s->state.sdk.value.gps_valid = true;
  s->state.sdk.value.latitude = 47.397742; s->state.sdk.value.longitude = 8.545594;
  view.snapshot(s, s->produced_steady);
  EXPECT_TRUE(view.property("gps_visible").toBool());
  EXPECT_FALSE(view.property("local_geographic_visible").toBool());
  auto d = datum(); d.origin = {47.397742, 8.545594, 500}; view.set_datum(d);
  EXPECT_TRUE(view.property("local_geographic_visible").toBool());
  view.findChild<QPushButton *>("geoCenterGps")->click();
  auto * canvas = view.findChild<QWidget *>("geographicCanvas"); ASSERT_NE(canvas, nullptr);
  QTest::keyClick(canvas, Qt::Key_Plus); QTest::keyClick(canvas, Qt::Key_Right);
  QTest::qWait(30);
  const auto output = qEnvironmentVariable("EASYFLY_GEOGRAPHIC_SCREENSHOT_DIR");
  if (!output.isEmpty()) {
    QDir().mkpath(output); ASSERT_TRUE(view.grab().save(output + "/geography.png"));
  }
  view.snapshot(s, s->produced_steady + 1.1);
  EXPECT_FALSE(view.property("gps_visible").toBool());
  EXPECT_FALSE(view.property("local_geographic_visible").toBool());
  ++s->time_epoch; view.snapshot(s, s->produced_steady);
  EXPECT_FALSE(view.property("datum_configured").toBool());
  QFile land(":/easyfly/geography/ne_110m_land.geojson");
  ASSERT_TRUE(land.open(QIODevice::ReadOnly));
  EXPECT_GT(geo::land_polygons(land.readAll()).size(), 100u);
}
TEST(GeographicView, AsyncOfflineImportAndConnectionChangeClearLocalOverlay)
{
  desktop_test::application(); QTemporaryDir temp; fixture(temp.filePath("source.mbtiles"));
  ui::GeographicView view(nullptr, temp.filePath("cache")); view.resize(800, 500); view.show();
  auto s = desktop_test::sample(); view.snapshot(s, s->produced_steady); view.set_datum(datum());
  view.import_map(temp.filePath("source.mbtiles"));
  ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return view.property("offline_tiles_loaded").toInt() == 2;
      }, 5000));
  ++s->observation_session_id; view.snapshot(s, s->produced_steady);
  EXPECT_FALSE(view.property("datum_configured").toBool());
  view.set_datum(datum()); s->stopped = true; view.snapshot(s, s->produced_steady);
  EXPECT_FALSE(view.property("datum_configured").toBool());
  EXPECT_FALSE(view.property("local_geographic_visible").toBool());
}

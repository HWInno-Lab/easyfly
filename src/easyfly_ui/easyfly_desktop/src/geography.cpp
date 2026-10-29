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
#include <QBuffer>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QImageReader>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <sqlite3.h>
#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <utility>
#include <GeographicLib/LocalCartesian.hpp>
#include "easyfly_desktop/geography.hpp"
namespace easyfly_desktop::geo
{
namespace
{
constexpr double pi = 3.14159265358979323846;
constexpr qint64 file_limit = 512LL * 1024 * 1024;
void require(bool condition, const char * message)
{if (!condition) {throw std::runtime_error(message);}}
bool finite(const Point & p)
{return std::all_of(p.begin(), p.end(), [](double x) {return std::isfinite(x);});}
struct Statement
{
  sqlite3_stmt * value{nullptr};
  Statement(sqlite3 * db, const char * query)
  {
    require(
      sqlite3_prepare_v2(
        db, query, -1, &value,
        nullptr) == SQLITE_OK, "Invalid MBTiles schema");
  }
  ~Statement() {sqlite3_finalize(value);}
};
QByteArray digest(QFile & file)
{
  QCryptographicHash hash(QCryptographicHash::Sha256);
  require(file.seek(0) && hash.addData(&file), "Cannot hash map file");
  return hash.result().toHex();
}
}  // namespace
bool valid(const Geodetic & p)
{
  return std::isfinite(p.latitude) && std::abs(p.latitude) <= 90 &&
         std::isfinite(p.longitude) && std::abs(p.longitude) <= 180 &&
         std::isfinite(p.ellipsoid_height) && std::abs(p.ellipsoid_height) <= 100000;
}
bool valid(const Datum & d)
{
  return valid(d.origin) && finite(d.local_origin) &&
         std::all_of(
    d.local_origin.begin(), d.local_origin.end(),
    [](double x) {return std::abs(x) <= 1e6;}) &&
         std::isfinite(d.yaw_rad) && std::abs(d.yaw_rad) <= pi &&
         !d.frame.isEmpty() && d.frame.size() <= 128 &&
         !d.evidence.trimmed().isEmpty() && d.evidence.size() <= 1024;
}
Point to_local(const Datum & d, const Geodetic & p)
{
  require(valid(d) && valid(p), "Invalid WGS84 datum or coordinate");
  GeographicLib::LocalCartesian earth(d.origin.latitude, d.origin.longitude,
    d.origin.ellipsoid_height);
  Point enu;
  earth.Forward(p.latitude, p.longitude, p.ellipsoid_height, enu[0], enu[1], enu[2]);
  require(std::hypot(enu[0], enu[1], enu[2]) <= 5000, "Local datum radius exceeds 5 km");
  const auto c = std::cos(d.yaw_rad), s = std::sin(d.yaw_rad);
  return {c * enu[0] + s * enu[1] + d.local_origin[0],
    -s * enu[0] + c * enu[1] + d.local_origin[1], enu[2] + d.local_origin[2]};
}
Geodetic to_geodetic(const Datum & d, const Point & p)
{
  require(valid(d) && finite(p), "Invalid WGS84 datum or local coordinate");
  Point a{p[0] - d.local_origin[0], p[1] - d.local_origin[1], p[2] - d.local_origin[2]};
  require(std::hypot(a[0], a[1], a[2]) <= 5000, "Local datum radius exceeds 5 km");
  const auto c = std::cos(d.yaw_rad), s = std::sin(d.yaw_rad);
  GeographicLib::LocalCartesian earth(d.origin.latitude, d.origin.longitude,
    d.origin.ellipsoid_height);
  Geodetic result;
  earth.Reverse(
    c * a[0] - s * a[1], s * a[0] + c * a[1], a[2],
    result.latitude, result.longitude, result.ellipsoid_height);
  require(valid(result), "Invalid projected coordinate"); return result;
}
QPointF mercator(double lat, double lon)
{
  require(
    std::isfinite(lat) && std::abs(lat) <= 85.0511287798066 && std::isfinite(lon),
    "Web Mercator does not represent polar coordinates");
  lon = std::remainder(lon, 360.0);
  return {(lon + 180) / 360, (1 - std::asinh(std::tan(lat * pi / 180)) / pi) / 2};
}
Geodetic unmercator(const QPointF & p)
{
  require(
    std::isfinite(p.x()) && std::isfinite(p.y()) && p.y() >= 0 && p.y() <= 1,
    "Invalid Web Mercator coordinate");
  return {std::atan(std::sinh(pi * (1 - 2 * p.y()))) * 180 / pi,
    std::remainder(p.x() * 360 - 180, 360.0), 0};
}
std::vector<QVector<QPointF>> land_polygons(const QByteArray & bytes)
{
  require(bytes.size() <= 2 * 1024 * 1024, "Land dataset exceeds size limit");
  QJsonParseError error; const auto doc = QJsonDocument::fromJson(bytes, &error);
  require(
    error.error == QJsonParseError::NoError && doc.isObject() &&
    doc.object()["type"] == "FeatureCollection", "Invalid land GeoJSON");
  std::vector<QVector<QPointF>> output; int count = 0;
  for (const auto & feature : doc.object()["features"].toArray()) {
    const auto g = feature.toObject()["geometry"].toObject();
    QJsonArray polygons;
    if (g["type"] == "Polygon") {
      polygons.append(g["coordinates"]);
    } else if (g["type"] == "MultiPolygon") {polygons = g["coordinates"].toArray();} else {
      throw std::runtime_error("Unsupported land geometry");
    }
    for (const auto & polygon : polygons) {
      for (const auto & ring : polygon.toArray()) {
        QVector<QPointF> points;
        for (const auto & coordinate : ring.toArray()) {
          const auto a = coordinate.toArray();
          require(
            a.size() >= 2 && a[0].isDouble() && a[1].isDouble() && ++count <= 50000,
            "Invalid or excessive land coordinates");
          const auto lon = a[0].toDouble(), lat = a[1].toDouble();
          require(std::abs(lon) <= 180 && std::abs(lat) <= 90, "Invalid land coordinate");
          points.append(mercator(std::clamp(lat, -85.0511287798066, 85.0511287798066), lon));
        }
        require(points.size() >= 4 && points.front() == points.back(), "Unclosed land ring");
        output.push_back(std::move(points));
      }
    }
  }
  require(!output.empty(), "Empty land dataset"); return output;
}
struct Tiles::Impl
{
  sqlite3 * db{nullptr}; TileMetadata metadata; int budget{0};
  ~Impl() {if (db) {sqlite3_close(db);}}
  void start() {budget = 10000;}
};
Tiles::Tiles(const QString & path)
: impl_(std::make_unique<Impl>())
{
  const QFileInfo file(path);
  require(
    file.isFile() && !file.isSymLink() && file.size() > 0 && file.size() <= file_limit,
    "Map must be a regular MBTiles file of at most 512 MiB");
  require(
    sqlite3_open_v2(
      QFile::encodeName(file.absoluteFilePath()), &impl_->db,
      SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX, nullptr) == SQLITE_OK, "Cannot open offline map");
  sqlite3_limit(impl_->db, SQLITE_LIMIT_LENGTH, 2 * 1024 * 1024);
  sqlite3_limit(impl_->db, SQLITE_LIMIT_SQL_LENGTH, 8192);
  sqlite3_exec(
    impl_->db, "PRAGMA trusted_schema=OFF; PRAGMA query_only=ON;", nullptr, nullptr,
    nullptr);
  sqlite3_progress_handler(
    impl_->db, 100, [](void * p) {
      auto & i = *static_cast<Impl *>(p); return --i.budget <= 0 ? 1 : 0;
    }, impl_.get());
  impl_->start(); Statement schema(impl_->db,
    "SELECT name,type FROM sqlite_master WHERE name IN ('metadata','tiles')"); int tables = 0;
  while (sqlite3_step(schema.value) == SQLITE_ROW) {
    require(
      QByteArray(reinterpret_cast<const char *>(sqlite3_column_text(schema.value, 1))) == "table",
      "MBTiles views are not accepted"); ++tables;
  }
  require(tables == 2, "Missing MBTiles tables");
  impl_->start(); Statement metadata(impl_->db, "SELECT name,value FROM metadata LIMIT 65");
  std::map<QString, QString> values; int rc;
  while ((rc = sqlite3_step(metadata.value)) == SQLITE_ROW) {
    require(
      sqlite3_column_bytes(metadata.value, 0) <= 128 &&
      sqlite3_column_bytes(metadata.value, 1) <= 4096, "Oversized map metadata");
    const auto key =
      QString::fromUtf8(reinterpret_cast<const char *>(sqlite3_column_text(metadata.value, 0)));
    const auto value =
      QString::fromUtf8(reinterpret_cast<const char *>(sqlite3_column_text(metadata.value, 1)));
    require(!values.count(key) && values.size() < 64, "Duplicate or excessive map metadata");
    values[key] = value;
  }
  require(rc == SQLITE_DONE, "Cannot read map metadata");
  bool min_ok = false, max_ok = false;
  auto & m = impl_->metadata;
  m.name = values["name"]; m.attribution = values["attribution"]; m.format = values["format"];
  m.minimum_zoom = values["minzoom"].toInt(&min_ok); m.maximum_zoom = values["maxzoom"].toInt(
    &max_ok);
  require(
    min_ok && max_ok && m.minimum_zoom >= 0 && m.maximum_zoom >= m.minimum_zoom &&
    m.maximum_zoom <= 19 && !m.name.isEmpty() && !m.attribution.isEmpty() &&
    (m.format == "png" || m.format == "jpg") &&
    (!values.count(
      "scheme") || values["scheme"] == "tms"), "Unsupported or incomplete raster MBTiles metadata");
  impl_->start(); Statement plan(impl_->db,
    "EXPLAIN QUERY PLAN SELECT tile_data FROM tiles "
    "WHERE zoom_level=0 AND tile_column=0 AND tile_row=0");
  bool indexed = false;
  while (sqlite3_step(plan.value) == SQLITE_ROW) {
    indexed |=
      QByteArray(reinterpret_cast<const char *>(sqlite3_column_text(plan.value, 3))).contains(
      "SEARCH");
  }
  require(indexed, "Offline tile lookup requires an index");
}
Tiles::~Tiles() = default;
TileMetadata Tiles::metadata() const {return impl_->metadata;}
QImage Tiles::tile(int z, int x, int y) const
{
  const auto m = metadata();
  if (z < m.minimum_zoom || z > m.maximum_zoom) {return {};}
  const auto size = 1 << z;
  if (x < 0 || x >= size || y < 0 || y >= size) {return {};}
  impl_->start(); Statement query(impl_->db,
    "SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=? LIMIT 2");
  sqlite3_bind_int(query.value, 1, z); sqlite3_bind_int(query.value, 2, x);
  sqlite3_bind_int(query.value, 3, size - 1 - y);
  const int rc = sqlite3_step(query.value);
  if (rc == SQLITE_DONE) {return {};}
  require(rc == SQLITE_ROW, "Offline tile read failed");
  const auto length = sqlite3_column_bytes(query.value, 0);
  require(length > 0 && length <= 1024 * 1024, "Tile exceeds encoded size limit");
  QByteArray bytes(static_cast<const char *>(sqlite3_column_blob(query.value, 0)), length);
  require(sqlite3_step(query.value) == SQLITE_DONE, "Duplicate offline tile");
  QBuffer buffer(&bytes); buffer.open(QIODevice::ReadOnly);
  QImageReader reader(&buffer, m.format.toLatin1()); reader.setAutoDetectImageFormat(false);
  const auto dimensions = reader.size();
  require(
    dimensions == QSize(256, 256) || dimensions == QSize(
      512,
      512), "Unsupported tile dimensions");
  const auto image = reader.read(); require(!image.isNull(), "Invalid raster tile"); return image;
}
QString Tiles::cache(const QString & source, const QString & directory)
{
  Tiles check(source);  // Validate before copying into the bounded local cache.
  QDir target(directory); require(
    !QFileInfo(directory).isSymLink() && target.mkpath(
      "."), "Invalid cache directory");
  QFile input(source); require(input.open(QIODevice::ReadOnly), "Cannot read map import");
  const auto hash = digest(input); const auto output = target.filePath(
    QString::fromLatin1(
      hash) + ".mbtiles");
  if (QFileInfo::exists(output)) {
    QFile existing(output);
    require(
      !QFileInfo(output).isSymLink() && existing.open(QIODevice::ReadOnly) && digest(
        existing) == hash,
      "Cached map hash mismatch"); return output;
  }
  qint64 used = 0;
  for (const auto & file : target.entryInfoList(QDir::Files | QDir::Hidden)) {
    used += file.size();
  }
  require(
    input.size() <= file_limit && used + input.size() <= 2 * file_limit,
    "Offline map cache exceeds 1 GiB; remove an unused cached map first");
  require(input.seek(0), "Cannot seek map import");
  QSaveFile destination(output); require(
    destination.open(
      QIODevice::WriteOnly), "Cannot create cached map");
  QCryptographicHash copied(QCryptographicHash::Sha256); qint64 total = 0;
  while (!input.atEnd()) {
    const auto bytes = input.read(1024 * 1024);
    require(
      !bytes.isEmpty() && (total += bytes.size()) <= file_limit,
      "Map import changed or read failed");
    copied.addData(bytes); require(
      destination.write(
        bytes) == bytes.size(), "Cannot write cached map");
  }
  require(
    copied.result().toHex() == hash && destination.commit(),
    "Map import changed or atomic save failed");
  Tiles verify(output); return output;
}
}  // namespace easyfly_desktop::geo

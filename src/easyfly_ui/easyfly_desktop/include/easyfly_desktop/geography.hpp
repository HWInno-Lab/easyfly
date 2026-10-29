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
#ifndef EASYFLY_DESKTOP__GEOGRAPHY_HPP_
#define EASYFLY_DESKTOP__GEOGRAPHY_HPP_
#include <QImage>
#include <QPointF>
#include <QString>
#include <QVector>
#include <array>
#include <memory>
#include <vector>
namespace easyfly_desktop::geo
{
struct Geodetic {double latitude{0}, longitude{0}, ellipsoid_height{0};};
using Point = std::array<double, 3>;
// Explicit surveyed local-frame anchor, never inferred from same-named TF frames.
// yaw is the local +X direction measured counterclockwise from true east.
struct Datum
{
  Geodetic origin;
  Point local_origin{0, 0, 0};
  double yaw_rad{0};
  QString frame;
  QString evidence;
};
bool valid(const Geodetic & point);
bool valid(const Datum & datum);
Point to_local(const Datum & datum, const Geodetic & point);
Geodetic to_geodetic(const Datum & datum, const Point & point);
// Normalized Web Mercator coordinates. Longitude wraps; poles are refused.
QPointF mercator(double latitude, double longitude);
Geodetic unmercator(const QPointF & point);
std::vector<QVector<QPointF>> land_polygons(const QByteArray & geojson);
struct TileMetadata
{
  QString name, attribution, format;
  int minimum_zoom{0}, maximum_zoom{0};
};
// Immutable, bounded, read-only raster MBTiles. No URL or network operations.
class Tiles
{
public:
  explicit Tiles(const QString & path);
  ~Tiles();
  Tiles(const Tiles &) = delete;
  Tiles & operator=(const Tiles &) = delete;
  TileMetadata metadata() const;
  QImage tile(int zoom, int x, int y_xyz) const;
  static QString cache(const QString & source, const QString & directory);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_desktop::geo
#endif  // EASYFLY_DESKTOP__GEOGRAPHY_HPP_

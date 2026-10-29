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
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QResource>
#include <QStandardPaths>
#include <QTimer>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <future>
#include <map>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>
#include "easyfly_desktop/geographic_view.hpp"
#include "easyfly_desktop/presentation.hpp"
static void initialize_geographic_resources() {Q_INIT_RESOURCE(easyfly_assets);}
namespace easyfly_desktop
{
namespace
{
constexpr double pi = 3.14159265358979323846;
using TileKey = std::tuple<int, int, int>;
struct TileBatch
{
  QString path, error; std::map<TileKey, QImage> images;
};
class GeoCanvas : public QWidget
{
public:
  QPointF center{.5, .5}; int zoom{1};
  std::optional<QPointF> gps, local;
  std::optional<double> heading;
  QString path, error;
  std::map<TileKey, QImage> tiles;
  std::vector<QVector<QPointF>> land;
  std::function<void()> changed;
  explicit GeoCanvas(QWidget * parent)
  : QWidget(parent)
  {
    initialize_geographic_resources();
    setObjectName("geographicCanvas"); setFocusPolicy(Qt::StrongFocus);
    setAccessibleName(QStringLiteral("离线地理地图；方向键平移，加减键缩放"));
    setMinimumSize(300, 220);
    QFile file(":/easyfly/geography/ne_110m_land.geojson");
    if (file.open(QIODevice::ReadOnly)) {
      try {land = geo::land_polygons(file.readAll());} catch (const std::exception & e) {
        error = e.what();
      }
    } else {error = QStringLiteral("内置底图不可用");}
  }
  double scale() const {return std::ldexp(256., zoom);}
  QPointF pixel(QPointF p) const
  {
    return {width() / 2. + std::remainder(p.x() - center.x(), 1.) * scale(),
      height() / 2. + (p.y() - center.y()) * scale()};
  }
  void move(QPointF delta)
  {
    center += delta / scale(); center.setX(center.x() - std::floor(center.x()));
    center.setY(std::clamp(center.y(), 0., 1.)); if (changed) {changed();} update();
  }
  void zoom_by(int delta)
  {zoom = std::clamp(zoom + delta, 0, path.isEmpty() ? 5 : 19); if (changed) {changed();} update();}
  std::vector<TileKey> needed() const
  {
    std::vector<TileKey> result; const int n = 1 << zoom;
    const int left = static_cast<int>(std::floor(center.x() * n - width() / 512.));
    const int top = static_cast<int>(std::floor(center.y() * n - height() / 512.));
    for (int y = top; y <= top + height() / 256 + 1; ++y) {
      for (int x = left; x <= left + width() / 256 + 1; ++x) {
        if (y >= 0 && y < n && result.size() < 64) {result.emplace_back(zoom, (x % n + n) % n, y);}
      }
    }
    return result;
  }

protected:
  QPoint last;
  void mousePressEvent(QMouseEvent * e) override {last = e->pos(); setFocus();}
  void mouseMoveEvent(QMouseEvent * e) override
  {if (e->buttons() & Qt::LeftButton) {move(last - e->pos()); last = e->pos();}}
  void wheelEvent(QWheelEvent * e) override
  {
    zoom_by(e->angleDelta().y() > 0 ? 1 : -1); e->accept();
  }
  void keyPressEvent(QKeyEvent * e) override
  {
    switch (e->key()) {
      case Qt::Key_Left: move({-80, 0}); break;
      case Qt::Key_Right: move({80, 0}); break;
      case Qt::Key_Up: move({0, -80}); break;
      case Qt::Key_Down: move({0, 80}); break;
      case Qt::Key_Plus: case Qt::Key_Equal: zoom_by(1); break;
      case Qt::Key_Minus: zoom_by(-1); break;
      default: QWidget::keyPressEvent(e); return;
    }
    e->accept();
  }
  void resizeEvent(QResizeEvent * e) override {QWidget::resizeEvent(e); if (changed) {changed();}}
  void paintEvent(QPaintEvent *) override
  {
    QPainter p(this); p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), QColor("#e9eff4"));
    if (path.isEmpty()) {
      p.fillRect(rect(), QColor("#dcebf3")); p.setPen(QPen(QColor("#647e70"), 1));
      p.setBrush(QColor("#e7ead9")); QPainterPath continents;
      for (const auto & ring : land) {
        // Draw three world copies with an unwrapped x coordinate. Never join
        // opposite sides of the date line through the screen center.
        for (int world = -1; world <= 1; ++world) {
          QPolygonF poly;
          for (const auto & v : ring) {
            poly.append(
              {width() / 2. + (v.x() + world - center.x()) * scale(),
                height() / 2. + (v.y() - center.y()) * scale()});
          }
          continents.addPolygon(poly);
        }
      }
      continents.setFillRule(Qt::OddEvenFill); p.drawPath(continents);
    } else {
      p.fillRect(rect(), QBrush(QColor("#c9d1d9"), Qt::Dense7Pattern));
      const int n = 1 << zoom;
      for (const auto & entry : tiles) {
        if (std::get<0>(entry.first) != zoom || entry.second.isNull()) {continue;}
        const QPointF normalized((std::get<1>(entry.first) + .5) / n,
          (std::get<2>(entry.first) + .5) / n);
        const auto point = pixel(normalized);
        p.drawImage(QRectF(point.x() - 128, point.y() - 128, 256, 256), entry.second);
      }
    }
    if (gps) {
      const auto point = pixel(*gps); p.setPen(QPen(QColor("#144c8c"), 2));
      p.setBrush(QColor("#ffffff")); p.drawEllipse(point, 7, 7);
      p.drawText(point + QPointF(11, -7), QStringLiteral("GPS"));
    }
    if (local) {
      const auto point = pixel(*local); p.setPen(QPen(QColor("#80520d"), 2));
      p.setBrush(QColor("#e9a938"));
      if (heading) {
        p.save(); p.translate(point); p.rotate(*heading);
        p.drawPolygon(QPolygonF(QVector<QPointF>{{0, -14}, {8, 9}, {0, 5}, {-8, 9}})); p.restore();
      } else {p.drawRect(QRectF(point.x() - 5, point.y() - 5, 10, 10));}
      p.drawText(point + QPointF(11, 17), QStringLiteral("已设基准的局部位置"));
    }
    const auto lat = geo::unmercator(center).latitude;
    const double metres = 100. / scale() * 40075016.68557849 * std::cos(lat * pi / 180);
    p.fillRect(QRect(8, height() - 47, 190, 38), QColor(255, 255, 255, 235));
    p.setPen(QPen(QColor("#25364b"), 2)); p.drawLine(18, height() - 18, 118, height() - 18);
    p.drawText(
      18, height() - 25, metres >= 1000 ? QString::number(metres / 1000, 'f', 1) + " km" :
      QString::number(metres, 'f', 1) + " m");
    p.drawText(width() - 70, 25, QStringLiteral("北 ↑"));
    if (!error.isEmpty()) {
      p.drawText(rect().adjusted(20, 35, -20, -60), Qt::AlignCenter | Qt::TextWordWrap, error);
    }
  }
};
}  // namespace
struct GeographicView::Impl
{
  GeographicView * owner; GeoCanvas * canvas; QLabel * status; QLabel * attribution;
  QPushButton * import_button; QString cache;
  std::shared_ptr<const bridge::UiSnapshot> state; double now{0};
  std::optional<geo::Datum> datum; uint64_t session{0}, epoch{0};
  std::future<QString> importing; std::future<TileBatch> loading; bool dirty{true};
  Impl(GeographicView * widget, QString directory)
  : owner(widget), cache(std::move(directory))
  {
    if (cache.isEmpty()) {
      cache = QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation) +
        "/offline-maps";
    }
    auto * layout = new QVBoxLayout(owner); auto * toolbar = new QHBoxLayout; layout->addLayout(
      toolbar);
    auto add = [&](const QString & title, const char * name, const std::function<void()> & action) {
        auto * b = new QPushButton(title, owner); b->setObjectName(name); b->setMinimumHeight(32);
        toolbar->addWidget(b); QObject::connect(b, &QPushButton::clicked, owner, action); return b;
      };
    add(QStringLiteral("＋"), "geoZoomIn", [this] {canvas->zoom_by(1);})->setAccessibleName(
      QStringLiteral(
        "放大地图"));
    add(QStringLiteral("−"), "geoZoomOut", [this] {canvas->zoom_by(-1);})->setAccessibleName(
      QStringLiteral(
        "缩小地图"));
    add(
      QStringLiteral("定位 GPS"), "geoCenterGps", [this] {
        if (canvas->gps) {canvas->center = *canvas->gps; dirty = true; canvas->update();}
      });
    toolbar = new QHBoxLayout; layout->addLayout(toolbar);
    import_button = add(
      QStringLiteral("导入离线底图"), "geoImport", [this] {
        const auto file =
        QFileDialog::getOpenFileName(
          owner, QStringLiteral("选择有使用权的栅格 MBTiles"), cache,
          "MBTiles (*.mbtiles)");
        if (!file.isEmpty()) {owner->import_map(file);}
      });
    import_button->setToolTip(QStringLiteral("离线地图缓存目录：%1").arg(cache));
    add(
      QStringLiteral("世界底图"), "geoWorld", [this] {
        canvas->path.clear(); canvas->tiles.clear(); canvas->zoom = std::min(canvas->zoom, 5);
        canvas->error.clear(); attribution->setText(world_credit()); dirty = true; canvas->update();
      });
    auto * anchor = new QPushButton(QStringLiteral("设置局部坐标基准…"), owner);
    anchor->setObjectName("geoSetDatum"); layout->addWidget(anchor);
    QObject::connect(anchor, &QPushButton::clicked, owner, [this] {edit_datum();});
    canvas = new GeoCanvas(owner); canvas->changed = [this] {dirty = true;}; layout->addWidget(
      canvas, 1);
    status = new QLabel(owner); status->setObjectName("geographicStatus");
    status->setWordWrap(true);
    status->setTextFormat(Qt::PlainText); layout->addWidget(status);
    attribution = new QLabel(world_credit(), owner); attribution->setWordWrap(true);
    attribution->setTextFormat(Qt::PlainText); layout->addWidget(attribution);
    auto * timer = new QTimer(owner); timer->setInterval(100);
    QObject::connect(timer, &QTimer::timeout, owner, [this] {tick();}); timer->start(); refresh();
  }
  QString world_credit() const
  {
    return QStringLiteral(
      "Natural Earth 5.1.2 · 公共领域 · 1:1.1 亿陆地轮廓，"
      "仅供全球位置参考。\n离线栅格采用 WGS84 / Web Mercator；"
      "网纹区域没有缓存，不联网补图。");
  }
  void refresh()
  {
    canvas->gps.reset(); canvas->local.reset(); canvas->heading.reset();
    QString detail = QStringLiteral("GPS 不可用；等待有效地理定位。");
    if (state) {
      const auto & v = state->state.sdk.value;
      const bool fresh = state->state.sdk.received && state->state.sdk.fresh &&
        display_quality(
        state->state.evidence, *state, now, 1,
        true) == bridge::Quality::ValidAtObservation;
      if (fresh && state->state.gps_valid && v.gps_valid &&
        std::isfinite(v.latitude) && std::isfinite(v.longitude) && std::abs(v.longitude) <= 180)
      {
        if (std::abs(v.latitude) <= 85.0511287798066) {
          canvas->gps = geo::mercator(v.latitude, v.longitude);
          detail = QStringLiteral("GPS · WGS84 %1°, %2°").arg(v.latitude, 0, 'f', 6).arg(
            v.longitude, 0, 'f', 6);
        } else {detail = QStringLiteral("GPS 位于极区，当前墨卡托底图无法显示。");}
      }
      if (datum && fresh && state->state.position_valid && v.pose_valid &&
        QString::fromStdString(v.header.frame_id) == datum->frame)
      {
        try {
          const auto p = geo::to_geodetic(*datum, {v.position_x, v.position_y, v.position_z});
          canvas->local = geo::mercator(p.latitude, p.longitude);
          const auto instruments = flight_instruments(*state, now);
          if (instruments.attitude_valid) {
            canvas->heading = enu_heading_degrees(v.yaw + datum->yaw_rad);
          }
        } catch (const std::exception & e) {
          detail += QStringLiteral("\n局部投影不可用：") + e.what();
        }
      }
    }
    detail += datum ? QStringLiteral(
      "\n基准：%1；证据：%2；椭球高，不是海拔。\n黄色机"
      "位只依赖该人工基准，未自动与 GPS 对齐。").arg(
      datum->frame, datum->evidence) :
      QStringLiteral("\n未设置局部坐标基准；局部 ENU 与 GPS 分别显示。");
    status->setText(detail); owner->setProperty("gps_visible", canvas->gps.has_value());
    owner->setProperty("local_geographic_visible", canvas->local.has_value());
    owner->setProperty("datum_configured", datum.has_value()); canvas->update();
  }
  void edit_datum()
  {
    QDialog dialog(owner); dialog.setWindowTitle(QStringLiteral("只读地图 · 显式 WGS84 基准"));
    auto * form = new QFormLayout(&dialog);
    auto * explanation =
      new QLabel(
      QStringLiteral(
        "填入测量得到的对应点和真东方向。局部框架名称相"
        "同并不能证明原点或朝向相同。切换连接或时间域会"
        "清除本基准。"), &dialog);
    explanation->setWordWrap(true); form->addRow(explanation);
    std::vector<QDoubleSpinBox *> fields;
    const QStringList labels{QStringLiteral("纬度 °"), QStringLiteral("经度 °"), QStringLiteral(
        "WGS84 椭球高 m"),
      QStringLiteral("对应局部 X m"), QStringLiteral("对应局部 Y m"),
      QStringLiteral("对应局部 Z m"),
      QStringLiteral("局部 X 相对真东逆时针角 °")};
    for (int i = 0; i < labels.size(); ++i) {
      auto * f = new QDoubleSpinBox(&dialog); f->setDecimals(i < 2 ? 8 : 4);
      const double limit = i == 0 ? 90 : i == 1 || i == 6 ? 180 : i == 2 ? 100000 : 1e6;
      f->setRange(-limit, limit); fields.push_back(f); form->addRow(labels[i], f);
    }
    auto * frame = new QLineEdit(&dialog); frame->setMaxLength(128);
    auto * evidence = new QLineEdit(&dialog); evidence->setMaxLength(1024);
    form->addRow(QStringLiteral("局部 frame"), frame); form->addRow(
      QStringLiteral(
        "测量／标定记录"), evidence);
    auto * error = new QLabel(&dialog); error->setWordWrap(true);
    error->setTextFormat(Qt::PlainText); form->addRow(error);
    auto * buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    form->addRow(buttons);
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    QObject::connect(
      buttons, &QDialogButtonBox::accepted, &dialog, [&] {
        geo::Datum d{{fields[0]->value(), fields[1]->value(), fields[2]->value()},
          {fields[3]->value(), fields[4]->value(), fields[5]->value()},
          fields[6]->value() * pi / 180,
          frame->text().trimmed(), evidence->text().trimmed()};
        try {owner->set_datum(d); dialog.accept();} catch (const std::exception & e) {
          error->setText(e.what());
        }
      });
    dialog.exec();
  }
  void tick()
  {
    if (importing.valid() &&
      importing.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      import_button->setEnabled(true);
      try {
        canvas->path = importing.get(); geo::Tiles map(canvas->path); const auto m = map.metadata();
        canvas->tiles.clear(); canvas->zoom = std::clamp(
          canvas->zoom, m.minimum_zoom,
          m.maximum_zoom);
        attribution->setText(
          m.name.left(128) + " · " + m.attribution.left(1024) +
          QStringLiteral("\n已缓存离线栅格；网纹区域无覆盖，不联网补图。"));
        canvas->error.clear(); dirty = true;
      } catch (const std::exception & e) {canvas->error = QStringLiteral("导入失败：") + e.what();}
    }
    if (loading.valid() && loading.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      const auto batch = loading.get();
      if (batch.path == canvas->path) {
        canvas->tiles = batch.images; canvas->error = batch.error; canvas->update();
        owner->setProperty("offline_tiles_loaded", static_cast<int>(batch.images.size()));
      }
    }
    if (dirty && !canvas->path.isEmpty() && !loading.valid()) {
      dirty = false; const auto needed = canvas->needed(); const auto path = canvas->path;
      loading = std::async(
        std::launch::async, [needed, path] {
          TileBatch result; result.path = path;
          try {
            geo::Tiles map(path);
            for (const auto & key : needed) {
              const auto tile = map.tile(std::get<0>(key), std::get<1>(key), std::get<2>(key));
              if (!tile.isNull()) {result.images.emplace(key, tile);}
            }
            if (result.images.empty()) {
              result.error = QStringLiteral("此区域／缩放级别没有离线瓦片");
            }
          } catch (const std::exception & e) {result.images.clear(); result.error = e.what();}
          return result;
        });
    }
  }
};
GeographicView::GeographicView(QWidget * parent, QString cache_directory)
: QWidget(parent), impl_(std::make_unique<Impl>(this, std::move(cache_directory))) {}
GeographicView::~GeographicView() = default;
void GeographicView::snapshot(std::shared_ptr<const bridge::UiSnapshot> value, double now)
{
  auto & p = *impl_;
  if (!value || !value->running || value->stopped || value->time_epoch != p.epoch ||
    value->observation_session_id != p.session) {p.datum.reset();}
  p.session = value ? value->observation_session_id : 0; p.epoch = value ? value->time_epoch : 0;
  p.state = std::move(value); p.now = now; p.refresh();
}
void GeographicView::set_datum(const geo::Datum & datum)
{
  auto & p = *impl_;
  if (!geo::valid(datum) || !p.state || !p.state->running || p.state->stopped ||
    QString::fromStdString(p.state->state.sdk.value.header.frame_id) != datum.frame)
  {
    throw std::runtime_error("基准无效，或与当前连接的局部 frame 不一致");
  }
  p.datum = datum; p.refresh();
}
void GeographicView::import_map(const QString & path)
{
  auto & p = *impl_; if (p.importing.valid()) {return;}
  p.import_button->setEnabled(false); p.canvas->error = QStringLiteral("正在校验并缓存离线底图…");
  const auto directory = p.cache;
  p.importing = std::async(
    std::launch::async, [path, directory] {
      return geo::Tiles::cache(path, directory);
    });
}
}  // namespace easyfly_desktop

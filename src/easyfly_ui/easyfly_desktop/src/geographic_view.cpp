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
#include <QImageReader>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QNetworkAccessManager>
#include <QNetworkDiskCache>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QElapsedTimer>
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
#include <QSignalBlocker>
#include <QStandardPaths>
#include <QTimer>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <future>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>
#include "easyfly_desktop/geographic_view.hpp"
#include "easyfly_desktop/presentation.hpp"
#include "easyfly_desktop/overview_task.hpp"
#include "easyfly_desktop/task_step_info.hpp"
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
QImage bounded_tile(const QByteArray & bytes)
{
  QBuffer buffer; buffer.setData(bytes); buffer.open(QIODevice::ReadOnly);
  QImageReader reader(&buffer);
  const auto size = reader.size();
  if (size.width() <= 0 || size.height() <= 0 || size.width() > 512 || size.height() > 512) {
    return {};
  }
  return reader.read();
}
class GeoCanvas : public QWidget
{
public:
  QPointF center{.5, .5}; int zoom{1};
  std::optional<QPointF> gps, local;
  std::optional<double> heading;
  QString path, error, source{"world"};
  bool follow{true};
  struct Trace {QPointF point; uint64_t segment;};
  std::vector<Trace> trajectory;
  struct TaskPoint {QPointF point; int step; QString type; bool current, completed;};
  std::vector<TaskPoint> task;
  std::vector<std::pair<std::size_t, std::size_t>> links;
  bool task_fresh{false};
  std::optional<QPointF> target;
  struct Area {QPolygonF polygon; bool forbidden; QString title;};
  std::vector<Area> areas;
  std::map<TileKey, QImage> tiles;
  std::vector<QVector<QPointF>> land;
  std::function<void()> changed;
  explicit GeoCanvas(QWidget * parent)
  : QWidget(parent)
  {
    initialize_geographic_resources();
    setObjectName("geographicCanvas"); setFocusPolicy(Qt::StrongFocus);
    setAccessibleName(QStringLiteral("地理地图；方向键平移，加减键缩放"));
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
    follow = false; center += delta / scale(); center.setX(center.x() - std::floor(center.x()));
    center.setY(std::clamp(center.y(), 0., 1.)); if (changed) {changed();} update();
  }
  void zoom_by(int delta)
  {
    zoom = std::clamp(zoom + delta, 0, source == "world" ? 5 : 19); if (changed) {
      changed();
    }
    update();
  }
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
    if (source == "world") {
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
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(QColor("#6c7f91"), 2));
    for (std::size_t i = 1; i < trajectory.size(); ++i) {
      if (trajectory[i].segment == trajectory[i - 1].segment &&
        std::abs(trajectory[i].point.x() - trajectory[i - 1].point.x()) < .5)
      {p.drawLine(pixel(trajectory[i - 1].point), pixel(trajectory[i].point));}
    }
    if (!gps && !trajectory.empty()) {
      const auto historical = pixel(trajectory.back().point);
      p.setPen(QPen(QColor("#6c7f91"), 2)); p.setBrush(Qt::NoBrush);
      p.drawEllipse(historical, 5, 5);
      p.drawText(historical + QPointF(8, -8), QStringLiteral("历史 GPS"));
    }
    p.setPen(QPen(QColor("#9674bd"), 2, Qt::DashLine));
    p.setBrush(QColor(150, 116, 189, 30));
    for (const auto & area : areas) {
      QPolygonF polygon; for (const auto & point : area.polygon) {polygon.append(pixel(point));}
      p.setPen(QPen(area.forbidden ? QColor("#b72b35") : QColor("#9674bd"), 2, Qt::DashLine));
      p.setBrush(area.forbidden ? QColor(183, 43, 53, 40) : QColor(150, 116, 189, 30));
      p.drawPolygon(polygon);
      p.drawText(polygon.boundingRect().topLeft() + QPointF(4, 16), area.title);
    }
    p.setPen(QPen(QColor("#9674bd"), 2, Qt::DashLine));
    for (const auto & link : links) {
      if (link.first < task.size() && link.second < task.size() &&
        std::abs(task[link.first].point.x() - task[link.second].point.x()) < .5)
      {p.drawLine(pixel(task[link.first].point), pixel(task[link.second].point));}
    }
    for (std::size_t i = 0; i < task.size(); ++i) {
      const auto & item = task[i]; const auto point = pixel(item.point);
      p.setPen(
        QPen(
          item.current ? QColor("#a94400") : task_fresh ? QColor("#754b9e") : QColor(
            "#7b8188"), 2));

      p.setBrush(item.completed ? QColor("#d4e8d6") : QColor("#fff5db"));
      p.drawEllipse(point, item.current ? 10 : 7, item.current ? 10 : 7);
      p.drawText(point + QPointF(10, -10), QString::number(item.step + 1) + " " + item.type);
    }
    if (target) {
      const auto point = pixel(*target); p.setBrush(Qt::NoBrush);
      p.setPen(QPen(QColor("#c34a00"), 3)); p.drawEllipse(point, 15, 15);
      p.drawText(point + QPointF(18, 15), QStringLiteral("当前任务目标"));
    }
    if (gps) {
      const auto point = pixel(*gps); p.setPen(QPen(QColor("#144c8c"), 2));
      p.setBrush(QColor("#ffffff")); p.drawEllipse(point, 7, 7);
      p.drawText(point + QPointF(11, -7), QStringLiteral("飞机 · GPS"));
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
  QNetworkAccessManager network;
  QNetworkDiskCache * network_cache;
  QElapsedTimer request_window;
  int window_requests{0}, total_requests{0};
  std::set<TileKey> pending;
  std::map<TileKey, qint64> retry_after;
  bool compact{false};
  QString offline_path;
  QString task_note;
  OverviewTaskOverlay overlay;
  QComboBox * source_select;
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
    network_cache = new QNetworkDiskCache(&network);
    network_cache->setCacheDirectory(cache + "/osm-tiles");
    network_cache->setMaximumCacheSize(256 * 1024 * 1024);
    network.setCache(network_cache); request_window.start();
    auto * layout = new QVBoxLayout(owner); layout->setContentsMargins(4, 4, 4, 4);
    auto * toolbar = new QHBoxLayout; layout->addLayout(toolbar);
    auto add = [&](const QString & title, const char * name, const std::function<void()> & action) {
        auto * b = new QPushButton(title, owner); b->setObjectName(name); b->setMinimumHeight(32);
        toolbar->addWidget(b); QObject::connect(b, &QPushButton::clicked, owner, action); return b;
      };
    add(QStringLiteral("＋"), "geoZoomIn", [this] {canvas->zoom_by(1);})->setAccessibleName(
      QStringLiteral("放大地图"));
    add(QStringLiteral("−"), "geoZoomOut", [this] {canvas->zoom_by(-1);})->setAccessibleName(
      QStringLiteral("缩小地图"));
    add(
      QStringLiteral(
        "定位"), "geoCenterGps", [this] {
        if (canvas->gps) {
          canvas->center = *canvas->gps;
          if (canvas->source == "world") {canvas->zoom = 5;} else if (canvas->source == "osm") {
            canvas->zoom = 16;
          }
          dirty = true; canvas->update();
        }
      });
    auto * follow = add(
      QStringLiteral("跟随"), "geoFollow", [this] {
        canvas->follow = true; refresh();
      });
    follow->setToolTip(QStringLiteral("跟随当前有效 GPS；拖动或方向键平移可暂停跟随"));
    add(QStringLiteral("地图设置…"), "geoSettings", [this] {settings();});
    source_select = new QComboBox(owner); source_select->setObjectName("geoSource");
    source_select->setAccessibleName(QStringLiteral("地图底图来源"));
    source_select->addItem(QStringLiteral("世界轮廓"), "world");
    source_select->addItem(QStringLiteral("街道地图 · 在线"), "osm");
    source_select->addItem(QStringLiteral("离线底图"), "offline");
    toolbar->addWidget(source_select);
    QObject::connect(
      source_select, QOverload<int>::of(&QComboBox::currentIndexChanged), owner,
      [this](int index) {owner->set_map_source(source_select->itemData(index).toString());});
    toolbar->addStretch();
    // Kept as named actions for automation; detailed controls live in the dialog.
    import_button = new QPushButton(owner); import_button->setObjectName("geoImport");
    import_button->hide();
    QObject::connect(import_button, &QPushButton::clicked, owner, [this] {choose_map();});
    auto * world = new QPushButton(owner); world->setObjectName("geoWorld"); world->hide();
    QObject::connect(world, &QPushButton::clicked, owner, [this] {owner->set_map_source("world");});
    auto * anchor = new QPushButton(owner); anchor->setObjectName("geoSetDatum"); anchor->hide();
    QObject::connect(anchor, &QPushButton::clicked, owner, [this] {edit_datum();});
    canvas = new GeoCanvas(owner); canvas->changed = [this] {dirty = true;}; layout->addWidget(
      canvas, 1);
    status = new QLabel(owner); status->setObjectName("geographicStatus");
    status->setWordWrap(true);
    status->setTextFormat(Qt::PlainText); layout->addWidget(status);
    attribution = new QLabel(world_credit(), owner); attribution->setWordWrap(true);
    attribution->setTextFormat(Qt::RichText); attribution->setOpenExternalLinks(true);
    layout->addWidget(attribution);
    auto * timer = new QTimer(owner); timer->setInterval(100);
    QObject::connect(timer, &QTimer::timeout, owner, [this] {tick();}); timer->start(); refresh();
  }
  void notify() {if (owner->configuration_changed) {owner->configuration_changed();}}
  void choose_map()
  {
    const auto file = QFileDialog::getOpenFileName(
      owner, QStringLiteral("选择栅格 MBTiles"),
      cache, "MBTiles (*.mbtiles)");
    if (!file.isEmpty()) {owner->import_map(file);}
  }
  void settings()
  {
    QDialog dialog(owner); dialog.setWindowTitle(QStringLiteral("地图设置"));
    auto * layout = new QFormLayout(&dialog);
    auto * source = new QComboBox(&dialog); source->setObjectName("geoMapSource");
    source->addItem(QStringLiteral("内置世界轮廓 · 离线"), "world");
    source->addItem(QStringLiteral("导入的 MBTiles · 离线"), "offline");
    source->addItem(QStringLiteral("OpenStreetMap · 在线＋缓存"), "osm");
    source->setCurrentIndex(source->findData(canvas->source));
    layout->addRow(QStringLiteral("底图"), source);
    auto * note = new QLabel(
      QStringLiteral(
        "在线底图仅请求当前可见瓦片，不上传飞行遥测。请求中的瓦片范围仍可反映查看区域。"
        "断网可显示已有缓存；网纹表示无缓存。缓存上限 256 MiB。"), &dialog);
    note->setWordWrap(true); layout->addRow(note);
    auto * load = new QPushButton(QStringLiteral("导入 MBTiles…"), &dialog);
    layout->addRow(load); QObject::connect(
      load, &QPushButton::clicked, &dialog, [this, source] {
        choose_map(); source->setCurrentIndex(source->findData("offline"));
      });
    auto * anchor = new QPushButton(QStringLiteral("设置实测局部坐标基准…"), &dialog);
    layout->addRow(anchor); QObject::connect(
      anchor, &QPushButton::clicked, &dialog,
      [this] {edit_datum();});
    auto * clear = new QPushButton(QStringLiteral("清除局部坐标基准"), &dialog);
    layout->addRow(clear); QObject::connect(
      clear, &QPushButton::clicked, &dialog,
      [this] {owner->clear_datum();});
    auto * credit = new QLabel(attribution->text(), &dialog); credit->setWordWrap(true);
    layout->addRow(credit);
    auto * buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    layout->addRow(buttons);
    QObject::connect(
      buttons, &QDialogButtonBox::accepted, &dialog, [&] {
        owner->set_map_source(source->currentData().toString()); dialog.accept();
      });
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    dialog.resize(480, 360); dialog.exec();
  }
  QUrl tile_url(const TileKey & key) const
  {
    return QUrl(
      QString("https://tile.openstreetmap.org/%1/%2/%3.png")
      .arg(std::get<0>(key)).arg(std::get<1>(key)).arg(std::get<2>(key)));
  }
  void online_tiles()
  {
    if (request_window.elapsed() >= 60000) {window_requests = 0; request_window.restart();}
    const auto needed = canvas->needed();
    for (auto it = canvas->tiles.begin(); it != canvas->tiles.end(); ) {
      if (std::find(needed.begin(), needed.end(), it->first) == needed.end()) {
        it = canvas->tiles.erase(it);
      } else {++it;}
    }
    for (const auto & key : needed) {
      if (pending.count(key)) {continue;}
      const auto url = tile_url(key);
      if (canvas->tiles.count(key) && network_cache->metaData(url).expirationDate() >=
        QDateTime::currentDateTimeUtc()) {continue;}
      // Read stale cached tiles too, so an offline connection never hides a prior download.
      std::unique_ptr<QIODevice> cached(network_cache->data(url));
      if (cached) {
        const auto image = bounded_tile(cached->read(1024 * 1024));
        if (!image.isNull() && image.width() <= 512 && image.height() <= 512) {
          canvas->tiles.emplace(key, image);
          const auto metadata = network_cache->metaData(url);
          if (metadata.expirationDate() >= QDateTime::currentDateTimeUtc()) {continue;}
        }
      }
      const auto now_ms = QDateTime::currentMSecsSinceEpoch();
      if (pending.size() >= 4 || window_requests >= 120 || total_requests >= 10000 ||
        (retry_after.count(key) && retry_after.at(key) > now_ms)) {continue;}
      QNetworkRequest request(url);
      request.setRawHeader("User-Agent", "EasyFlyDesktop/1.0 (robotics ground station)");
      request.setAttribute(
        QNetworkRequest::RedirectPolicyAttribute,
        QNetworkRequest::ManualRedirectPolicy);
      request.setTransferTimeout(7000); request.setMaximumRedirectsAllowed(0);
      auto * reply = network.get(request); pending.insert(key); ++window_requests; ++total_requests;
      owner->setProperty("online_request_count", total_requests);
      QObject::connect(
        reply, &QNetworkReply::readyRead, owner, [reply] {
          if (reply->bytesAvailable() > 1024 * 1024) {reply->abort();}
        });
      QObject::connect(
        reply, &QNetworkReply::finished, owner, [this, reply, key, url] {
          pending.erase(key); retry_after[key] = QDateTime::currentMSecsSinceEpoch() + 60000;
          const auto bytes = reply->read(1024 * 1024);
          const auto image = bounded_tile(bytes);
          if (reply->error() == QNetworkReply::NoError && !image.isNull() &&
          image.width() <= 512 && image.height() <= 512)
          {
            if (canvas->source == "osm") {canvas->tiles[key] = image; canvas->error.clear();}
            auto metadata = network_cache->metaData(url);
            const auto minimum = QDateTime::currentDateTimeUtc().addDays(7);
            if (metadata.isValid() && !metadata.expirationDate().isValid()) {
              metadata.setExpirationDate(minimum); network_cache->updateMetaData(metadata);
            }
          } else if (canvas->source == "osm") {
            canvas->error = QStringLiteral("在线底图暂不可用；保留已有缓存");
          }
          reply->deleteLater(); canvas->update();
        });
    }
    canvas->update();
  }
  ~Impl()
  {
    for (auto * reply : network.findChildren<QNetworkReply *>()) {
      QObject::disconnect(reply, nullptr, owner, nullptr); reply->abort();
    }
  }
  QString world_credit() const
  {
    return QStringLiteral("Natural Earth 5.1.2 · 公共领域 · 世界轮廓");
  }
  void refresh_task()
  {
    canvas->task.clear(); canvas->areas.clear(); canvas->links.clear(); canvas->target.reset();
    canvas->task_fresh = overlay.bound && overlay.fresh; task_note.clear();
    owner->setProperty("geographic_task_point_count", 0);
    owner->setProperty("geographic_task_target_visible", false);
    if (!overlay.geometry.accepted) {return;}
    if (overlay.geometry.points.size() > 512 || overlay.geometry.areas.size() > 128) {
      task_note = QStringLiteral("任务路径超过地图显示预算"); return;
    }
    if (!datum) {task_note = QStringLiteral("任务路径未投影：尚无实测局部坐标基准"); return;}
    if (overlay.frame.isEmpty() || overlay.frame != datum->frame) {
      task_note = QStringLiteral("任务路径未投影：任务与基准 frame 不匹配"); return;
    }
    try {
      auto project = [this](double x, double y, double z) {
          const auto global = geo::to_geodetic(*datum, {x, y, z});
          return geo::mercator(global.latitude, global.longitude);
        };
      for (const auto & point : overlay.geometry.points) {
        canvas->task.push_back(
          {project(point.x, point.y, point.z), static_cast<int>(point.step),
            task_step_title(point.type),
            canvas->task_fresh && static_cast<int>(point.step) == overlay.current_step,
            static_cast<int>(point.step) < overlay.completed_steps});
      }
      for (const auto & link : overlay.geometry.links) {
        canvas->links.emplace_back(link.from, link.to);
      }
      for (const auto & area : overlay.geometry.areas) {
        canvas->areas.push_back(
          {QPolygonF(
              QVector<QPointF>{project(area.x, area.y, area.z),
                project(area.x + area.width, area.y, area.z),
                project(area.x + area.width, area.y + area.height, area.z),
                project(area.x, area.y + area.height, area.z)}), area.forbidden,
            area.forbidden ? QStringLiteral("禁飞区域") : area.type == "explore" ?
            QStringLiteral("探索范围") : QStringLiteral("搜索区域")});
      }
      if (overlay.current_target && canvas->task_fresh) {
        const auto & target = *overlay.current_target;
        canvas->target = project(target[0], target[1], target[2]);
      }
      if (!canvas->task_fresh) {task_note = QStringLiteral("历史任务路径 · ") + overlay.reason;}
    } catch (const std::exception & error) {
      canvas->task.clear(); canvas->links.clear(); canvas->areas.clear(); canvas->target.reset();
      task_note = QStringLiteral("任务路径投影不可用：") + error.what();
    }
    owner->setProperty("geographic_task_point_count", static_cast<int>(canvas->task.size()));
    owner->setProperty("geographic_task_target_visible", canvas->target.has_value());
  }
  void refresh()
  {
    const bool had_gps = canvas->gps.has_value();
    canvas->gps.reset(); canvas->local.reset(); canvas->heading.reset();
    canvas->trajectory.clear();
    QString detail = QStringLiteral("GPS 不可用；等待有效地理定位。");
    if (state) {
      const auto & v = state->state.sdk.value;
      const bool fresh = state->state.sdk.received && state->state.sdk.fresh &&
        display_quality(
        state->state.evidence, *state, now, 1,
        true) == bridge::Quality::ValidAtObservation;
      const auto & global = state->global_position;
      const bool global_fresh = global.value && display_quality(
        global.evidence, *state, now,
        2.0, true) == bridge::Quality::ValidAtObservation;
      if (global_fresh && geo::valid(
          geo::Geodetic{global.value->latitude, global.value->longitude,
            global.value->altitude}))
      {
        if (std::abs(global.value->latitude) <= 85.0511287798066) {
          canvas->gps = geo::mercator(global.value->latitude, global.value->longitude);
          detail = QStringLiteral("GPS · %1°, %2°").arg(global.value->latitude, 0, 'f', 6).arg(
            global.value->longitude, 0, 'f', 6);
        } else {detail = QStringLiteral("GPS 位于极区，当前墨卡托底图无法显示。");}
      }
      canvas->trajectory.clear();
      const auto first = state->global_trajectory.size() > 4096 ?
        state->global_trajectory.size() - 4096 : 0;
      for (std::size_t i = first; i < state->global_trajectory.size(); ++i) {
        const auto & point = state->global_trajectory[i];
        if (std::isfinite(point.steady) && now - point.steady <= 300 &&
          now >= point.steady &&
          geo::valid(geo::Geodetic{point.latitude, point.longitude, point.altitude}) &&
          std::abs(point.latitude) <= 85.0511287798066)
        {
          canvas->trajectory.push_back(
            {geo::mercator(
                point.latitude,
                point.longitude), point.segment});
        }
      }
      if (!canvas->gps && !canvas->trajectory.empty()) {
        detail += QStringLiteral(" · 灰线仅为历史 GPS 轨迹");
      }
      if (canvas->gps && canvas->follow) {
        canvas->center = *canvas->gps;
        if (!had_gps && canvas->source == "osm" && canvas->zoom < 12) {canvas->zoom = 16;}
        dirty = true;
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
    refresh_task();
    if (!task_note.isEmpty()) {detail += "\n" + task_note;}
    status->setToolTip(detail);
    status->setText(
      detail.section(
        '\n', 0,
        0) + (task_note.isEmpty() ? QString() : " · " + task_note));
    attribution->setToolTip(attribution->text());
    if (compact && canvas->source == "world") {attribution->setText("Natural Earth · 公共领域");}
    owner->setProperty("gps_available", canvas->gps.has_value());
    owner->setProperty("global_trajectory_count", static_cast<int>(canvas->trajectory.size()));
    owner->setProperty("gps_visible", canvas->gps.has_value());
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
    if (datum) {
      const double values[] = {datum->origin.latitude, datum->origin.longitude,
        datum->origin.ellipsoid_height, datum->local_origin[0], datum->local_origin[1],
        datum->local_origin[2], datum->yaw_rad * 180 / pi};
      for (std::size_t i = 0; i < fields.size(); ++i) {fields[i]->setValue(values[i]);}
      frame->setText(datum->frame); evidence->setText(datum->evidence);
    }
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
        canvas->path = importing.get(); offline_path = canvas->path;
        canvas->source = "offline";
        {QSignalBlocker block(source_select); source_select->setCurrentIndex(
            source_select->findData(
              "offline"));}
        geo::Tiles map(canvas->path); const auto m = map.metadata();
        canvas->tiles.clear(); canvas->zoom = std::clamp(
          canvas->zoom, m.minimum_zoom,
          m.maximum_zoom);
        attribution->setText(
          m.name.left(128).toHtmlEscaped() + " · " + m.attribution.left(1024).toHtmlEscaped() +
          QStringLiteral("\n已缓存离线栅格；网纹区域无覆盖，不联网补图。"));
        canvas->error.clear(); dirty = true; notify();
      } catch (const std::exception & e) {canvas->error = QStringLiteral("导入失败：") + e.what();}
    }
    if (loading.valid() && loading.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      const auto batch = loading.get();
      if (canvas->source == "offline" && batch.path == canvas->path) {
        canvas->tiles = batch.images; canvas->error = batch.error; canvas->update();
        owner->setProperty("offline_tiles_loaded", static_cast<int>(batch.images.size()));
      }
    }
    if (owner->isVisible() && canvas->source == "osm") {online_tiles();}
    if (dirty && canvas->source == "offline" && !canvas->path.isEmpty() && !loading.valid()) {
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
    value->observation_session_id != p.session)
  {
    const bool had_datum = p.datum.has_value(); p.datum.reset();
    if (had_datum) {p.notify();}
  }
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
  p.datum = datum; p.refresh(); p.notify();
}
void GeographicView::import_map(const QString & path)
{
  auto & p = *impl_; if (p.importing.valid()) {return;}
  if (path == p.offline_path) {set_map_source("offline"); return;}
  p.import_button->setEnabled(false); p.canvas->error = QStringLiteral("正在校验并缓存离线底图…");
  const auto directory = p.cache;
  p.importing = std::async(
    std::launch::async, [path, directory] {
      return geo::Tiles::cache(path, directory);
    });
}
bool GeographicView::gps_available() const {return impl_->canvas->gps.has_value();}
void GeographicView::set_compact(bool compact)
{
  impl_->compact = compact; impl_->refresh();
}
std::optional<geo::Datum> GeographicView::datum() const {return impl_->datum;}
void GeographicView::clear_datum()
{
  if (!impl_->datum) {return;}
  impl_->datum.reset(); impl_->refresh(); impl_->notify();
}
QString GeographicView::map_source() const {return impl_->canvas->source;}
QString GeographicView::offline_map_path() const {return impl_->offline_path;}
void GeographicView::set_map_source(const QString & source)
{
  auto & p = *impl_; if (source == p.canvas->source) {return;}
  if (source != "world" && source != "offline" && source != "osm") {
    throw std::runtime_error("unknown map source");
  }
  if (source == "offline" && p.offline_path.isEmpty()) {
    {QSignalBlocker block(p.source_select);
      p.source_select->setCurrentIndex(p.source_select->findData(p.canvas->source));}
    p.canvas->error = QStringLiteral("请先导入 MBTiles 离线底图"); p.canvas->update(); return;
  }
  {QSignalBlocker block(p.source_select);
    p.source_select->setCurrentIndex(p.source_select->findData(source));}
  p.canvas->source = source;
  if (source != "osm") {
    for (auto * reply : p.network.findChildren<QNetworkReply *>()) {
      reply->abort();
    }
  }
  p.canvas->path = source == "offline" ? p.offline_path : QString();
  p.canvas->tiles.clear(); p.canvas->error.clear(); p.dirty = true;
  if (source == "world") {
    p.canvas->zoom = std::min(p.canvas->zoom, 5); p.attribution->setText(p.world_credit());
  } else if (source == "osm") {
    if (p.canvas->gps) {p.canvas->center = *p.canvas->gps; p.canvas->zoom = 16;}
    p.attribution->setText(
      QStringLiteral(
        "<a href=\"https://www.openstreetmap.org/copyright\">© OpenStreetMap contributors</a>"));
  } else {
    geo::Tiles map(p.offline_path); const auto metadata = map.metadata();
    p.canvas->zoom = std::clamp(p.canvas->zoom, metadata.minimum_zoom, metadata.maximum_zoom);
    p.attribution->setText(
      metadata.name.toHtmlEscaped() + " · " + metadata.attribution.toHtmlEscaped());
  }
  p.canvas->update(); p.notify();
}
void GeographicView::set_task_overlay(const OverviewTaskOverlay & overlay)
{
  impl_->overlay = overlay; impl_->refresh();
}
}  // namespace easyfly_desktop

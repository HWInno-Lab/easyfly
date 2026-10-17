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

#include "easyfly_desktop/playback_window.hpp"
#include <QAbstractTableModel>
#include <QCloseEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSlider>
#include <QSplitter>
#include <QTableView>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>
#include <limits>
#include "theme.hpp"
namespace easyfly_desktop
{
namespace
{
class EventModel : public QAbstractTableModel
{
public:
  std::shared_ptr<const PlaybackRecord> record;
  explicit EventModel(QObject * parent)
  : QAbstractTableModel(parent) {}
  void replace(std::shared_ptr<const PlaybackRecord> value)
  {
    beginResetModel(); record = std::move(value); endResetModel();
  }
  int rowCount(const QModelIndex & parent = {}) const override
  {
    return parent.isValid() || !record ? 0 : static_cast<int>(record->events.size());
  }
  int columnCount(const QModelIndex & parent = {}) const override {return parent.isValid() ? 0 : 4;}
  QVariant headerData(int section, Qt::Orientation direction, int role) const override
  {
    const QStringList headers{QStringLiteral("写入 UTC"), QStringLiteral("历史事件类型"),
      QStringLiteral("归属（不猜测）"), QStringLiteral("已知执行 ID")};
    return role == Qt::DisplayRole && direction == Qt::Horizontal ?
           QVariant(headers.value(section)) : QVariant{};
  }
  QVariant data(const QModelIndex & index, int role) const override
  {
    if (!record || !index.isValid() || index.row() >= rowCount() || role != Qt::DisplayRole) {
      return {};
    }
    const auto & event = record->events[index.row()];
    const QStringList keys{"written_utc", "kind", "attribution", "execution_id"};
    const auto value = event[keys.value(index.column())].toString();
    return value.isEmpty() ? QStringLiteral("未记录") : value.left(512);
  }
};
QLabel * plain(QWidget * owner, const char * name)
{
  auto * label = new QLabel(owner); label->setObjectName(name); label->setWordWrap(true);
  label->setTextFormat(Qt::PlainText); label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum); return label;
}
QString recorded(const QJsonObject & object, const char * key)
{
  const auto value = object[key].toString();
  return value.isEmpty() ? QStringLiteral("未记录") : value;
}
}  // namespace
struct PlaybackWindow::Impl
{
  PlaybackWindow * owner;
  PlaybackWorker worker;
  QLineEdit * root, * id;
  QPushButton * open, * choose, * play, * previous, * next;
  QSlider * slider;
  QLabel * summary, * frame_info, * image;
  QPlainTextEdit * details;
  QTableView * events;
  EventModel * model;
  bool playing{false}, closing{false};
  uint64_t rendered{std::numeric_limits<uint64_t>::max()};
  QSize rendered_size;
  std::shared_ptr<const PlaybackRecord> displayed;
  explicit Impl(PlaybackWindow * window)
  : owner(window)
  {
    owner->setObjectName("playbackWindow");
    owner->setWindowTitle(QStringLiteral("EasyFly — 本地历史回放（不连接飞机）"));
    owner->setPalette(theme::palette()); owner->setStyleSheet(theme::stylesheet());
    owner->resize(1280, 800); owner->setMinimumSize(1024, 720);
    auto * layout = new QVBoxLayout(owner);
    auto * banner = plain(owner, "warningBanner");
    banner->setText(
      QStringLiteral(
        "历史回放 · 不连接 ROS / 飞机 · 无任务、参数或飞行控制入口\n"
        "仅 UI 抽样图像；10fps 固定逐帧播放，不补丢帧，不表示真实时间。"));
    layout->addWidget(banner);
    auto * split = new QSplitter(Qt::Horizontal, owner);
    auto * controls = new QWidget(split); auto * left = new QVBoxLayout(controls);
    root = new QLineEdit(controls); root->setObjectName("playbackDirectory");
    root->setPlaceholderText(QStringLiteral("专用记录根目录绝对路径")); left->addWidget(root);
    id = new QLineEdit(controls); id->setObjectName("playbackId");
    id->setPlaceholderText(QStringLiteral("记录 UUID")); left->addWidget(id);
    choose = new QPushButton(QStringLiteral("选择 UUID 会话目录…"), controls);
    choose->setObjectName("playbackChoose"); left->addWidget(choose);
    open = new QPushButton(QStringLiteral("只读校验并打开（默认暂停）"), controls);
    open->setObjectName("playbackOpen"); left->addWidget(open);
    summary = plain(controls, "playbackSummary"); left->addWidget(summary);
    frame_info = plain(controls, "playbackFrameInfo"); left->addWidget(frame_info);
    auto * buttons = new QHBoxLayout;
    previous = new QPushButton(QStringLiteral("上一帧"), controls);
    previous->setObjectName("playbackPrevious");
    play = new QPushButton(QStringLiteral("播放 10fps"), controls);
    play->setObjectName("playbackPlay");
    next = new QPushButton(QStringLiteral("下一帧"), controls); next->setObjectName("playbackNext");
    buttons->addWidget(previous); buttons->addWidget(play); buttons->addWidget(next);
    left->addLayout(buttons);
    slider = new QSlider(Qt::Horizontal, controls); slider->setObjectName("playbackSeek");
    left->addWidget(slider);
    auto * notice = plain(controls, "playbackLimitations");
    notice->setText(
      QStringLiteral(
        "source_ns 属于记录相机的时钟域；steady 仅属于录制进程，"
        "UTC 是写入索引。三者不互相等同。\n"
        "事件按原日志顺序显示，保留各自时间字段；"
        "不根据写入 UTC 自动对齐飞行或帧时间。\n"
        "全局事件未归属，不推断任务身份；丢帧计数不是完整上游损失。\n"
        "本版没有保存轨迹、点云或完整飞机遥测，因此这些历史数据不可用。"));
    left->addWidget(notice); left->addStretch();
    auto * scroll = new QScrollArea(split); scroll->setWidget(controls);
    scroll->setWidgetResizable(true); scroll->setMinimumWidth(360);
    auto * right = new QSplitter(Qt::Vertical, split);
    image = new QLabel(QStringLiteral("历史图像 — 请选择已收尾记录"), right);
    image->setObjectName("playbackImage"); image->setAlignment(Qt::AlignCenter);
    image->setMinimumSize(320, 180);
    image->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    auto * log = new QWidget(right); auto * log_layout = new QVBoxLayout(log);
    events = new QTableView(log); events->setObjectName("playbackEvents");
    model = new EventModel(owner); events->setModel(model);
    events->setSelectionBehavior(QAbstractItemView::SelectRows);
    events->setSelectionMode(QAbstractItemView::SingleSelection);
    events->setEditTriggers(QAbstractItemView::NoEditTriggers);
    events->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
    events->horizontalHeader()->setStretchLastSection(true);
    events->setColumnWidth(0, 190); events->setColumnWidth(1, 180);
    log_layout->addWidget(events, 1);
    details = new QPlainTextEdit(log); details->setObjectName("playbackEventDetails");
    details->setReadOnly(true); details->setMaximumHeight(140); log_layout->addWidget(details);
    right->setSizes({420, 260}); split->setSizes({400, 850}); layout->addWidget(split, 1);
    QObject::connect(
      choose, &QPushButton::clicked, owner, [this] {
        const auto selected = QFileDialog::getExistingDirectory(
          owner, QStringLiteral("选择专用记录根目录内的 UUID 会话目录"));
        if (!selected.isEmpty()) {
          const QFileInfo info(selected); root->setText(info.absolutePath());
          id->setText(info.fileName());
        }
      });
    QObject::connect(
      open, &QPushButton::clicked, owner, [this] {
        playing = false; worker.open(root->text(), id->text()); render();
      });
    QObject::connect(play, &QPushButton::clicked, owner, [this] {playing = !playing; render();});
    QObject::connect(
      previous, &QPushButton::clicked, owner, [this] {
        playing = false; worker.seek(worker.state().frame - 1); render();
      });
    QObject::connect(
      next, &QPushButton::clicked, owner, [this] {
        playing = false; worker.seek(worker.state().frame + 1); render();
      });
    QObject::connect(
      slider, &QSlider::valueChanged, owner, [this](int value) {
        playing = false; worker.seek(value); render();
      });
    QObject::connect(
      events->selectionModel(), &QItemSelectionModel::currentRowChanged,
      owner, [this](const QModelIndex & selected) {
        details->clear();
        if (displayed && selected.isValid() &&
        static_cast<std::size_t>(selected.row()) < displayed->events.size())
        {
          details->setPlainText(
            QString::fromUtf8(
              QJsonDocument(
                displayed->events[selected.row()]).toJson(QJsonDocument::Indented)));
        }
      });
    auto * timer = new QTimer(owner); timer->setInterval(100);
    QObject::connect(
      timer, &QTimer::timeout, owner, [this] {
        const auto s = worker.state();
        if (closing) {if (s.stopped) {owner->close();} return;}
        if (playing && s.record && !s.loading && !s.pending) {
          if (s.frame + 1 < static_cast<int>(s.record->frames.size())) {
            worker.seek(
              s.frame + 1);
          } else {playing = false;}
        }
        render();
      }); timer->start(); render();
  }
  void render()
  {
    const auto s = worker.state();
    const bool ready = !closing && !s.loading && s.record && s.error.isEmpty();
    if (!ready) {playing = false;}
    choose->setEnabled(!closing); open->setEnabled(
      !closing && !root->text().isEmpty() &&
      !id->text().isEmpty()); root->setEnabled(!closing); id->setEnabled(!closing);
    previous->setEnabled(ready && !s.pending && s.frame > 0);
    next->setEnabled(
      ready && !s.pending &&
      s.frame + 1 < static_cast<int>(s.record->frames.size()));
    play->setEnabled(ready);
    play->setText(playing ? QStringLiteral("暂停") : QStringLiteral("播放 10fps"));
    slider->setEnabled(ready);
    if (s.record != displayed) {
      displayed = s.record; model->replace(displayed); details->clear();
    }
    if (s.revision == rendered && image->size() == rendered_size && !s.loading) {return;}
    rendered = s.revision; rendered_size = image->size();
    if (!ready) {
      image->clear(); image->setText(QStringLiteral("历史图像不可用"));
      frame_info->clear();
      summary->setText(
        s.loading ? QStringLiteral("本地完整性校验中…不连接飞机") :
        s.error.isEmpty() ? QStringLiteral("尚未选择历史记录") :
        QStringLiteral("拒绝回放：%1").arg(s.error)); return;
    }
    const auto & r = *s.record;
    summary->setText(
      QStringLiteral(
        "历史记录 %1\n%2 帧 · %3 个事件\n相机 %4\nprofile 标签（未确认）%5\n"
        "时钟域 %6 · 会话/epoch %7 / %8\n结束原因 %9")
      .arg(r.id).arg(r.frames.size()).arg(r.events.size())
      .arg(
        recorded(r.config, "source_topic"), recorded(r.config, "profile_label_unconfirmed"),
        r.config["simulation_clock"].toBool() ? QStringLiteral("模拟") : QStringLiteral("系统"),
        recorded(r.config, "observation_session"), recorded(r.config, "epoch"),
        recorded(r.manifest, "reason")));
    if (s.frame >= 0) {
      const auto & f = r.frames.at(s.frame);
      frame_info->setText(
        QStringLiteral(
          "历史帧 %1 / %2\nsource_ns：%3\n接收 steady：%4\n编码 steady：%5\n写入 UTC：%6")
        .arg(s.frame + 1).arg(r.frames.size()).arg(f["source_ns"].toString())
        .arg(f["first_received_steady"].toDouble(), 0, 'f', 6)
        .arg(f["encoded_steady"].toDouble(), 0, 'f', 6).arg(f["written_utc"].toString()));
    }
    {const QSignalBlocker block(slider);
      slider->setRange(0, r.frames.size() - 1); slider->setValue(s.frame);}
    image->setPixmap(
      QPixmap::fromImage(s.image).scaled(
        image->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
  }
};
PlaybackWindow::PlaybackWindow(QWidget * parent)
: QWidget(parent), impl_(std::make_unique<Impl>(this)) {}
PlaybackWindow::~PlaybackWindow() = default;
void PlaybackWindow::load(const QString & directory, const QString & id)
{
  impl_->root->setText(directory); impl_->id->setText(id);
  impl_->playing = false; impl_->worker.open(directory, id); impl_->render();
}
PlaybackState PlaybackWindow::state() const {return impl_->worker.state();}
void PlaybackWindow::closeEvent(QCloseEvent * event)
{
  if (impl_->worker.state().stopped) {event->accept(); return;}
  impl_->closing = true; impl_->playing = false; impl_->worker.stop();
  impl_->render(); event->ignore();
}
}  // namespace easyfly_desktop

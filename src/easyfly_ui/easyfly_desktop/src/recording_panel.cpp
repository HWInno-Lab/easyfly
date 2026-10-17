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

#include "easyfly_desktop/recording_panel.hpp"
#include <QFileDialog>
#include <QDateTime>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPointer>
#include <QPushButton>
#include <QTableWidget>
#include <QHeaderView>
#include <QTimer>
#include <QToolButton>
#include <QCoreApplication>
#include <QProcess>
#include <QVBoxLayout>
#include <utility>
namespace easyfly_desktop
{
struct RecordingPanel::Impl
{
  RecordingPanel * owner;
  RecordingWorker worker;
  RecordingPacket packet;
  RecordingConfig config, frozen;
  bool connected{false}, stopping{false};
  QLineEdit * directory;
  QPushButton * start, * finish, * scan, * choose, * replay;
  QLabel * status;
  QTableWidget * index;
  QPointer<QMessageBox> prompt;
  QString rendered_index;
  QString indexed_directory, replay_error;
  explicit Impl(RecordingPanel * p, std::vector<std::string> paths)
  : owner(p), worker(std::move(paths))
  {
    owner->setAttribute(Qt::WA_StyledBackground, true);
    auto * layout = new QVBoxLayout(p);
    auto * title = new QLabel(QStringLiteral("记录与回放"), p);
    title->setProperty("heading", true); layout->addWidget(title);
    auto * summary = new QLabel(
      QStringLiteral(
        "录制当前选定相机，停止后刷新列表并选择记录回放。\n"
        "录像是界面抽样数据；时钟或连接中断会自动收尾，恢复后需手动开始。"), p);
    summary->setWordWrap(true); layout->addWidget(summary);
    auto * explanation = new QLabel(
      QStringLiteral(
        "仅记录开始时选定的相机，最多 10fps / 1280×720，MJPEG/AVI 分段（最多 600 帧）。\n"
        "固定帧率回放不表示真实帧间隔；源时间、steady、UTC 和丢帧另记。\n"
        "非原始完整数据。录像最多 512MiB，保留 256MiB 空闲空间；\n"
        "满额或写入失败即停止，不自动清理。相机/观测会话/时钟来源变化、\n"
        "时钟暂停/失效、断开或退出即收尾；恢复后须重新人工开始，不取消远端任务。\n"
        "只关联已知请求/执行/Action 身份；全局阶段单独标记未归属。"), p);
    explanation->setWordWrap(true);
    explanation->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
    auto * details = new QToolButton(p); details->setText(QStringLiteral("展开录制规格与说明"));
    details->setCheckable(true); explanation->hide(); layout->addWidget(details);
    layout->addWidget(explanation);
    QObject::connect(details, &QToolButton::toggled, explanation, &QWidget::setVisible);
    auto * row = new QHBoxLayout;
    directory = new QLineEdit(p); directory->setObjectName("recordingDirectory");
    directory->setPlaceholderText(
      QStringLiteral(
        "新建独立私有目录的绝对路径，例如 /tmp/my-easyfly-records"));
    choose = new QPushButton(QStringLiteral("选择目录"), p);
    choose->setObjectName("recordingChoose");
    row->addWidget(directory, 1); row->addWidget(choose); layout->addLayout(row);
    auto * buttons = new QHBoxLayout;
    start = new QPushButton(QStringLiteral("开始录像…"), p);
    start->setObjectName("recordingStart");
    finish = new QPushButton(QStringLiteral("停止并校验"), p);
    finish->setObjectName("recordingFinish");
    scan = new QPushButton(QStringLiteral("刷新本地索引"), p); scan->setObjectName("recordingScan");
    buttons->addWidget(start); buttons->addWidget(finish); buttons->addWidget(scan);
    replay = new QPushButton(QStringLiteral("独立历史回放"), p);
    replay->setObjectName("recordingReplay");
    buttons->addStretch();
    layout->addLayout(buttons);
    status = new QLabel(p); status->setObjectName("recordingStatus"); status->setWordWrap(true);
    status->setTextFormat(Qt::PlainText);
    status->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
    status->setTextInteractionFlags(Qt::TextSelectableByMouse); layout->addWidget(status);
    index = new QTableWidget(0, 4, p); index->setObjectName("recordingIndex");
    index->setColumnCount(7); index->setIconSize(QSize(72, 40));
    index->setHorizontalHeaderLabels(
      {QStringLiteral("预览 / ID"), QStringLiteral("状态"),
        QStringLiteral("帧数"), QStringLiteral("开始时间 UTC"), QStringLiteral("观测时长"),
        QStringLiteral("来源 / 执行"), QStringLiteral("结束原因")});
    index->setEditTriggers(QAbstractItemView::NoEditTriggers);
    index->setMinimumHeight(120); owner->setMinimumHeight(580);
    index->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
    const int widths[]{140, 62, 44, 104, 66, 110, 86};
    for (int col = 0; col < 7; ++col) {index->setColumnWidth(col, widths[col]);}
    index->horizontalHeader()->setStretchLastSection(true); layout->addWidget(index, 1);
    auto * replay_row = new QHBoxLayout;
    replay_row->addWidget(replay); replay_row->addStretch(); layout->addLayout(replay_row);
    layout->addWidget(
      new QLabel(
        QStringLiteral(
          "索引只读，无删除。选择已校验的记录后可打开独立本地历史窗口（不连接 ROS）。\n"
          "独立窗口自行关闭，不随本窗口关闭；刷新校验最多 256 条/512MiB。"), p));
    QObject::connect(
      choose, &QPushButton::clicked, p, [this] {
        const auto path = QFileDialog::getExistingDirectory(
          owner, QStringLiteral(
            "选择已有专用记录目录；首次可取消后在输入框填写新目录路径"));
        if (!path.isEmpty()) {directory->setText(path);}
      });
    QObject::connect(start, &QPushButton::clicked, p, [this] {confirm();});
    QObject::connect(
      finish, &QPushButton::clicked, p, [this] {
        worker.finish("operator_stop"); render();
      });
    QObject::connect(
      scan, &QPushButton::clicked, p, [this] {
        if (worker.scan(directory->text().toStdString())) {
          indexed_directory = directory->text();
        }
        render();
      });
    QObject::connect(index, &QTableWidget::itemSelectionChanged, p, [this] {render();});
    QObject::connect(
      replay, &QPushButton::clicked, p, [this] {
        const auto state = worker.state(); const auto row = index->currentRow();
        if (!replay->isEnabled() || row < 0 ||
        static_cast<std::size_t>(row) >= state.index.size() || active() || prompt ||
        state.index_pending || !state.index_error.empty() ||
        indexed_directory != directory->text() || state.index[row].status != "verified")
        {return;}
        QProcess player;
        player.setProgram(QCoreApplication::applicationDirPath() + "/easyfly_recording_player");
        player.setArguments(
          {"--directory", indexed_directory, "--record",
            QString::fromStdString(state.index[row].id)});
        replay_error = player.startDetached() ? QString{} :
        QStringLiteral("本地回放窗口启动失败；未发送 ROS 请求。");
        render();
      });
    auto * timer = new QTimer(p); timer->setInterval(100);
    QObject::connect(timer, &QTimer::timeout, p, [this] {render();}); timer->start(); render();
  }
  bool fresh() const
  {
    if (!connected || !packet.frame || !packet.observation || !packet.frame->error.empty() ||
      packet.frame->image.isNull() || !(packet.frame->binding == config.binding)) {return false;}
    const auto now = bridge::detail::steady_seconds();
    const auto & s = *packet.observation;
    const auto age = static_cast<double>(s.clock.now_ns - packet.frame->stamp_ns) / 1e9;
    return recording_clock_end_reason(packet, config.binding).empty() &&
           now - s.produced_steady >= 0 && now - s.produced_steady <= 1 &&
           now - packet.frame->first_received >= 0 && now - packet.frame->first_received <= 1 &&
           age >= -.2 && age <= 1;
  }
  bool active() const
  {
    const auto p = worker.state().phase;
    return p == RecordingPhase::Starting || p == RecordingPhase::Recording ||
           p == RecordingPhase::Stopping;
  }
  void dismiss()
  {
    const auto old = prompt; prompt.clear();
    if (old) {
      QObject::disconnect(old, nullptr, owner, nullptr);
      old->setObjectName("dismissedRecordingConfirmation"); old->reject(); old->deleteLater();
    }
  }
  void confirm()
  {
    if (prompt || active() || stopping || !fresh() || directory->text().isEmpty()) {return;}
    frozen = config; frozen.directory = directory->text().toStdString();
    prompt = new QMessageBox(
      QMessageBox::Question, QStringLiteral("确认本地录像"),
      QStringLiteral(
        "相机 topic：%1\n观测会话/epoch：%2 / %3\n目录：%4\n"
        "每次新建独立记录，不覆盖、不自动删除。仅保存 UI 抽样图像与有限元数据；不发送飞行命令。")
      .arg(QString::fromStdString(frozen.binding.topic)).arg(frozen.binding.session)
      .arg(frozen.binding.epoch).arg(
        directory->text()), QMessageBox::Ok | QMessageBox::Cancel, owner);
    prompt->setObjectName("recordingConfirmation"); prompt->setDefaultButton(QMessageBox::Cancel);
    prompt->setTextFormat(Qt::PlainText);
    const auto accepted_prompt = prompt;
    QObject::connect(
      prompt, &QMessageBox::finished, owner, [this, accepted_prompt](int result) {
        if (prompt != accepted_prompt) {return;}
        prompt.clear();
        if (accepted_prompt) {
          accepted_prompt->setObjectName("dismissedRecordingConfirmation");
          accepted_prompt->deleteLater();
        }
        if (result == QMessageBox::Ok && !stopping && !active() && fresh() &&
        config.binding == frozen.binding && config.simulation_clock == frozen.simulation_clock &&
        config.profile == frozen.profile && directory->text().toStdString() == frozen.directory)
        {
          worker.start(frozen, packet);
        }
        render();
      });
    prompt->open(); render();
  }
  void render()
  {
    // Also expire a frozen snapshot when no new observe() call arrives; same existing 1s bound.
    const auto reason = recording_clock_end_reason(packet, frozen.binding);
    if (active() && !reason.empty()) {worker.finish(reason);}
    if (prompt && !reason.empty()) {dismiss();}
    const auto s = worker.state(); const bool running = active();
    start->setEnabled(
      !stopping && !running && !s.index_pending && !prompt && fresh() &&
      !directory->text().isEmpty());
    finish->setEnabled(
      !stopping &&
      (s.phase == RecordingPhase::Starting || s.phase == RecordingPhase::Recording));
    directory->setEnabled(!stopping && !running && !s.index_pending && !prompt); choose->setEnabled(
      directory->isEnabled());
    scan->setEnabled(
      !stopping && !running && !s.index_pending && !prompt &&
      !directory->text().isEmpty());
    const auto selected = index->currentRow();
    replay->setEnabled(
      !stopping && !running && !s.index_pending && !prompt &&
      s.index_error.empty() && indexed_directory == directory->text() && selected >= 0 &&
      static_cast<std::size_t>(selected) < s.index.size() &&
      s.index[selected].status == "verified");
    const QStringList phases{QStringLiteral("未录像"), QStringLiteral("创建中"),
      QStringLiteral("录像中"), QStringLiteral("收尾/逐帧校验中"),
      QStringLiteral("已收尾"), QStringLiteral("失败/可能有未完成文件")};
    status->setText(
      QStringLiteral(
        "%1 · 已写 %2 帧 · 已校验 %3 段 · 待编码 %4/1\n"
        "编码排队替换 %5 · 限速丢帧 %6 · 过期丢帧 %7 · 元数据排队替换 %8\n%9\n%10\n%11")
      .arg(phases[static_cast<int>(s.phase)]).arg(s.frames).arg(s.segments).arg(s.pending)
      .arg(s.replaced).arg(s.rate_dropped).arg(s.stale_dropped).arg(s.metadata_replaced)
      .arg(
        QString::fromStdString(s.path), QString::fromStdString(s.reason + " " + s.error),
        QString::fromStdString(s.index_error) + replay_error));
    QString fingerprint;
    for (const auto & item : s.index) {
      fingerprint += QString::fromStdString(item.id + item.status + item.reason) + QString::number(
        item.frames);
    }
    if (fingerprint == rendered_index) {return;}
    rendered_index = fingerprint; index->setRowCount(s.index.size());
    for (std::size_t row = 0; row < s.index.size(); ++row) {
      const auto & entry = s.index[row];
      const auto started = QDateTime::fromString(entry.started, Qt::ISODateWithMs);
      const auto status_text = entry.status == "verified" ? QStringLiteral("已校验") :
        entry.status == "empty" ? QStringLiteral("无有效帧") :
        entry.status == "failed" ? QStringLiteral("失败") : QStringLiteral("未完成/损坏");
      const auto reason_text = entry.reason == "operator_stop" ? QStringLiteral("手动停止") :
        QString::fromStdString(entry.reason);
      const QStringList values{QString::fromStdString(entry.id).left(8), status_text,
        QString::number(entry.frames), started.isValid() ?
        started.toUTC().toString("yyyy-MM-dd\nHH:mm:ss") : entry.started,
        entry.duration >= 0 ? QString::number(entry.duration, 'f', 1) + " s" : QStringLiteral("—"),
        entry.source.section('/', -2) +
        (entry.execution.isEmpty() ? QString{} : "\n" + entry.execution.left(8)), reason_text};
      const QStringList details{QString::fromStdString(entry.id),
        QString::fromStdString(entry.status), QString::number(entry.frames), entry.started,
        values[4], entry.source + "\n" + entry.execution, QString::fromStdString(entry.reason)};
      for (int col = 0; col < values.size(); ++col) {
        index->setItem(row, col, new QTableWidgetItem(values[col]));
        index->item(row, col)->setToolTip(details[col]);
      }
      if (!entry.thumbnail.isNull()) {
        index->item(row, 0)->setIcon(QPixmap::fromImage(entry.thumbnail));
      }
      index->setRowHeight(row, 62);
    }
  }
};
RecordingPanel::RecordingPanel(std::vector<std::string> paths, QWidget * parent)
: QWidget(parent), impl_(std::make_unique<Impl>(this, std::move(paths))) {}
RecordingPanel::~RecordingPanel() {impl_->dismiss();}
void RecordingPanel::observe(
  const RecordingPacket & packet, const ImageBinding & binding, bool connected,
  bool simulation_clock, const std::string & profile)
{
  impl_->packet = packet; impl_->config.binding = binding; impl_->connected = connected;
  impl_->config.simulation_clock = simulation_clock; impl_->config.profile = profile;
  if (impl_->active() && (!connected || !(binding == impl_->frozen.binding) ||
    simulation_clock != impl_->frozen.simulation_clock || profile != impl_->frozen.profile))
  {
    impl_->worker.finish(!connected ? "observation_disconnected" : "camera_session_clock_changed");
  } else if (impl_->active()) {impl_->worker.offer(packet);}
  if (impl_->prompt && (!connected || !(binding == impl_->frozen.binding))) {impl_->dismiss();}
  impl_->render();
}
void RecordingPanel::finish(const std::string & reason)
{
  impl_->dismiss(); impl_->worker.finish(reason);
}
void RecordingPanel::stop()
{
  impl_->stopping = true; impl_->dismiss(); impl_->worker.stop(); impl_->render();
}
bool RecordingPanel::stopped() const {return impl_->worker.state().stopped;}
}  // namespace easyfly_desktop

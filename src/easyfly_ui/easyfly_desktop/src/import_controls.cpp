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

#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTimer>
#include <QUuid>
#include <QVariant>
#include <QVBoxLayout>
#include <stdexcept>
#include <utility>
#include "easyfly_desktop/import_controls.hpp"
namespace easyfly_desktop
{
namespace ctl = easyfly_ui_control;
namespace
{
class RealImport final : public ImportPort
{
public:
  explicit RealImport(const ctl::ImportConfig & c)
  : importer_(c) {}
  std::shared_ptr<const ctl::ImportSnapshot> snapshot() const override
  {
    return importer_.snapshot();
  }
  bool read_destination() override {return importer_.read_destination();}
  bool import_confirmed(
    const ctl::ImportDocument & d, const ctl::ImportDestination & to,
    const std::string & id) override {return importer_.import_confirmed(d, to, id);}
  void inhibit(bool active) override {importer_.inhibit(active);}
  void stop() override {importer_.stop();}

private:
  ctl::MissionImporter importer_;
};
QString phase_text(ctl::ImportPhase phase)
{
  switch (phase) {
    case ctl::ImportPhase::Idle: return QStringLiteral("未读取目的地，未导入");
    case ctl::ImportPhase::Reading: return QStringLiteral("正在读取目的地");
    case ctl::ImportPhase::Ready: return QStringLiteral("已读取目的地，尚未导入");
    case ctl::ImportPhase::Precheck: return QStringLiteral("正在复核目的地，尚未提交");
    case ctl::ImportPhase::Sent: return QStringLiteral("已单次提交，等待导入回复");
    case ctl::ImportPhase::Uncertain: return QStringLiteral("结果未确认，只查询原请求，不重新提交");
    case ctl::ImportPhase::Readback: return QStringLiteral("正在独立回读目录内容");
    case ctl::ImportPhase::Completed: return QStringLiteral("导入已回读确认（未执行）");
    case ctl::ImportPhase::Rejected: return QStringLiteral("未确认导入成功 / 操作被拒绝");
    case ctl::ImportPhase::Blocked: return QStringLiteral("故障锁：身份或结果未确认");
  }
  return QStringLiteral("状态未知");
}
}  // namespace
ImportFactory import_factory()
{return [](const ctl::ImportConfig & c) {return std::make_unique<RealImport>(c);};}
struct ImportPanel::Impl
{
  ImportPanel * owner;
  std::unique_ptr<ImportPort> port;
  std::optional<ExportedDraft> draft;
  std::function<std::optional<ExportedDraft>()> live;
  QLabel * target{nullptr}, *status{nullptr}, *document{nullptr};
  QPushButton * read{nullptr}, *create{nullptr};
  QMessageBox * prompt{nullptr};
  bool available{false}, inhibited{false}, closing{false};
  QString error;
  Impl(
    ImportPanel * widget, const ImportOptions & options, ImportFactory factory,
    std::function<std::optional<ExportedDraft>()> provider)
  : owner(widget), live(std::move(provider))
  {
    auto * layout = new QVBoxLayout(owner);
    auto * boundary = new QLabel(
      QStringLiteral(
        "受控任务导入：仅新增已校验且已导出的缓存字节，不覆盖、不执行，不重新读取草稿文件。\n"
        "服务端须单独启用私有任务目录；超时/重启只查询原实例和请求，不重发。"), owner);
    boundary->setWordWrap(true); layout->addWidget(boundary);
    status = new QLabel(owner); status->setObjectName("importStatus");
    status->setTextFormat(Qt::PlainText); status->setWordWrap(true);
    status->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    if (!options.enabled) {
      status->setText(
        QStringLiteral(
          "默认关闭。须 --enable-mission-import 与独立 --import-journal 显式启用。"));
      layout->addWidget(status); return;
    }
    port = factory({options.journal_directory, 2});
    if (!port) {throw std::runtime_error("import controller not created");}
    target = new QLabel(owner); target->setObjectName("importDestination");
    document = new QLabel(owner); document->setObjectName("importDocument");
    for (auto * label : {target, document}) {
      label->setTextFormat(Qt::PlainText); label->setWordWrap(true);
      label->setTextInteractionFlags(Qt::TextSelectableByMouse);
      label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred); layout->addWidget(label);
    }
    auto * row = new QHBoxLayout;
    read = new QPushButton(QStringLiteral("读取服务导入目的地"), owner);
    read->setObjectName("importReadDestination"); row->addWidget(read);
    create = new QPushButton(QStringLiteral("确认仅新增任务…"), owner);
    create->setObjectName("importCreate"); row->addWidget(create);
    layout->addLayout(row); layout->addWidget(status);
    QObject::connect(
      read, &QPushButton::clicked, owner, [this] {
        if (permitted() && port->read_destination()) {error.clear();} else {
          error = QStringLiteral("读取未入队：忙、未决、故障或断开");
        } refresh();
      });
    QObject::connect(create, &QPushButton::clicked, owner, [this] {confirm();});
    auto * timer = new QTimer(owner); timer->setInterval(50);
    QObject::connect(timer, &QTimer::timeout, owner, [this] {refresh();}); timer->start();
    refresh();
  }
  bool permitted() const {return port && available && !closing && !inhibited && !prompt;}
  void retire()
  {
    if (!prompt) {return;} auto * p = prompt; prompt = nullptr;
    p->setObjectName(""); p->done(QMessageBox::Cancel); p->deleteLater();
  }
  bool unresolved() const
  {
    if (!port) {return false;} const auto s = port->snapshot();
    return (!s->ready && !s->stopped) || s->fault || (s->intent && !s->intent->resolved);
  }
  void refresh()
  {
    if (!port) {return;} const auto s = port->snapshot();
    const bool ready = permitted() && s->ready && !s->stopped && !s->pending && !unresolved();
    read->setEnabled(ready);
    create->setEnabled(
      ready && draft && s->destination && s->destination->enabled &&
      s->destination->retained < s->destination->capacity && s->confirmations < 256);
    target->setText(
      s->destination ? QStringLiteral("服务目的地：%1\n实例：%2 · 目录身份：%3 · 启用：%4")
      .arg(
        QString::fromStdString(s->destination->directory),
        QString::fromStdString(s->destination->instance),
        QString::fromStdString(s->destination->catalog),
        s->destination->enabled ? QStringLiteral("是") : QStringLiteral("否")) :
      QStringLiteral("未读取目的地，不用本地路径代替服务目录"));
    document->setText(
      draft ? QStringLiteral("当前已导出：%1\nSHA-256：%2")
      .arg(QString::fromStdString(draft->name), QString::fromStdString(draft->hash)) :
      QStringLiteral("请先校验并导出当前 v1 草稿；未应用改动/更换文件名后需重新导出"));
    QString message = phase_text(s->phase) + "\n" + QString::fromStdString(s->message);
    if (s->intent) {
      message += QStringLiteral("\n原请求：%1 · 文件：%2\n原 SHA-256：%3")
        .arg(
        QString::fromStdString(s->intent->request), QString::fromStdString(s->intent->name),
        QString::fromStdString(s->intent->hash));
    }
    if (!available) {message += QStringLiteral("\n观测断开，禁止新导入；原请求仍只查询恢复");}
    if (!error.isEmpty()) {message += "\n" + error;}
    status->setProperty("importPhase", static_cast<int>(s->phase)); status->setText(message);
  }
  void confirm()
  {
    if (!permitted() || !draft) {return;}
    const auto s = port->snapshot();
    if (s->pending || unresolved() || !s->destination || !s->destination->enabled) {return;}
    const auto captured = *draft; const auto destination = *s->destination;
    const auto id = QUuid::createUuid().toString(QUuid::Id128).toStdString();
    prompt = new QMessageBox(
      QMessageBox::Warning, QStringLiteral("仅新增任务文件，不执行？"),
      QStringLiteral(
        "服务端目录：%1\n实例：%2\n目录身份：%3\n文件：%4\nSHA-256：%5\n"
        "确认后复核目的地并单次提交；存在同名文件则拒绝，不覆盖。\n"
        "结果不明时持久保留，只查询原请求；任务不会自动执行。")
      .arg(
        QString::fromStdString(destination.directory), QString::fromStdString(destination.instance),
        QString::fromStdString(destination.catalog), QString::fromStdString(captured.name),
        QString::fromStdString(captured.hash)), QMessageBox::Yes | QMessageBox::Cancel, owner);
    auto * p = prompt; p->setObjectName("importPrompt"); p->setTextFormat(Qt::PlainText);
    p->setDefaultButton(QMessageBox::Cancel); p->setEscapeButton(QMessageBox::Cancel);
    p->setButtonText(QMessageBox::Yes, QStringLiteral("确认仅新增"));
    QObject::connect(
      p, &QMessageBox::finished, owner, [this, p, captured, destination, id](int answer) {
        if (prompt != p) {return;} prompt = nullptr; p->setObjectName(""); p->deleteLater();
        if (answer == QMessageBox::Yes && permitted()) {
          const auto current = live ? live() : draft;
          if (current && current->revision == captured.revision && current->name == captured.name &&
          current->content == captured.content && current->hash == captured.hash &&
          port->import_confirmed({captured.name, captured.content, captured.hash}, destination, id))
          {
            error.clear();
          } else {error = QStringLiteral("确认失效或导入未入队；未自动重试");}
        }
        refresh();
      }); p->open(); refresh();
  }
};
ImportPanel::ImportPanel(
  const ImportOptions & o, ImportFactory f, QWidget * parent,
  std::function<std::optional<ExportedDraft>()> live)
: QWidget(parent), impl_(std::make_unique<Impl>(this, o, std::move(f), std::move(live))) {}
ImportPanel::~ImportPanel() = default;
void ImportPanel::observe(std::optional<ExportedDraft> draft, bool available)
{
  impl_->draft = std::move(draft); impl_->available = available;
  if (!available) {impl_->retire();}
  if (impl_->port) {impl_->port->inhibit(!available || impl_->inhibited || impl_->closing);}
  impl_->refresh();
}
void ImportPanel::exit_question(bool active)
{
  impl_->inhibited = active; if (active) {impl_->retire();}
  if (impl_->port) {impl_->port->inhibit(active || !impl_->available || impl_->closing);}
  impl_->refresh();
}
bool ImportPanel::busy() const
{return impl_->prompt || (impl_->port && impl_->port->snapshot()->active_create);}
bool ImportPanel::unresolved() const {return impl_->unresolved();}
bool ImportPanel::stopped() const {return !impl_->port || impl_->port->snapshot()->stopped;}
void ImportPanel::stop()
{impl_->closing = true; impl_->retire(); if (impl_->port) {impl_->port->stop();} impl_->refresh();}
}  // namespace easyfly_desktop

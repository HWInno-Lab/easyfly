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
#include <QCheckBox>
#include <QComboBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTimer>
#include <QUuid>
#include <QVBoxLayout>
#include <algorithm>
#include <stdexcept>
#include <utility>
#include "easyfly_desktop/parameter_controls.hpp"
namespace easyfly_desktop
{
namespace params = easyfly_ui_control;
namespace
{
QString phase_text(params::ParameterPhase phase)
{
  switch (phase) {
    case params::ParameterPhase::Idle: return QStringLiteral("未读取，未提交");
    case params::ParameterPhase::Reading: return QStringLiteral("正在读取当前值");
    case params::ParameterPhase::Reviewed: return QStringLiteral("已读取，尚未提交");
    case params::ParameterPhase::Precheck: return QStringLiteral("正在复核旧值，尚未写入");
    case params::ParameterPhase::Writing: return QStringLiteral("已单次提交，等待回复");
    case params::ParameterPhase::Checking: return QStringLiteral("已收到回复，正在独立回读");
    case params::ParameterPhase::Verified: return QStringLiteral("写入已回读确认（仅该次观测）");
    case params::ParameterPhase::Rejected: return QStringLiteral("操作被拒绝，未确认成功");
    case params::ParameterPhase::Uncertain: return QStringLiteral("结果未确认，禁止后续写入");
  }
  return QStringLiteral("状态未知");
}
class RealParameters final : public ParameterPort
{
public:
  std::shared_ptr<const params::ParameterSnapshot> snapshot() const override
  {
    return control_.snapshot();
  }
  bool read(params::LocalParameter p) override {return control_.read(p);}
  bool write_confirmed(
    const params::ParameterReview & r, const params::ParameterValue & v,
    const std::string & id) override {return control_.write_confirmed(r, v, id);}
  void inhibit(bool active) override {control_.inhibit(active);}
  void stop() override {control_.stop();}

private:
  params::ParameterControl control_;
};
}  // namespace
ParameterFactory parameter_factory()
{return [] {return std::make_unique<RealParameters>();};}
struct ParameterPanel::Impl
{
  ParameterPanel * owner;
  std::unique_ptr<ParameterPort> port;
  QComboBox * key{nullptr};
  QPlainTextEdit * note{nullptr}, *current{nullptr};
  QCheckBox * pretty{nullptr};
  QPushButton * read{nullptr}, *write{nullptr}, *copy{nullptr};
  QLabel * status{nullptr};
  QMessageBox * prompt{nullptr};
  QString local_error;
  bool available{false}, inhibited{false}, closing{false};
  Impl(ParameterPanel * widget, const ParameterOptions & options, ParameterFactory factory)
  : owner(widget)
  {
    auto * layout = new QVBoxLayout(owner);
    auto * boundary = new QLabel(
      QStringLiteral(
        "仅平台配置元数据：operator_note / export_pretty。飞行边界、frame、外参、设备连接仍只读。\n"
        "不自动重试/回滚；无原子旧值比较、多客户端或跨重启恢复保证。"), owner);
    boundary->setWordWrap(true); layout->addWidget(boundary);
    status = new QLabel(owner); status->setObjectName("parameterEditStatus");
    status->setTextFormat(Qt::PlainText); status->setWordWrap(true);
    status->setMinimumWidth(0); status->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    if (!options.enabled) {
      status->setText(
        QStringLiteral(
          "默认不创建写入客户端。须用 --enable-parameter-edit 显式启用。"));
      layout->addWidget(status); return;
    }
    try {
      port = factory();
      if (!port) {
        throw std::runtime_error("no parameter editor port");
      }
    } catch (const std::exception &) {
      status->setText(QStringLiteral("本地参数编辑器启动失败；未写入"));
      layout->addWidget(status); return;
    }
    auto * row = new QHBoxLayout;
    key = new QComboBox(owner); key->setObjectName("parameterEditKey"); key->addItems(
      {"operator_note", "export_pretty"});
    row->addWidget(key); read = new QPushButton(QStringLiteral("读取当前值"), owner);
    read->setObjectName("parameterEditRead"); row->addWidget(read); layout->addLayout(row);
    current = new QPlainTextEdit(owner); current->setObjectName("parameterEditCurrent");
    current->setReadOnly(true); current->setMaximumHeight(65); current->setPlaceholderText(
      QStringLiteral(
        "未读取此参数，不用默认值冒充当前值"));
    layout->addWidget(current);
    auto * pending = new QLabel(QStringLiteral("待写值（不会随窗口关闭提交）"), owner);
    layout->addWidget(pending);
    note = new QPlainTextEdit(owner); note->setObjectName("parameterEditNote");
    note->setMaximumHeight(80);
    note->setPlaceholderText(QStringLiteral("UTF-8备注，最多512字节，不含NUL；允许空字符串"));
    layout->addWidget(note);
    pretty = new QCheckBox(QStringLiteral("平台配置导出 JSON 使用缩进（不影响草稿 YAML）"), owner);
    pretty->setObjectName("parameterEditPretty"); layout->addWidget(pretty);
    auto * buttons = new QHBoxLayout;
    copy = new QPushButton(QStringLiteral("复制读取值到待写值"), owner); copy->setObjectName(
      "parameterEditCopy"); buttons->addWidget(copy);
    write = new QPushButton(QStringLiteral("确认单次写入…"), owner); write->setObjectName(
      "parameterEditWrite"); buttons->addWidget(write);
    layout->addLayout(buttons); layout->addWidget(status);
    QObject::connect(
      read, &QPushButton::clicked, owner, [this] {
        if (!permitted() || !port->read(parameter())) {
          local_error = QStringLiteral("读取未入队：忙、观测断开或编辑器关闭");
        } else {local_error.clear();} refresh();
      });
    QObject::connect(write, &QPushButton::clicked, owner, [this] {confirm();});
    QObject::connect(
      copy, &QPushButton::clicked, owner, [this] {
        const auto s = port->snapshot();
        if (!s->review || s->review->parameter != parameter() || !permitted()) {return;}
        const auto value = params::parameter_value(parameter(), s->review->json); if (!value) {
          return;
        }
        if (parameter() == params::LocalParameter::OperatorNote) {
          note->setPlainText(QString::fromStdString(std::get<std::string>(*value)));
        } else {pretty->setChecked(std::get<bool>(*value));} refresh();
      });
    QObject::connect(
      key, &QComboBox::currentTextChanged, owner, [this] {
        retire(); local_error.clear(); refresh();
      });
    QObject::connect(note, &QPlainTextEdit::textChanged, owner, [this] {refresh();});
    QObject::connect(pretty, &QCheckBox::toggled, owner, [this] {refresh();});
    auto * timer = new QTimer(owner); timer->setInterval(50);
    QObject::connect(timer, &QTimer::timeout, owner, [this] {refresh();}); timer->start();
    refresh();
  }
  params::LocalParameter parameter() const
  {
    return key->currentIndex() ==
           0 ? params::LocalParameter::OperatorNote : params::LocalParameter::ExportPretty;
  }
  params::ParameterValue desired() const
  {
    if (parameter() == params::LocalParameter::OperatorNote) {
      return note->toPlainText().toUtf8().toStdString();
    }
    return pretty->isChecked();
  }
  bool permitted() const {return port && available && !inhibited && !closing && !prompt;}
  void retire()
  {
    if (!prompt) {return;} auto * p = prompt; prompt = nullptr;
    p->setObjectName(""); p->done(QMessageBox::Cancel); p->deleteLater();
  }
  void refresh()
  {
    if (!port) {return;}
    const auto s = port->snapshot();
    const bool enabled = permitted() && s->ready && !s->stopped && !s->pending;
    const bool reviewed = s->review && s->review->parameter == parameter();
    const auto value_text = reviewed ? QString::fromStdString(s->review->json) : QString{};
    if (current->toPlainText() != value_text) {current->setPlainText(value_text);}
    bool valid = true; QString invalid;
    try {params::parameter_json(parameter(), desired());} catch (const std::exception &) {
      valid = false; invalid = QStringLiteral("待写值类型/UTF-8或512字节边界无效");
    }
    read->setEnabled(enabled); key->setEnabled(enabled); note->setEnabled(enabled);
    pretty->setEnabled(enabled);
    note->setVisible(parameter() == params::LocalParameter::OperatorNote);
    pretty->setVisible(parameter() == params::LocalParameter::ExportPretty);
    copy->setEnabled(enabled && reviewed); write->setEnabled(
      enabled && reviewed && valid && !s->fault && s->confirmations < 256);
    status->setProperty("parameterPhase", static_cast<int>(s->phase));
    QString message = phase_text(s->phase) + "\n" + QString::fromStdString(s->message);
    if (s->fault) {
      message = QStringLiteral("结果未确认 · 参数故障锁（重启不恢复，读取不清锁）\n") + message;
    }
    if (!s->last_write.empty()) {message += "\n" + QString::fromStdString(s->last_write);}
    if (reviewed) {
      message += QStringLiteral("\n读取年龄 %1s（不是永久当前值），确认后再复核旧值。")
        .arg(std::max(0.0, params::parameter_steady_seconds() - s->review->received), 0, 'f', 1);
    }
    if (!available) {message += QStringLiteral("\n本地观测未连接：禁止新写入");}
    if (s->confirmations >= 256) {message += QStringLiteral("\n确认记录已满：不淘汰，拒绝新写入");}
    if (!invalid.isEmpty()) {message += "\n" + invalid;}
    if (!local_error.isEmpty()) {message += "\n" + local_error;}
    status->setText(message.isEmpty() ? QStringLiteral("未读取；未提交任何写入") : message);
  }
  void confirm()
  {
    if (!permitted()) {return;} const auto s = port->snapshot();
    if (!s->ready || s->pending || s->fault || !s->review || s->review->parameter != parameter()) {
      return;
    }
    const auto review = *s->review; const auto candidate = desired(); std::string json;
    try {json = params::parameter_json(parameter(), candidate);} catch (const std::exception &) {
      return;
    }
    const auto id = QUuid::createUuid().toString(QUuid::Id128).toStdString();
    prompt = new QMessageBox(
      QMessageBox::Warning, QStringLiteral("只写平台元数据，确认单次提交？"),
      QStringLiteral(
        "固定服务 /platform/params/query\n参数 %1\n上次读取 JSON：%2\n待写 JSON：%3\n"
        "确认后先复核旧值；非原子比较，不保证并发或跨重启恢复。\n"
        "回复超时不代表没写入；不会自动重试、撤销或回滚。")
      .arg(
        QString::fromStdString(params::local_parameter_name(parameter())),
        QString::fromStdString(review.json), QString::fromStdString(json)),
      QMessageBox::Yes | QMessageBox::Cancel, owner);
    auto * p = prompt; p->setObjectName("parameterEditPrompt"); p->setTextFormat(Qt::PlainText);
    p->setDefaultButton(QMessageBox::Cancel); p->setEscapeButton(QMessageBox::Cancel);
    p->setButtonText(QMessageBox::Yes, QStringLiteral("确认单次写入")); p->setButtonText(
      QMessageBox::Cancel, QStringLiteral("不写入"));
    QObject::connect(
      p, &QMessageBox::finished, owner, [this, p, review, candidate, json, id](int answer) {
        if (prompt != p) {return;} prompt = nullptr; p->setObjectName(""); p->deleteLater();
        if (answer == QMessageBox::Yes && permitted()) {
          try {
            if (parameter() == review.parameter &&
            params::parameter_json(parameter(), desired()) == json &&
            port->write_confirmed(review, candidate, id)) {local_error.clear();} else {
              local_error = QStringLiteral("确认已失效或写入未入队；请重新读取/确认，不自动重试");
            }
          } catch (const std::exception &) {
            local_error = QStringLiteral("待写值已改变或无效，未提交");
          }
        }
        refresh();
      }); p->open(); refresh();
  }
};
ParameterPanel::ParameterPanel(
  const ParameterOptions & options, ParameterFactory factory,
  QWidget * parent)
: QWidget(parent), impl_(std::make_unique<Impl>(this, options, std::move(factory))) {}
ParameterPanel::~ParameterPanel() = default;
void ParameterPanel::observer_available(bool available)
{
  impl_->available = available;
  if (impl_->port) {impl_->port->inhibit(!available || impl_->inhibited || impl_->closing);}
  if (!available) {impl_->retire();} impl_->refresh();
}
void ParameterPanel::exit_question(bool active)
{
  impl_->inhibited = active; if (active) {impl_->retire();}
  if (impl_->port) {
    impl_->port->inhibit(active || !impl_->available || impl_->closing);
  }
  impl_->refresh();
}
bool ParameterPanel::busy() const
{
  return impl_->prompt || (impl_->port && impl_->port->snapshot()->pending);
}
bool ParameterPanel::fault() const {return impl_->port && impl_->port->snapshot()->fault;}
bool ParameterPanel::stopped() const {return !impl_->port || impl_->port->snapshot()->stopped;}
void ParameterPanel::stop()
{impl_->closing = true; impl_->retire(); if (impl_->port) {impl_->port->stop();} impl_->refresh();}
}  // namespace easyfly_desktop

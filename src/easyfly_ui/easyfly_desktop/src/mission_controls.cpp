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
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QUuid>
#include <QVBoxLayout>
#include <cmath>
#include <array>
#include <cstdlib>
#include <stdexcept>
#include <utility>
#include "easyfly_desktop/mission_controls.hpp"
#include "easyfly_desktop/presentation.hpp"
namespace easyfly_desktop
{
namespace
{
class ManagedPort final : public ControlPort
{
public:
  explicit ManagedPort(const control::ControlConfig & config)
  : client_(config) {}
  std::shared_ptr<const control::ControlSnapshot> snapshot() const override
  {
    return client_.snapshot();
  }
  bool start_confirmed(
    const easyfly_msgs::srv::GetMissionDetails::Response & review, double received,
    const std::string & instance, const std::string & confirmation) override
  {
    return client_.start_confirmed(review, received, instance, confirmation);
  }
  bool cancel_confirmed() override {return client_.cancel_confirmed();}
  void stop() override {client_.stop();}

private:
  control::MissionControl client_;
};
QLabel * plain(const char * name, QWidget * parent)
{
  auto * label = new QLabel(parent); label->setObjectName(name);
  label->setTextFormat(Qt::PlainText); label->setWordWrap(true);
  label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  label->setMinimumWidth(0); label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  return label;
}
QString phase(control::Phase value)
{
  switch (value) {
    case control::Phase::Idle: return QStringLiteral("无本地未决意图");
    case control::Phase::Sent: return QStringLiteral("已派发；接纳待确认");
    case control::Phase::Uncertain: return QStringLiteral("结果不明；保留未决锁，仅查询恢复");
    case control::Phase::Running: return QStringLiteral("执行中（不是完成）");
    case control::Phase::CancelPending: return QStringLiteral("取消待定；ACK不证明结束");
    case control::Phase::DrainPending: return QStringLiteral("公开终态；仍等待所有权排空");
    case control::Phase::Completed: return QStringLiteral("该意图已确认解决（不等于任务成功）");
    case control::Phase::Blocked: return QStringLiteral("故障锁；不重发、不清锁、不接管");
  }
  return QStringLiteral("未知");
}
bool recent(double received, double now, double ttl)
{
  return std::isfinite(received) && received > 0 && now >= received && now - received <= ttl;
}
bool same(const control::Intent & a, const control::Intent & b)
{
  return a.scope == b.scope && a.instance == b.instance && a.request == b.request &&
         a.name == b.name && a.hash == b.hash && a.execution == b.execution && a.goal == b.goal;
}
}  // namespace
ControlFactory control_factory()
{
  return [](const control::ControlConfig & config) {return std::make_unique<ManagedPort>(config);};
}
struct MissionPanel::Impl
{
  MissionPanel * panel;
  MissionOptions options;
  ReviewRead read;
  struct Check
  {
    std::shared_ptr<const easyfly_msgs::srv::GetMissionDetails::Response> review;
    std::string instance, profile, confirmation;
    uint64_t session, epoch;
    double started;
    std::array<uint64_t, 5> requests{};
    std::array<double, 5> next_try{};
  };
  std::optional<Check> check;
  bool advancing{false};
  std::unique_ptr<ControlPort> port;
  std::shared_ptr<const bridge::UiSnapshot> observed;
  std::string selected;
  bool waiting{true}, available{false}, simulation{false}, exiting{false}, stopping{false};
  bool local_queued{false};
  std::string queued_after;
  QString notice;
  uint64_t seen_session{0}, seen_epoch{0};
  bool start_prompt{false};
  QCheckBox * permit;
  QLabel * gate, * status, * dependencies;
  QPushButton * start, * cancel, * stop_check;
  QMessageBox * prompt{nullptr};
  Impl(
    MissionPanel * owner, const MissionOptions & value, ControlFactory factory,
    ReviewRead reader)
  : panel(owner), options(value), read(std::move(reader))
  {
    panel->setObjectName("managedMissionPanel");
    if (!options.enabled || options.journal_directory.empty()) {
      throw std::invalid_argument("explicit enable and persistent directory required");
    }
    auto * layout = new QVBoxLayout(panel); layout->setContentsMargins(0, 0, 0, 0);
    permit = new QCheckBox(
      QStringLiteral("允许新的仿真任务"), panel);
    permit->setObjectName("managedSimulationPermit"); permit->setChecked(false);
    layout->addWidget(permit);
    auto * permit_hint = plain("managedSimulationPermitHint", panel);
    permit_hint->setText(QStringLiteral("勾选即确认本 ROS domain 仅用于仿真且未连接实机。"));
    layout->addWidget(permit_hint);
    dependencies = plain("missionAlgorithmDependencies", panel); layout->addWidget(dependencies);
    gate = plain("missionPreflight", panel); layout->addWidget(gate);
    auto * row = new QVBoxLayout;
    start = new QPushButton(QStringLiteral("审阅并确认启动"), panel);
    start->setObjectName("startMission");
    cancel = new QPushButton(QStringLiteral("确认取消本地登记执行"), panel);
    cancel->setObjectName("cancelOwnedMission"); row->addWidget(start); row->addWidget(cancel);
    layout->addLayout(row);
    stop_check = new QPushButton(QStringLiteral("取消本次启动复核（不发飞行请求）"), panel);
    stop_check->setObjectName("stopMissionRecheck"); layout->addWidget(stop_check);
    status = plain("managedMissionStatus", panel); layout->addWidget(status);
    auto * boundary = plain("managedMissionBoundary", panel);
    boundary->setText(
      QStringLiteral(
        "仅显式启用的仿真任务。配置服务不是实际飞控认证，localhost也不等于没有实机；"
        "前置检查不是飞行准入，后端仍逐动作裁决。取消不要求当前遥测新鲜。"
        "取消只针对登记身份，不采用手输ID。取消勾选仅禁止新启动。"
        "关闭仅停本地通信，远端可能继续执行；记录保留，重启只查询，不重放。"));
    layout->addWidget(boundary);
    try {
      port = factory({options.journal_directory, 2.0});
      if (!port) {throw std::runtime_error("control facade not created");}
    } catch (const std::exception & exc) {status->setText(text(exc.what(), 2048));}
    QObject::connect(permit, &QCheckBox::toggled, panel, [this] {render();});
    QObject::connect(start, &QPushButton::clicked, panel, [this] {confirm_start();});
    QObject::connect(cancel, &QPushButton::clicked, panel, [this] {confirm_cancel();});
    QObject::connect(
      stop_check, &QPushButton::clicked, panel, [this] {
        abort_check(QStringLiteral("已取消只读复核；未发送启动")); render();
      });
    render();
  }
  std::shared_ptr<const control::ControlSnapshot> state() const
  {
    return port ? port->snapshot() : nullptr;
  }
  bool unresolved() const
  {
    const auto s = state(); return check.has_value() || local_queued ||
           (s && (s->phase == control::Phase::Blocked || (s->intent && !s->intent->resolved)));
  }
  bool valid(const bridge::Evidence & e, double now, double ttl, bool clock = false) const
  {
    return observed && recent(e.received_steady, now, ttl) &&
           display_quality(e, *observed, now, ttl, clock) == bridge::Quality::ValidAtObservation;
  }
  QJsonValue parameter(bridge::Key key, double now) const
  {
    if (!observed) {return {};}
    const auto found = observed->parameters.find(key);
    if (found == observed->parameters.end() || !found->second.value ||
      !found->second.value->success || !valid(found->second.evidence, now, 2))
    {return {};}
    const auto & json = found->second.value->current_value;
    if (!bridge::detail::parameter_json(json, key)) {return {};}
    const auto parsed = QJsonDocument::fromJson(
      QByteArray("[") + QByteArray::fromStdString(
        json) + "]");
    if (!parsed.isArray()) {return {};}
    const auto values = parsed.array();
    return values.size() == 1 ? values.at(0) : QJsonValue{};
  }
  QString missing_backends(double now) const
  {return observed ? missing_mission_backends(*observed, now) : QString{};}
  QString preflight(double now, bool reviewing = false) const
  {
    if (check) {return QStringLiteral("正在只读复核已确认内容；尚未发送启动，不接受重复操作");}
    if (exiting || stopping || prompt) {return QStringLiteral("正在确认操作或退出，不接受新操作");}
    if (!port) {return QStringLiteral("控制门面未创建；不会回退原任务接口");}
    const auto s = state();
    if (!s || s->stopped || s->phase == control::Phase::Blocked) {
      return QStringLiteral("控制门面停止/故障；未决锁不得绕过");
    }
    if (local_queued || s->request_pending || (s->intent && !s->intent->resolved) ||
      (s->phase != control::Phase::Idle && s->phase != control::Phase::Completed))
    {return QStringLiteral("已有待定/执行/排空操作，不允许重复启动");}
    if (!permit->isChecked()) {return QStringLiteral("默认只读：请先人工确认隔离仿真环境");}
    const auto localhost = std::getenv("ROS_LOCALHOST_ONLY");
    if (!simulation || !localhost || std::string(localhost) != "1") {
      return QStringLiteral("须模拟时钟与 ROS_LOCALHOST_ONLY=1；不开放系统时钟/实机入口");
    }
    if (!observed || !available || !observed->running || observed->stopped ||
      !recent(observed->produced_steady, now, 1) ||
      observed->clock.quality != bridge::Quality::ValidAtObservation)
    {return QStringLiteral("观测器/模拟时钟不新鲜或已暂停、重置");}
    const auto missing = missing_backends(now);
    if (!missing.isEmpty()) {return missing;}
    if (reviewing) {
      const auto & d = observed->mission_details;
      if (!read || waiting || selected.empty() || !d.value || !d.value->success ||
        d.value->status != "valid_snapshot" || d.value->mission_name != selected ||
        !observed->mission_plan || d.evidence.time_epoch != observed->time_epoch ||
        (d.evidence.quality != bridge::Quality::ValidAtObservation &&
        d.evidence.quality != bridge::Quality::Stale) ||
        (observed->mission_format != "yaml_v1" && observed->mission_format != "legacy_flat"))
      {return QStringLiteral("须先读取并审阅所选任务；确认后将只读复核同一内容");}
      if (observed->command_uncertain_observed || !s->session_ready ||
        !bridge::detail::execution_id(s->server_instance))
      {return QStringLiteral("故障历史或实例未知；不开放启动确认");}
      return {};
    }
    const auto profile = parameter(bridge::Key::Profile, now);
    const auto sim = parameter(bridge::Key::UseSimTime, now);
    const auto name = profile.toString();
    if (!profile.isString()) {return QStringLiteral("profile 查询证据缺失/失效；请只读刷新");}
    if (name != "base_sim" && name != "full_perception" &&
      name != "navigation_dev" && name != "landing_dev")
    {return QStringLiteral("已查询 profile 不在仿真配置白名单");}
    if (name.toStdString() != observed->selected_profile) {
      return QStringLiteral("已查询 profile 与 UI 选择不一致");
    }
    if (!sim.isBool() || !sim.toBool()) {
      return QStringLiteral("use_sim_time 查询缺失/失效或不是布尔 true；请只读刷新");
    }
    const auto & v = observed->state;
    if (!valid(v.evidence, now, 1, true) || !v.sdk.received || !v.sdk.fresh ||
      !v.sdk.source_time_valid || !v.sdk.value.connected || !v.position_valid ||
      !v.velocity_valid || !v.attitude_valid || !v.battery_valid)
    {return QStringLiteral("飞控连接、定位/速度/姿态/电池或时间证据无效；不使用默认零值");}
    if (observed->command_uncertain_observed) {
      return QStringLiteral("已观察到飞控命令结果不明；禁止新启动，不清故障锁");
    }
    const auto flight = observed->capabilities.find(bridge::Key::FlightCapabilities);
    if (flight == observed->capabilities.end() || !flight->second.value ||
      !valid(flight->second.evidence, now, 2, true))
    {return QStringLiteral("飞控能力/命令故障状态缺少新鲜证据");}
    bool fault_known = false;
    for (const auto & detail : flight->second.value->details) {
      const auto uncertain = bridge::detail::command_uncertain(detail);
      if (uncertain && *uncertain) {
        return QStringLiteral("飞控报告 command_uncertain；禁止新启动");
      }
      fault_known = fault_known || uncertain.has_value();
    }
    if (!fault_known) {return QStringLiteral("飞控未报告明确 command_uncertain 状态");}
    const auto & d = observed->mission_details;
    if (waiting || selected.empty() || !d.value || !d.value->success ||
      d.value->status != "valid_snapshot" || d.value->mission_name != selected ||
      !observed->mission_plan || !valid(d.evidence, now, 2) ||
      (observed->mission_format != "yaml_v1" && observed->mission_format != "legacy_flat"))
    {return QStringLiteral("须读取并审阅所选任务的当前详情；旧副本或等待中不可启动");}
    const auto caps = observed->capabilities.find(bridge::Key::MissionCapabilities);
    if (caps == observed->capabilities.end() || !caps->second.value ||
      !valid(caps->second.evidence, now, 2, true))
    {return QStringLiteral("任务后端能力/可用性证据已失效");}
    const auto & c = *caps->second.value;
    const auto kind = observed->mission_format == "yaml_v1" ? "versioned_yaml" : "legacy_mission";
    bool ready = false;
    for (size_t i = 0; i < c.names.size(); ++i) {
      ready = ready || (c.names[i] == kind && i < c.implemented.size() && c.implemented[i] &&
        i < c.ready.size() && c.ready[i]);
    }
    if (!ready) {return QStringLiteral("所选任务格式未实现/后端忙或不可用；ready不是飞行许可");}
    if (!s->session_ready || !recent(s->session_received, now, 2) ||
      !bridge::detail::execution_id(s->server_instance))
    {return QStringLiteral("受管服务实例未取得或已过期；不会回退原 start");}
    return {};
  }
  bool can_cancel() const
  {
    const auto s = state();
    return !exiting && !stopping && !prompt && !check && s && !s->stopped && !s->request_pending &&
           s->phase != control::Phase::Blocked && s->intent && !s->intent->resolved &&
           !s->intent->cancel_dispatched && bridge::detail::execution_id(s->intent->execution);
  }
  void render()
  {
    const auto s = state();
    if (local_queued && s && s->intent && s->intent->request != queued_after) {
      local_queued = false; notice.clear();
    }
    const auto dependency = missing_backends(bridge::detail::steady_seconds());
    dependencies->setText(
      !observed || !observed->mission_plan ? QStringLiteral("读取任务后显示算法依赖。") :
      dependency.isEmpty() ? QStringLiteral(
        "任务所需接口已发现；输入数据、算法就绪"
        "和飞行准入仍由后端检查。") : dependency);
    const auto reason = preflight(bridge::detail::steady_seconds(), true);
    gate->setText(
      (reason.isEmpty() ? QStringLiteral(
        "可审阅已读取副本；确认后只读复核同一内容与新鲜配置/能力，不是飞行许可。") :
      reason) + (notice.isEmpty() ? QString{} : "\n" + notice));
    start->setEnabled(reason.isEmpty()); cancel->setEnabled(can_cancel());
    permit->setEnabled(!exiting && !stopping && !prompt && !check);
    stop_check->setEnabled(check.has_value() && !exiting && !stopping);
    if (!s) {return;}
    QString description = phase(s->phase) + " · " + text(s->status, 64) + "\n" + text(
      s->message,
      2048);
    if (local_queued) {
      description.prepend(QStringLiteral("新意图本地入队，等待身份；不是已执行。\n"));
    }
    if (s->intent) {
      const auto & v = *s->intent;
      description += QStringLiteral(
        "\n登记任务：%1\nscope：%2\n服务器实例：%3\n客户端请求：%4\n"
        "execution_id：%5\nAction UUID：%6\n原内容 SHA-256：%7\n"
        "取消已派发：%8 · 已确认解决：%9\n与目录选择/手输查询 ID 无关。")
        .arg(
        text(v.name, 128), text(v.scope), text(v.instance), text(v.request),
        text(v.execution), v.goal.empty() ? QStringLiteral("未接纳/未知") : text(v.goal),
        text(v.hash), v.cancel_dispatched ? "true" : "false", v.resolved ? "true" : "false");
    }
    description += QStringLiteral("\n持久记录目录：%1").arg(text(options.journal_directory, 4096));
    status->setText(description);
  }
  void finish_prompt()
  {
    if (!prompt) {return;}
    prompt->setObjectName(""); prompt->deleteLater(); prompt = nullptr;
  }
  QMessageBox * question(const QString & title, const QString & body, const char * name)
  {
    auto * box = new QMessageBox(
      QMessageBox::Warning, title, body,
      QMessageBox::Yes | QMessageBox::Cancel, panel);
    box->setObjectName(name); box->setTextFormat(Qt::PlainText);
    box->setDefaultButton(QMessageBox::Cancel); box->setEscapeButton(QMessageBox::Cancel);
    box->setButtonText(QMessageBox::Yes, QStringLiteral("确认此操作"));
    box->setButtonText(QMessageBox::Cancel, QStringLiteral("返回"));
    return box;
  }
  void confirm_start()
  {
    if (!preflight(bridge::detail::steady_seconds(), true).isEmpty()) {return;}
    const auto before = state();
    const auto review = observed->mission_details.value;
    const auto ev = observed->mission_details.evidence;
    const auto session = observed->observation_session_id, epoch = observed->time_epoch;
    const auto instance = before->server_instance;
    const auto profile = observed->selected_profile;
    const auto confirmation = QUuid::createUuid().toString(QUuid::Id128).toStdString();
    const auto step_count = observed->mission_plan->steps.size();
    const auto units = observed->mission_format == "yaml_v1" ?
      QStringLiteral("文件声明") : QStringLiteral("legacy 格式执行约定，非文件显式声明");
    start_prompt = true; notice.clear();
    prompt = question(
      QStringLiteral("确认启动已审阅仿真任务"),
      QStringLiteral(
        "任务：%1\n步骤：%2\n单位/坐标：odom · m/rad/s（%6）\nSHA-256：%3\n"
        "UI 所选仿真 profile：%4（启动前只读核对，不是飞控认证或任务字段）\n服务实例：%5\n"
        "严格顺序，失败即终止；不自动追加返航/降落/上锁。\n"
        "可充分审阅。确认后只读复核同一内容及新鲜证据；变化/失败/超时不启动，须重新审阅。"
        "复核成功才发送一次启动，不自动重试飞行请求。")
      .arg(text(review->mission_name)).arg(step_count)
      .arg(text(review->content_sha256), text(profile), text(instance), units),
      "missionStartPrompt");
    const auto source_prompt = prompt;
    control::audit_event("prompt_opened", confirmation, instance, {}, review->content_sha256);
    QObject::connect(
      prompt, &QMessageBox::finished, panel,
      [this, source_prompt, review, ev, session, epoch, instance, profile, confirmation](
        int answer) {
        if (prompt != source_prompt) {return;}  // Only the currently open prompt may decide once.
        finish_prompt();
        control::audit_event(
          answer == QMessageBox::Yes ? "prompt_yes" : "prompt_dismissed",
          confirmation, instance, {}, review->content_sha256);
        if (answer == QMessageBox::Yes && observed &&
        preflight(bridge::detail::steady_seconds(), true).isEmpty() &&
        observed->observation_session_id == session && observed->time_epoch == epoch &&
        observed->selected_profile == profile && observed->mission_details.value &&
        observed->mission_details.evidence.request_id == ev.request_id &&
        observed->mission_details.value->mission_name == review->mission_name &&
        observed->mission_details.value->content_sha256 == review->content_sha256 &&
        state()->server_instance == instance)
        {
          check = Check{review, instance, profile, confirmation, session, epoch,
            bridge::detail::steady_seconds()};
          notice = QStringLiteral("确认已收到；只读复核期间尚未发送启动");
        } else if (answer == QMessageBox::Yes) {
          notice = QStringLiteral("确认所依据的内容/会话/时间/前置证据已变化；未发送，不自动重试");
        }
        render();
      });
    render(); prompt->open();
  }
  void confirm_cancel()
  {
    if (!can_cancel()) {return;}
    const auto owned = *state()->intent;
    start_prompt = false;
    prompt = question(
      QStringLiteral("确认精确取消登记执行"),
      QStringLiteral(
        "任务：%1\n服务实例：%2\n请求：%3\nexecution_id：%4\nAction UUID：%5\n"
        "只取消该登记执行，不采用手输ID、不取消全部。\n取消ACK不证明结束，"
        "仍等待真实终态与所有权排空；丢回复不自动重试。")
      .arg(
        text(owned.name), text(owned.instance), text(owned.request), text(owned.execution),
        owned.goal.empty() ? QStringLiteral("尚未取得") : text(owned.goal)), "missionCancelPrompt");
    QObject::connect(
      prompt, &QMessageBox::finished, panel, [this, source_prompt = prompt, owned](int answer) {
        if (prompt != source_prompt) {return;}
        finish_prompt(); const auto s = state();
        if (answer == QMessageBox::Yes && can_cancel() && s->intent && same(*s->intent, owned)) {
          notice = port->cancel_confirmed() ? QStringLiteral(
            "取消已本地入队；仍须等待真实终态与排空") :
          QStringLiteral("取消未能入队；未自动重试");
        } else if (answer == QMessageBox::Yes) {
          notice = QStringLiteral(
            "取消确认所依据的登记身份/状态已变化；未发送，不自动重试");
        }
        render();
      });
    render(); prompt->open();
  }
  void dismiss()
  {
    if (prompt) {prompt->reject();}
  }
  void abort_check(const QString & reason)
  {
    if (check) {
      control::audit_event(
        "recheck_aborted", check->confirmation, check->instance, {},
        check->review->content_sha256);
      check.reset(); notice = reason;
    }
  }
  void advance_check()
  {
    if (!check || advancing) {return;}
    const auto now = bridge::detail::steady_seconds();
    const auto s = state();
    if (!observed || exiting || stopping || !available || !permit->isChecked() ||
      observed->observation_session_id != check->session || observed->time_epoch != check->epoch ||
      observed->selected_profile != check->profile || selected != check->review->mission_name ||
      !s || s->stopped || s->phase == control::Phase::Blocked ||
      s->server_instance != check->instance || (s->intent && !s->intent->resolved))
    {abort_check(QStringLiteral("复核身份/会话/选择已变化；未发送启动")); return;}
    if (now - check->started > 5) {
      abort_check(QStringLiteral("只读复核超时；未发送启动，不自动重试")); return;
    }
    const std::array<bridge::Key, 5> keys{bridge::Key::Profile, bridge::Key::UseSimTime,
      bridge::Key::MissionDetails, bridge::Key::FlightCapabilities,
      bridge::Key::MissionCapabilities};
    bool complete = true;
    advancing = true;
    for (size_t i = 0; i < keys.size() && check; ++i) {
      const auto key = keys[i];
      const auto evidence = [&]() -> const bridge::Evidence * {
          if (i == 2) {return &observed->mission_details.evidence;}
          if (i < 2) {
            const auto item = observed->parameters.find(key);
            return item == observed->parameters.end() ? nullptr : &item->second.evidence;
          }
          const auto item = observed->capabilities.find(key);
          return item == observed->capabilities.end() ? nullptr : &item->second.evidence;
        }();
      // Capability polling is already bounded by the Observer. A newer reply in this
      // same session/epoch is sufficient, even if its polling request superseded ours.
      if (evidence && evidence->received_steady >= check->started &&
        ((i >= 3) || (check->requests[i] && evidence->request_id == check->requests[i])))
      {
        if (!valid(*evidence, now, 2, i >= 3)) {
          abort_check(QStringLiteral("只读复核证据失败/过期；未发送启动")); break;
        }
        continue;
      }
      if (i < 3 && check->requests[i] && evidence && evidence->request_id > check->requests[i]) {
        abort_check(QStringLiteral("新的读取替代了确认复核；未发送启动")); break;
      }
      complete = false;
      if (check->requests[i] || now < check->next_try[i]) {continue;}
      const auto receipt = read(key, check->review->mission_name);
      if (!check) {break;}
      if (receipt.status == bridge::RequestStatus::Queued && receipt.id) {
        check->requests[i] = receipt.id; continue;
      }
      if (receipt.status == bridge::RequestStatus::Busy ||
        receipt.status == bridge::RequestStatus::RateLimited)
      {check->next_try[i] = now + .2; continue;}
      abort_check(QStringLiteral("无法排队只读复核；未发送启动")); break;
    }
    advancing = false;
    if (!check || !complete || s->request_pending) {return;}
    const auto original = *check; check.reset();
    const auto & d = observed->mission_details;
    const auto reason = preflight(bridge::detail::steady_seconds());
    if (!reason.isEmpty() || !d.value || d.value->mission_name != original.review->mission_name ||
      d.value->content_sha256 != original.review->content_sha256 ||
      d.value->content != original.review->content)
    {
      notice = QStringLiteral("复核内容/准入证据已变化或无效；未发送，必须重新审阅。") + reason;
      return;
    }
    queued_after = s->intent ? s->intent->request : "";
    control::audit_event(
      "recheck_consumed", original.confirmation, original.instance, {},
      d.value->content_sha256);
    local_queued = port->start_confirmed(
      *d.value, d.evidence.received_steady, original.instance,
      original.confirmation);
    control::audit_event(
      local_queued ? "frontend_queued" : "frontend_refused",
      original.confirmation, original.instance, {}, d.value->content_sha256);
    notice = local_queued ? QStringLiteral("同内容复核通过；启动已本地入队，等待接纳") :
      QStringLiteral("控制门面拒绝入队；没有自动重试");
  }
};
MissionPanel::MissionPanel(
  const MissionOptions & options, ControlFactory factory, QWidget * parent, ReviewRead read)
: QWidget(parent), impl_(std::make_unique<Impl>(this, options, std::move(factory), std::move(read)))
{
}
MissionPanel::~MissionPanel() {impl_->dismiss();}
void MissionPanel::observe(
  std::shared_ptr<const bridge::UiSnapshot> snapshot, const std::string & selection,
  bool waiting, bool available, bool simulation)
{
  impl_->observed = std::move(snapshot); impl_->selected = selection;
  impl_->waiting = waiting; impl_->available = available; impl_->simulation = simulation;
  if (impl_->observed && (impl_->seen_session != impl_->observed->observation_session_id ||
    impl_->seen_epoch != impl_->observed->time_epoch))
  {
    impl_->seen_session = impl_->observed->observation_session_id;
    impl_->seen_epoch = impl_->observed->time_epoch;
    impl_->permit->setChecked(false);
    impl_->abort_check(QStringLiteral("观测会话/时钟已变化；复核终止，未发送启动"));
    if (impl_->start_prompt) {impl_->dismiss();}
  }
  impl_->advance_check();
  impl_->render();
}
bool MissionPanel::unresolved() const {return impl_->unresolved();}
std::shared_ptr<const control::ControlSnapshot> MissionPanel::record_state() const
{
  return impl_->state();
}
bool MissionPanel::stopped() const {const auto s = impl_->state(); return !s || s->stopped;}
void MissionPanel::stop()
{
  impl_->stopping = true; impl_->dismiss();
  impl_->abort_check(QStringLiteral("本地已停止；复核终止，未发送启动"));
  if (impl_->port) {impl_->port->stop();}
  impl_->render();
}
void MissionPanel::exit_question(bool active)
{
  impl_->exiting = active;
  if (active) {
    impl_->dismiss(); impl_->abort_check(QStringLiteral("正在退出；复核终止，未发送启动"));
  }
  impl_->render();
}
}  // namespace easyfly_desktop

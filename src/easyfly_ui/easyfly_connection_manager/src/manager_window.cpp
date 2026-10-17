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
#include <QAction>
#include <QCoreApplication>
#include <QFileInfo>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QElapsedTimer>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QMenu>
#include <QMenuBar>
#include <QPlainTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QSplitter>
#include <QTimer>
#include <QVBoxLayout>
#include <QUuid>

#include "easyfly_connection_manager/manager_window.hpp"
#include "easyfly_connection_manager/profile.hpp"
#include "easyfly_connection_manager/rpc.hpp"

namespace easyfly_connection_manager
{
struct ManagerWindow::Impl
{
  ManagerWindow * owner;
  QString config_file, runtime;
  QVector<Profile> profiles;
  QMap<QString, QString> persisted_hashes;
  QElapsedTimer response_clock;
  int selected{-1}, retries{0};
  bool healthy{true}, loading{false}, service_started{false}, queried{false};
  QString poll_request, exit_request;
  QJsonObject actual;
  RpcClient rpc;
  QTimer timer;
  QListWidget * list;
  QLineEdit * name, * front, * down;
  QSpinBox * domain;
  QComboBox * scope, * profile;
  QCheckBox * simulation;
  QLabel * note, * summary, * effective;
  QPlainTextEdit * diagnostics;
  QPushButton * save, * remove, * start, * stop;

  Impl(ManagerWindow * window, const QString & file, const QString & path)
  : owner(window), config_file(file), runtime(private_directory(path))
  {
    owner->setWindowTitle(QStringLiteral("EasyFly 连接管理"));
    auto * service_menu = owner->menuBar()->addMenu(QStringLiteral("管理服务"));
    auto * retry_service = service_menu->addAction(QStringLiteral("重试连接管理服务"));
    QObject::connect(
      retry_service, &QAction::triggered, owner, [this] {
        retries = 0; service_started = false;
        if (!rpc.connected()) {rpc.connect_to(runtime);}
      });
    auto * stop_service = service_menu->addAction(QStringLiteral("结束空闲管理服务并退出"));
    QObject::connect(
      stop_service, &QAction::triggered, owner, [this] {
        if (!queried || actual["process_running"].toBool()) {
          notice(QStringLiteral("请先正常关闭工作会话；当前不能结束管理服务。")); return;
        }
        exit_request = rpc.send("exit_service");
      });
    owner->resize(1080, 740);
    auto * central = new QWidget(owner); owner->setCentralWidget(central);
    auto * main = new QVBoxLayout(central);
    auto * title = new QLabel(QStringLiteral("连接已有 EasyFly 平台"), central);
    title->setStyleSheet("font-size:22px;font-weight:600;color:#17263b"); main->addWidget(title);
    auto * hint = new QLabel(
      QStringLiteral(
        "选择通信配置后打开独立工作窗口。关闭本窗口会保留工作会话；连接不启动后端。"), central);
    hint->setWordWrap(true); main->addWidget(hint);
    auto * split = new QSplitter(central); main->addWidget(split, 1);
    auto * left = new QWidget(split); auto * left_layout = new QVBoxLayout(left);
    list = new QListWidget(left); list->setObjectName("connectionList");
    left_layout->addWidget(list);
    auto * actions = new QHBoxLayout; left_layout->addLayout(actions);
    const auto button = [this](const QString & text, const char * id, QLayout * layout) {
        auto * b = new QPushButton(text, owner); b->setObjectName(id); layout->addWidget(b);
        return b;
      };
    auto * add = button(QStringLiteral("新建"), "newConnection", actions);
    auto * copy = button(QStringLiteral("复制"), "copyConnection", actions);
    remove = button(QStringLiteral("删除"), "deleteConnection", actions);
    auto * transfer = new QHBoxLayout; left_layout->addLayout(transfer);
    auto * import = button(QStringLiteral("导入"), "importConnections", transfer);
    auto * export_button = button(QStringLiteral("导出"), "exportConnections", transfer);
    auto * right = new QWidget(split); auto * layout = new QVBoxLayout(right);
    auto * group = new QGroupBox(QStringLiteral("已有平台观测配置"), right);
    auto * form = new QFormLayout(group); layout->addWidget(group);
    name = new QLineEdit(group); name->setObjectName("connectionName"); name->setMaxLength(64);
    domain = new QSpinBox(group); domain->setObjectName("connectionDomain");
    domain->setRange(0, 232);
    domain->setToolTip(QStringLiteral("与目标平台保持一致；新建环境建议选择 0–101。"));
    scope = new QComboBox(group); scope->setObjectName("connectionScope");
    scope->addItems({QStringLiteral("仅本机"), QStringLiteral("局域网")});
    simulation = new QCheckBox(QStringLiteral("使用模拟时间，需要持续有效的 /clock"), group);
    simulation->setObjectName("connectionSimulationTime");
    profile = new QComboBox(group); profile->setObjectName("connectionProfile");
    profile->addItems({"base_sim", "full_perception", "navigation_dev", "landing_dev"});
    front = new QLineEdit(group); front->setObjectName("connectionFrontTopic"); front->setMaxLength(
      256);
    down = new QLineEdit(group); down->setObjectName("connectionDownTopic");
    down->setMaxLength(256);
    form->addRow(QStringLiteral("名称"), name); form->addRow(QStringLiteral("ROS domain"), domain);
    form->addRow(QStringLiteral("通信范围"), scope); form->addRow(simulation);
    form->addRow(QStringLiteral("UI profile 标签"), profile);
    form->addRow(QStringLiteral("前视 topic（空为禁用）"), front);
    form->addRow(QStringLiteral("下视 topic（空为禁用）"), down);
    auto * supported = new QLabel(
      QStringLiteral(
        "Humble / Fast DDS 默认发现。局域网需要网络可达和相容的通信设置。"
        "此入口连接已运行的平台；高级发现配置尚不可用。"), group);
    supported->setWordWrap(true); form->addRow(supported);
    auto * commands = new QHBoxLayout; layout->addLayout(commands);
    save = button(QStringLiteral("保存配置"), "saveConnection", commands);
    start = button(QStringLiteral("连接观测"), "startConnection", commands);
    stop = button(QStringLiteral("正常关闭工作会话"), "closeConnection", commands);
    summary = new QLabel(right); summary->setObjectName("connectionActualState");
    summary->setTextFormat(Qt::PlainText); summary->setWordWrap(true); layout->addWidget(summary);
    effective = new QLabel(right); effective->setTextFormat(Qt::PlainText);
    effective->setObjectName("connectionEffectiveConfig"); effective->setWordWrap(true);
    layout->addWidget(effective);
    diagnostics = new QPlainTextEdit(right); diagnostics->setObjectName("connectionDiagnostics");
    diagnostics->setReadOnly(true); diagnostics->setMaximumBlockCount(200);
    auto * details = new QPushButton(QStringLiteral("展开连接诊断"), right);
    details->setCheckable(true); layout->addWidget(details);
    diagnostics->hide();
    QObject::connect(details, &QPushButton::toggled, diagnostics, &QWidget::setVisible);
    layout->addWidget(diagnostics, 1);
    note = new QLabel(central); note->setObjectName("connectionNotice"); note->setTextFormat(
      Qt::PlainText);
    note->setWordWrap(true); main->addWidget(note);
    split->setSizes({260, 780});
    owner->setStyleSheet(
      "QWidget {font-size:14px;} QMainWindow {background:#f7f9fc;} "
      "QGroupBox {margin-top:10px;padding:12px;} QPushButton {padding:7px;} "
      "QLineEdit,QSpinBox,QComboBox {min-height:26px;}");

    try {
      profiles = read_profiles(config_file);
      for (const auto & p : profiles) {persisted_hashes.insert(p.id, p.hash());}
    } catch (const std::exception & error) {
      healthy = false;
      notice(QString::fromUtf8(error.what()) + QStringLiteral("；保留原文件，不覆盖。"));
    }
    if (healthy && profiles.isEmpty()) {
      profiles.push_back(Profile::create(QStringLiteral("本机观测")));
    }
    QObject::connect(
      list, &QListWidget::currentRowChanged, owner, [this](int row) {
        if (loading || row < 0) {return;}
        if (selected >= 0 && selected < profiles.size() && dirty() &&
        QMessageBox::question(
          owner, QStringLiteral("未保存配置"),
          QStringLiteral("放弃当前未保存的编辑并切换？"), QMessageBox::Yes | QMessageBox::No,
          QMessageBox::No) != QMessageBox::Yes)
        {
          QSignalBlocker guard(list); list->setCurrentRow(selected); return;
        }
        load(row);
      });
    const auto changed = [this] {if (!loading) {update();}};
    for (auto * edit : {name, front, down}) {
      QObject::connect(edit, &QLineEdit::textChanged, owner, changed);
    }
    QObject::connect(domain, QOverload<int>::of(&QSpinBox::valueChanged), owner, changed);
    QObject::connect(scope, QOverload<int>::of(&QComboBox::currentIndexChanged), owner, changed);
    QObject::connect(profile, QOverload<int>::of(&QComboBox::currentIndexChanged), owner, changed);
    QObject::connect(simulation, &QCheckBox::toggled, owner, changed);
    QObject::connect(save, &QPushButton::clicked, owner, [this] {save_current();});
    QObject::connect(add, &QPushButton::clicked, owner, [this] {create(false);});
    QObject::connect(copy, &QPushButton::clicked, owner, [this] {create(true);});
    QObject::connect(
      remove, &QPushButton::clicked, owner, [this] {
        if (!editable() || selected < 0) {return;}
        if (QMessageBox::question(
          owner, QStringLiteral("删除连接"),
          QStringLiteral("只删除此配置，保留日志、录像和任务记录。"),
          QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) {return;}
        auto next = profiles; next.removeAt(selected);
        try {write_profiles(config_file, next); profiles = next; selected = -1; refresh(0);} catch (
          const std::exception & error) {notice(error.what());}
      });
    QObject::connect(
      export_button, &QPushButton::clicked, owner, [this] {
        const auto path = QFileDialog::getSaveFileName(
          owner, QStringLiteral("导出到私有目录"),
          QFileInfo(config_file).absolutePath() + "/connections-export.json", "JSON (*.json)");
        if (path.isEmpty()) {return;}
        try {
          write_profiles(path, profiles);
          notice(QStringLiteral("已导出保存的配置，未连接。"));
        } catch (const std::exception & error) {notice(error.what());}
      });
    QObject::connect(
      import, &QPushButton::clicked, owner, [this] {
        if (!healthy) {return;}
        const auto path = QFileDialog::getOpenFileName(
          owner, QStringLiteral("从私有目录导入"),
          QFileInfo(config_file).absolutePath(), "JSON (*.json)");
        if (path.isEmpty()) {return;}
        try {
          const auto imported = read_profiles(path); auto next = profiles;
          for (auto p : imported) {
            p.id = QUuid::createUuid().toString(QUuid::WithoutBraces); next << p;
          }
          write_profiles(config_file, next); profiles = next;
          for (const auto & p : profiles) {persisted_hashes.insert(p.id, p.hash());}
          refresh(profiles.size() - 1);
          notice(QStringLiteral("已导入为新配置；未启动任何连接。"));
        } catch (const std::exception & error) {notice(error.what());}
      });
    QObject::connect(
      start, &QPushButton::clicked, owner, [this] {
        if (!start->isEnabled()) {return;}
        start->setEnabled(false);
        rpc.send("start", {{"profile", form_profile().json()}});
      });
    QObject::connect(
      stop, &QPushButton::clicked, owner, [this] {
        rpc.send("close", {{"session", actual["session"]}});
      });
    QObject::connect(
      &rpc, &RpcClient::ready, owner, [this] {
        retries = 0; queried = false;
        poll_request = rpc.send("attach_manager", {{"config_file", config_file}});
      });
    QObject::connect(
      &rpc, &RpcClient::reply, owner, [this](const QJsonObject & reply) {
        if (reply["event"] == "raise") {
          owner->show(); owner->raise(); owner->activateWindow(); return;
        }
        if (reply["id"].toString() == poll_request) {poll_request.clear();}
        if (!exit_request.isEmpty() && reply["id"].toString() == exit_request &&
        reply["ok"].toBool()) {owner->close(); return;}
        if (!reply["ok"].toBool()) {notice(reply["error"].toString());}
        if (reply["status"].isObject()) {queried = true; actual = reply["status"].toObject();}
        response_clock.restart();
        update();
      });
    QObject::connect(
      &rpc, &RpcClient::failure, owner, [this](const QString & error) {
        queried = false; poll_request.clear(); notice(error); update();
      });
    timer.setInterval(500);
    QObject::connect(
      &timer, &QTimer::timeout, owner, [this] {
        if (response_clock.isValid() && response_clock.elapsed() > 2500) {
          queried = false; poll_request.clear(); update();
        }
        if (rpc.connected()) {
          if (poll_request.isEmpty()) {poll_request = rpc.send("status");}
        } else if (++retries <= 10) {
          if (!service_started) {
            service_started = true;
            const auto program = QCoreApplication::applicationDirPath() +
            "/easyfly_connection_supervisor";
            if (!QProcess::startDetached(program, {"--runtime-directory", runtime})) {
              notice(QStringLiteral("无法启动连接管理服务"));
            }
          }
          rpc.connect_to(runtime);
        }
      });
    refresh(0); timer.start(); rpc.connect_to(runtime);
  }
  void notice(const QString & text) {note->setText(text.left(2048));}
  ~Impl()
  {
    timer.stop();
    QObject::disconnect(&rpc, nullptr, owner, nullptr);
  }
  Profile form_profile() const
  {
    Profile p = selected >= 0 && selected < profiles.size() ? profiles[selected] : Profile{};
    p.name = name->text(); p.domain = domain->value(); p.local_only = scope->currentIndex() == 0;
    p.simulation_time = simulation->isChecked(); p.selected_profile = profile->currentText();
    p.front_topic = front->text(); p.down_topic = down->text(); return p;
  }
  bool dirty() const
  {
    return selected >= 0 && selected < profiles.size() &&
           form_profile().json() != profiles[selected].json();
  }
  bool editable() const
  {
    return healthy && selected >= 0 && selected < profiles.size() &&
           queried && !(actual["process_running"].toBool() &&
           actual["profile"].toObject()["id"] == profiles[selected].id);
  }
  void load(int row)
  {
    selected = row; loading = true;
    const auto p = profiles.value(row);
    name->setText(p.name); domain->setValue(p.domain); scope->setCurrentIndex(p.local_only ? 0 : 1);
    simulation->setChecked(p.simulation_time);
    profile->setCurrentText(p.selected_profile); front->setText(p.front_topic); down->setText(
      p.down_topic);
    loading = false; update();
  }
  void refresh(int row)
  {
    loading = true; list->clear();
    for (const auto & p : profiles) {
      list->addItem(p.name + QStringLiteral(" · domain %1").arg(p.domain));
    }
    loading = false;
    if (!profiles.isEmpty()) {list->setCurrentRow(qBound(0, row, profiles.size() - 1));} else {
      selected = -1; load(-1);
    }
    update();
  }
  void create(bool duplicate)
  {
    if (!healthy || profiles.size() >= 32) {
      notice(QStringLiteral("配置不可用或已达 32 条上限")); return;
    }
    if (dirty()) {notice(QStringLiteral("请先保存或放弃当前编辑")); return;}
    auto p = duplicate &&
      selected >= 0 ? profiles[selected] : Profile::create(QStringLiteral("新连接"));
    p.id = QUuid::createUuid().toString(QUuid::WithoutBraces);
    if (duplicate) {p.name = p.name.left(60) + QStringLiteral(" 副本");}
    profiles.push_back(p); refresh(profiles.size() - 1);
    notice(QStringLiteral("新配置尚未保存；请检查后点击保存。"));
  }
  void save_current()
  {
    if (!editable()) {return;}
    try {
      auto p = form_profile(); p.validate();
      auto next = profiles; next[selected] = p;
      write_profiles(config_file, next); profiles = next;
      for (const auto & item : profiles) {persisted_hashes.insert(item.id, item.hash());}
      const auto row = selected; selected = -1; refresh(row);
      notice(QStringLiteral("配置已保存；保存不会启动连接。"));
    } catch (const std::exception & error) {notice(error.what());}
  }
  void update()
  {
    const bool edit = editable();
    for (QWidget * widget :
      QList<QWidget *>{name, domain, scope, simulation, profile, front, down})
    {
      widget->setEnabled(edit);
    }
    save->setEnabled(edit); remove->setEnabled(edit);
    bool valid = false;
    try {form_profile().validate(); valid = true;} catch (const std::exception &) {}
    const bool persisted = healthy && !dirty() && selected >= 0 &&
      persisted_hashes.value(profiles[selected].id) == profiles[selected].hash();
    start->setEnabled(
      queried && rpc.connected() && !actual["process_running"].toBool() &&
      !actual["close_pending"].toBool() && valid && persisted);
    const auto workspace = actual["workspace"].toObject();
    stop->setEnabled(
      queried && actual["process_running"].toBool() &&
      actual["workspace_fresh"].toBool() && !workspace["unresolved"].toBool(true));
    QString state = QStringLiteral("管理服务连接中");
    if (queried) {
      if (!actual["process_running"].toBool()) {
        state = actual["phase"] == "failed" ? QStringLiteral("工作会话失败") :
          QStringLiteral("未连接观测");
      } else if (!actual["workspace_fresh"].toBool()) {
        state = QStringLiteral("工作进程运行，观测状态未知");
      } else {state = workspace["summary"].toString(QStringLiteral("等待工作快照"));}
    }
    summary->setText(state);
    const auto active = actual["profile"].toObject();
    effective->setText(
      active.isEmpty() ? QStringLiteral("当前无生效连接") :
      QStringLiteral("生效连接：%1 · domain %2 · %3\n生效版本：%4")
      .arg(active["name"].toString()).arg(active["domain"].toInt())
      .arg(
        active["simulation_time"].toBool() ? QStringLiteral(
          "模拟时间") : QStringLiteral("系统时间"))
      .arg(actual["config_hash"].toString().left(12)));
    const auto text = QJsonDocument(actual).toJson(QJsonDocument::Indented);
    if (diagnostics->toPlainText() != QString::fromUtf8(text)) {
      diagnostics->setPlainText(QString::fromUtf8(text));
    }
  }
};
ManagerWindow::ManagerWindow(const QString & file, const QString & runtime, QWidget * parent)
: QMainWindow(parent), impl_(std::make_unique<Impl>(this, file, runtime)) {}
ManagerWindow::~ManagerWindow() = default;
}  // namespace easyfly_connection_manager

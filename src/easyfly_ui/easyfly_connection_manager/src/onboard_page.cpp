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

#include "easyfly_connection_manager/onboard_page.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QCryptographicHash>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QGridLayout>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollArea>
#include <QSet>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QSplitter>
#include <QStackedWidget>
#include <QTabWidget>
#include <QTableWidget>
#include <QTimer>
#include <QTreeWidget>
#include <QUuid>
#include <QVBoxLayout>
#include <algorithm>

namespace easyfly_connection_manager
{
namespace
{
QString json(const QJsonObject & value)
{
  return QString::fromUtf8(QJsonDocument(value).toJson(QJsonDocument::Indented));
}
QString phaseText(const QString & phase)
{
  static const QMap<QString, QString> names{
    {"stopped", QStringLiteral("已停止")}, {"failed", QStringLiteral("失败")},
    {"starting", QStringLiteral("启动中")}, {"running", QStringLiteral("运行中")},
    {"degraded", QStringLiteral("部分输出未就绪")}, {"ready", QStringLiteral("已就绪")},
    {"waiting_for_data", QStringLiteral(
        "等待"
        "数据")}, {"failed_owned", QStringLiteral(
        "异常，后"
        "台保留")},
    {"stopping", QStringLiteral(
        "正常停止"
        "中")}, {"stop_unknown", QStringLiteral(
        "停止结果"
        "未确认")},
    {"owned_descendants_remain", QStringLiteral("仍有受管子进程")},
    {"unresolved_after_service_restart", QStringLiteral(
        "服务重启，原会话待核"
        "对")}};
  return names.value(phase, phase);
}
QString groupFor(const QString & id, const QJsonObject & spec)
{
  if (spec.contains("group")) {return spec["group"].toString();}
  if (QStringList{"mavlink", "realsense", "livox", "camera_calib", "coord"}.contains(id)) {
    return "devices";
  }
  if (QStringList{"fast_lio", "map_odom"}.contains(id)) {return "localization";}
  if (QStringList{"pointcloud_filter", "semantic_map"}.contains(id)) {return "mapping";}
  if (QStringList{"navigation", "local_planner"}.contains(id)) {return "navigation";}
  if (QStringList{"flight_control", "mission_fsm", "mission_services"}.contains(id)) {
    return "services";
  }
  return "perception";
}
}  // namespace

struct OnboardPage::Impl
{
  OnboardPage * owner;
  QString runtime, pending, kind, operation, loaded_hash, upload_component, upload_key,
    task_request;
  // QTextDocument normalizes line endings; retain the original source for transfer and drafts.
  QString mission_source, focused_mission;
  QJsonObject catalog, recipe, snapshot, target, draft_target, review, saved_mission,
    editor_overrides, parameter_templates, parameter_sources, mission_candidate,
    submitted_mission, submitted_mission_target;
  QByteArray input;
  QProcess bridge;
  QTimer timer;
  QElapsedTimer fresh_clock, poll_clock;
  bool loading{false}, dirty{false}, local_dirty{false}, mission_dirty{false};
  bool task_pending{false}, last_ok{false}, closing{false}, mission_center_active{true};
  quint64 mission_document_serial{0};
  quint64 mission_saved_serial{0};
  quint64 recipe_revision{0}, preparing_revision{0};
  QSet<QString> prepare_queue;
  QStringList preparing_enable;
  QPushButton * advanced_edit{nullptr};
  QLabel * identity, * result, * recipe_state, * plan, * conditions, * task_identity, * task_policy;
  QLabel * connection_status, * runtime_status, * draft_status;
  QLabel * algorithm_title, * algorithm_description, * algorithm_inputs, * algorithm_dependencies;
  QLabel * algorithm_outputs, * algorithm_notes;
  QComboBox * recent, * log_component;
  QListWidget * missions;
  QLineEdit * alias, * host, * recipe_name, * mission_name;
  QSpinBox * domain;
  QCheckBox * allow_missions, * autostart, * permit;
  QPushButton * local, * remote, * recent_connect, * read, * save, * check, * start, * stop;
  QPushButton * logs, * export_connection, * install_service, * task_read, * task_save;
  QPushButton * task_start, * task_lookup, * task_cancel, * task_edit;
  QLabel * transfer_target, * transfer_candidate;
  QTreeWidget * devices, * algorithms;
  QFormLayout * device_form, * algorithm_form, * device_overview_form, * algorithm_overview_form;
  QScrollArea * device_parameter_editor, * algorithm_parameter_editor;
  QStackedWidget * algorithm_side_panel;
  QSplitter * device_columns, * algorithm_columns;
  QPlainTextEdit * raw_state, * log_text, * mission_text;
  QTableWidget * states;
  QWidget * mission_page, * mission_transfer_page;
  QString selected_device, selected_algorithm;

  explicit Impl(OnboardPage * page, const QString & directory)
  : owner(page), runtime(directory)
  {
    auto * layout = new QVBoxLayout(owner);
    layout->setContentsMargins(0, 0, 0, 0); layout->setSpacing(6);
    auto * status_row = new QHBoxLayout; status_row->setSpacing(6);
    const auto status_card = [this, status_row](const char * id) {
        auto * card = new QFrame(owner); card->setObjectName("connectionStatusCard");
        auto * box = new QVBoxLayout(card); box->setContentsMargins(8, 4, 8, 4);
        auto * value = label(id, box); value->setMinimumWidth(0);
        value->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
        status_row->addWidget(card, 1); return value;
      };
    connection_status = status_card("onboardConnectionStatus");
    runtime_status = status_card("onboardRuntimeStatus");
    draft_status = status_card("onboardDraftStatus"); layout->addLayout(status_row);
    auto * connections = new QHBoxLayout;
    connections->addWidget(new QLabel(QStringLiteral("常用平台")));
    recent = new QComboBox(owner); recent->setObjectName("onboardRecentPlatforms");
    recent->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    recent->setMinimumContentsLength(8); recent->setMinimumWidth(0);
    connections->addWidget(recent, 1);
    recent_connect = button(QStringLiteral("连接所选平台"), "onboardConnectRecent", connections);
    layout->addLayout(connections);
    auto * tabs = new QTabWidget; tabs->setObjectName("onboardManagementTabs");
    tabs->setMinimumHeight(120); layout->addWidget(tabs, 1);
    auto * environment = new QScrollArea(tabs); environment->setWidgetResizable(true);
    environment->setObjectName("onboardEnvironmentPage");
    environment->setFrameShape(QFrame::NoFrame);
    environment->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    auto * environment_body = new QWidget; environment->setWidget(environment_body);
    auto * environment_layout = new QVBoxLayout(environment_body);
    environment_layout->setContentsMargins(16, 12, 16, 12); environment_layout->setSpacing(10);
    auto * environment_form = new QFormLayout;
    environment_form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    environment_form->setRowWrapPolicy(QFormLayout::WrapLongRows);
    environment_form->setVerticalSpacing(10); environment_layout->addLayout(environment_form);
    local = new QPushButton(QStringLiteral("初始化 / 连接本机平台"), owner);
    local->setObjectName("onboardConnectLocal");
    environment_form->addRow(QStringLiteral("本机平台"), local);
    auto * remote_row = new QWidget; auto * remote_layout = new QHBoxLayout(remote_row);
    remote_layout->setContentsMargins(0, 0, 0, 0);
    alias = new QLineEdit; alias->setPlaceholderText(QStringLiteral("平台别名（可选）"));
    alias->setObjectName("onboardPlatformAlias");
    alias->setAccessibleName(QStringLiteral("平台别名"));
    remote_layout->addWidget(alias, 1);
    remote = button(QStringLiteral("连接远端平台…"), "onboardConnectRemote", remote_layout);
    environment_form->addRow(QStringLiteral("远端平台"), remote_row);
    identity = label("onboardIdentity", environment_layout);
    recipe_name = new QLineEdit; recipe_name->setObjectName("onboardRecipeName");
    auto * recipe_row = new QWidget; auto * recipe_layout = new QHBoxLayout(recipe_row);
    recipe_layout->setContentsMargins(0, 0, 0, 0); recipe_layout->addWidget(recipe_name, 1);
    read = button(QStringLiteral("读取目标方案"), "onboardReadRecipe", recipe_layout);
    environment_form->addRow(QStringLiteral("运行方案"), recipe_row);
    autostart = new QCheckBox(QStringLiteral("服务启动时自动运行方案（不执行任务）"));
    environment_form->addRow(autostart);
    recipe_state = label("onboardRecipeState", environment_layout);
    auto * draft_actions = new QHBoxLayout;
    auto * draft_save = button(QStringLiteral("保存本地草稿"), "onboardSaveDraft", draft_actions);
    auto * draft_load = button(QStringLiteral("恢复本地草稿"), "onboardLoadDraft", draft_actions);
    auto * advanced = button(
      QStringLiteral("高级方案 JSON…"), "onboardAdvancedRecipe", draft_actions);
    advanced_edit = advanced; draft_actions->addStretch();
    environment_layout->addLayout(draft_actions);
    auto * deployment = new QGroupBox(QStringLiteral("首次部署与本机服务设置（展开）"));
    deployment->setObjectName("onboardDeploymentSettings");
    deployment->setCheckable(true); deployment->setChecked(false);
    auto * deploy_layout = new QVBoxLayout(deployment);
    auto * deploy_body = new QWidget; auto * deploy_form = new QFormLayout(deploy_body);
    deploy_form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    deploy_form->setRowWrapPolicy(QFormLayout::WrapLongRows); deploy_form->setVerticalSpacing(10);
    domain = new QSpinBox; domain->setRange(0, 232);
    domain->setValue(qEnvironmentVariableIntValue("ROS_DOMAIN_ID"));
    host = new QLineEdit; host->setPlaceholderText(
      QStringLiteral("IPv4／主机名；留空仅允许本机管理"));
    allow_missions = new QCheckBox(QStringLiteral("首次部署允许受管任务执行"));
    deploy_form->addRow(QStringLiteral("本机平台 ROS Domain"), domain);
    deploy_form->addRow(QStringLiteral("对外提供的机载地址"), host);
    deploy_form->addRow(allow_missions);
    auto * deploy_hint = new QLabel(
      QStringLiteral(
        "仅首次初始化本机时应用，已有部署复用原配置。这里的本机指当前电脑；"
        "远端连接使用导入的连接配置。"));
    deploy_hint->setWordWrap(true); deploy_form->addRow(deploy_hint);
    export_connection = new QPushButton(QStringLiteral("导出地面连接配置…"), owner);
    export_connection->setObjectName("onboardExportConnection");
    install_service = new QPushButton(QStringLiteral("启用本机启动服务"), owner);
    install_service->setObjectName("onboardInstallService");
    deploy_form->addRow(export_connection); deploy_form->addRow(install_service);
    deploy_layout->addWidget(deploy_body); deploy_body->hide();
    QObject::connect(deployment, &QGroupBox::toggled, deploy_body, &QWidget::setVisible);
    environment_layout->addWidget(deployment);
    auto * heading = new QLabel(
      QStringLiteral(
        "管理本机或远端机载平台；关闭管理页面后，机载服务继续运行。"));
    heading->setWordWrap(true); environment_layout->addWidget(heading);
    environment_layout->addStretch(); tabs->addTab(environment, QStringLiteral("运行环境"));
    createEditor(tabs, QStringLiteral("设备管理"), &devices, &device_form);
    createEditor(tabs, QStringLiteral("算法配置"), &algorithms, &algorithm_form);
    auto * status_area = new QScrollArea(tabs); status_area->setWidgetResizable(true);
    status_area->setFrameShape(QFrame::NoFrame);
    status_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    auto * status_page = new QWidget; status_area->setWidget(status_page);
    auto * status_layout = new QVBoxLayout(status_page);
    status_layout->setContentsMargins(16, 12, 16, 12); status_layout->setSpacing(10);
    states = new QTableWidget(0, 3); states->setObjectName("onboardComponentStates");
    states->setMinimumHeight(120);
    states->setHorizontalHeaderLabels(
      {QStringLiteral("模块"), QStringLiteral("状态"), QStringLiteral("详情")});
    states->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    states->setEditTriggers(QAbstractItemView::NoEditTriggers); status_layout->addWidget(states, 1);
    auto * log_actions = new QHBoxLayout;
    log_component = new QComboBox; log_component->setMinimumWidth(0);
    log_component->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    log_component->setMinimumContentsLength(8); log_actions->addWidget(log_component, 1);
    logs = button(QStringLiteral("读取模块日志"), "onboardReadLog", log_actions);
    status_layout->addLayout(log_actions);
    log_text = new QPlainTextEdit; log_text->setReadOnly(true); log_text->setMinimumHeight(100);
    log_text->setObjectName("onboardModuleLog"); status_layout->addWidget(log_text, 1);
    auto * details = new QPushButton(QStringLiteral("展开原始状态")); details->setCheckable(true);
    raw_state = new QPlainTextEdit; raw_state->setReadOnly(true); raw_state->hide();
    status_layout->addWidget(details); status_layout->addWidget(raw_state);
    QObject::connect(details, &QPushButton::toggled, raw_state, &QWidget::setVisible);
    tabs->addTab(status_area, QStringLiteral("平台状态与故障"));
    auto * feedback = new QScrollArea(owner); feedback->setWidgetResizable(true);
    feedback->setObjectName("onboardOperationFeedback"); feedback->setFrameShape(QFrame::NoFrame);
    feedback->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    feedback->setFixedHeight(48);
    auto * feedback_body = new QWidget; auto * feedback_layout = new QVBoxLayout(feedback_body);
    feedback_layout->setContentsMargins(0, 0, 0, 0); feedback_layout->setSpacing(4);
    feedback_layout->setAlignment(Qt::AlignTop);
    conditions = label("onboardOperationConditions", feedback_layout);
    result = label("onboardResult", feedback_layout); feedback->setWidget(feedback_body);
    layout->addWidget(feedback);
    auto * actions = new QGridLayout;
    save = button(QStringLiteral("保存到目标平台"), "onboardSaveRecipe", actions);
    check = button(QStringLiteral("检查设备与依赖"), "onboardCheck", actions);
    start = button(QStringLiteral("启动平台"), "onboardStart", actions);
    stop = button(QStringLiteral("正常停止平台"), "onboardStop", actions);
    start->setProperty("primary", true); stop->setProperty("danger", true);
    const QList<QWidget *> platform_actions{save, check, start, stop};
    for (int i = 0; i < platform_actions.size(); ++i) {
      actions->removeWidget(platform_actions[i]);
      actions->addWidget(platform_actions[i], 0, i);
    }
    layout->addLayout(actions);
    auto * shortcuts = new QHBoxLayout;
    auto * observe = button(QStringLiteral("打开观察配置"), "onboardOpenObserver", shortcuts);
    auto * tasks = button(QStringLiteral("前往任务中心"), "onboardOpenTasks", shortcuts);
    shortcuts->addStretch(); layout->addLayout(shortcuts);
    buildMissionPage();
    QObject::connect(
      recipe_name, &QLineEdit::textEdited, owner, [this](const QString & text) {
        recipe["name"] = text; markDirty();
      });
    QObject::connect(
      autostart, &QCheckBox::toggled, owner, [this](bool value) {
        if (!loading) {recipe["autostart"] = value; markDirty();}});
    QObject::connect(
      local, &QPushButton::clicked, owner, [this] {
        if (!confirmDiscard()) {return;}
        send(
          "connect_local", {{"domain", domain->value()}, {"host", host->text().trimmed()},
            {"allow_missions", allow_missions->isChecked()}});
      });
    QObject::connect(
      remote, &QPushButton::clicked, owner, [this] {
        if (!confirmDiscard()) {return;}
        const auto path =
        QFileDialog::getOpenFileName(
          owner, QStringLiteral(
            "选择机载"
            "端连接配"
            "置"), {}, "JSON (*.json)");
        if (!path.isEmpty()) {send("connect_remote", {{"path", path}, {"alias", alias->text()}});}
      });
    QObject::connect(
      recent_connect, &QPushButton::clicked, owner, [this] {
        if (confirmDiscard() && !recent->currentData().toString().isEmpty()) {
          send(
            "connect_remote",
            {{"path", recent->currentData().toString()},
              {"alias", recent->itemData(recent->currentIndex(), Qt::UserRole + 1).toString()}});
        }
      });
    QObject::connect(
      read, &QPushButton::clicked, owner, [this] {
        if (confirmDiscard() && snapshot.contains("recipe")) {
          loadRecipe(
            snapshot.value("recipe").toObject(), false); draft_target = target;
          loaded_hash = snapshot.value("recipe_hash").toString();
        }
      });
    QObject::connect(
      save, &QPushButton::clicked, owner, [this] {
        QJsonObject values{{"recipe", recipe}, {"expected_hash", loaded_hash}};
        const auto selected = templateComponents();
        if (!selected.isEmpty()) {values["templates"] = selected;}
        operate("configure", values);
      });
    QObject::connect(
      check, &QPushButton::clicked, owner, [this] {
        operate("check", {{"recipe_hash", loaded_hash}});
      });
    QObject::connect(
      start, &QPushButton::clicked, owner, [this] {
        const auto confirmed_target = target;
        const auto confirmed_hash = loaded_hash;
        if (QMessageBox::question(
          owner, QStringLiteral("启动机载平台"),
          QStringLiteral(
            "平台：%1\n方案：%2\n%3\n"
            "启动这些模块？此操作"
            "不执行飞行任务。")
          .arg(
            target.value("server_id").toString(), recipe.value("name").toString(),
            plan->text())) == QMessageBox::Yes)
        {
          if (!sameTarget(confirmed_target) || loaded_hash != confirmed_hash || dirty) {
            result->setText(
              QStringLiteral(
                "确认期间平台或方案版"
                "本已改变，请重新核对"
                "模块清单；未启动。")); return;
          }
          operate("start", {{"recipe_hash", confirmed_hash}});
        }
      });
    QObject::connect(
      stop, &QPushButton::clicked, owner, [this] {
        const auto confirmed_target = target;
        const auto confirmed_session = snapshot.value("state").toObject()["session"];
        if (QMessageBox::question(
          owner, QStringLiteral("正常停止机载平台"),
          QStringLiteral(
            "请求正常停止平台 %1 "
            "的当前受管会话？").arg(target.value("server_id").toString())) ==
        QMessageBox::Yes)
        {
          if (!sameTarget(confirmed_target) ||
          snapshot.value("state").toObject()["session"] != confirmed_session)
          {
            result->setText(
              QStringLiteral(
                "确认期间平台会话已改"
                "变，请重新核对；未发"
                "送停止。")); return;
          }
          operate("stop", {{"session", confirmed_session}});
        }
      });
    QObject::connect(
      logs, &QPushButton::clicked, owner, [this] {
        send("logs", {{"component", log_component->currentData().toString()}});
      });
    QObject::connect(draft_save, &QPushButton::clicked, owner, [this] {saveDraft();});
    QObject::connect(
      draft_load, &QPushButton::clicked, owner, [this] {
        if (confirmDiscard()) {send("load_draft");}});
    QObject::connect(advanced, &QPushButton::clicked, owner, [this] {editAdvanced();});
    QObject::connect(
      export_connection, &QPushButton::clicked, owner, [this] {
        const auto path = QFileDialog::getExistingDirectory(
          owner, QStringLiteral(
            "选择新的"
            "空连接包"
            "目录"));
        if (!path.isEmpty()) {send("export_connection", {{"directory", path}});}});
    QObject::connect(
      install_service, &QPushButton::clicked, owner, [this] {
        if (QMessageBox::question(
          owner, QStringLiteral("启用本机启动服务"),
          QStringLiteral(
            "仅对当前电脑安装用户"
            "启动服务。保存的自启"
            "方案会在满足条件时启"
            "动模块，但不执行任务"
            "。继续？")) == QMessageBox::Yes)
        {
          send("install_service");
        }
      });
    QObject::connect(tasks, &QPushButton::clicked, owner, [this] {emit owner->missionRequested();});
    QObject::connect(
      observe, &QPushButton::clicked, owner, [this] {
        if (!fresh()) {
          result->setText(
            QStringLiteral(
              "请先连接平台并获取新"
              "鲜状态")); return;
        }
        QJsonObject context = target;
        context["simulation_time"] = false;
        context["front_topic"] =
        recipe.value("components").toObject()["realsense"].toObject()["output_topic"].toString(
          "/camera/color/image_raw");
        context["down_topic"] = "/camera_down/color/image_raw";
        context["target_association_confirmed"] = false;
        emit owner->observeRequested(context);
      });
    QObject::connect(&bridge, &QProcess::started, owner, [this] {send("hello");});
    QObject::connect(&bridge, &QProcess::readyReadStandardOutput, owner, [this] {readReplies();});
    QObject::connect(
      &bridge, &QProcess::readyReadStandardError, owner, [this] {
        const auto detail = QString::fromUtf8(bridge.readAllStandardError()).left(2048);
        if (!detail.isEmpty()) {
          result->setText(
            QStringLiteral(
              "机载管理"
              "适配器：") + detail);
        }
      });
    QObject::connect(
      &bridge, &QProcess::errorOccurred, owner, [this](QProcess::ProcessError) {
        pending.clear(); fresh_clock.invalidate();
        result->setText(
          QStringLiteral(
            "机载管理适配器不可用"
            "：") + bridge.errorString()); update();
      });
    QObject::connect(
      &bridge, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), owner,
      [this](int, QProcess::ExitStatus) {
        pending.clear(); fresh_clock.invalidate();
        result->setText(
          QStringLiteral(
            "管理适配器已断开；机"
            "载后台继续运行。重新"
            "连接后先查询原状态。")); update();
      });
    timer.setInterval(200);
    QObject::connect(
      &timer, &QTimer::timeout, owner, [this] {
        update(); pumpPrepare();
        if (!closing && pending.isEmpty() && !target.isEmpty() &&
        (!poll_clock.isValid() || poll_clock.elapsed() >= 800)) {send("status");}
      });
    timer.start(); ensureBridge(); update();
  }

  QPushButton * button(const QString & text, const char * id, QLayout * layout)
  {
    auto * b = new QPushButton(text, owner); b->setObjectName(id); layout->addWidget(b); return b;
  }
  QLabel * label(const char * id, QLayout * layout)
  {
    auto * l = new QLabel(owner); l->setObjectName(id); l->setWordWrap(true);
    l->setTextFormat(Qt::PlainText); l->setMinimumWidth(0);
    l->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    layout->addWidget(l); return l;
  }
  void createEditor(
    QTabWidget * tabs, const QString & title, QTreeWidget ** tree,
    QFormLayout ** form)
  {
    auto * editor_page = new QWidget; auto * editor_layout = new QVBoxLayout(editor_page);
    editor_layout->setContentsMargins(0, 0, 0, 0); editor_layout->setSpacing(6);
    const bool device_editor = title == QStringLiteral("设备管理");
    auto * split = new QSplitter; split->setChildrenCollapsible(false);
    split->setObjectName(device_editor ? "onboardDeviceColumns" : "onboardAlgorithmColumns");
    *tree = new QTreeWidget; (*tree)->setHeaderHidden(true); (*tree)->setMinimumWidth(150);
    (*tree)->setTextElideMode(Qt::ElideRight);
    (*tree)->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    (*tree)->setObjectName(
      device_editor ? "onboardDeviceTree" : "onboardAlgorithmTree");
    auto * area = new QScrollArea; area->setWidgetResizable(true);
    area->setFrameShape(QFrame::NoFrame);
    area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    auto * body = new QWidget; auto * body_layout = new QVBoxLayout(body);
    body_layout->setContentsMargins(16, 12, 16, 12); body_layout->setSpacing(10);
    auto * overview_fields = new QWidget; auto * overview = new QFormLayout(overview_fields);
    overview->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    overview->setRowWrapPolicy(QFormLayout::WrapLongRows);
    overview->setContentsMargins(0, 0, 0, 0); overview->setVerticalSpacing(10);
    body_layout->addWidget(overview_fields); body_layout->addStretch(); area->setWidget(body);
    auto * parameter_editor = new QScrollArea; parameter_editor->setWidgetResizable(true);
    parameter_editor->setObjectName(
      device_editor ? "onboardDeviceParameterEditor" :
      "onboardAlgorithmParameterEditor");
    parameter_editor->setMinimumWidth(230); parameter_editor->setFrameShape(QFrame::NoFrame);
    parameter_editor->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    auto * parameter_body = new QWidget; auto * parameter_layout = new QVBoxLayout(parameter_body);
    parameter_layout->setContentsMargins(16, 12, 16, 12); parameter_layout->setSpacing(10);
    auto * close = button(
      device_editor ? QStringLiteral("收起编辑") : QStringLiteral("返回说明"),
      device_editor ? "onboardCloseDeviceParameters" : "onboardBackToAlgorithmDetails",
      parameter_layout);
    auto * fields = new QWidget; *form = new QFormLayout(fields);
    (*form)->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    (*form)->setRowWrapPolicy(QFormLayout::WrapLongRows);
    (*form)->setContentsMargins(0, 0, 0, 0); (*form)->setVerticalSpacing(10);
    parameter_layout->addWidget(fields); parameter_layout->addStretch();
    parameter_editor->setWidget(parameter_body);
    split->addWidget(*tree); split->addWidget(area); split->setStretchFactor(1, 2);
    if (device_editor) {
      device_overview_form = overview; device_parameter_editor = parameter_editor;
      device_columns = split; split->addWidget(parameter_editor); parameter_editor->hide();
      split->setStretchFactor(2, 2); split->setSizes({180, 620, 400});
      QObject::connect(
        close, &QPushButton::clicked, owner, [parameter_editor] {
          parameter_editor->hide();
        });
    } else {
      algorithm_overview_form = overview; algorithm_parameter_editor = parameter_editor;
      algorithm_columns = split; area->setMinimumWidth(230);
      algorithm_side_panel = new QStackedWidget; algorithm_side_panel->setMinimumWidth(190);
      algorithm_side_panel->setObjectName("onboardAlgorithmSidePanel");
      auto * details = new QScrollArea; details->setWidgetResizable(true);
      details->setObjectName("onboardAlgorithmDetails"); details->setMinimumWidth(190);
      details->setFrameShape(QFrame::NoFrame);
      details->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
      auto * details_body = new QWidget; details->setWidget(details_body);
      auto * detail_layout = new QVBoxLayout(details_body);
      detail_layout->setContentsMargins(16, 12, 16, 12); detail_layout->setSpacing(10);
      algorithm_title = label("onboardAlgorithmTitle", detail_layout);
      auto title_font = algorithm_title->font(); title_font.setBold(true);
      algorithm_title->setFont(title_font);
      const auto section = [this, detail_layout](const QString & heading, const char * id) {
          auto * caption = new QLabel(heading); auto font = caption->font(); font.setBold(true);
          caption->setFont(font); detail_layout->addWidget(caption);
          return label(id, detail_layout);
        };
      algorithm_description = section(QStringLiteral("功能"), "onboardAlgorithmDescription");
      algorithm_inputs = section(QStringLiteral("输入"), "onboardAlgorithmInputs");
      algorithm_dependencies = section(QStringLiteral("依赖模块"), "onboardAlgorithmDependencies");
      algorithm_outputs = section(QStringLiteral("输出与就绪检测"), "onboardAlgorithmOutputs");
      algorithm_notes = section(QStringLiteral("使用说明"), "onboardAlgorithmNotes");
      plan = section(QStringLiteral("当前方案"), "onboardStartupPlan");
      detail_layout->addStretch(); algorithm_side_panel->addWidget(details);
      algorithm_side_panel->addWidget(parameter_editor); split->addWidget(algorithm_side_panel);
      split->setStretchFactor(2, 2); split->setSizes({180, 320, 400});
      QObject::connect(
        close, &QPushButton::clicked, owner, [this] {
          algorithm_side_panel->setCurrentIndex(0);
        });
    }
    editor_layout->addWidget(split, 1);
    tabs->addTab(editor_page, title);
    auto * selected_tree = *tree; auto * selected_form = *form;
    QObject::connect(
      selected_tree, &QTreeWidget::currentItemChanged, owner,
      [this, selected_form](QTreeWidgetItem * item, QTreeWidgetItem *) {
        if (item && !item->data(0, Qt::UserRole).toString().isEmpty()) {
          populateForm(selected_form, item->data(0, Qt::UserRole).toString());
        }
      });
    QObject::connect(
      selected_tree, &QTreeWidget::itemChanged, owner,
      [this](QTreeWidgetItem * item, int) {
        if (loading) {return;}
        const auto id = item->data(0, Qt::UserRole).toString(); if (id.isEmpty()) {return;}
        setEntry(id, "enabled", item->checkState(0) == Qt::Checked); markDirty();
        if (item->checkState(0) == Qt::Checked) {prepare({id});} else {renderPlan();}
      });
  }

  QString selectedMission() const
  {
    return missions->currentItem() ? missions->currentItem()->text() : QString();
  }
  QJsonObject taskStepSupport() const
  {
    auto support = snapshot.value("task_step_support").toObject();
    for (const auto & type : QStringList{
        "takeoff", "hover", "waypoint", "navigate", "track", "search", "land",
        "precision_land", "explore"})
    {
      if (support.value(type).isObject()) {continue;}
      support[type] = QJsonObject{
        {"state", "unknown"},
        {"reason", QStringLiteral(
            "机载服务尚未提供该步骤的依赖检查结果，请更新服务并检查平台状态。")}};
    }
    return support;
  }
  QString missionStepProblem() const
  {
    if (review.isEmpty()) {return {};}
    const auto types = saved_mission.value("step_types").toArray();
    if (types.isEmpty()) {
      return QStringLiteral("机载服务未提供原生步骤检查结果，请更新服务后重新读取任务。");
    }
    const auto support = taskStepSupport(); QStringList problems;
    for (const auto & type : types) {
      const auto state = support.value(type.toString()).toObject();
      if (state.value("state").toString() != "supported") {
        const auto reason = state.value("reason").toString();
        problems << (reason.isEmpty() ?
        QStringLiteral("尚未确认任务步骤的依赖状态：") + type.toString() : reason);
      }
    }
    problems.removeDuplicates(); return problems.join(QStringLiteral("；"));
  }
  QString missionCandidateProblem() const
  {
    if (mission_candidate.isEmpty() || !mission_candidate.value("valid").toBool(true)) {
      return QStringLiteral("当前任务尚未通过校验，或表单改动尚未应用。");
    }
    const auto name = mission_candidate.value("name").toString();
    if (name.toUtf8().size() > 128) {
      return QStringLiteral("机载任务文件名的 UTF-8 长度不能超过 128 字节，请缩短名称。");
    }
    const QRegularExpression names(
      "\\A[\\w-]{1,100}\\.yaml\\z", QRegularExpression::UseUnicodePropertiesOption);
    bool valid_name = names.match(name).hasMatch();
    // Python re's Unicode \w is alphanumeric plus underscore, without combining marks.
    // Check code points as well so PCRE Unicode word-class extensions cannot widen it.
    for (const auto point : name.left(name.size() - 5).toUcs4()) {
      if (point != '_' && point != '-' && !QChar::isLetterOrNumber(point)) {valid_name = false;}
    }
    if (!valid_name) {
      return QStringLiteral(
        "机载任务文件名须为 1～100 个字母、数字、下划线或短横线，"
        "扩展名仅支持 .yaml；名称不能含其他点号。");
    }
    const auto content = mission_candidate.value("content").toString().toUtf8();
    if (content.isEmpty() || content.size() > 65536) {
      return QStringLiteral("任务正文不能为空，且 UTF-8 内容不能超过 64 KiB。");
    }
    const auto hash = QString::fromLatin1(
      QCryptographicHash::hash(content, QCryptographicHash::Sha256).toHex());
    if (hash != mission_candidate.value("hash").toString()) {
      return QStringLiteral("任务内容与已校验的 SHA-256 不一致，请重新校验。");
    }
    const auto candidate_target = mission_candidate.value("target").toObject();
    if (target.isEmpty() || candidate_target.isEmpty() || !sameTarget(candidate_target)) {
      return QStringLiteral("候选任务绑定的目标与当前认证平台不一致，请核对目标。");
    }
    if (!fresh()) {return QStringLiteral("平台认证状态已过期，请等待连接恢复后保存。");}
    return {};
  }
  bool missionCandidateReady() const {return missionCandidateProblem().isEmpty();}
  void buildMissionPage()
  {
    mission_page = new QWidget(owner); mission_page->setObjectName("onboardMissionPage");
    auto * layout = new QVBoxLayout(mission_page); layout->setContentsMargins(0, 0, 0, 0);
    auto * columns = new QSplitter(Qt::Horizontal, mission_page);
    columns->setObjectName("onboardMissionColumns"); columns->setChildrenCollapsible(false);
    auto * directory = new QWidget(columns); auto * directory_layout = new QVBoxLayout(directory);
    directory_layout->setContentsMargins(8, 8, 8, 8);
    label("onboardMissionCatalogHeading", directory_layout)->setText(
      QStringLiteral(
        "机载任务"
        "目录"));
    missions = new QListWidget(directory); missions->setObjectName("onboardMissionCatalog");
    missions->setMinimumWidth(150); directory_layout->addWidget(missions, 1);
    label("onboardMissionCatalogNote", directory_layout)->setText(
      QStringLiteral("目录随平台状态刷新。选择后读取目标平台保存的版本。"));
    auto * details = new QWidget(columns); auto * details_layout = new QVBoxLayout(details);
    details_layout->setContentsMargins(8, 8, 8, 8);
    task_identity = label("onboardTaskTarget", details_layout);
    task_policy = label("onboardTaskPolicy", details_layout);
    mission_name = new QLineEdit(details); mission_name->setObjectName("onboardMissionName");
    mission_name->setReadOnly(true); mission_name->setPlaceholderText(
      QStringLiteral(
        "尚未读取"
        "机载任务"));
    details_layout->addWidget(mission_name);
    mission_text = new QPlainTextEdit(details); mission_text->setObjectName("onboardMissionEditor");
    mission_text->setReadOnly(true);
    mission_text->setPlaceholderText(
      QStringLiteral(
        "选择任务并读取审阅；"
        "修改任务请在统一任务编辑器中打开。"));
    details_layout->addWidget(mission_text, 1);
    auto * task_result = label("onboardTaskResult", details_layout);
    QObject::connect(
      &timer, &QTimer::timeout, task_result, [this, task_result] {
        task_result->setText(result->text());
      });
    auto * document_actions = new QHBoxLayout;
    task_read = button(
      QStringLiteral(
        "读取并审阅"
        "机载版本"), "onboardMissionRead", document_actions);
    task_edit = button(QStringLiteral("在任务编辑中打开"), "onboardMissionEdit", document_actions);
    details_layout->addLayout(document_actions);
    permit = new QCheckBox(QStringLiteral("已审阅本次任务，允许提交执行"), details);
    permit->setObjectName("onboardMissionPermit"); details_layout->addWidget(permit);
    auto * execution_actions = new QHBoxLayout;
    task_start = button(QStringLiteral("执行所审阅任务"), "onboardMissionStart", execution_actions);
    task_lookup = button(QStringLiteral("查询原任务"), "onboardMissionLookup", execution_actions);
    task_cancel = button(
      QStringLiteral(
        "取消本客户端"
        "任务"), "onboardMissionCancel", execution_actions);
    details_layout->addLayout(execution_actions);
    columns->addWidget(directory); columns->addWidget(details);
    columns->setStretchFactor(0, 0); columns->setStretchFactor(1, 1);
    columns->setSizes({250, 750}); layout->addWidget(columns);
    mission_transfer_page = new QWidget(owner);
    mission_transfer_page->setObjectName("onboardMissionTransferPage");
    auto * transfer_layout = new QVBoxLayout(mission_transfer_page);
    transfer_target = label("onboardMissionTransferTarget", transfer_layout);
    transfer_candidate = label("onboardMissionCandidate", transfer_layout);
    transfer_layout->addStretch();
    auto * transfer_actions = new QHBoxLayout;
    task_save = button(
      QStringLiteral("校验并保存到目标平台"), "onboardMissionSave", transfer_actions);
    transfer_layout->addLayout(transfer_actions);
    auto * transfer_result = label("onboardMissionTransferResult", transfer_layout);
    QObject::connect(
      &timer, &QTimer::timeout, transfer_result, [this, transfer_result] {
        transfer_result->setText(result->text());
      });
    QObject::connect(
      missions, &QListWidget::currentRowChanged, owner, [this] {
        if (loading || selectedMission() == review.value("name").toString()) {return;}
        review = {}; permit->setChecked(false); update();
      });
    QObject::connect(permit, &QCheckBox::toggled, owner, [this] {update();});
    QObject::connect(
      task_read, &QPushButton::clicked, owner, [this] {
        if (!mission_center_active || selectedMission().isEmpty()) {return;}
        operate("mission_read", {{"name", selectedMission()}});
      });
    QObject::connect(
      task_edit, &QPushButton::clicked, owner, [this] {
        if (!mission_center_active || mission_source.isEmpty()) {return;}
        owner->setProperty(
          "onboardMissionDocument", QJsonObject{
          {"name", mission_name->text()}, {"content", mission_source},
          {"target", target}, {"serial", static_cast<double>(++mission_document_serial)}});
      });
    QObject::connect(
      task_save, &QPushButton::clicked, owner, [this] {
        if (!mission_center_active || !pending.isEmpty() || !missionCandidateReady()) {return;}
        const auto confirmed_target = target; const auto candidate = mission_candidate;
        if (QMessageBox::question(
          owner, QStringLiteral("保存任务到机载平台"),
          QStringLiteral(
            "目标：%1\n地址：%2\n任务：%3\nSHA-256：%4\n"
            "保存新任务，不执行；不覆盖同名不同内容。")
          .arg(
            target.value("server_id").toString(), target.value("url").toString(),
            candidate.value("name").toString(), candidate.value("hash").toString()),
          QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel) != QMessageBox::Yes)
        {
          return;
        }
        if (!mission_center_active || !sameTarget(confirmed_target) ||
        candidate != mission_candidate || !missionCandidateReady())
        {
          result->setText(QStringLiteral("确认期间目标或编辑内容已变化，未保存。")); return;
        }
        submitted_mission = candidate; submitted_mission_target = confirmed_target;
        operate("mission_save", {{"name", candidate["name"]}, {"content", candidate["content"]}});
      });
    QObject::connect(
      task_start, &QPushButton::clicked, owner, [this] {
        if (!mission_center_active || review.isEmpty() || !permit->isChecked()) {return;}
        const auto confirmed_target = target; const auto confirmed_review = review;
        if (QMessageBox::question(
          owner, QStringLiteral("执行机载受管任务"),
          QStringLiteral("平台：%1\n任务：%2\n内容 SHA256：%3\n仅提交本次审阅内容？")
          .arg(
            target.value("server_id").toString(), review.value("name").toString(),
            review.value("sha256").toString())) == QMessageBox::Yes)
        {
          if (!mission_center_active || !sameTarget(confirmed_target) ||
          review != confirmed_review || !permit->isChecked() || !task_start->isEnabled())
          {
            result->setText(
              QStringLiteral(
                "确认期间平台、执行条件或审阅已改变，"
                "请重新读取审阅；未执行。"));
            return;
          }
          operate(
            "mission_start", {{"name", confirmed_review["name"]},
              {"sha256", confirmed_review["sha256"]}, {"confirmed", true}});
        }
      });
    QObject::connect(
      task_lookup, &QPushButton::clicked, owner, [this] {
        if (mission_center_active && !task_request.isEmpty()) {
          operate("mission_lookup", {{"request_id", task_request}});
        }
      });
    QObject::connect(
      task_cancel, &QPushButton::clicked, owner, [this] {
        if (!mission_center_active || task_request.isEmpty()) {return;}
        const auto confirmed_target = target; const auto confirmed_request = task_request;
        if (QMessageBox::question(
          owner, QStringLiteral("取消机载任务"),
          QStringLiteral(
            "向当前平台取消本客户端"
            "原请求 %1？").arg(task_request)) == QMessageBox::Yes)
        {
          if (!mission_center_active || !sameTarget(confirmed_target) ||
          task_request != confirmed_request)
          {
            result->setText(
              QStringLiteral(
                "确认期间任务会话已改变，请重新查询原任务；"
                "未取消其他任务。"));
            return;
          }
          operate("mission_cancel", {{"request_id", confirmed_request}});
        }
      });
    mission_page->hide(); mission_transfer_page->hide();
  }

  void ensureBridge()
  {
    if (bridge.state() != QProcess::NotRunning) {return;}
    bridge.setProcessChannelMode(QProcess::SeparateChannels);
    bridge.start(
      "python3", {"-s", "-m", "easyfly_connection_manager.onboard_ui_bridge",
        "--state-directory", runtime + "/onboard-ui"});
  }
  bool send(const QString & op, const QJsonObject & data = {})
  {
    if (!pending.isEmpty()) {
      result->setText(
        QStringLiteral(
          "请等待当前请求完成；"
          "未重复提交。")); return false;
    }
    if (bridge.state() == QProcess::NotRunning) {
      ensureBridge(); result->setText(
        QStringLiteral(
          "管理适配器正在恢复，"
          "请稍后重新连接。")); return false;
    }
    if (bridge.state() != QProcess::Running) {return false;}
    const auto id = QUuid::createUuid().toString(QUuid::WithoutBraces).remove('-');
    const auto bytes = QJsonDocument(QJsonObject{{"id", id}, {"op", op}, {"data", data}}).toJson(
      QJsonDocument::Compact) + '\n';
    if (bytes.size() > 524288) {
      result->setText(
        QStringLiteral(
          "请求超过允许大小，未"
          "发送或截断。")); return false;
    }
    pending = id; kind = op; last_ok = false;
    if (bridge.write(bytes) != bytes.size()) {
      pending.clear(); result->setText(
        QStringLiteral(
          "请求未完整送达；请先"
          "查询状态，未自动重发"
          "。")); return false;
    }
    if (op != "status") {result->setText(QStringLiteral("正在处理：") + displayOperation(op));}
    update(); return true;
  }
  QString displayOperation(const QString & op) const
  {
    static const QMap<QString, QString> names{{"hello", QStringLiteral("加载机载配置目录")},
      {"connect_local", QStringLiteral(
          "连接本机"
          "平台")}, {"connect_remote", QStringLiteral(
          "连接远端"
          "平台")},
      {"save_draft", QStringLiteral(
          "保存本地"
          "草稿")}, {"load_draft", QStringLiteral(
          "恢复本地"
          "草稿")},
      {"prepare", QStringLiteral(
          "整理组件"
          "依赖")}, {"logs", QStringLiteral(
          "读取模块"
          "日志")}};
    return op == "operate" ? operation : names.value(op, op);
  }
  void operate(const QString & op, const QJsonObject & values)
  {
    if (!pending.isEmpty()) {
      result->setText(
        QStringLiteral(
          "请等待当前请求完成，"
          "未重复提交。")); return;
    }
    if (!fresh()) {
      result->setText(
        QStringLiteral(
          "请先获取新鲜、经过认"
          "证的平台状态。")); return;
    }
    operation = op; send("operate", {{"operation", op}, {"values", values}});
  }
  void readReplies()
  {
    input.append(bridge.readAllStandardOutput());
    if (input.size() > 1048576) {
      bridge.closeWriteChannel(); input.clear(); result->setText(
        QStringLiteral(
          "适配器响应超限，未解"
          "析。"));
      return;
    }
    while (input.contains('\n')) {
      const auto end = input.indexOf('\n'); const auto line = input.left(end); input.remove(
        0,
        end +
        1);
      if (line.size() > 524288) {continue;}
      QJsonParseError error; const auto document = QJsonDocument::fromJson(line, &error);
      if (error.error != QJsonParseError::NoError || !document.isObject()) {continue;}
      const auto reply = document.object(); if (reply["id"].toString() != pending) {continue;}
      const auto completed = kind; pending.clear(); last_ok = reply["ok"].toBool();
      if (reply.contains("task_request")) {
        task_request = reply["task_request"].toString();
        task_pending = reply["task_pending"].toBool();
      }
      if (!last_ok) {
        if (completed == "prepare") {preparing_enable.clear();}
        fresh_clock.invalidate(); result->setText(
          QStringLiteral("未完成：") + reply["error"].toString() +
          QStringLiteral(
            "；平台操作不自动重发"
            "，请先查询原状态。"));
      } else {handleValue(completed, reply["value"].toObject());}
      poll_clock.restart(); update(); pumpPrepare();
    }
  }

  void handleValue(const QString & completed, const QJsonObject & value)
  {
    if (completed == "hello") {
      catalog = value["catalog"].toObject();
      parameter_templates = value.value("parameter_templates").toObject();
      if (recipe.isEmpty()) {loadRecipe(value["template"].toObject(), false);}
      renderRecent(value["recent"].toArray());
      result->setText(
        QStringLiteral(
          "选择本机或远端平台。"
          "离线时可编辑并保存本"
          "地方案草稿。")); return;
    }
    if (completed == "prepare") {
      if (recipe_revision != preparing_revision) {
        for (const auto & id : preparing_enable) {prepare_queue.insert(id);}
        preparing_enable.clear(); return;
      }
      preparing_enable.clear();
      loadRecipe(value["recipe"].toObject(), true, true);
      QStringList added; for (const auto & item : value["added"].toArray()) {
        added << title(item.toString());
      }
      result->setText(
        added.isEmpty() ? QStringLiteral(
          "依赖已核对；保存和启"
          "动仍会完整校验。") : QStringLiteral(
          "自动加入依赖：") + added.join(QStringLiteral("、")));
      return;
    }
    if (completed == "save_draft") {
      local_dirty = false; mission_dirty = false;
      result->setText(
        QStringLiteral(
          "本地草稿已保存；目标"
          "平台配置未改变，也未"
          "启动任何模块。")); return;
    }
    if (completed == "load_draft") {
      const auto stored_target = value["target"].toObject();
      if (!target.isEmpty() && !stored_target.isEmpty() &&
        stored_target["server_id"] != target.value("server_id"))
      {
        result->setText(
          QStringLiteral(
            "草稿属于另一平台，未"
            "覆盖当前编辑。请先连"
            "接草稿对应平台。")); return;
      }
      editor_overrides = value["editor_overrides"].toObject();
      parameter_sources = value.value("parameter_sources").toObject();
      loadRecipe(value["recipe"].toObject(), true, true); draft_target = stored_target;
      loaded_hash = value["expected_hash"].toString();
      loading = true; const auto mission = value["mission"].toObject();
      if (mission.contains("content")) {
        mission_name->setText(mission["name"].toString());
        mission_source = mission["content"].toString(); mission_text->setPlainText(mission_source);
      }
      loading = false; review = {}; permit->setChecked(false); local_dirty = false;
      mission_dirty = false;
      result->setText(
        QStringLiteral(
          "已恢复本地草稿；未写"
          "入机载端。提交时仍检"
          "查目标和保存版本。")); return;
    }
    if (value.contains("snapshot")) {
      const auto old_id = target.value("server_id").toString();
      const auto old_boot = target.value("boot_id").toString();
      snapshot = value["snapshot"].toObject(); target = value["target"].toObject();
      fresh_clock.restart(); task_request = value["task_request"].toString();
      task_pending = value["task_pending"].toBool();
      if (old_id != target.value("server_id").toString() ||
        old_boot != target.value("boot_id").toString())
      {
        review = {}; saved_mission = {}; permit->setChecked(false);
        mission_name->clear(); mission_source.clear(); mission_text->clear();
        focused_mission.clear();
      }
      if (completed.startsWith("connect")) {
        // Explicit target switching has already offered Save/Discard/Cancel.
        // Always display the authenticated target's recipe, never an old target's draft.
        prepare_queue.clear(); preparing_enable.clear();
        loaded_hash = snapshot.value("recipe_hash").toString();
        loadRecipe(snapshot.value("recipe").toObject(), false); draft_target = target;
        if (old_id != target.value("server_id").toString()) {
          loading = true; mission_name->setText("mission_001.yaml");
          mission_source.clear(); mission_text->clear(); loading = false;
          mission_dirty = false; saved_mission = {}; review = {}; permit->setChecked(false);
        }
      } else if (loaded_hash.isEmpty() && !dirty) {
        loaded_hash = snapshot.value("recipe_hash").toString();
        loadRecipe(snapshot.value("recipe").toObject(), false); draft_target = target;
      } else if (!dirty && loaded_hash != snapshot.value("recipe_hash").toString()) {
        loaded_hash = snapshot.value("recipe_hash").toString(); loadRecipe(
          snapshot.value("recipe").toObject(), false);
      }
      renderRecent(value["recent"].toArray()); renderSnapshot();
    }
    const auto receipt = value["result"].toObject();
    if (completed == "status") {return;}
    if (completed.startsWith("connect")) {
      result->setText(
        QStringLiteral(
          "管理服务已连接。DDS "
          "观察另行建立；连接不"
          "会提交任务。")); return;
    }
    if (completed == "export_connection") {
      result->setText(
        QStringLiteral(
          "已导出连"
          "接配置：") + value["path"].toString()); return;
    }
    if (receipt.contains("accepted") && !receipt["accepted"].toBool()) {
      result->setText(
        QStringLiteral("未执行：") +
        receipt["reason"].toString(receipt["phase"].toString())); return;
    }
    if (operation == "configure" && completed == "operate") {
      if (receipt.value("recipe").isObject()) {
        loadRecipe(receipt.value("recipe").toObject(), false);
      }
      loaded_hash = receipt["recipe_hash"].toString(); dirty = false; local_dirty = false;
    }
    if (operation == "mission_read" && completed == "operate") {
      loading = true; mission_name->setText(receipt["name"].toString());
      mission_source = receipt["content"].toString(); mission_text->setPlainText(mission_source);
      loading = false;
      review = QJsonObject{{"name", receipt["name"]}, {"sha256", receipt["sha256"]}};
      saved_mission = receipt;
      mission_dirty = false; permit->setChecked(false);
    }
    if (operation == "mission_save" && completed == "operate") {
      review = {}; permit->setChecked(false); mission_dirty = false;
      if (sameTarget(submitted_mission_target) &&
        receipt.value("name") == submitted_mission.value("name") &&
        receipt.value("sha256") == submitted_mission.value("hash"))
      {
        owner->setProperty(
          "onboardMissionSaved", QJsonObject{
            {"name", receipt.value("name")}, {"sha256", receipt.value("sha256")},
            {"target", submitted_mission_target},
            {"serial", static_cast<double>(++mission_saved_serial)}});
      }
      poll_clock.invalidate();
    }
    if (operation == "configuration_upload" && completed == "operate") {
      if (upload_key == "parameter_file") {
        useCustomParameters(upload_component);
        setEntry(upload_component, "parameter_file", receipt["path"]);
      } else {setParameter(upload_component, upload_key, receipt["path"]);}
      markDirty(); rebuildTrees();
    }
    if (completed == "logs") {
      log_text->setPlainText(
        receipt["content"].toString(
          QStringLiteral(
            "模块尚无"
            "日志输出"
            "。")));
    }
    if (receipt.contains("startup_order")) {
      QStringList ordered; for (const auto & id : receipt["startup_order"].toArray()) {
        ordered << title(id.toString());
      }
      result->setText(
        QStringLiteral(
          "检查通过"
          "，启动顺"
          "序：") + ordered.join(
          QStringLiteral(
            " "
            "→ ")));
    } else {
      result->setText(
        QStringLiteral(
          "已受"
          "理：") + phaseText(receipt["phase"].toString()));
    }
  }

  QString title(const QString & id) const {return catalog[id].toObject()["title"].toString(id);}
  void setEntry(const QString & id, const QString & key, const QJsonValue & value)
  {
    auto all = recipe.value("components").toObject(); auto entry = all[id].toObject();
    entry[key] = value; all[id] = entry; recipe["components"] = all;
  }
  void setParameter(const QString & id, const QString & key, const QJsonValue & value)
  {
    auto parameters = effectiveParameters(id);
    parameters[key] = value; setEntry(id, "parameters", parameters);
  }
  void useCustomParameters(const QString & id)
  {
    if (builtinParameters(id)) {
      const auto component = recipe.value("components").toObject().value(id).toObject();
      const auto previous = component.value("parameters").toObject();
      QJsonObject explicit_parameters;
      const QStringList retained = id == "navigation" ?
        QStringList{"strategy", "localization_backend"} : id == "camera_calib" ?
      QStringList{"calib_file"} : id == "livox" ? QStringList{"user_config_path"} : QStringList{};
      for (const auto & key : retained) {
        if (previous.contains(key)) {explicit_parameters[key] = previous.value(key);}
      }
      setEntry(id, "parameters", explicit_parameters);
      for (auto it = editor_overrides.begin(); it != editor_overrides.end(); ) {
        if (it.key().startsWith(id + "/")) {it = editor_overrides.erase(it);} else {++it;}
      }
    }
    parameter_sources[id] = "custom";
  }
  bool builtinParameters(const QString & id) const
  {
    const auto selected = parameter_sources.value(id).toString();
    if (selected == "custom") {return false;}
    if (selected == "builtin") {return true;}
    const auto path = recipe.value("components").toObject().value(id).toObject().value(
      "parameter_file").toString();
    const QRegularExpression generated(
      "^[a-f0-9]{16}-easyfly-template-" + QRegularExpression::escape(id) + "\\.yaml$");
    return path.isEmpty() || generated.match(QFileInfo(path).fileName()).hasMatch();
  }
  QJsonObject effectiveParameters(const QString & id) const
  {
    auto parameters = builtinParameters(id) ? parameter_templates.value(id).toObject().value(
      "parameters").toObject() : QJsonObject{};
    const auto current = recipe.value("components").toObject().value(id).toObject().value(
      "parameters").toObject();
    for (auto it = current.begin(); it != current.end(); ++it) {parameters[it.key()] = it.value();}
    return parameters;
  }
  QJsonArray templateComponents() const
  {
    QJsonArray result;
    const auto all = recipe.value("components").toObject();
    for (auto it = all.begin(); it != all.end(); ++it) {
      if (it.value().toObject().value("enabled").toBool() && builtinParameters(it.key()) &&
        parameter_templates.value(it.key()).toObject().value("accepts_file").toBool())
      {result.append(it.key());}
    }
    return result;
  }
  void refreshForm(QFormLayout * form, const QString & id)
  {
    QTimer::singleShot(
      0, owner, [this, form, id] {
        if ((form == device_form ? selected_device : selected_algorithm) == id) {
          populateForm(form, id);
        }
      });
  }
  void markDirty()
  {
    if (loading) {return;} ++recipe_revision; dirty = true; local_dirty = true; renderPlan();
    update();
  }
  void loadRecipe(const QJsonObject & value, bool modified, bool preserve_overrides = false)
  {
    if (!preserve_overrides) {editor_overrides = {}; parameter_sources = {};}
    if (modified) {++recipe_revision;}
    loading = true; recipe = value; recipe_name->setText(recipe.value("name").toString());
    autostart->setChecked(recipe.value("autostart").toBool());
    dirty = modified; if (modified) {local_dirty = true;} else {local_dirty = false;}
    rebuildTrees(); loading = false; renderPlan();
  }
  void rebuildTrees()
  {
    const bool was_loading = loading; loading = true;
    const auto old_device = selected_device; const auto old_algorithm = selected_algorithm;
    devices->clear(); algorithms->clear();
    QMap<QString, QTreeWidgetItem *> groups;
    const QMap<QString, QString> names{{"devices", QStringLiteral("其他设备")},
      {"device_flight", QStringLiteral("飞控通信")},
      {"device_sensors", QStringLiteral("传感设备")},
      {"device_alignment", QStringLiteral("标定与坐标")},
      {"localization", QStringLiteral(
          "定位与建"
          "图")}, {"mapping", QStringLiteral(
          "地图"
          "处理")},
      {"navigation", QStringLiteral(
          "导航与规"
          "划")}, {"perception", QStringLiteral(
          "图像"
          "感知")},
      {"services", QStringLiteral("平台运行服务（高级）")}};
    QTreeWidgetItem * select_device = nullptr; QTreeWidgetItem * select_algorithm = nullptr;
    for (auto it = catalog.begin(); it != catalog.end(); ++it) {
      if (!recipe.value("components").toObject().contains(it.key())) {continue;}
      auto group = groupFor(it.key(), it.value().toObject());
      auto * tree = group == "devices" ? devices : algorithms;
      if (tree == devices) {
        if (it.key() == "mavlink") {
          group = "device_flight";
        } else if (QStringList{"realsense", "down_camera", "livox"}.contains(it.key())) {
          group = "device_sensors";
        } else if (QStringList{"camera_calib", "coord"}.contains(it.key())) {
          group = "device_alignment";
        }
      }
      if (!groups.contains(group)) {
        auto * item = new QTreeWidgetItem(tree, {names.value(group, group)});
        item->setExpanded(true);
        item->setFlags(item->flags() & ~Qt::ItemIsSelectable); groups[group] = item;
      }
      auto * item = new QTreeWidgetItem(groups[group], {title(it.key())}); item->setData(
        0,
        Qt::UserRole,
        it.key());
      item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
      const auto configured_component = recipe.value("components").toObject()[it.key()].toObject();
      item->setCheckState(
        0, configured_component["enabled"].toBool() ? Qt::Checked : Qt::Unchecked);
      if (it.key() == old_device) {select_device = item;}
      if (it.key() == old_algorithm) {select_algorithm = item;}
    }
    int position = 0;
    for (const auto & group : QStringList{
        "device_flight", "device_sensors", "device_alignment", "devices"})
    {
      auto * item = groups.value(group, nullptr); if (!item) {continue;}
      devices->takeTopLevelItem(devices->indexOfTopLevelItem(item));
      devices->insertTopLevelItem(position++, item);
      item->setExpanded(true);
    }
    if (!select_device && devices->topLevelItemCount() && devices->topLevelItem(0)->childCount()) {
      select_device = devices->topLevelItem(0)->child(0);
    }
    if (!select_algorithm && algorithms->topLevelItemCount() &&
      algorithms->topLevelItem(0)->childCount())
    {
      select_algorithm = algorithms->topLevelItem(0)->child(0);
    }
    if (select_device) {
      devices->setCurrentItem(select_device);
      populateForm(device_form, select_device->data(0, Qt::UserRole).toString());
    }
    if (select_algorithm) {
      algorithms->setCurrentItem(select_algorithm); populateForm(
        algorithm_form, select_algorithm->data(
          0,
          Qt::UserRole).toString());
    }
    loading = was_loading;
  }
  void populateForm(QFormLayout * form, const QString & id)
  {
    if (form == device_form) {selected_device = id;} else {selected_algorithm = id;}
    const bool was_loading = loading; loading = true;
    auto * overview = form == device_form ? device_overview_form : algorithm_overview_form;
    while (form->rowCount()) {form->removeRow(0);}
    while (overview->rowCount()) {overview->removeRow(0);}
    const auto entry = recipe.value("components").toObject()[id].toObject();
    overview->addRow(new QLabel(title(id)));
    const auto spec = parameter_templates.value(id).toObject();
    const bool has_template = !spec.isEmpty();
    const bool file_supported = spec.value("accepts_file").toBool(
      id != "mavlink" && id != "local_planner");
    const bool builtin = has_template && builtinParameters(id);
    auto * heading = new QWidget; auto * editor_heading = new QHBoxLayout(heading);
    editor_heading->setContentsMargins(0, 0, 0, 0);
    editor_heading->addWidget(new QLabel(QStringLiteral("参数编辑 · %1").arg(title(id))), 1);
    if (has_template) {
      auto * reset = button(QStringLiteral("恢复模板"), "onboardResetParameters", editor_heading);
      reset->setObjectName("onboardResetParameters_" + id);
      reset->setAccessibleName(QStringLiteral("恢复%1内置模板").arg(title(id)));
      reset->setToolTip(QStringLiteral("仅恢复当前组件的模板参数；硬件相关项需按实际设备填写"));
      QObject::connect(
        reset, &QPushButton::clicked, owner, [this, id, form, file_supported] {
          setEntry(id, "parameters", parameter_templates.value(id).toObject().value("parameters"));
          if (file_supported) {setEntry(id, "parameter_file", "");}
          parameter_sources[id] = "builtin";
          for (auto it = editor_overrides.begin(); it != editor_overrides.end(); ) {
            if (it.key().startsWith(id + "/")) {it = editor_overrides.erase(it);} else {++it;}
          }
          markDirty(); refreshForm(form, id);
        });
    }
    form->addRow(heading);
    auto * editor_hint = new QLabel(
      QStringLiteral(
        "编辑保留在当前本地方案中；保存到目标平台后生效。收起编辑不会丢弃修改。"));
    editor_hint->setWordWrap(true); form->addRow(editor_hint);
    auto * row = new QWidget; auto * choices = new QHBoxLayout(row);
    choices->setContentsMargins(0, 0, 0, 0);
    if (has_template && file_supported) {
      auto * source = new QComboBox; source->setObjectName("onboardParameterSource_" + id);
      source->setAccessibleName(QStringLiteral("%1参数来源").arg(title(id)));
      source->addItem(QStringLiteral("内置模板"), "builtin");
      source->addItem(QStringLiteral("自定义文件"), "custom");
      source->setMinimumWidth(0);
      source->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
      source->setMinimumContentsLength(6); source->setCurrentIndex(builtin ? 0 : 1);
      source->setToolTip(
        QStringLiteral(
          "切换自定义文件会移除模板参数；设备配置路径和导航策略仍在表单中单独设置。"));
      choices->addWidget(source, 1);
      QObject::connect(
        source, QOverload<int>::of(&QComboBox::currentIndexChanged), owner,
        [this, id, form, source] {
          if (loading) {return;}
          if (source->currentData().toString() == "builtin") {
            parameter_sources[id] = "builtin";
            setEntry(id, "parameters", effectiveParameters(id));
          } else {useCustomParameters(id);}
          setEntry(id, "parameter_file", ""); markDirty(); refreshForm(form, id);
        });
    } else {
      choices->addWidget(
        new QLabel(
          file_supported ? QStringLiteral("自定义参数文件") :
          QStringLiteral("内置启动参数")), 1);
    }
    auto * upload = button(QStringLiteral("上传"), "onboardUploadFile", choices);
    upload->setObjectName(
      (id == "camera_calib" || id == "livox" ?
      "onboardUploadParameterFile_" : "onboardUploadFile_") + id);
    upload->setAccessibleName(QStringLiteral("上传%1参数文件").arg(title(id)));
    upload->setEnabled(file_supported);
    upload->setToolTip(
      file_supported ? QStringLiteral("上传自定义参数文件，成功后切换文件来源") :
      QStringLiteral("当前适配器使用启动参数，不支持 ROS 参数文件上传"));
    QObject::connect(
      upload, &QPushButton::clicked, owner, [this, id] {
        uploadFile(id, "parameter_file");
      });
    auto * edit = button(QStringLiteral("编辑"), "onboardEditParameters", choices);
    edit->setObjectName("onboardEditParameters_" + id);
    edit->setAccessibleName(QStringLiteral("编辑%1参数").arg(title(id)));
    QObject::connect(
      edit, &QPushButton::clicked, owner, [this, form] {
        if (form == device_form) {
          device_parameter_editor->show(); device_columns->setSizes({180, 300, 400});
        } else {
          algorithm_side_panel->setCurrentWidget(algorithm_parameter_editor);
          algorithm_columns->setSizes({180, 300, 400});
        }
      });
    overview->addRow(QStringLiteral("参数配置"), row);
    auto * hint = new QLabel(
      !file_supported ? QStringLiteral("点击编辑，在右侧配置启动参数。") :
      (builtin ? QStringLiteral("点击编辑，在右侧配置参数；保存时生成所需文件。") :
      (id == "realsense" ?
      QStringLiteral("自定义文件优先于表单同名参数。点击编辑可查看显式参数。") :
      QStringLiteral("使用自定义文件；显式表单参数覆盖文件同名项。点击编辑可查看。"))));
    hint->setWordWrap(true); overview->addRow(hint);
    if (has_template) {
      auto extra_notes = spec.value("notes").toString();
      const auto general_note = QStringLiteral("直接编辑下方参数；保存到平台时生成对应配置文件。");
      if (extra_notes.startsWith(general_note)) {extra_notes.remove(0, general_note.size());}
      if (!extra_notes.isEmpty()) {
        auto * note = new QLabel(extra_notes); note->setWordWrap(true); form->addRow(note);
      }
    }
    if (file_supported || !has_template) {
      auto * file = new QLineEdit(entry["parameter_file"].toString()); file->setObjectName(
        "onboardParameterFile_" + id);
      file->setAccessibleName(QStringLiteral("%1参数文件（机载路径）").arg(title(id)));
      file->setEnabled(file_supported); file->setReadOnly(builtin);
      file->setPlaceholderText(builtin ? QStringLiteral("保存到目标平台时自动生成") : QString());
      overview->addRow(
        builtin ? QStringLiteral("生成的参数文件") :
        QStringLiteral("ROS 参数文件（机载路径）"), file);
      QObject::connect(
        file, &QLineEdit::textEdited, owner, [this, id](const QString & text) {
          useCustomParameters(id); setEntry(id, "parameter_file", text); markDirty();
        });
    }
    if (!catalog[id].toObject()["output"].toString().isEmpty()) {
      auto * topic = new QLineEdit(entry["output_topic"].toString());
      topic->setToolTip(
        QStringLiteral(
          "此项是就绪检测条件；修改输出还需同步设置模块参数或话题重映射。"));
      overview->addRow(QStringLiteral("就绪检测话题"), topic);
      QObject::connect(
        topic, &QLineEdit::textEdited, owner, [this, id](const QString & text) {
          setEntry(id, "output_topic", text); markDirty();
        });
    }
    auto parameters = effectiveParameters(id);
    if (id == "camera_calib" && !parameters.contains("calib_file")) {
      parameters["calib_file"] = "";
    }
    if (id == "livox" && !parameters.contains("user_config_path")) {
      parameters["user_config_path"] = "";
    }
    if (parameters.isEmpty()) {
      auto * empty = new QLabel(
        QStringLiteral(
          "当前没有显式表单参数。自定义文件内容可修改后重新上传，或切换内置模板编辑。"));
      empty->setWordWrap(true); form->addRow(empty);
    }
    for (auto it = parameters.begin(); it != parameters.end(); ++it) {
      const auto key = it.key(); const auto value = it.value();
      const auto required_hint = builtin ||
        !file_supported ? spec.value("required").toObject().value(
        key).toString() : QString();
      const auto caption = required_hint.isEmpty() ? key : key + QStringLiteral("（必填）");
      if (id == "navigation" && key == "strategy") {
        auto * selector = new QComboBox; selector->setObjectName("onboardNavigationStrategy");
        selector->addItem("A*", "astar"); selector->addItem("EGO", "ego"); selector->addItem(
          "A* + EGO", "combined");
        if (catalog.contains("gcopter") && catalog.contains("fuel")) {
          selector->addItem("GCOPTER / FUEL", "gcopter");
        }
        selector->setCurrentIndex(selector->findData(value.toString())); form->addRow(
          QStringLiteral(
            "导航策略"), selector);
        QObject::connect(
          selector, QOverload<int>::of(&QComboBox::currentIndexChanged), owner,
          [this, id, key, selector] {
            if (loading) {return;} setParameter(
              id, key,
              selector->currentData().toString()); markDirty();
            if (recipe.value("components").toObject()[id].toObject()["enabled"].toBool()) {
              prepare({id});
            }
          });
      } else if (value.isBool()) {
        auto * field = new QCheckBox; field->setObjectName("onboardParameter_" + id + "_" + key);
        field->setChecked(value.toBool()); form->addRow(caption, field);
        QObject::connect(
          field, &QCheckBox::toggled, owner, [this, id, key](bool v) {
            if (!loading) {
              setParameter(id, key, v); markDirty();
            }
          });
      } else if (value.isDouble()) {
        auto * field = new QDoubleSpinBox; field->setDecimals(
          value.toDouble() == static_cast<qint64>(value.toDouble()) ? 0 : 8);
        field->setObjectName("onboardParameter_" + id + "_" + key);
        field->setRange(-1e9, 1e9); field->setValue(value.toDouble());
        field->setToolTip(required_hint); form->addRow(caption, field);
        QObject::connect(
          field, QOverload<double>::of(&QDoubleSpinBox::valueChanged), owner,
          [this, id, key](double v) {if (!loading) {setParameter(id, key, v); markDirty();}});
      } else {
        const auto text =
          value.isArray() ? QString::fromUtf8(
          QJsonDocument(value.toArray()).toJson(
            QJsonDocument
            ::Compact)) : value.toString();
        auto * field = new QLineEdit(editor_overrides.value(id + "/" + key).toString(text));
        field->setObjectName("onboardParameter_" + id + "_" + key);
        field->setAccessibleName(QStringLiteral("%1 %2").arg(title(id), key));
        field->setToolTip(required_hint);
        const bool upload_target = (id == "camera_calib" && key == "calib_file") ||
          (id == "livox" && key == "user_config_path");
        form->addRow(caption, upload_target ? uploadRow(field, id, true, key) : field);
        if (!required_hint.isEmpty()) {
          auto * required = new QLabel(required_hint); required->setWordWrap(true);
          form->addRow(required);
        }
        auto * field_error = new QLabel(
          QStringLiteral(
            "请输入合法 JSON 数组，"
            "修改前不能提交到平台"
            "。"));
        field_error->setWordWrap(true); field_error->setStyleSheet("color:#b42318");
        field_error->setVisible(editor_overrides.contains(id + "/" + key));
        if (value.isArray()) {form->addRow(field_error);} else {
          delete field_error; field_error = nullptr;
        }
        QObject::connect(
          field, &QLineEdit::textEdited, owner,
          [this, id, key, field, field_error, value](const QString & text) {
            if (value.isArray()) {
              QJsonParseError parse;
              const auto document = QJsonDocument::fromJson(text.toUtf8(), &parse);
              if (parse.error != QJsonParseError::NoError || !document.isArray()) {
                editor_overrides[id + "/" + key] = text; field_error->show(); markDirty();
                field->setStyleSheet("border:1px solid #b42318");
                result->setText(
                  QStringLiteral(
                    "参数 %1 需要合法 JSON "
                    "数组；未提交。可在高"
                    "级 JSON 中保存未完成编"
                    "辑。").arg(
                    key)); return;
              }
              editor_overrides.remove(id + "/" + key); field_error->hide();
              field->setStyleSheet({}); setParameter(id, key, document.array());
            } else {setParameter(id, key, text);} markDirty();
          });
      }
    }
    if (form == device_form) {
      QStringList depends; for (const auto & dependency : entry["depends_on"].toArray()) {
        depends << title(dependency.toString());
      }
      auto * hint = new QLabel(
        QStringLiteral("依赖：%1").arg(
          depends.isEmpty() ? QStringLiteral("无声明的前置模块") :
          depends.join(QStringLiteral("、"))));
      hint->setWordWrap(true); overview->addRow(hint);
    } else {renderAlgorithmDetails(id);}
    loading = was_loading;
  }
  QWidget * uploadRow(QLineEdit * path, const QString & id, bool supported, const QString & key)
  {
    auto * row = new QWidget; auto * layout = new QHBoxLayout(row);
    layout->setContentsMargins(0, 0, 0, 0); path->setMinimumWidth(0);
    layout->addWidget(path, 1);
    auto * upload = button(QStringLiteral("上传…"), "onboardUploadFile", layout);
    upload->setObjectName(
      (key == "parameter_file" && (id == "camera_calib" || id == "livox") ?
      "onboardUploadParameterFile_" : "onboardUploadFile_") + id); upload->setEnabled(supported);
    upload->setAccessibleName(QStringLiteral("上传%1配置文件").arg(title(id)));
    upload->setToolTip(QStringLiteral("上传文件到当前目标平台，并回填左侧路径"));
    QObject::connect(upload, &QPushButton::clicked, owner, [this, id, key] {uploadFile(id, key);});
    return row;
  }
  void renderAlgorithmDetails(const QString & id)
  {
    const auto spec = catalog.value(id).toObject();
    const auto all = recipe.value("components").toObject();
    const auto entry = all.value(id).toObject();
    algorithm_title->setText(title(id));
    algorithm_description->setText(
      spec.value("description").toString(
        QStringLiteral("当前目录未提供功能说明。")));
    QStringList inputs;
    for (const auto & input : spec.value("inputs").toArray()) {inputs << input.toString();}
    algorithm_inputs->setText(
      inputs.isEmpty() ? QStringLiteral("未声明输入说明。") :
      inputs.join(QStringLiteral("\n")));
    QStringList required;
    for (const auto & dependency : entry.value("depends_on").toArray()) {
      if (!required.contains(dependency.toString())) {required << dependency.toString();}
    }
    const auto strategy = entry.value("parameters").toObject().value("strategy").toString("astar");
    if (id == "navigation" && (strategy == "ego" || strategy == "combined") &&
      !required.contains("local_planner")) {required << "local_planner";}
    if (id == "navigation" && strategy == "gcopter") {
      for (const auto & dependency : {"gcopter", "fuel"}) {
        if (!required.contains(dependency)) {required << dependency;}
      }
    }
    QStringList dependencies;
    for (const auto & dependency : required) {
      dependencies << QStringLiteral("%1（%2）").arg(
        title(dependency),
        all.value(dependency).toObject().value("enabled").toBool() ?
        QStringLiteral("已选入方案") : QStringLiteral("未启用"));
    }
    algorithm_dependencies->setText(
      dependencies.isEmpty() ?
      QStringLiteral("无声明的前置模块。") : dependencies.join(QStringLiteral("\n")));
    const auto output = entry.value("output_topic").toString();
    algorithm_outputs->setText(
      output.isEmpty() ? QStringLiteral("未配置单一就绪检测话题。") :
      QStringLiteral("就绪检测话题：%1\n修改检测话题不会自动改变模块实际输出。").arg(output));
    algorithm_notes->setText(
      spec.value("notes").toString() + QStringLiteral(
        "\n外部输入、话题重映射及额外参数可在高级方案 JSON 中配置；保存与启动仍由平台校验。"));
  }
  bool sameTarget(const QJsonObject & expected) const
  {
    return expected["server_id"] == target.value("server_id") &&
           expected["boot_id"] == target.value("boot_id") && expected["client_id"] == target.value(
      "client_id");
  }
  void prepare(const QStringList & ids)
  {
    for (const auto & id : ids) {prepare_queue.insert(id);}
    pumpPrepare(); update();
  }
  void pumpPrepare()
  {
    if (closing || !pending.isEmpty() || prepare_queue.isEmpty() ||
      bridge.state() != QProcess::Running) {return;}
    QJsonArray enable; QStringList selected;
    for (const auto & id : prepare_queue) {
      if (recipe.value("components").toObject()[id].toObject()["enabled"].toBool()) {
        enable.append(id); selected << id;
      }
    }
    prepare_queue.clear();
    if (selected.isEmpty()) {return;}
    preparing_revision = recipe_revision; preparing_enable = selected;
    if (!send("prepare", {{"recipe", recipe}, {"enable", enable}, {"validate", false}})) {
      for (const auto & id : selected) {prepare_queue.insert(id);}
      preparing_enable.clear();
    }
  }
  void renderPlan()
  {
    QStringList enabled; const auto all = recipe.value("components").toObject();
    for (auto it = all.begin(); it != all.end(); ++it) {
      if (it.value().toObject()["enabled"].toBool()) {
        enabled << title(it.key());
      }
    }
    plan->setText(
      QStringLiteral("最终启用模块：%1").arg(
        enabled.isEmpty() ? QStringLiteral("尚未选择") :
        enabled.join(QStringLiteral("、"))));
    if (!selected_algorithm.isEmpty()) {renderAlgorithmDetails(selected_algorithm);}
  }
  void renderRecent(const QJsonArray & rows)
  {
    const auto previous = recent->currentData(); QSignalBlocker block(recent); recent->clear();
    for (const auto & row : rows) {
      const auto entry = row.toObject();
      recent->addItem(
        entry["alias"].toString() + " · " + entry["url"].toString(), entry["path"].toString());
      recent->setItemData(recent->count() - 1, entry["alias"].toString(), Qt::UserRole + 1);
    }
    const auto index = recent->findData(previous); if (index >= 0) {recent->setCurrentIndex(index);}
  }
  void renderSnapshot()
  {
    const auto state = snapshot.value("state").toObject();
    raw_state->setPlainText(json(state));
    const auto all = state["components"].toObject(); states->setRowCount(all.size());
    const auto log_selected = log_component->currentData(); log_component->clear();
    int row = 0; for (auto it = all.begin(); it != all.end(); ++it, ++row) {
      const auto entry = it.value().toObject();
      states->setItem(row, 0, new QTableWidgetItem(title(it.key())));
      states->setItem(row, 1, new QTableWidgetItem(phaseText(entry["phase"].toString())));
      states->setItem(
        row, 2,
        new QTableWidgetItem(
          entry.contains("returncode") ? QStringLiteral("退出码 %1").arg(
            entry[
              "returncode"].toInt()) : QStringLiteral("就绪依赖实际输出")));
      log_component->addItem(title(it.key()), it.key());
    }
    const auto log_index = log_component->findData(log_selected); if (log_index >= 0) {
      log_component->setCurrentIndex(log_index);
    }
    const auto old_mission = selectedMission(); QSignalBlocker block(missions); missions->clear();
    for (const auto & mission : snapshot.value("missions").toArray()) {
      missions->addItem(mission.toObject()["name"].toString());
    }
    auto matches = missions->findItems(
      focused_mission.isEmpty() ? old_mission : focused_mission, Qt::MatchExactly);
    if (!matches.isEmpty() && !focused_mission.isEmpty()) {focused_mission.clear();}
    if (matches.isEmpty()) {matches = missions->findItems(old_mission, Qt::MatchExactly);}
    if (!matches.isEmpty()) {
      missions->setCurrentItem(matches.first());
    } else if (missions->count()) {
      missions->setCurrentRow(0);
    }
    if (selectedMission() != old_mission) {review = {}; permit->setChecked(false);}
  }
  bool fresh() const
  {
    return !snapshot.isEmpty() && fresh_clock.isValid() && fresh_clock.elapsed() <= 2500;
  }
  void update()
  {
    if (closing) {return;}
    const bool available = fresh() && pending.isEmpty();
    const auto state = snapshot.value("state").toObject();
    const auto rules = snapshot.value("conditions").toObject();
    const auto phase = state["phase"].toString();
    const bool idle = phase == "stopped" || phase == "failed";
    const bool busy = !pending.isEmpty();
    const auto connection_text = QStringLiteral("管理连接 · %1").arg(
      target.isEmpty() ? QStringLiteral("未连接") :
      (fresh() ? QStringLiteral("已认证") : QStringLiteral("状态未知")));
    const auto target_name = target.value("alias").toString().isEmpty() ?
      target.value("server_id").toString() : target.value("alias").toString();
    const auto current_target = target.isEmpty() ? connection_text :
      connection_text + QStringLiteral(" · ") + target_name;
    connection_status->setText(
      connection_status->fontMetrics().elidedText(
        current_target, Qt::ElideRight, qMax(1, connection_status->width())));
    connection_status->setAccessibleName(current_target);
    connection_status->setToolTip(
      current_target + QStringLiteral("\n地址：") + target.value("url").toString() +
      QStringLiteral("\n身份：") + target.value("server_id").toString());
    runtime_status->setText(
      QStringLiteral("平台运行 · %1").arg(
        target.isEmpty() ? QStringLiteral("未连接") :
        (fresh() ? phaseText(phase) : QStringLiteral("状态未知"))));
    draft_status->setText(
      QStringLiteral("运行方案 · %1").arg(
        (dirty || target.isEmpty()) ? QStringLiteral("本地编辑") : QStringLiteral("与目标一致")));
    const bool editing = !busy || kind == "status" || kind == "prepare";
    local->setEnabled(!busy); remote->setEnabled(!busy); recent_connect->setEnabled(
      !busy && recent->count());
    devices->setEnabled(editing); algorithms->setEnabled(editing);
    device_form->parentWidget()->setEnabled(editing);
    algorithm_form->parentWidget()->setEnabled(editing);
    device_overview_form->parentWidget()->setEnabled(editing);
    algorithm_overview_form->parentWidget()->setEnabled(editing);
    device_parameter_editor->setEnabled(editing);
    algorithm_parameter_editor->setEnabled(editing); recipe_name->setEnabled(editing);
    autostart->setEnabled(editing);
    advanced_edit->setEnabled(editing && prepare_queue.isEmpty() && preparing_enable.isEmpty());
    const bool draft_matches = draft_target.isEmpty() ||
      draft_target.value("server_id") == target.value("server_id");
    const bool template_service_required = !templateComponents().isEmpty() &&
      snapshot.value("capabilities").toObject().value("parameter_templates").toInt() != 1;
    read->setEnabled(available); save->setEnabled(
      available && idle && editor_overrides.isEmpty() && draft_matches &&
      !template_service_required);
    check->setEnabled(available && idle && !dirty); start->setEnabled(available && idle && !dirty);
    stop->setEnabled(
      available && rules["can_stop"].toBool() &&
      state["client_id"].toString() == target.value("client_id").toString() && !idle);
    logs->setEnabled(available && log_component->count()); export_connection->setEnabled(
      available && target.value("local").toBool());
    install_service->setEnabled(available && idle && target.value("local").toBool());
    QString block;
    if (!fresh()) {
      block = QStringLiteral(
        "管理状态未连接或已过"
        "期：后台状态未知，可"
        "继续编辑和保存本地草"
        "稿。");
    } else if (busy) {
      block = QStringLiteral(
        "正在处理请求，请等待"
        "；不会自动重发平台操"
        "作。");
    } else if (!draft_matches) {
      block = QStringLiteral(
        "当前草稿属于另一平台"
        "。请连接草稿对应平台"
        "，或读取当前目标方案"
        "；未允许跨平台提交。");
    } else if (!editor_overrides.isEmpty()) {
      block = QStringLiteral(
        "有未完成的数组参数，"
        "请修正字段错误后再提"
        "交；仍可保存本地草稿"
        "。");
    } else if (dirty) {
      block = QStringLiteral(
        "当前是本地编辑方案；"
        "检查或启动前先保存到"
        "目标平台。");
    } else if (!idle) {
      block = QStringLiteral(
        "平台正在运行或状态未"
        "决，方案可编辑；提交"
        "新方案前先按条件正常"
        "停止。");
    } else {
      block = QStringLiteral(
        "保存方案不会启动平台"
        "；启动平台不会执行任"
        "务。");
    }
    if (!idle && available && !stop->isEnabled()) {
      block += QStringLiteral(
        " 停止需本客户端会话"
        "、新鲜地面状态和任务"
        "排空，条件未满足时保"
        "留后台。");
    }
    if (available && template_service_required) {
      block = QStringLiteral(
        "目标服务尚未加载内置模板功能。"
        "请在平台正常停止后更新并重启管理服务，或选择自定义参数文件。");
    }
    conditions->setText(block);
    identity->setText(
      target.isEmpty() ? QStringLiteral(
        "尚未连接平台；可以先"
        "编辑本地草稿。") :
      QStringLiteral(
        "平台：%1 · %2\n地址：%3"
        " · 身份：%4 · Domain %5\n"
        "管理连接：%6 · 平台："
        "%7\n%8")
      .arg(
        target.value("alias").toString(),
        target.value("local").toBool() ? QStringLiteral("本机") : QStringLiteral("远端"),
        target.value("url").toString(), target.value("server_id").toString(),
        QString::number(target.value("ros_domain_id").toInt()),
        fresh() ? QStringLiteral(
          "已"
          "认证") : QStringLiteral(
          "状态未知"
          "／失联"), phaseText(phase),
        state["error"].toString()));
    recipe_state->setText(
      QStringLiteral(
        "编辑状态：%1 · 本地草"
        "稿：%2\n目标已保存版"
        "本：%3 · 当前运行会话"
        "：%4")
      .arg(
        (dirty || target.isEmpty()) ? QStringLiteral(
          "尚未提交"
          "到目标") : QStringLiteral(
          "与目标已"
          "保存方案"
          "一致"),
        local_dirty ? QStringLiteral("未保存") : QStringLiteral("已保存或无改动"),
        loaded_hash.isEmpty() ? QStringLiteral("尚未读取") : loaded_hash,
        state["session"].toString(QStringLiteral("无"))));
    const auto task_target_label = target.value("server_id").toString(QStringLiteral("未连接"));
    task_identity->setText(
      QStringLiteral(
        "机载任务目标：%1\n管"
        "理地址：%2\n原任务请"
        "求：%3")
      .arg(
        task_target_label, target.value("url").toString(),
        task_request));
    QString policy;
    if (!fresh()) {
      policy = QStringLiteral(
        "请在连接管理中选择机"
        "载平台，等待新鲜认证"
        "状态；任务草稿仍保留"
        "。");
    } else if (!missionStepProblem().isEmpty()) {
      policy = missionStepProblem();
    } else if (snapshot.value("maintenance").toBool()) {
      policy = QStringLiteral(
        "当前 QGC 双向连接配置"
        "限制受管任务执行。");
    } else if (!snapshot.value("mission_execution_enabled").toBool()) {
      policy = QStringLiteral(
        "当前部署未启用受管任"
        "务执行；仍可编辑和保"
        "存任务。");
    } else if (!rules["mission_stack_ready"].toBool()) {
      policy = QStringLiteral(
        "执行前需导航、飞控动"
        "作、任务状态机和任务"
        "服务完整就绪。");
    } else if (!rules["grounded"].toBool()) {
      policy = QStringLiteral(
        "等待唯一、新鲜的未解"
        "锁／在地面状态。");
    } else if (task_pending) {
      policy = QStringLiteral(
        "原任务尚未确认结束，"
        "请查询原请求，未允许"
        "重复提交。");
    } else {
      policy = QStringLiteral(
        "读取机载任务并审阅内"
        "容后，可显式提交本次"
        "执行。");
    }
    task_policy->setText(policy);
    transfer_target->setText(task_identity->text());
    transfer_candidate->setText(
      mission_candidate.isEmpty() ?
      QStringLiteral("请在任务编辑中完成当前任务校验。") :
      QStringLiteral("当前编辑版本：%1\nSHA-256：%2\n%3")
      .arg(
        mission_candidate.value("name").toString(), mission_candidate.value("hash").toString(),
        missionCandidateReady() ? QStringLiteral("可确认保存到当前目标；保存后需重新读取审阅。") :
        missionCandidateProblem()));
    mission_text->setEnabled(true); mission_name->setEnabled(true);
    task_read->setEnabled(mission_center_active && available && missions->count());
    task_edit->setEnabled(mission_center_active && !mission_source.isEmpty());
    task_save->setEnabled(mission_center_active && available && missionCandidateReady());
    permit->setEnabled(mission_center_active && available && !review.isEmpty());
    const QJsonObject context{{"target", target}, {"fresh", fresh()}, {"policy", policy},
      {"step_support", taskStepSupport()},
      {"task_request", task_request}, {"task_pending", task_pending}};
    if (owner->property("onboardMissionContext").value<QJsonObject>() != context) {
      owner->setProperty("onboardMissionContext", context);
    }
    const bool mission_execution_available = snapshot.value("mission_execution_enabled").toBool();
    task_start->setEnabled(
      mission_center_active && available && !review.isEmpty() &&
      missionStepProblem().isEmpty() &&
      review.value(
        "name").toString() == selectedMission() && permit->isChecked() && !task_pending &&
      mission_execution_available && phase == "running" && rules["grounded"].toBool() &&
      rules["mission_stack_ready"].toBool());
    task_lookup->setEnabled(mission_center_active && available && !task_request.isEmpty());
    task_cancel->setEnabled(mission_center_active && available && !task_request.isEmpty());
    owner->setProperty("onboardHasUnsavedChanges", local_dirty || mission_dirty);
    owner->setProperty("onboardTaskUnresolved", task_pending);
    mission_page->setProperty("onboardHasUnsavedChanges", local_dirty || mission_dirty);
    mission_page->setProperty("onboardTaskUnresolved", task_pending);
  }
  void uploadFile(const QString & component, const QString & key)
  {
    if (!fresh()) {
      result->setText(
        QStringLiteral(
          "上传配置需要先连接机"
          "载平台。")); return;
    }
    const auto uploading_target = target;
    const auto path = QFileDialog::getOpenFileName(
      owner, QStringLiteral(
        "上传硬件配置或标定"), {}, "配置 (*.yaml *.yml *.json)");
    if (path.isEmpty()) {return;} QFile file(path);
    if (!file.open(QIODevice::ReadOnly) || file.size() > 65536) {
      result->setText(
        QStringLiteral(
          "配置文件不可读或超过"
          " 64 KiB。")); return;
    }
    const auto content = file.readAll();
    if (QString::fromUtf8(content).toUtf8() != content) {
      result->setText(
        QStringLiteral(
          "配置文件不是有效 UTF-8"
          "，未转换或上传。")); return;
    }
    if (!sameTarget(uploading_target) || !fresh()) {
      result->setText(QStringLiteral("选择文件期间平台身份或连接状态已变化，未上传。")); return;
    }
    upload_component = component; upload_key = key;
    auto name = QFileInfo(path).fileName();
    if (key == "parameter_file" && name == "easyfly-template-" + component + ".yaml") {
      name = "user-" + name;
    }
    operate(
      "configuration_upload", {{"name", name}, {"content", QString::fromUtf8(content)}});
  }
  bool saveDraft()
  {
    return send(
      "save_draft", {{"recipe", recipe}, {"expected_hash", loaded_hash}, {"editor_overrides",
          editor_overrides}, {"parameter_sources", parameter_sources},
        {"draft_target", draft_target}, {"mission", QJsonObject{{"name",
            mission_name->text()}, {"content", mission_source}}}});
  }
  bool confirmDiscard()
  {
    if (!local_dirty && !mission_dirty) {return true;}
    if (!pending.isEmpty()) {
      result->setText(
        QStringLiteral(
          "仍有在途请求，请等待"
          "完成后保存草稿或切换"
          "平台。")); return false;
    }
    const auto answer = QMessageBox::question(
      owner, QStringLiteral("保留未保存编辑"),
      QStringLiteral(
        "当前有未保存到本地的"
        "方案或任务草稿。保存"
        "草稿后离开、放弃修改"
        "，还是返回编辑？"),
      QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel, QMessageBox::Cancel);
    if (answer == QMessageBox::Cancel) {return false;}
    if (answer == QMessageBox::Discard) {
      local_dirty = false; mission_dirty = false; update(); return true;
    }
    if (!saveDraft()) {return false;}
    QEventLoop loop; QTimer timeout; timeout.setSingleShot(true); timeout.start(5000);
    QTimer completion; completion.setInterval(20);
    QObject::connect(&timeout, &QTimer::timeout, &loop, &QEventLoop::quit);
    QObject::connect(
      &completion, &QTimer::timeout, &loop, [this, &loop] {
        if (pending.isEmpty()) {
          loop.quit();
        }
      });
    completion.start(); loop.exec();
    return pending.isEmpty() && last_ok && !local_dirty && !mission_dirty;
  }
  void editAdvanced()
  {
    const auto editing_target = target;
    const auto editing_hash = loaded_hash;
    QDialog dialog(owner); dialog.setWindowTitle(QStringLiteral("高级机载运行方案"));
    dialog.resize(720, 600);
    auto * layout = new QVBoxLayout(&dialog); auto * text = new QPlainTextEdit(json(recipe));
    layout->addWidget(text);
    auto * note = new QLabel(
      QStringLiteral(
        "编辑组件依赖、remapping"
        "s 和额外参数。只修改"
        "本地草稿，提交与启动"
        "前由机载服务完整校验"
        "。"));
    note->setWordWrap(true); layout->addWidget(note);
    auto * buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addWidget(buttons);
    QObject::connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    if (dialog.exec() != QDialog::Accepted) {return;}
    QJsonParseError error; const auto document = QJsonDocument::fromJson(
      text->toPlainText().toUtf8(), &error);
    if (error.error != QJsonParseError::NoError || !document.isObject()) {
      result->setText(
        QStringLiteral(
          "方案不是有效 JSON 对象"
          "，原草稿保留。")); return;
    }
    if (!sameTarget(editing_target)) {
      result->setText(
        QStringLiteral(
          "编辑期间目标平台已改"
          "变，未将旧平台 JSON 应"
          "用到新平台。")); return;
    }
    loaded_hash = editing_hash; draft_target = editing_target;
    loadRecipe(document.object(), true); update();
  }
};

OnboardPage::OnboardPage(const QString & runtime, QWidget * parent)
: QWidget(parent), impl_(std::make_unique<Impl>(this, runtime)) {}
OnboardPage::~OnboardPage()
{
  impl_->closing = true; impl_->timer.stop();
  QObject::disconnect(&impl_->timer, nullptr, nullptr, nullptr);
  QObject::disconnect(&impl_->bridge, nullptr, this, nullptr);
  impl_->bridge.closeWriteChannel();
  // Only the local adapter can be terminated; it never owns the platform service.
  if (!impl_->bridge.waitForFinished(1000)) {
    impl_->bridge.terminate(); impl_->bridge.waitForFinished(1000);
  }
}
QWidget * OnboardPage::missionPage() {return impl_->mission_page;}
QWidget * OnboardPage::missionTransferPage() {return impl_->mission_transfer_page;}
void OnboardPage::setMissionCandidate(const QJsonObject & candidate)
{
  if (impl_->mission_candidate == candidate) {return;}
  impl_->mission_candidate = candidate; impl_->review = {}; impl_->permit->setChecked(false);
  impl_->update();
}
void OnboardPage::setMissionCenterActive(bool active)
{
  if (impl_->mission_center_active == active) {return;}
  impl_->mission_center_active = active;
  if (!active) {impl_->permit->setChecked(false);}
  impl_->update();
}
void OnboardPage::focusMission(const QString & name)
{
  impl_->focused_mission = name;
  const auto matches = impl_->missions->findItems(name, Qt::MatchExactly);
  if (!matches.isEmpty()) {
    impl_->missions->setCurrentItem(matches.first()); impl_->focused_mission.clear();
  }
  impl_->poll_clock.invalidate();
}
bool OnboardPage::confirmDiscardChanges() {return impl_->confirmDiscard();}
}  // namespace easyfly_connection_manager

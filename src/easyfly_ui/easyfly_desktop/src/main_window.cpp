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

#include <QAction>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDockWidget>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMenuBar>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRegularExpressionValidator>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSplitter>
#include <QStackedWidget>
#include <QStatusBar>
#include <QTableView>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <utility>

#include "easyfly_desktop/image_pipeline.hpp"
#include "easyfly_desktop/main_window.hpp"
#include "easyfly_desktop/presentation.hpp"
#include "widgets.hpp"
#include "theme.hpp"

namespace easyfly_desktop
{
namespace
{
QLabel * label(const QString & value, const char * name, QWidget * parent)
{
  auto * result = new QLabel(value, parent); result->setObjectName(name);
  result->setTextFormat(Qt::PlainText); result->setWordWrap(true);
  result->setTextInteractionFlags(Qt::TextSelectableByMouse);
  result->setMinimumWidth(0); result->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  return result;
}
QPushButton * button(const QString & value, const char * name, QWidget * parent)
{
  auto * result = new QPushButton(value, parent); result->setObjectName(name); return result;
}
QTableView * table(TextTable * model, const char * name, QWidget * parent)
{
  auto * result = new QTableView(parent); result->setObjectName(name); result->setModel(model);
  result->setEditTriggers(QAbstractItemView::NoEditTriggers);
  result->setSelectionBehavior(QAbstractItemView::SelectRows);
  result->setAlternatingRowColors(true); result->setWordWrap(false);
  result->horizontalHeader()->setStretchLastSection(true);
  result->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
  result->verticalHeader()->setVisible(false); result->setMinimumHeight(150);
  result->setMinimumWidth(0); return result;
}
QString yes(bool value) {return value ? QStringLiteral("是") : QStringLiteral("否");}
QString key_name(bridge::Key key)
{
  switch (key) {
    case bridge::Key::FlightCapabilities: return QStringLiteral("飞控（分项准入）");
    case bridge::Key::MissionCapabilities: return QStringLiteral("任务服务");
    case bridge::Key::PerceptionCapabilities: return QStringLiteral("感知服务");
    case bridge::Key::LandingCapabilities: return QStringLiteral("精准降落（可选）");
    default: return QStringLiteral("只读查询");
  }
}
QString parameter_name(int index)
{
  return QStringList{"profile", "use_sim_time", "operator_note", "export_pretty"}.value(index);
}
QString json_string(const std::string & encoded)
{
  const auto doc = QJsonDocument::fromJson(QByteArray("[") + text(encoded).toUtf8() + "]");
  return doc.isArray() && doc.array().size() == 1 && doc.array()[0].isString() ?
         doc.array()[0].toString() : QString{};
}
QString json_type(const std::string & encoded)
{
  const auto doc = QJsonDocument::fromJson(QByteArray("[") + text(encoded).toUtf8() + "]");
  if (!doc.isArray() || doc.array().size() != 1) {return QStringLiteral("无效 JSON");}
  if (doc.array()[0].isString()) {return "string";}
  if (doc.array()[0].isBool()) {return "bool";}
  return QStringLiteral("未知类型");
}
}  // namespace

struct MainWindow::Impl
{
  MainWindow * window;
  bridge::BridgeConfig config;
  SourceFactory factory;
  std::unique_ptr<Source> source;
  std::shared_ptr<const bridge::UiSnapshot> current, history;
  std::unique_ptr<ImagePipeline> decoder;
  QTimer * timer{nullptr};
  bool closing{false}, disconnecting{false}, released{false};
  uint64_t last_session{0}, last_epoch{0};
  int last_camera{-1}; int64_t submitted_stamp{-1};
  std::map<bridge::Key, uint64_t> expected;
  std::string queried_id, validated_name;
  uint64_t catalog_revision{0};
  std::map<bridge::Key, uint64_t> geometry_revision;
  std::map<bridge::Key, bool> geometry_visible;
  double event_tail{-1}; std::size_t event_count{0}; uint64_t event_session{0};

  QStackedWidget * pages{nullptr}; QListWidget * navigation{nullptr};
  QLabel * connection{nullptr}, *clock{nullptr}, *backend_profile{nullptr}, *warning{nullptr};
  QLabel * request_note{nullptr}, *state_connection{nullptr}, *armed{nullptr}, *mode{nullptr};
  QLabel * position{nullptr}, *speed{nullptr}, *battery{nullptr}, *validity{nullptr},
  *state_age{nullptr};
  QLabel * global_overview{nullptr}, *global_mission{nullptr}, *catalog_quality{nullptr};
  QLabel * validation_result{nullptr}, *query_result{nullptr}, *image_evidence{nullptr};
  QLabel * target_summary{nullptr}, *obstacle_summary{nullptr};
  QPushButton * toggle{nullptr}, *validate{nullptr}, *query{nullptr};
  QListWidget * mission_list{nullptr}; QLineEdit * execution_id{nullptr};
  QComboBox * camera{nullptr}, *selected_profile{nullptr}, *parameter{nullptr};
  QCheckBox * simulation{nullptr}; QLineEdit * front{nullptr}, *down{nullptr};
  QWidget * configuration{nullptr};
  CameraCanvas * overview_camera{nullptr}, *camera_canvas{nullptr}; MapCanvas * map{nullptr};
  TextTable * capability_model{nullptr}, *target_model{nullptr}, *obstacle_model{nullptr};
  TextTable * parameter_model{nullptr}, *event_model{nullptr};
  QPlainTextEdit * diagnostics{nullptr};

  Impl(MainWindow * owner, const bridge::BridgeConfig & options, SourceFactory source_factory)
  : window(owner), config(options), factory(std::move(source_factory))
  {
    build(); connect_source();
    timer = new QTimer(window); timer->setInterval(100);
    QObject::connect(timer, &QTimer::timeout, window, [this] {tick();}); timer->start();
    tick();
  }
  QVBoxLayout * page(const QString & title, const QString & subtitle)
  {
    auto * scroll = new QScrollArea(pages); scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    auto * content = new QWidget(scroll); content->setObjectName("pageContent");
    content->setAttribute(Qt::WA_StyledBackground, true);
    auto * layout = new QVBoxLayout(content);
    layout->setContentsMargins(18, 12, 18, 16); layout->setSpacing(12);
    auto * heading = label(title, "pageHeading", content); heading->setProperty("heading", true);
    layout->addWidget(heading); layout->addWidget(label(subtitle, "pageSubtitle", content));
    scroll->setWidget(content); pages->addWidget(scroll); return layout;
  }
  void build()
  {
    window->setWindowTitle(QStringLiteral("EasyFly · 只读平台观测")); window->resize(1440, 900);
    window->setMinimumSize(1024, 720);
    window->setPalette(theme::palette());
    window->setStyleSheet(theme::stylesheet());
    auto * root = new QWidget(window); root->setObjectName("workspaceRoot");
    auto * layout = new QVBoxLayout(root);
    layout->setContentsMargins(12, 10, 12, 8); layout->setSpacing(8);
    auto * top_bar = new QWidget(root); top_bar->setObjectName("topBar");
    top_bar->setAttribute(Qt::WA_StyledBackground, true);
    auto * header = new QHBoxLayout(top_bar);
    header->setContentsMargins(8, 8, 12, 8);
    auto * brand = new QLabel(QStringLiteral("EF  EasyFly"), top_bar);
    brand->setObjectName("brand");
    header->addWidget(brand);
    auto * status = new QVBoxLayout;
    connection = label(QStringLiteral("本地观测器启动中"), "connectionSummary", top_bar);
    clock = label({}, "clockSummary", top_bar);
    backend_profile = label({}, "backendProfile", top_bar);
    status->addWidget(connection); status->addWidget(clock); status->addWidget(backend_profile);
    header->addLayout(status, 1);
    toggle = button(QStringLiteral("断开观测"), "observerToggle", top_bar);
    header->addWidget(toggle);
    QObject::connect(
      toggle, &QPushButton::clicked, window, [this] {
        if (disconnecting || closing) {return;}
        if (source && current && !current->stopped) {stop_source(false);} else {connect_source();}
      });
    layout->addWidget(top_bar);
    warning = label({}, "warningBanner", root);
    warning->hide(); layout->addWidget(warning);
    auto * splitter = new QSplitter(Qt::Horizontal, root);
    navigation = new QListWidget(splitter); navigation->setObjectName("navigation");
    navigation->addItems(
      {QStringLiteral("运行总览"), QStringLiteral("任务查询"), QStringLiteral("相机与感知"),
        QStringLiteral("局部二维"), QStringLiteral("配置与诊断"), QStringLiteral("工具边界")});
    navigation->setMaximumWidth(165); navigation->setMinimumWidth(125);
    pages = new QStackedWidget(splitter); splitter->addWidget(navigation);
    splitter->addWidget(pages);
    splitter->setStretchFactor(1, 1); layout->addWidget(splitter, 1);
    QObject::connect(
      navigation, &QListWidget::currentRowChanged, pages,
      &QStackedWidget::setCurrentIndex);
    overview_page(); mission_page(); perception_page(); spatial_page(); diagnostics_page();
    tools_page();
    navigation->setCurrentRow(0);
    request_note = label(
      QStringLiteral(
        "只读模式 · 查询不会启动/"
        "取消任务，也不会切换模式"
        "或写参数。"), "requestNote", root);
    layout->addWidget(request_note); window->setCentralWidget(root);
    auto * dock = new QDockWidget(QStringLiteral("本地会话事件（有界，非任务历史数据库）"), window);
    dock->setObjectName("eventDock");
    event_model =
      new TextTable(
      {QStringLiteral("接收 steady(s)"), "session / epoch", QStringLiteral(
          "事件文本")}, dock);
    auto * events = table(event_model, "eventTable", dock); events->setMinimumHeight(80);
    events->setColumnWidth(0, 140); events->setColumnWidth(1, 135); dock->setWidget(events);
    window->addDockWidget(Qt::BottomDockWidgetArea, dock); window->resizeDocks(
      {dock}, {135},
      Qt::Vertical);
    auto * view = window->menuBar()->addMenu(QStringLiteral("视图")); view->addAction(
      dock->toggleViewAction());
    auto * exit = window->menuBar()->addAction(QStringLiteral("退出只读界面"));
    QObject::connect(exit, &QAction::triggered, window, [this] {window->close();});
  }
  void overview_page()
  {
    auto * layout =
      page(
      QStringLiteral("运行总览"), QStringLiteral(
        "观测不是控制权；armed、OF"
        "FBOARD 或 ready 均不等于"
        "可飞。"));
    auto * row = new QHBoxLayout;
    auto * state = new QGroupBox(QStringLiteral("飞机状态 · 局部 odom/ENU"), pages);
    auto * form = new QFormLayout(state); form->setFieldGrowthPolicy(
      QFormLayout::AllNonFixedFieldsGrow);
    state_connection = label({}, "stateConnection", state); armed = label({}, "stateArmed", state);
    mode = label({}, "stateMode", state); position = label({}, "statePosition", state);
    speed = label({}, "stateSpeed", state); battery = label({}, "stateBattery", state);
    validity = label({}, "stateValidity", state); state_age = label({}, "stateAges", state);
    form->addRow(QStringLiteral("PX4 连接"), state_connection); form->addRow(
      QStringLiteral(
        "解锁状态"), armed);
    form->addRow(QStringLiteral("飞行模式"), mode); form->addRow(
      QStringLiteral(
        "X / Y / 局部 Z"), position);
    form->addRow(QStringLiteral("速度范数"), speed); form->addRow(QStringLiteral("电池"), battery);
    form->addRow(QStringLiteral("独立有效性"), validity);
    form->addRow(QStringLiteral("时间证据"), state_age);
    row->addWidget(state, 1);
    auto * box = new QGroupBox(QStringLiteral("选中相机 · 只读图像"), pages);
    auto * camera_layout = new QVBoxLayout(box);
    overview_camera = new CameraCanvas(box); overview_camera->setObjectName("overviewCamera");
    camera_layout->addWidget(overview_camera); row->addWidget(box, 1); layout->addLayout(row);
    auto * phase = new QGroupBox(QStringLiteral("全局阶段观察（不关联目录任务）"), pages);
    auto * p = new QVBoxLayout(phase);
    global_overview = label({}, "globalPhaseOverview", phase); p->addWidget(global_overview);
    layout->addWidget(phase);
    layout->addWidget(
      label(
        QStringLiteral(
          "相机、实测目标和几何障碍"
          "详情在相机与感知页；能力"
          "与拒绝原因在配置与诊断页"
          "。"), "overviewHint",
        pages));
    layout->addStretch();
  }
  void mission_page()
  {
    auto * layout =
      page(
      QStringLiteral("目录任务 / 已知执行查询"), QStringLiteral(

        "仅 list、validate、query"
        "。服务 execution_id 不等"
        "于 Action UUID；这里没有"
        "启动或取消操作。"));
    auto * refresh = button(QStringLiteral("刷新目录（只读）"), "listMissions", pages);
    layout->addWidget(refresh);
    QObject::connect(
      refresh, &QPushButton::clicked, window, [this] {
        if (available()) {record(bridge::Key::MissionList, source->list_missions());}
      });
    catalog_quality = label({}, "catalogQuality", pages); layout->addWidget(catalog_quality);
    mission_list = new QListWidget(pages); mission_list->setObjectName("missionList");
    mission_list->setMinimumHeight(130); mission_list->setMaximumHeight(210); layout->addWidget(
      mission_list);
    validate = button(QStringLiteral("校验所选目录任务（静态检查）"), "validateMission", pages);
    validate->setEnabled(false); layout->addWidget(validate);
    QObject::connect(
      mission_list, &QListWidget::currentRowChanged, window, [this] {
        update_buttons();
      });
    QObject::connect(
      validate, &QPushButton::clicked, window, [this] {
        if (available() && mission_list->currentItem()) {
          const auto name = mission_list->currentItem()->text().toUtf8().toStdString();
          const auto receipt = source->validate_mission(name);
          if (receipt.status == bridge::RequestStatus::Queued) {validated_name = name;}
          record(bridge::Key::MissionValidation, receipt);
        }
      });
    validation_result = label(
      QStringLiteral(
        "尚未校验。校验成功不是飞"
        "行许可。"), "validationResult", pages);
    layout->addWidget(validation_result);
    layout->addWidget(
      label(
        QStringLiteral(
          "任务正文、完整步骤树、活"
          "动 ID / 历史执行列表：现"
          "有服务未提供，本轮不读取"
          "服务器或任意本地文件补齐"
          "。"),
        "missionLimit", pages));
    auto * query_row = new QHBoxLayout;
    execution_id = new QLineEdit(pages); execution_id->setObjectName("executionId");
    execution_id->setMaxLength(32);
    execution_id->setPlaceholderText(
      QStringLiteral(
        "输入明确已知的服务 execut"
        "ion_id（32 位小写十六进制"
        "）"));
    execution_id->setValidator(
      new QRegularExpressionValidator(
        QRegularExpression("[0-9a-f]{0,32}"),
        execution_id));
    query = button(QStringLiteral("查询执行（只读）"), "queryMission", pages);
    query->setEnabled(false);
    query_row->addWidget(execution_id, 1); query_row->addWidget(query);
    layout->addLayout(query_row);
    QObject::connect(execution_id, &QLineEdit::textChanged, window, [this] {update_buttons();});
    QObject::connect(
      query, &QPushButton::clicked, window, [this] {
        if (!available()) {return;}
        const auto id = execution_id->text().toUtf8().toStdString();
        const auto receipt = source->query_mission(id);
        if (receipt.status == bridge::RequestStatus::Queued) {queried_id = id;}
        record(bridge::Key::MissionQuery, receipt);
      });
    query_result = label(
      QStringLiteral(
        "尚无已知执行查询；目录为"
        "空也不证明没有正在执行的"
        "动作。"), "queryResult", pages);
    layout->addWidget(query_result);
    global_mission = label({}, "globalPhaseMission", pages); layout->addWidget(global_mission);
    layout->addStretch();
  }
  void perception_page()
  {
    auto * layout =
      page(
      QStringLiteral("相机与实测感知"), QStringLiteral(
        "前视 / 下视独立选择，不自"
        "动替代；目标中心不是 bbox"
        "，不猜测图像关联。"));
    camera = new QComboBox(pages); camera->setObjectName("cameraSelector");
    camera->addItems({QStringLiteral("前视相机"), QStringLiteral("下视相机（可选）")});
    layout->addWidget(camera);
    QObject::connect(
      camera, qOverload<int>(&QComboBox::currentIndexChanged), window, [this] {
        submitted_stamp = -1;
        if (decoder) {decoder->clear();} clear_images(
          QStringLiteral(
            "选源已变更，等待该源的新"
            "鲜图像"));
      });
    camera_canvas = new CameraCanvas(pages); camera_canvas->setObjectName("cameraCanvas");
    layout->addWidget(camera_canvas);
    image_evidence = label({}, "imageEvidence", pages); layout->addWidget(image_evidence);
    target_summary = label({}, "targetSummary", pages); obstacle_summary = label(
      {},
      "obstacleSummary",
      pages);
    target_model = new TextTable(
      {"ID", QStringLiteral("类别"), QStringLiteral(
          "置信度"), "X (m)", "Y (m)", "Z (m)", QStringLiteral("相机距离 m")}, pages);
    obstacle_model = new TextTable(
      {"ID", QStringLiteral(
          "类型"), "X (m)", "Y (m)", "Z (m)", QStringLiteral("尺寸 X/Y/Z m"), QStringLiteral(
          "动态")}, pages);
    layout->addWidget(target_summary); layout->addWidget(table(target_model, "targetTable", pages));
    layout->addWidget(obstacle_summary);
    layout->addWidget(table(obstacle_model, "obstacleTable", pages));
    layout->addWidget(
      label(
        QStringLiteral(
          "标签/降落专用、预测与语义"
          "类别图层未接入；实测目标"
          "不能冒充这些来源。"), "perceptionLimit",
        pages));
  }
  void spatial_page()
  {
    auto * layout =
      page(
      QStringLiteral("局部二维 · odom/ENU"), QStringLiteral(
        "仅观测实际轨迹、当前有效机位/目标/几何障碍。无点击发目标、无完整地图或飞行边界推断。"));
    map = new MapCanvas(pages); map->setObjectName("spatialCanvas"); layout->addWidget(map, 1);
    layout->addWidget(
      label(
        QStringLiteral(

          "历史轨迹最多 300s / 4096 "
          "点。地图原点不等于起飞点"
          "或地面；局部 Z 不等于离地"
          "高度。超过 ±10⁶m 的几何不"
          "绘制，原值仍可查列表。"),
        "spatialHint",
        pages));
  }
  void diagnostics_page()
  {
    auto * layout =
      page(
      QStringLiteral("配置与只读诊断"), QStringLiteral(
        "界面选择不是后端实际配置"
        "，也不会重启平台。详情全"
        "部为纯文本，可复制。"));
    configuration = new QGroupBox(QStringLiteral("下次观测连接配置（先断开再修改）"), pages);
    configuration->setObjectName("configurationPanel");
    auto * form = new QFormLayout(configuration);
    selected_profile = new QComboBox(configuration); selected_profile->setObjectName(
      "selectedProfile");
    selected_profile->addItems({"base_sim", "full_perception", "navigation_dev", "landing_dev"});
    if (selected_profile->findText(text(config.selected_profile)) < 0) {
      selected_profile->addItem(text(config.selected_profile));
    }
    selected_profile->setCurrentText(text(config.selected_profile));
    simulation = new QCheckBox(QStringLiteral("使用模拟时钟（use_sim_time）"), configuration);
    simulation->setObjectName("simulationClock"); simulation->setChecked(config.use_sim_time);
    front = new QLineEdit(text(config.image_topics[0]), configuration); front->setObjectName(
      "frontTopic"); front->setMaxLength(256);
    down = new QLineEdit(text(config.image_topics[1]), configuration); down->setObjectName(
      "downTopic"); down->setMaxLength(256);
    form->addRow(QStringLiteral("UI 选择 profile"), selected_profile); form->addRow(simulation);
    form->addRow(QStringLiteral("前视图像 topic"), front); form->addRow(
      QStringLiteral(
        "下视图像 topic"), down);
    form->addRow(
      label(
        QStringLiteral(

          "ROS domain 由进程启动环境"
          "固定；换 domain 请关闭后"
          "重新启动。landing_dev 为"
          "准备配置，不等于完整精降"
          "已就绪。"), "domainHint",
        configuration));
    layout->addWidget(configuration);
    auto * refresh = button(
      QStringLiteral(
        "刷新能力（只读，受频率限"
        "制）"), "capabilityRefresh", pages);
    layout->addWidget(refresh);
    QObject::connect(
      refresh, &QPushButton::clicked, window, [this] {
        if (!available()) {return;}
        const std::array<bridge::CapabilitySource, 4> sources{bridge::CapabilitySource::Flight,
          bridge::CapabilitySource::Mission, bridge::CapabilitySource::Perception,
          bridge::CapabilitySource::Landing};
        for (int i = 0; i < 4; ++i) {
          record(static_cast<bridge::Key>(i), source->capabilities(sources[i]));
        }
      });
    capability_model = new TextTable(
      {QStringLiteral("来源"), QStringLiteral(
          "分项"), "implemented", "ready (查询时)", QStringLiteral("质量/原因")}, pages);
    layout->addWidget(table(capability_model, "capabilityTable", pages));
    auto * row = new QHBoxLayout; parameter = new QComboBox(pages); parameter->setObjectName(
      "parameterSelector");
    parameter->addItems({"profile", "use_sim_time", "operator_note", "export_pretty"});
    row->addWidget(parameter);
    auto * read = button(QStringLiteral("读取参数（白名单，只读）"), "readParameter", pages);
    row->addWidget(read); row->addStretch(); layout->addLayout(row);
    QObject::connect(
      read, &QPushButton::clicked, window, [this] {
        if (!available()) {return;}
        const std::array<bridge::Parameter, 4> values{bridge::Parameter::Profile,
          bridge::Parameter::UseSimTime,
          bridge::Parameter::OperatorNote, bridge::Parameter::ExportPretty};
        const std::array<bridge::Key, 4> keys{bridge::Key::Profile, bridge::Key::UseSimTime,
          bridge::Key::OperatorNote, bridge::Key::ExportPretty};
        record(
          keys[parameter->currentIndex()],
          source->read_parameter(values[parameter->currentIndex()]));
      });
    parameter_model = new TextTable(
      {QStringLiteral("参数"), QStringLiteral(
          "原 JSON 值"), QStringLiteral("类型"), QStringLiteral("查询质量/年龄")}, pages);
    layout->addWidget(table(parameter_model, "parameterTable", pages));
    diagnostics = new QPlainTextEdit(pages); diagnostics->setObjectName("diagnosticText");
    diagnostics->setReadOnly(true);
    diagnostics->setMaximumBlockCount(600); diagnostics->setMinimumHeight(180); layout->addWidget(
      diagnostics);
  }
  void tools_page()
  {
    auto * layout =
      page(
      QStringLiteral("专业工具边界"), QStringLiteral(
        "本轮不启动或连接 QGC / RV"
        "iz；没有伪装成可用的按钮"
        "。"));
    layout->addWidget(
      label(
        QStringLiteral(
          "RViz2：后续先用独立进程与只读预设；目标输入仍需经过统一导航。\n\nQGC"
          "：平台之外的命令源；只关闭 EasyFly 按钮不能限制其控制权。连接端点、心跳"
          "与安全边界尚未验收。\n\nWeb、任务编辑/导入、录像和回放属于后续阶段。本程"
          "序不查找或管理用户已启动的工具。"),
        "toolsBoundary", pages));
    layout->addStretch();
  }
  bool available() const
  {
    return source && current && current->running && !current->stopped && !closing && !disconnecting;
  }
  void update_buttons()
  {
    validate->setEnabled(available() && mission_list->currentItem());
    query->setEnabled(
      available() && bridge::detail::execution_id(
        execution_id->text().toStdString()));
  }
  void record(bridge::Key key, bridge::RequestReceipt receipt)
  {
    if (receipt.status == bridge::RequestStatus::Queued) {
      expected[key] = receipt.id;
      request_note->setText(
        QStringLiteral(
          "只读请求已在本地排队 #%1"
          "；等待服务回复，不表示准"
          "入/执行成功。").arg(receipt.id));
      return;
    }
    const auto status = receipt.status == bridge::RequestStatus::Busy ? QStringLiteral(
      "忙：已有请求或超并发预算") :
      receipt.status == bridge::RequestStatus::RateLimited ?
      QStringLiteral("受频率限制：每端点最多 1Hz") :
      receipt.status == bridge::RequestStatus::Invalid ?
      QStringLiteral("输入无效") : QStringLiteral("观测已关闭");
    request_note->setText(status + QStringLiteral("；未发送新的请求。"));
  }
  bool pending(bridge::Key key, uint64_t id) const
  {
    const auto found = expected.find(key); return found != expected.end() && found->second != id;
  }
  void clear_images(const QString & message)
  {
    overview_camera->frame({}, message); camera_canvas->frame({}, message);
  }
  void connect_source()
  {
    if (closing || disconnecting || (source && current && !current->stopped)) {return;}
    if (configuration) {
      config.selected_profile = selected_profile->currentText().toStdString();
      config.use_sim_time = simulation->isChecked();
      config.image_topics = {front->text().toStdString(), down->text().toStdString()};
    }
    try {
      source = factory(config, history);
      if (!source) {throw std::runtime_error("只读数据源未创建");}
      decoder = std::make_unique<ImagePipeline>();
      current = source->snapshot();
      if (!current) {throw std::runtime_error("只读数据源没有初始快照");}
      released = false; configuration->setEnabled(false);
      expected.clear(); catalog_revision = 0; geometry_revision.clear(); geometry_visible.clear();
      queried_id.clear(); validated_name.clear(); mission_list->clear();
      last_session = 0; last_camera = -1; submitted_stamp = -1;
      toggle->setText(QStringLiteral("断开观测")); toggle->setEnabled(true);
    } catch (const std::exception & e) {
      auto failed = std::make_shared<bridge::UiSnapshot>(); failed->stopped = true;
      failed->produced_steady = bridge::detail::steady_seconds(); failed->local_error = e.what();
      current = failed; released = !source && !decoder;
      request_note->setText(QStringLiteral("启动只读观测失败：") + text(e.what(), 2048));
      if (!released) {
        stop_source(false);
      } else {
        configuration->setEnabled(true); toggle->setText(QStringLiteral("重新连接观测"));
      }
    }
  }
  void stop_source(bool close_window)
  {
    closing = close_window || closing; disconnecting = true; toggle->setEnabled(false);
    if (source) {source->stop();}
    if (decoder) {decoder->stop();}
    clear_images(QStringLiteral("正在释放本地只读资源")); update_buttons();
    request_note->setText(
      QStringLiteral(
        "清理中：窗口仍可响应；不"
        "取消外部动作、不关闭用户"
        "节点或工具。"));
  }
  void cleanup_tick()
  {
    if (source && (!current || !current->stopped)) {return;}
    if (decoder && !decoder->stats().stopped) {return;}
    history = current; source.reset(); decoder.reset(); released = true; disconnecting = false;
    configuration->setEnabled(!closing); toggle->setEnabled(!closing); toggle->setText(
      QStringLiteral(
        "重新连接观测"));
    if (closing) {QTimer::singleShot(0, window, [this] {window->close();});} else {
      request_note->setText(
        QStringLiteral(
          "本地观测已断开；保留历史"
          "警告。修改配置后可重建新"
          "的只读会话。"));
    }
  }
  void state_view(double now)
  {
    const auto & s = *current; const auto & v = s.state;
    const auto q = display_quality(v.evidence, s, now, 1, true);
    const bool live = q == bridge::Quality::ValidAtObservation && v.sdk.received && v.sdk.fresh;
    const auto unknown = v.sdk.received ?
      QStringLiteral("最后已知 / 不新鲜") : QStringLiteral("未收到 / —");
    state_connection->setText(
      live ? (v.sdk.value.connected ? QStringLiteral("已连接（本次新鲜观测）") :
      QStringLiteral("未连接（本次新鲜观测）")) : unknown);
    armed->setText(live ? yes(v.sdk.value.armed) : unknown);
    mode->setText(live ? text(v.sdk.value.flight_mode_name, 256) : unknown);
    position->setText(
      live &&
      v.position_valid ? QStringLiteral("%1 / %2 / %3 m").arg(
        number(v.sdk.value.position_x),
        number(v.sdk.value.position_y),
        number(v.sdk.value.position_z)) : QStringLiteral("—（定位不可用）"));
    speed->setText(
      live && v.velocity_valid && v.speed_mps ? number(
        *v.speed_mps) + " m/s" : QStringLiteral("—（速度不可用）"));
    battery->setText(
      live && v.battery_valid && v.battery_percent ? number(
        *v.battery_percent,
        1) + "%" : QStringLiteral(
        "—（电池不可用）"));
    validity->setText(
      QStringLiteral("位置 %1 · 速度 %2 · 姿态 %3\n电池 %4 · GPS %5\n显示质量：%6")
      .arg(
        yes(live && v.position_valid), yes(live && v.velocity_valid), yes(live && v.attitude_valid),
        yes(live && v.battery_valid), yes(live && v.gps_valid), quality_text(q)));
    state_age->setText(ages(v.evidence, s, now));
    global_overview->setText(phase_event(s, now)); global_mission->setText(phase_event(s, now));
  }
  void mission_view(double now)
  {
    const auto & s = *current;
    const auto list = s.missions.find(bridge::Key::MissionList);
    if (list != s.missions.end()) {
      const auto & r = list->second;
      catalog_quality->setText(
        pending(
          bridge::Key::MissionList,
          r.evidence.request_id) ? QStringLiteral("等待目录查询回复（不推断活动任务）") :
        quality_text(
          display_quality(
            r.evidence, s, now, 2,
            false)) + " · " + ages(r.evidence, s, now));
      if (!pending(
          bridge::Key::MissionList,
          r.evidence.request_id) && r.evidence.request_id != catalog_revision)
      {
        const auto selection =
          mission_list->currentItem() ? mission_list->currentItem()->text() : QString{};
        mission_list->clear(); catalog_revision = r.evidence.request_id;
        if (r.value && r.value->success) {
          for (const auto & name : r.value->missions) {
            if (bridge::detail::basename(name)) {mission_list->addItem(text(name, 255));}
          }
        }
        const auto matches = mission_list->findItems(selection, Qt::MatchExactly);
        if (!matches.empty()) {mission_list->setCurrentItem(matches[0]);}
      }
    }
    const auto validation = s.missions.find(bridge::Key::MissionValidation);
    if (validation != s.missions.end() && !validated_name.empty()) {
      const auto & r = validation->second;
      validation_result->setText(
        pending(bridge::Key::MissionValidation, r.evidence.request_id) ?
        QStringLiteral("等待静态校验：") +
        text(validated_name, 255) : QStringLiteral(
          "校验请求文件：%1\n%2 · 上"
          "游状态：%3\n%4\n仅静态检"
          "查，不是飞行许可。")
        .arg(
          text(validated_name, 255), quality_text(display_quality(r.evidence, s, now, 2, false)),
          r.value ? text(r.value->status) : QStringLiteral("无可核对响应"),
          r.value ? text(r.value->message, 2048) : text(r.evidence.reason, 2048)));
    }
    const auto query_status = s.missions.find(bridge::Key::MissionQuery);
    if (query_status != s.missions.end() && !queried_id.empty()) {
      const auto & r = query_status->second;
      query_result->setText(
        pending(bridge::Key::MissionQuery, r.evidence.request_id) ?
        QStringLiteral("等待只读查询：") + text(queried_id) :
        mission_result(r, s, now, queried_id));
    }
  }
  void geometry_view(bridge::Key key, double now)
  {
    const auto & s = *current; const auto found = s.perception.find(key);
    auto * summary = key == bridge::Key::Targets ? target_summary : obstacle_summary;
    auto * model = key == bridge::Key::Targets ? target_model : obstacle_model;
    if (found == s.perception.end()) {
      summary->setText(QStringLiteral("未收到感知查询")); model->replace({}); return;
    }
    const auto & observation = found->second; const bool valid = geometry_current(s, key, now);
    const auto quality = display_quality(observation.evidence, s, now, 1, true);
    QString report;
    const auto meta = s.perception_reports.find(key);
    if (meta != s.perception_reports.end() && meta->second.received) {
      report = QStringLiteral(
        " · 上游 status=%1 / age=%"
        "2s · 源=%3").arg(meta->second.upstream_status)
        .arg(number(meta->second.reported_age_sec)).arg(text(meta->second.source_topic, 256));
    }
    const auto count =
      valid ? (key ==
      bridge::Key::Targets ? observation.value->targets.targets.size() : observation.value->
      obstacles.obstacles.size()) : 0;
    summary->setText(
      (valid ? QStringLiteral("有效查询：%1 个").arg(count) : quality_text(quality) +
      QStringLiteral(
        "（不显示当前几何，不是有"
        "效空）")) + report + "\n" + ages(observation.evidence, s, now));
    if (geometry_revision[key] == observation.evidence.request_id &&
      geometry_visible[key] == valid)
    {
      return;
    }
    geometry_revision[key] = observation.evidence.request_id; geometry_visible[key] = valid;
    QVector<QStringList> rows;
    if (valid && key == bridge::Key::Targets) {
      for (const auto & t : observation.value->targets.targets) {
        rows.push_back(
          {QString::number(t.track_id), text(t.class_name, 256), number(
              t.confidence), number(t.position_x), number(t.position_y), number(
              t.position_z), number(t.distance)});
      }
    } else if (valid) {
      for (const auto & o : observation.value->obstacles.obstacles) {
        rows.push_back(
          {QString::number(o.obstacle_id), text(o.obstacle_type, 256), number(o.center_x),
            number(o.center_y), number(o.center_z),
            QStringLiteral("%1 / %2 / %3").arg(
              number(o.size_x), number(o.size_y), number(
                o.size_z)), yes(o.is_dynamic)});
      }
    }
    model->replace(std::move(rows));
  }
  void image_view(double now)
  {
    const auto & s = *current; const int channel = camera->currentIndex();
    const auto & frame = s.images[channel];
    const ImageBinding binding{s.observation_session_id, s.time_epoch, channel,
      config.image_topics[channel]};
    if (last_session != s.observation_session_id || last_epoch != s.time_epoch ||
      last_camera != channel)
    {
      last_session = s.observation_session_id; last_epoch = s.time_epoch; last_camera = channel;
      submitted_stamp = -1; clear_images(QStringLiteral("会话/源已变更，等待新观测"));
    }
    const auto q = display_quality(frame.evidence, s, now, 1, true);
    image_evidence->setText(
      text(
        binding.topic,
        256) + " · " + quality_text(q) + "\n" + ages(frame.evidence, s, now));
    if (!decoder || !available() || q != bridge::Quality::ValidAtObservation || !frame.value) {
      if (decoder) {decoder->clear();} submitted_stamp = -1;
      clear_images(text(binding.topic, 256) + " · " + quality_text(q)); return;
    }
    decoder->bind(binding);
    const auto ns = bridge::detail::stamp_ns(frame.value->header.stamp);
    if (ns != submitted_stamp) {
      decoder->submit(frame.value, frame.evidence.first_source_steady); submitted_stamp = ns;
    }
    const auto decoded = decoder->latest();
    if (!decoded || !(decoded->binding == binding)) {return;}
    const double age = static_cast<double>(s.clock.now_ns - decoded->stamp_ns) / 1e9;
    const bool fresh = decoded->stamp_ns <= ns && age >= -.2 && age <= 1 &&
      now - decoded->first_received <= 1;
    if (!fresh || !decoded->error.empty()) {
      clear_images(
        decoded->error.empty() ?
        QStringLiteral("转换结果已过期，不显示旧帧") : text(decoded->error));
      return;
    }
    const auto caption = QStringLiteral("%1 · %2 · 帧源年龄 %3s · stamp=%4").arg(
      camera->currentText(), text(binding.topic, 256), number(age)).arg(decoded->stamp_ns);
    overview_camera->frame(decoded->image, caption); camera_canvas->frame(decoded->image, caption);
  }
  void diagnostics_view(double now)
  {
    const auto & s = *current; QVector<QStringList> capabilities; QStringList details;
    details <<
      QStringLiteral("只读 UI，不作为飞行许可。ROS_DOMAIN_ID=%1（仅启动环境，非鉴权）").arg(
      qEnvironmentVariable(
        "ROS_DOMAIN_ID",
        "0"));
    details << QStringLiteral("会话 %1 / epoch %2").arg(s.observation_session_id).arg(s.time_epoch);
    details << text(s.local_error, 2048);
    int observed = 0;
    for (const auto & item : s.capabilities) {
      const auto & r = item.second; const auto q = display_quality(r.evidence, s, now, 2, true);
      const bool valid_shape = r.value &&
        bridge::detail::validate_capabilities(*r.value) == bridge::Quality::ValidAtObservation;
      if (valid_shape) {
        ++observed;
        for (std::size_t i = 0; i < r.value->names.size(); ++i) {
          const auto qualifier = q ==
            bridge::Quality::ValidAtObservation ?
            QStringLiteral("仅查询时") : QStringLiteral("最后已知");
          capabilities.push_back(
            {key_name(item.first), text(r.value->names[i], 256), yes(r.value->implemented[i]),
              yes(r.value->ready[i]) + " (" + qualifier + ")", quality_text(q)});
          details << key_name(item.first) + "/" + text(r.value->names[i], 256) +
            QStringLiteral(" · 观测 steady=%1 s").arg(number(r.evidence.received_steady, 3));
          details << text(r.value->details[i]);
          if (item.first == bridge::Key::FlightCapabilities &&
            !bridge::detail::command_uncertain(r.value->details[i]))
          {
            details << QStringLiteral(
              "command_uncertain 诊断："
              "未知（未解析/缺字段），不"
              "能默认健康。");
          }
        }
      } else {capabilities.push_back({key_name(item.first), "—", "—", "—", quality_text(q)});}
      details << text(r.evidence.endpoint, 256) + " · " + text(r.evidence.reason, 2048);
    }
    capability_model->replace(std::move(capabilities));
    connection->setText(
      QStringLiteral("只读 · %1 · 曾观测平台能力端点 %2/4（非当前可飞）")
      .arg(
        disconnecting ? QStringLiteral("本地资源清理中") :
        s.running ? QStringLiteral("本地观测器运行中") :
        QStringLiteral("本地观测已停止")).arg(observed));
    clock->setText(
      QStringLiteral("%1 · %2 · 界面快照年龄 %3s").arg(
        config.use_sim_time ? QStringLiteral(
          "模拟时间") : QStringLiteral("系统时间"), quality_text(s.clock.quality),
        number(snapshot_delay(s, now))));
    QVector<QStringList> parameters;
    QString actual = QStringLiteral("后端 profile 未确认");
    const std::array<bridge::Key, 4> keys{bridge::Key::Profile, bridge::Key::UseSimTime,
      bridge::Key::OperatorNote, bridge::Key::ExportPretty};
    for (int i = 0; i < 4; ++i) {
      const auto found = s.parameters.find(keys[i]); if (found == s.parameters.end()) {continue;}
      const auto & r = found->second; const auto q = display_quality(r.evidence, s, now, 2, false);
      parameters.push_back(
        {parameter_name(i), r.value ? text(
            r.value->current_value) : "—", r.value ? json_type(
            r.value->current_value) : "—", quality_text(q) + " · " + ages(r.evidence, s, now)});
      if (i == 0 && r.value && r.value->success && !json_string(r.value->current_value).isEmpty()) {
        actual = QStringLiteral("后端 profile（%1）：%2").arg(
          q == bridge::Quality::ValidAtObservation ? QStringLiteral(
            "查询时") : QStringLiteral("最后已知"), json_string(r.value->current_value));
      }
    }
    parameter_model->replace(std::move(parameters));
    backend_profile->setText(
      QStringLiteral("UI 选择：%1 · %2（界面不启动/切换后端）").arg(
        text(
          config.
          selected_profile, 256), actual));
    const auto detail_text = details.join('\n');
    if (diagnostics->toPlainText() != detail_text) {diagnostics->setPlainText(detail_text);}
    if (s.command_uncertain_observed) {
      warning->setText(
        QStringLiteral(

          "⚠ 曾观察到飞控 command_un"
          "certain：需人工核查。历史"
          "来源 session=%1，steady=%"
          "2s；新遥测/重连不表示故障"
          "锁已解除。")
        .arg(s.command_uncertain_observed->observation_session_id).arg(
          number(
            s.
            command_uncertain_observed->steady, 1))); warning->show();
    } else {warning->hide();}
    const auto stats = decoder ? decoder->stats() : ImageQueueStats{};
    window->statusBar()->showMessage(
      QStringLiteral(
        "只读请求 %1/8 · 超时 %2 ·"
        " 图像丢弃 %3 · 转换 pendi"
        "ng %4/1 · 无效样本 %5 · "
        "不持有飞行操作")
      .arg(s.pending_queries).arg(s.counters.timed_out).arg(
        s.counters.dropped_images +
        stats.dropped).arg(stats.pending).arg(s.counters.invalid_samples));
  }
  void events_view()
  {
    const auto & events = current->events;
    const auto tail = events.empty() ? -1 : events.back().steady;
    const auto session = events.empty() ? 0 : events.back().observation_session_id;
    if (tail == event_tail && events.size() == event_count && session == event_session) {return;}
    event_tail = tail; event_count = events.size(); event_session = session;
    QVector<QStringList> rows;
    const std::size_t start = events.size() > 1000 ? events.size() - 1000 : 0;
    for (std::size_t i = start; i < events.size(); ++i) {
      rows.push_back(
        {number(events[i].steady, 3),
          QStringLiteral("%1 / %2").arg(events[i].observation_session_id).arg(
            events[i].epoch), text(events[i].text, 2048)});
    }
    event_model->replace(std::move(rows));
  }
  void tick()
  {
    if (source) {current = source->snapshot();}
    if (disconnecting) {cleanup_tick();}
    if (!current) {return;}
    const double now = bridge::detail::steady_seconds();
    if (source && current->stopped && !disconnecting && !released) {stop_source(false);}
    state_view(now); mission_view(now); geometry_view(bridge::Key::Targets, now);
    geometry_view(bridge::Key::Obstacles, now); image_view(now); diagnostics_view(now);
    events_view();
    map->snapshot(current, now); update_buttons();
  }
};

MainWindow::MainWindow(const bridge::BridgeConfig & config, SourceFactory factory, QWidget * parent)
: QMainWindow(parent), impl_(std::make_unique<Impl>(this, config, std::move(factory))) {}
MainWindow::~MainWindow() = default;
bool MainWindow::cleanup_complete() const {return impl_->released;}
void MainWindow::closeEvent(QCloseEvent * event)
{
  if (impl_->released) {event->accept(); return;}
  event->ignore(); if (!impl_->closing) {impl_->stop_source(true);}
}
}  // namespace easyfly_desktop

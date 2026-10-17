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
#include <QCoreApplication>
#include <QDockWidget>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMenuBar>
#include <QMessageBox>
#include <QFileDialog>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRegularExpressionValidator>
#include <QScrollArea>
#include <QScrollBar>
#include <QSignalBlocker>
#include <QSplitter>
#include <QStackedWidget>
#include <QStatusBar>
#include <QTableView>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <utility>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_packages_with_prefixes.hpp>

#include "easyfly_desktop/image_pipeline.hpp"
#include "easyfly_desktop/main_window.hpp"
#include "easyfly_desktop/mission_editor.hpp"
#include "easyfly_desktop/mission_draft.hpp"
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
  std::unique_ptr<ImagePipeline> overlay_decoder;
  int64_t overlay_submitted_stamp{-1};
  std::optional<bridge::CameraObservations> overlay_observations;
  QComboBox * overlay_mode{nullptr};
  QLabel * overlay_evidence{nullptr};
  TextTable * overlay_model{nullptr};
  QTimer * timer{nullptr};
  bool closing{false}, disconnecting{false}, released{false};
  ToolOptions tool_options;
  ToolProcess rviz_process, qgc_process;
  QMessageBox * exit_prompt{nullptr};
  QMessageBox * mission_exit_prompt{nullptr};
  bool mission_exit_confirmed{false};
  MissionOptions mission_options;
  ControlFactory controls_factory;
  MissionPanel * mission_panel{nullptr};
  ParameterOptions parameter_options;
  ParameterFactory parameter_factory;
  ParameterPanel * parameter_panel{nullptr};
  QMessageBox * parameter_exit_prompt{nullptr};
  bool parameter_exit_confirmed{false};
  ImportOptions import_options;
  ImportFactory import_factory;
  RvizOptions spatial_options;
  RecordingOptions recording_options;
  RecordingPanel * recording_panel{nullptr};
  std::shared_ptr<const ConvertedFrame> recording_frame;
  RvizView * rviz_view{nullptr};
  QWidget * rviz_full_page{nullptr}, *overview_rviz_host{nullptr};
  QVBoxLayout * rviz_full_layout{nullptr}, *overview_rviz_layout{nullptr};
  QTabWidget * overview_map_tabs{nullptr};
  ImportPanel * import_panel{nullptr};
  QMessageBox * import_exit_prompt{nullptr};
  bool import_exit_confirmed{false};
  QLineEdit * rviz_path{nullptr}, *qgc_path{nullptr};
  QComboBox * rviz_preset{nullptr};
  QLabel * rviz_state{nullptr}, *qgc_state{nullptr};
  QPlainTextEdit * tool_log{nullptr};
  QPushButton * rviz_start{nullptr}, *qgc_start{nullptr};
  QPushButton * rviz_stop{nullptr}, *qgc_stop{nullptr};
  uint64_t last_session{0}, last_epoch{0};
  int last_camera{-1}; int64_t submitted_stamp{-1};
  std::map<bridge::Key, uint64_t> expected;
  std::string queried_id, validated_name;
  std::string details_name;
  uint64_t rendered_details{0}, rendered_details_session{0};
  QPushButton * inspect{nullptr};
  QLabel * details_summary{nullptr};
  QPlainTextEdit * mission_content{nullptr};
  QTabWidget * document_tabs{nullptr};
  TextTable * steps_model{nullptr};
  uint64_t catalog_revision{0};
  std::map<bridge::Key, uint64_t> geometry_revision;
  std::map<bridge::Key, bool> geometry_visible;
  double event_tail{-1}; std::size_t event_count{0}; uint64_t event_session{0};

  QStackedWidget * pages{nullptr}; QListWidget * navigation{nullptr};
  MissionEditor * editor{nullptr};
  QMessageBox * draft_exit_prompt{nullptr};
  bool draft_exit_confirmed{false};
  QLabel * connection{nullptr}, *clock{nullptr}, *backend_profile{nullptr}, *warning{nullptr};
  QLabel * request_note{nullptr}, *state_connection{nullptr}, *armed{nullptr}, *mode{nullptr};
  QLabel * position{nullptr}, *speed{nullptr}, *battery{nullptr}, *validity{nullptr},
  *state_age{nullptr};
  QLabel * altitude_metric{nullptr};
  FlightInstrumentCanvas * instruments{nullptr};
  QLabel * global_overview{nullptr}, *global_mission{nullptr}, *catalog_quality{nullptr};
  QLabel * validation_result{nullptr}, *query_result{nullptr}, *image_evidence{nullptr};
  QLabel * target_summary{nullptr}, *obstacle_summary{nullptr};
  QPushButton * toggle{nullptr}, *validate{nullptr}, *query{nullptr};
  QListWidget * mission_list{nullptr}; QLineEdit * execution_id{nullptr};
  QComboBox * camera{nullptr}, *selected_profile{nullptr}, *parameter{nullptr};
  QCheckBox * simulation{nullptr}; QLineEdit * front{nullptr}, *down{nullptr};
  QWidget * configuration{nullptr};
  CameraCanvas * overview_camera{nullptr}, *camera_canvas{nullptr}; MapCanvas * map{nullptr};
  MapCanvas * overview_map{nullptr};
  TextTable * capability_model{nullptr}, *target_model{nullptr}, *obstacle_model{nullptr};
  TextTable * parameter_model{nullptr}, *event_model{nullptr};
  QPlainTextEdit * diagnostics{nullptr};

  Impl(
    MainWindow * owner, const bridge::BridgeConfig & options, SourceFactory source_factory,
    const ToolOptions & tools, const MissionOptions & missions, ControlFactory controls,
    const ParameterOptions & parameters, ParameterFactory parameter_controls,
    const ImportOptions & imports, ImportFactory import_controls, const RvizOptions & spatial,
    const RecordingOptions & recording)
  : window(owner), config(options), factory(std::move(source_factory)), tool_options(tools),
    mission_options(missions), controls_factory(std::move(controls)), parameter_options(parameters),
    parameter_factory(std::move(parameter_controls)), import_options(imports),
    import_factory(std::move(import_controls)), spatial_options(spatial), recording_options(
      recording)
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
    window->setWindowTitle(
      mission_options.enabled ? QStringLiteral("EasyFly · 受管仿真任务") :
      import_options.enabled ? QStringLiteral("EasyFly · 飞行观测只读 / 任务导入启用") :
      parameter_options.enabled ? QStringLiteral("EasyFly · 飞行观测只读 / 元数据编辑启用") :
      QStringLiteral("EasyFly · 只读平台观测")); window->resize(1440, 900);
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
    auto * brand = new QLabel(top_bar); brand->setObjectName("brand");
    brand->setFixedSize(190, 66); brand->setAlignment(Qt::AlignCenter);
    const auto logo = brand_pixmap();
    brand->setPixmap(logo.scaled(180, 60, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    brand->setAccessibleName(QStringLiteral("EasyFly Logo"));
    // Bound the X11 _NET_WM_ICON payload; do not send the full 2752x1536 source.
    window->setWindowIcon(
      QIcon(
        logo.scaled(
          64, 64, Qt::KeepAspectRatio,
          Qt::SmoothTransformation)));
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
      {QStringLiteral("运行总览"), mission_options.enabled ? QStringLiteral("仿真任务") :
        QStringLiteral("任务查询"), QStringLiteral("相机与感知"),
        QStringLiteral("局部二维"), QStringLiteral("配置与诊断"), QStringLiteral("工具边界"),
        import_options.enabled ? QStringLiteral("任务编辑/导入") :
        QStringLiteral("任务编辑（本地）")});
    const QStringList icons{"overview", "tasks", "camera", "map", "diagnostics", "tools", "edit"};
    for (int i = 0; i < navigation->count(); ++i) {
      navigation->item(i)->setIcon(dashboard_icon(icons[i]));
    }
    navigation->setIconSize(QSize(20, 20));
    navigation->setMaximumWidth(175); navigation->setMinimumWidth(145);
    pages = new QStackedWidget(splitter); splitter->addWidget(navigation);
    splitter->addWidget(pages);
    splitter->setStretchFactor(1, 1); layout->addWidget(splitter, 1);
    QObject::connect(
      navigation, &QListWidget::currentRowChanged, pages,
      &QStackedWidget::setCurrentIndex);
    overview_page(); mission_page(); perception_page(); spatial_page(); diagnostics_page();
    tools_page();
    auto * draft_scroll = new QScrollArea(pages); draft_scroll->setWidgetResizable(true);
    std::vector<std::string> protected_paths{EASYFLY_DRAFT_SOURCE_ROOT, EASYFLY_DRAFT_INSTALL_ROOT};
    std::map<std::string, std::string> installed_packages;
    try {
      installed_packages = ament_index_cpp::get_packages_with_prefixes();
    } catch (const std::exception &) {
      // The installed executable prefix is also protected independently of package discovery.
    }
    protected_paths = local_storage_protected_paths(
      std::move(protected_paths), installed_packages,
      QCoreApplication::applicationFilePath().toStdString());
    try {
      protected_paths.push_back(
        ament_index_cpp::get_package_share_directory(
          "easyfly_mission_fsm") + "/missions");
    } catch (const std::exception &) {
      // Source/install trees remain protected even if the package index is unavailable offline.
    }
    auto * draft_page = new QWidget(draft_scroll);
    const auto recording_protected_paths = protected_paths;
    auto * draft_layout = new QVBoxLayout(draft_page);
    editor = new MissionEditor(
      draft_page, std::move(protected_paths), [this] {
        draft_exit_confirmed = false;
      });
    draft_layout->addWidget(editor);
    auto * import_box = new QGroupBox(QStringLiteral("受控导入（独立显式启用）"), draft_page);
    auto * import_layout = new QVBoxLayout(import_box);
    import_panel = new ImportPanel(
      import_options, import_factory, import_box,
      [this] {return editor->exported_draft();});
    import_layout->addWidget(import_panel); draft_layout->addWidget(import_box);
    draft_scroll->setWidget(draft_page); pages->addWidget(draft_scroll);
    if (spatial_options.enabled) {
      navigation->addItem(QStringLiteral("三维观测（实验）"));
      navigation->item(navigation->count() - 1)->setIcon(dashboard_icon("map"));
      rviz_full_page = new QWidget(pages); rviz_full_layout = new QVBoxLayout(rviz_full_page);
      rviz_full_layout->setContentsMargins(0, 0, 0, 0);
      rviz_view = new RvizView(rviz_full_page); rviz_view->setObjectName("embeddedRvizPage");
      rviz_full_layout->addWidget(rviz_view);
      pages->addWidget(rviz_full_page);
      overview_rviz_host = new QWidget(overview_map_tabs);
      overview_rviz_layout = new QVBoxLayout(overview_rviz_host);
      overview_rviz_layout->setContentsMargins(0, 0, 0, 0);
      overview_map_tabs->addTab(
        overview_rviz_host, dashboard_icon("map"),
        QStringLiteral("RViz 三维"));
      QObject::connect(
        overview_map_tabs, &QTabWidget::currentChanged, window, [this] {
          mount_rviz();
        });
      QObject::connect(
        navigation, &QListWidget::currentRowChanged, window, [this] {
          mount_rviz();
        });
    }
    if (recording_options.enabled && RecordingWorker::available()) {
      auto protected_records = recording_protected_paths;
      protected_records.push_back(mission_options.journal_directory);
      protected_records.push_back(import_options.journal_directory);
      navigation->addItem(QStringLiteral("录像与记录"));
      auto * recording_scroll = new QScrollArea(pages);
      recording_scroll->setWidgetResizable(true); recording_scroll->setFrameShape(QFrame::NoFrame);
      recording_panel = new RecordingPanel(std::move(protected_records), recording_scroll);
      recording_panel->setObjectName("recordingPage");
      recording_scroll->setWidget(recording_panel); pages->addWidget(recording_scroll);
    }
    navigation->setCurrentRow(0);
    request_note = label(
      QStringLiteral(
        "只读模式 · 查询不会启动/"
        "取消任务，也不会切换模式"
        "或写参数。"), "requestNote", root);
    layout->addWidget(request_note); window->setCentralWidget(root);
    if (mission_options.enabled) {
      request_note->setText(QStringLiteral("查询仍只读；仿真任务启动/取消只走独立确认入口。"));
    }
    if (parameter_options.enabled) {
      request_note->setText(
        QStringLiteral(
          "观察查询只读；两项元数据写入仅走独立确认入口，不开放飞行参数。"));
    }
    if (import_options.enabled) {
      request_note->setText(
        QStringLiteral(
          "文件导入单独确认，不自动执行；查询只读，任务控制须另行启用。"));
    }
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
    dock->hide();  // Keep the overview visual; the event log remains available from View.
    auto * exit = window->menuBar()->addAction(QStringLiteral("退出 EasyFly"));
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
    auto * metrics = new QGridLayout; metrics->setSpacing(10);
    const auto metric =
      [&](const QString & title, const QString & icon, const char * name, int col) {
        auto * card = new QFrame(pages); card->setProperty("dashboardCard", true);
        auto * content = new QVBoxLayout(card); content->setContentsMargins(12, 9, 12, 9);
        auto * heading = new QHBoxLayout;
        auto * glyph = new QLabel(card); glyph->setPixmap(dashboard_icon(icon).pixmap(22, 22));
        glyph->setFixedSize(24, 24); heading->addWidget(glyph);
        auto * caption = label(title, "metricTitle", card);
        caption->setProperty("metricTitle", true);
        heading->addWidget(caption, 1); content->addLayout(heading);
        auto * value = label({}, name, card); value->setProperty("metricValue", true);
        content->addWidget(value); metrics->addWidget(card, 0, col); metrics->setColumnStretch(
          col,
          1);
        return value;
      };
    state_connection = metric(QStringLiteral("飞控连接"), "link", "stateConnection", 0);
    mode = metric(QStringLiteral("飞行模式"), "mode", "stateMode", 1);
    altitude_metric = metric(QStringLiteral("局部高度 Z · m"), "location", "overviewAltitude", 2);
    speed = metric(QStringLiteral("速度 · m/s"), "speed", "stateSpeed", 3);
    battery = metric(QStringLiteral("电池"), "battery", "stateBattery", 4);
    layout->addLayout(metrics);

    auto * split = new QSplitter(Qt::Horizontal, pages); split->setObjectName("overviewSplitter");
    split->setChildrenCollapsible(false);
    split->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Ignored);
    split->setMinimumHeight(430);
    overview_map_tabs = new QTabWidget(split); overview_map_tabs->setObjectName("overviewMapTabs");
    overview_map_tabs->setMinimumWidth(320);
    overview_map_tabs->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Ignored);
    overview_map_tabs->setMinimumHeight(380);
    auto * local = new QWidget(overview_map_tabs); auto * local_layout = new QVBoxLayout(local);
    local_layout->setContentsMargins(4, 4, 4, 4);
    overview_map = new MapCanvas(local); overview_map->setObjectName("overviewMapCanvas");
    overview_map->setMinimumHeight(340); local_layout->addWidget(overview_map, 1);
    local_layout->addWidget(
      label(
        QStringLiteral("局部位置 / 实际轨迹；不是卫星底图。SLAM 与点云请选 RViz 三维。"),
        "overviewMapHint", local));
    overview_map_tabs->addTab(local, dashboard_icon("map"), QStringLiteral("局部地图 · ENU"));
    if (!spatial_options.enabled) {
      auto * unavailable = new QWidget(overview_map_tabs);
      auto * help = new QVBoxLayout(unavailable);
      auto * notice = label(
        RvizView::available() ? QStringLiteral(
          "本次未启用三维入口。使用 --enable-rviz-embed 启动后，可在总览手动打开 RViz。") :
        QStringLiteral("此构建未包含内嵌 RViz；可使用“工具边界”页的独立 RViz。"),
        "overviewRvizUnavailable", unavailable);
      notice->setMaximumHeight(120); help->addWidget(notice);
      auto * tools = button(QStringLiteral("查看独立 RViz 入口"), "overviewRvizTools", unavailable);
      help->addWidget(tools); help->addStretch();
      QObject::connect(
        tools, &QPushButton::clicked, window,
        [this] {navigation->setCurrentRow(5);});
      overview_map_tabs->addTab(unavailable, QStringLiteral("三维入口"));
    }
    auto * side = new QWidget(split); side->setMinimumWidth(220); side->setMaximumWidth(380);
    auto * side_layout = new QVBoxLayout(side); side_layout->setContentsMargins(8, 0, 0, 0);
    auto * instrument_box = new QGroupBox(QStringLiteral("姿态与航向 · 只读"), side);
    auto * instrument_layout = new QVBoxLayout(instrument_box);
    instruments = new FlightInstrumentCanvas(instrument_box);
    instruments->setMinimumHeight(160);
    instruments->setObjectName("overviewInstruments"); instrument_layout->addWidget(instruments);
    side_layout->addWidget(instrument_box);
    auto * box = new QGroupBox(QStringLiteral("选中相机 · 当前视角"), side);
    auto * camera_layout = new QVBoxLayout(box);
    overview_camera = new CameraCanvas(box); overview_camera->setObjectName("overviewCamera");
    overview_camera->setMinimumHeight(145); camera_layout->addWidget(overview_camera);
    side_layout->addWidget(box, 1);
    split->setStretchFactor(0, 1); split->setStretchFactor(1, 0); split->setSizes({740, 320});
    layout->addWidget(split, 1);

    auto * evidence_panel = new QWidget(pages);
    evidence_panel->setObjectName("overviewEvidencePanel");
    auto * evidence_layout = new QVBoxLayout(evidence_panel);
    evidence_layout->setContentsMargins(0, 0, 0, 0);
    auto * state = new QGroupBox(QStringLiteral("位置与观测证据"), evidence_panel);
    auto * form = new QFormLayout(state); form->setFieldGrowthPolicy(
      QFormLayout::AllNonFixedFieldsGrow);
    armed = label({}, "stateArmed", state); position = label({}, "statePosition", state);
    validity = label({}, "stateValidity", state); state_age = label({}, "stateAges", state);
    form->addRow(QStringLiteral("解锁状态"), armed);
    form->addRow(QStringLiteral("X / Y / Z（odom，m）"), position);
    form->addRow(QStringLiteral("独立有效性"), validity);
    form->addRow(QStringLiteral("时间证据"), state_age);
    evidence_layout->addWidget(state);
    auto * phase = new QGroupBox(QStringLiteral("全局阶段观察（不关联目录任务）"), evidence_panel);
    auto * p = new QVBoxLayout(phase);
    global_overview = label({}, "globalPhaseOverview", phase); p->addWidget(global_overview);
    evidence_layout->addWidget(phase);
    evidence_layout->addWidget(
      label(
        QStringLiteral(
          "航向是局部 ENU 方位，不是真北或磁北；局部 Z 不是离地高度。"
          "地图、姿态仪与相机分别判断有效性；缺少观测不填充演示数据。"), "overviewHint",
        evidence_panel));
    auto * actions = new QHBoxLayout;
    auto * expand = button(
      QStringLiteral("展开观测证据 / 全局阶段"), "overviewEvidenceToggle", pages);
    expand->setCheckable(true);
    QObject::connect(
      expand, &QPushButton::toggled, window, [evidence_panel, expand](bool shown) {
        evidence_panel->setVisible(shown);
        expand->setText(
          shown ? QStringLiteral("收起观测证据 / 全局阶段") :
          QStringLiteral("展开观测证据 / 全局阶段"));
      });
    actions->addWidget(expand);
    auto * events = button(QStringLiteral("会话事件"), "overviewEvents", pages);
    QObject::connect(
      events, &QPushButton::clicked, window, [this] {
        auto * dock = window->findChild<QDockWidget *>("eventDock");
        if (dock) {dock->setVisible(!dock->isVisible());}
      });
    actions->addWidget(events); actions->addStretch(); layout->addLayout(actions);
    layout->addWidget(evidence_panel); evidence_panel->hide();
  }
  void mount_rviz()
  {
    if (!rviz_view) {return;}
    const bool in_overview = navigation->currentRow() == 0 &&
      overview_map_tabs->currentIndex() == 1;
    // QTabWidget does not propagate the mounted page's minimum width. Keep
    // both RViz splitter children inside the tab rather than overlapping them
    // when the overview's instrument column reserves the remaining width.
    overview_map_tabs->setMinimumWidth(
      in_overview ? std::max(320, rviz_view->minimumSizeHint().width() + 4) : 320);
    auto * host = in_overview ? overview_rviz_host : rviz_full_page;
    if (rviz_view->parentWidget() == host) {return;}
    // One surface only. Moving it releases the old renderer; never auto-reopen it.
    rviz_view->stop();
    rviz_full_layout->removeWidget(rviz_view); overview_rviz_layout->removeWidget(rviz_view);
    rviz_view->setParent(host);
    (in_overview ? overview_rviz_layout : rviz_full_layout)->addWidget(rviz_view);
    rviz_view->show();
    rviz_view->set_connection(
      source && current && current->running && !current->stopped &&
      !closing && !disconnecting && !released, config.use_sim_time);
  }
  void mission_page()
  {
    auto * layout =
      page(
      mission_options.enabled ? QStringLiteral("目录审阅 / 受管仿真任务") :
      QStringLiteral("目录任务 / 已知执行查询"), mission_options.enabled ?
      QStringLiteral(
        "查询仍只读；启动/取消须单独确认，仅使用本地登记身份。"
        "手输 ID 始终只用于查询。") :
      QStringLiteral(

        "仅 list、validate、details、query"
        "。服务 execution_id 不等"
        "于 Action UUID；手输 ID"
        "始终只用于查询。"));
    auto * refresh = button(QStringLiteral("刷新目录（只读）"), "listMissions", pages);
    layout->addWidget(refresh);
    QObject::connect(
      refresh, &QPushButton::clicked, window, [this] {
        if (available()) {record(bridge::Key::MissionList, source->list_missions());}
      });
    catalog_quality = label({}, "catalogQuality", pages); layout->addWidget(catalog_quality);
    mission_list = new QListWidget(pages); mission_list->setObjectName("missionList");
    mission_list->setMinimumHeight(70); mission_list->setMaximumHeight(100); layout->addWidget(
      mission_list);
    validate = button(QStringLiteral("校验所选目录任务（静态检查）"), "validateMission", pages);
    validate->setEnabled(false);
    auto * preview_actions = new QHBoxLayout(); preview_actions->addWidget(validate);
    layout->addLayout(preview_actions);
    QObject::connect(
      mission_list, &QListWidget::currentRowChanged, window, [this] {
        const auto name = mission_list->currentItem() ?
        mission_list->currentItem()->text().toStdString() : std::string{};
        if (name != details_name) {details_name.clear(); clear_details();}
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
    inspect = button(QStringLiteral("读取所选任务详情（只读）"), "inspectMission", pages);
    inspect->setEnabled(false); preview_actions->addWidget(inspect);
    QObject::connect(
      inspect, &QPushButton::clicked, window, [this] {
        if (!available() || !mission_list->currentItem()) {return;}
        const auto name = mission_list->currentItem()->text().toStdString();
        const auto receipt = source->mission_details(name);
        if (receipt.status == bridge::RequestStatus::Queued) {
          details_name = name; clear_details();
        }
        record(bridge::Key::MissionDetails, receipt);
      });
    details_summary = label(QStringLiteral("尚未读取任务详情"), "missionDetailsSummary", pages);
    layout->addWidget(details_summary);
    steps_model = new TextTable(
      {QStringLiteral("顺序"), QStringLiteral("步骤"),
        QStringLiteral("有效参数（含校验器默认值）"), QStringLiteral("超时 s")}, pages);
    document_tabs = new QTabWidget(pages); document_tabs->setObjectName("missionDocumentTabs");
    auto * steps_table = table(steps_model, "missionSteps", document_tabs);
    steps_table->horizontalHeader()->setStretchLastSection(false);
    steps_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    steps_table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    steps_table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
    steps_table->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
    steps_table->setWordWrap(true); steps_table->setTextElideMode(Qt::ElideNone);
    steps_table->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    steps_table->setMinimumHeight(150);
    document_tabs->addTab(steps_table, QStringLiteral("步骤与参数"));
    mission_content = new QPlainTextEdit(pages); mission_content->setObjectName("missionContent");
    mission_content->setReadOnly(true); mission_content->setMinimumHeight(160);
    document_tabs->addTab(mission_content, QStringLiteral("原始正文（只读）"));
    layout->addWidget(document_tabs);
    layout->addWidget(
      label(
        QStringLiteral(
          "详情是读取时的已校验副本，不保证稍后目录内容未变；SHA-256不是 execution_id、"
          "Action UUID 或飞行许可。任务文件没有 profile 字段，UI 选择不等于后端配置。"
          "受管操作须启动参数显式启用；没有任意执行接管或完整历史列表。") +
        (mission_options.enabled ? QString{} : QStringLiteral("默认没有启动/取消入口。")),
        "missionLimit", pages));
    if (mission_options.enabled) {
      mission_panel = new MissionPanel(
        mission_options, controls_factory, pages,
        [this](bridge::Key key, const std::string & name) {return review_read(key, name);});
      mission_panel->setObjectName("managedMissionPanel"); layout->addWidget(mission_panel);
    }
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
    auto * page = new QWidget(pages); page->setObjectName("cameraPerceptionPage");
    auto * row = new QHBoxLayout(page); row->setContentsMargins(0, 0, 0, 0);
    auto * splitter = new QSplitter(Qt::Horizontal, page);
    splitter->setObjectName("cameraPerceptionSplitter"); splitter->setChildrenCollapsible(false);
    auto * scroll = new QScrollArea(splitter); scroll->setObjectName("cameraControlsScroll");
    scroll->setWidgetResizable(true); scroll->setFrameShape(QFrame::NoFrame);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setMinimumWidth(260); scroll->setMaximumWidth(560);
    auto * controls = new QWidget(scroll); controls->setObjectName("cameraControls");
    controls->setAttribute(Qt::WA_StyledBackground, true);
    auto * layout = new QVBoxLayout(controls);
    layout->setContentsMargins(16, 12, 16, 16); layout->setSpacing(12);
    auto * heading = label(QStringLiteral("相机与实测感知"), "cameraHeading", controls);
    heading->setProperty("heading", true); layout->addWidget(heading);
    layout->addWidget(
      label(
        QStringLiteral(
          "前视 / 下视独立选择，不自动替代。感知模式只显示匹配同帧；"
          "总览与录像仍用实时原图。"), "cameraSubtitle", controls));
    camera = new QComboBox(controls); camera->setObjectName("cameraSelector");
    camera->addItems({QStringLiteral("前视相机"), QStringLiteral("下视相机（可选）")});
    layout->addWidget(camera);
    overlay_mode = new QComboBox(controls); overlay_mode->setObjectName("cameraOverlayMode");
    overlay_mode->addItems(
      {QStringLiteral("实时原图（默认）"),
        QStringLiteral("同帧检测框 · 无身份保证"),
        QStringLiteral("同帧跟踪框 · 原始 SORT ID")});
    layout->addWidget(overlay_mode);
    overlay_evidence = label({}, "cameraOverlayEvidence", controls);
    layout->addWidget(overlay_evidence);
    overlay_model = new TextTable(
      {"ID", QStringLiteral("类别"), QStringLiteral("置信度"),
        QStringLiteral("光轴深度 m"), QStringLiteral("深度关联")}, controls);
    layout->addWidget(table(overlay_model, "cameraOverlayTable", controls));
    QObject::connect(
      overlay_mode, qOverload<int>(&QComboBox::currentIndexChanged), window, [this] {
        overlay_submitted_stamp = -1;
        overlay_observations.reset();
        if (overlay_decoder) {overlay_decoder->clear();}
        camera_canvas->frame({}, QStringLiteral("显示模式已变更，等待同帧观测"));
        overlay_model->replace({});
      });
    QObject::connect(
      camera, qOverload<int>(&QComboBox::currentIndexChanged), window, [this] {
        submitted_stamp = -1;
        overlay_submitted_stamp = -1;
        overlay_observations.reset();
        if (overlay_decoder) {overlay_decoder->clear();}
        if (decoder) {decoder->clear();} clear_images(
          QStringLiteral(
            "选源已变更，等待该源的新"
            "鲜图像"));
      });
    camera_canvas = new CameraCanvas(splitter); camera_canvas->setObjectName("cameraCanvas");
    camera_canvas->setMinimumWidth(320);
    image_evidence = label({}, "imageEvidence", controls); layout->addWidget(image_evidence);
    target_summary = label({}, "targetSummary", controls); obstacle_summary = label(
      {},
      "obstacleSummary",
      controls);
    target_model = new TextTable(
      {"ID", QStringLiteral("类别"), QStringLiteral(
          "置信度"), "X (m)", "Y (m)", "Z (m)", QStringLiteral("上游光轴深度 m")}, controls);
    obstacle_model = new TextTable(
      {"ID", QStringLiteral(
          "类型"), "X (m)", "Y (m)", "Z (m)", QStringLiteral("尺寸 X/Y/Z m"), QStringLiteral(
          "动态")}, controls);
    layout->addWidget(target_summary);
    layout->addWidget(table(target_model, "targetTable", controls));
    layout->addWidget(obstacle_summary);
    layout->addWidget(table(obstacle_model, "obstacleTable", controls));
    layout->addWidget(
      label(
        QStringLiteral(
          "蓝框=检测；琥珀框=上游跟踪，可能包含预测框；SORT编号不是真实身份，"
          "不与稳定ID或其他源互换。深度为上游融合光轴深度，不是斜距；"
          "需要同帧、唯一ID/类别和像素中心匹配，无效不填零。\n"
          "上游未报告实际深度帧时间/TF回退；这里只核对事件关联，不证明RGB-D同步或定位精度。\n"
          "标签/降落专用、预测与语义"
          "类别图层未接入；实测目标"
          "不能冒充这些来源。"), "perceptionLimit",
        controls));
    layout->addStretch(); scroll->setWidget(controls);
    splitter->addWidget(scroll); splitter->addWidget(camera_canvas);
    splitter->setStretchFactor(0, 0); splitter->setStretchFactor(1, 1);
    splitter->setSizes({300, 900}); row->addWidget(splitter); pages->addWidget(page);
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
    auto * local_edits = new QGroupBox(QStringLiteral("受限平台元数据编辑（独立显式启用）"), pages);
    auto * edit_layout = new QVBoxLayout(local_edits);
    parameter_panel = new ParameterPanel(parameter_options, parameter_factory, local_edits);
    parameter_panel->setObjectName("localParameterPanel"); edit_layout->addWidget(parameter_panel);
    layout->addWidget(local_edits);
    diagnostics = new QPlainTextEdit(pages); diagnostics->setObjectName("diagnosticText");
    diagnostics->setReadOnly(true);
    diagnostics->setMaximumBlockCount(600); diagnostics->setMinimumHeight(180); layout->addWidget(
      diagnostics);
  }
  void tools_page()
  {
    auto * layout = page(
      QStringLiteral("专业工具"), QStringLiteral(
        "可选独立窗口；不会自动启动。进程运行不是已连接、可飞或控制权确认。"));
    layout->addWidget(
      label(
        QStringLiteral(
          "RViz2：默认基础飞行预设以 map 显示 MAVROS 位姿坐标轴；导航/感知预设使用 odom，"
          "需对应发布节点和真实 TF。继承本进程 ROS domain；启动时绑定当前观测"
          "配置，并允许 DDS 访问 /dev/shm；这是可信 ROS 查看工具，不是强安全沙箱。"
          "所有 RViz 图层是最后收到的内容，不承诺新鲜度；模拟时钟"
          "暂停时标记寿命也可能暂停。预测/语义副本尚无显示适配，不混入实测图层。"
          "切换 UI 配置后请手动停止并重启 RViz。\n\nQGC：仅离线隔离预览，独立临时配置，"
          "无主机网络/设备访问；缺少隔离条件就拒绝启动，无联网回退。QGC 本身不是只读"
          "软件；本入口不管理用户从其他方式启动的 QGC，也不代表联网控制已验收。"),
        "toolsBoundary", pages));
    auto * form = new QFormLayout();
    rviz_path = new QLineEdit(tool_options.rviz_path, pages); rviz_path->setObjectName("rvizPath");
    qgc_path = new QLineEdit(tool_options.qgc_path, pages); qgc_path->setObjectName("qgcPath");
    form->addRow(QStringLiteral("RViz2 绝对路径"), rviz_path);
    auto * qgc_row = new QHBoxLayout(); qgc_row->addWidget(qgc_path);
    auto * choose = button(QStringLiteral("选择 AppImage"), "selectQgc", pages); qgc_row->addWidget(
      choose);
    form->addRow(QStringLiteral("QGC AppImage"), qgc_row);
    QObject::connect(
      choose, &QPushButton::clicked, window, [this] {
        if (qgc_process.active() || closing) {return;}
        const auto path = QFileDialog::getOpenFileName(
          window, QStringLiteral("选择可信来源的 QGC AppImage"), qgc_path->text(),
          "AppImage (*.AppImage);;All files (*)");
        if (!path.isEmpty()) {qgc_path->setText(path);}
      });
    rviz_preset = new QComboBox(pages); rviz_preset->setObjectName("rvizPreset");
    rviz_preset->addItems(
      {QStringLiteral("导航/SLAM 查看（需相关节点）"), QStringLiteral("感知查看"),
        QStringLiteral("精降相机查看"), QStringLiteral("基础飞行（MAVROS / map）")});
    rviz_preset->setCurrentIndex(3);
    form->addRow(QStringLiteral("RViz 查看预设"), rviz_preset); layout->addLayout(form);
    auto * row = new QHBoxLayout();
    rviz_start = button(QStringLiteral("打开 RViz 查看"), "startRviz", pages);
    rviz_stop = button(QStringLiteral("停止本次 RViz"), "stopRviz", pages);
    qgc_start = button(QStringLiteral("打开 QGC 离线预览"), "startQgc", pages);
    qgc_stop = button(QStringLiteral("停止本次 QGC"), "stopQgc", pages);
    for (auto * item : {rviz_start, rviz_stop, qgc_start, qgc_stop}) {row->addWidget(item);}
    layout->addLayout(row);
    rviz_state = label("", "rvizState", pages); qgc_state = label("", "qgcState", pages);
    layout->addWidget(rviz_state); layout->addWidget(qgc_state);
    tool_log = new QPlainTextEdit(pages); tool_log->setObjectName("toolLog");
    tool_log->setReadOnly(true); tool_log->setMaximumBlockCount(400);
    tool_log->setMinimumHeight(180);
    layout->addWidget(tool_log);
    QObject::connect(
      rviz_start, &QPushButton::clicked, window, [this] {
        if (closing || disconnecting) {return;}
        tool_options.rviz_path = rviz_path->text();
        rviz_process.start(ToolKind::Rviz, tool_options, config, rviz_preset->currentIndex());
        tools_tick();
      });
    QObject::connect(
      qgc_start, &QPushButton::clicked, window, [this] {
        if (closing) {return;}
        tool_options.qgc_path = qgc_path->text();
        qgc_process.start(ToolKind::Qgc, tool_options, config); tools_tick();
      });
    QObject::connect(rviz_stop, &QPushButton::clicked, window, [this] {rviz_process.stop();});
    QObject::connect(qgc_stop, &QPushButton::clicked, window, [this] {qgc_process.stop();});
    layout->addStretch();
  }
  bool tools_active() const {return rviz_process.active() || qgc_process.active();}
  void tools_tick()
  {
    rviz_path->setEnabled(!rviz_process.active() && !closing);
    qgc_path->setEnabled(!qgc_process.active() && !closing);
    rviz_preset->setEnabled(!rviz_process.active() && !closing);
    rviz_start->setEnabled(!rviz_process.active() && !closing && !disconnecting);
    qgc_start->setEnabled(!qgc_process.active() && !closing);
    rviz_stop->setEnabled(rviz_process.active() && !closing);
    qgc_stop->setEnabled(qgc_process.active() && !closing);
    rviz_state->setText("RViz: " + rviz_process.status());
    qgc_state->setText("QGC: " + qgc_process.status());
    const auto logs = QString("RViz (last 32K characters)\n") + rviz_process.log() +
      "\nQGC offline (last 32K characters)\n" + qgc_process.log();
    if (tool_log->toPlainText() != logs) {tool_log->setPlainText(logs);}
    if (closing && released && !tools_active() && (!mission_panel || mission_panel->stopped()) &&
      editor->stopped() && parameter_panel->stopped() && import_panel->stopped() &&
      (!recording_panel || recording_panel->stopped()))
    {
      window->close();
    }
  }
  void request_close()
  {
    if (closing || exit_prompt || mission_exit_prompt || draft_exit_prompt ||
      parameter_exit_prompt || import_exit_prompt)
    {
      return;
    }
    if (import_panel->busy()) {
      window->statusBar()->showMessage(
        QStringLiteral(
          "请先关闭导入确认框或等待单次提交收尾；不自动撤销。"), 5000);
      return;
    }
    if (parameter_panel->busy()) {
      window->statusBar()->showMessage(
        QStringLiteral(
          "请先关闭参数确认框或等待本次参数操作收尾；不自动撤销写入。"), 5000);
      return;
    }
    if (editor->busy()) {
      window->statusBar()->showMessage(
        QStringLiteral("请先完成或关闭本地草稿操作，再退出窗口。"), 5000); return;
    }
    parameter_panel->exit_question(true);
    import_panel->exit_question(true);
    if (editor->dirty() && !draft_exit_confirmed) {
      draft_exit_prompt = new QMessageBox(
        QMessageBox::Warning, QStringLiteral("本地草稿尚未导出"),
        QStringLiteral("退出会丢弃未导出的本地改动；不改原任务文件，不取消远端任务。是否仍退出？"),
        QMessageBox::Yes | QMessageBox::Cancel, window);
      auto * p = draft_exit_prompt; p->setObjectName("draftExitPrompt"); p->setTextFormat(
        Qt::PlainText);
      p->setDefaultButton(QMessageBox::Cancel); p->setEscapeButton(QMessageBox::Cancel);
      p->setButtonText(QMessageBox::Yes, QStringLiteral("丢弃本地改动并退出"));
      p->setButtonText(QMessageBox::Cancel, QStringLiteral("保留窗口"));
      QObject::connect(
        p, &QMessageBox::finished, window, [this, p](int answer) {
          if (draft_exit_prompt != p) {return;}
          draft_exit_prompt = nullptr; p->setObjectName(""); p->deleteLater();
          if (answer == QMessageBox::Yes) {draft_exit_confirmed = true; request_close();} else {
            cancel_close();
          }
        }); p->open(); return;
    }
    if (import_panel->unresolved() && !import_exit_confirmed) {
      import_exit_prompt = new QMessageBox(
        QMessageBox::Warning, QStringLiteral("任务导入结果未确认"),
        QStringLiteral(
          "文件可能已导入，但本入口不执行任务。关闭只停止本地通信，不删除/回滚文件。\n"
          "未决记录保留于：%1\n重启只查询原身份；实例变化仍保锁。是否知悉并退出？")
        .arg(
          QString::fromStdString(
            import_options.journal_directory)), QMessageBox::Yes | QMessageBox::Cancel, window);
      auto * p = import_exit_prompt; p->setObjectName("importExitPrompt"); p->setTextFormat(
        Qt::PlainText);
      p->setDefaultButton(QMessageBox::Cancel); p->setEscapeButton(QMessageBox::Cancel);
      QObject::connect(
        p, &QMessageBox::finished, window, [this, p](int answer) {
          if (import_exit_prompt != p) {return;} import_exit_prompt = nullptr;
          p->setObjectName(""); p->deleteLater();
          if (answer == QMessageBox::Yes) {import_exit_confirmed = true; request_close();} else {
            cancel_close();
          }
        }); p->open(); return;
    }
    if (parameter_panel->fault() && !parameter_exit_confirmed) {
      parameter_exit_prompt = new QMessageBox(
        QMessageBox::Warning, QStringLiteral("参数写入结果未确认"),
        QStringLiteral(
          "仅两项元数据，不影响飞行。关闭只停止本地通信，不证明远端未写入。\n"
          "参数故障锁不跨重启恢复，旧写入不重放；请先人工核对后端。是否仍退出？"),
        QMessageBox::Yes | QMessageBox::Cancel, window);
      auto * p = parameter_exit_prompt; p->setObjectName("parameterExitPrompt"); p->setTextFormat(
        Qt::PlainText);
      p->setDefaultButton(QMessageBox::Cancel); p->setEscapeButton(QMessageBox::Cancel);
      p->setButtonText(QMessageBox::Yes, QStringLiteral("知悉结果未确认并退出"));
      QObject::connect(
        p, &QMessageBox::finished, window, [this, p](int answer) {
          if (parameter_exit_prompt != p) {return;} parameter_exit_prompt = nullptr;
          p->setObjectName(""); p->deleteLater();
          if (answer == QMessageBox::Yes) {parameter_exit_confirmed = true; request_close();} else {
            cancel_close();
          }
        }); p->open(); return;
    }
    if (mission_panel) {mission_panel->exit_question(true);}
    if (mission_panel && mission_panel->unresolved() && !mission_exit_confirmed) {
      mission_exit_prompt = new QMessageBox(
        QMessageBox::Warning, QStringLiteral("控制/未决任务状态尚未完全确认"),
        QStringLiteral(
          "关闭只停止本地通信，不取消远端任务；任务可能继续执行。\n"
          "未决记录保留在：%1\n重启只查询原身份，不重发、不清锁。是否仍关闭？")
        .arg(text(mission_options.journal_directory, 4096)),
        QMessageBox::Yes | QMessageBox::Cancel, window);
      mission_exit_prompt->setObjectName("missionExitPrompt");
      mission_exit_prompt->setTextFormat(Qt::PlainText);
      mission_exit_prompt->setDefaultButton(QMessageBox::Cancel);
      mission_exit_prompt->setEscapeButton(QMessageBox::Cancel);
      mission_exit_prompt->setButtonText(QMessageBox::Yes, QStringLiteral("保留记录并退出"));
      mission_exit_prompt->setButtonText(QMessageBox::Cancel, QStringLiteral("保持窗口"));
      QObject::connect(
        mission_exit_prompt, &QMessageBox::finished, window, [this](int answer) {
          mission_exit_prompt->setObjectName(""); mission_exit_prompt->deleteLater();
          mission_exit_prompt = nullptr;
          if (answer == QMessageBox::Yes) {mission_exit_confirmed = true; request_close();} else {
            cancel_close();
          }
        });
      mission_exit_prompt->open(); return;
    }
    if (!tools_active()) {stop_source(true); return;}
    exit_prompt = new QMessageBox(
      QMessageBox::Question, QStringLiteral("本次工具仍在运行"),
      QStringLiteral(
        "是否停止本次启动的 RViz/QGC 后关闭 EasyFly？\n"
        "只停止本次受管进程，不关闭用户原有工具或平台节点。QGC 是离线预览。"),
      QMessageBox::Yes | QMessageBox::Cancel, window);
    exit_prompt->setObjectName("toolExitPrompt"); exit_prompt->setTextFormat(Qt::PlainText);
    exit_prompt->setDefaultButton(QMessageBox::Cancel);
    exit_prompt->setButtonText(QMessageBox::Yes, QStringLiteral("停止本次工具并关闭"));
    exit_prompt->setButtonText(QMessageBox::Cancel, QStringLiteral("保持窗口和工具"));
    QObject::connect(
      exit_prompt, &QMessageBox::finished, window, [this](int answer) {
        exit_prompt->setObjectName("");
        exit_prompt->deleteLater(); exit_prompt = nullptr;
        if (answer == QMessageBox::Yes) {
          rviz_process.stop(); qgc_process.stop(); stop_source(true);
        } else {
          cancel_close();
        }
      });
    exit_prompt->open();
  }
  void cancel_close()
  {
    draft_exit_confirmed = mission_exit_confirmed = parameter_exit_confirmed = false;
    import_exit_confirmed = false; import_panel->exit_question(false);
    if (mission_panel) {mission_panel->exit_question(false);}
    parameter_panel->exit_question(false);
  }
  bool available() const
  {
    return source && current && current->running && !current->stopped && !closing && !disconnecting;
  }
  void update_buttons()
  {
    import_panel->observe(editor->exported_draft(), available());
    parameter_panel->observer_available(available());
    if (mission_panel) {
      const auto name = mission_list->currentItem() ?
        mission_list->currentItem()->text().toStdString() : std::string{};
      mission_panel->observe(
        current, name, !current || details_name != name ||
        pending(bridge::Key::MissionDetails, current->mission_details.evidence.request_id),
        available(), config.use_sim_time);
    }
    if (inspect) {inspect->setEnabled(available() && mission_list->currentItem());}
    validate->setEnabled(available() && mission_list->currentItem());
    query->setEnabled(
      available() && bridge::detail::execution_id(
        execution_id->text().toStdString()));
  }
  void record(bridge::Key key, bridge::RequestReceipt receipt)
  {
    if (receipt.status == bridge::RequestStatus::Queued) {
      expected[key] = receipt.id;
      update_buttons();
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
  bridge::RequestReceipt review_read(bridge::Key key, const std::string & name)
  {
    if (!available()) {return {0, bridge::RequestStatus::Closed};}
    bridge::RequestReceipt receipt;
    switch (key) {
      case bridge::Key::Profile:
        receipt = source->read_parameter(bridge::Parameter::Profile); break;
      case bridge::Key::UseSimTime:
        receipt = source->read_parameter(bridge::Parameter::UseSimTime); break;
      case bridge::Key::FlightCapabilities:
        receipt = source->capabilities(bridge::CapabilitySource::Flight); break;
      case bridge::Key::MissionCapabilities:
        receipt = source->capabilities(bridge::CapabilitySource::Mission); break;
      case bridge::Key::MissionDetails:
        receipt = source->mission_details(name);
        if (receipt.status == bridge::RequestStatus::Queued) {
          details_name = name; clear_details();
        }
        break;
      default: return {0, bridge::RequestStatus::Invalid};
    }
    record(key, receipt); return receipt;
  }
  bool pending(bridge::Key key, uint64_t id) const
  {
    const auto found = expected.find(key); return found != expected.end() && found->second != id;
  }
  void clear_images(const QString & message)
  {
    recording_frame.reset();
    overlay_observations.reset(); overlay_submitted_stamp = -1;
    overview_camera->frame({}, message); camera_canvas->frame({}, message);
    overview_camera->setProperty("display_stamp_ns", QVariant{});
    camera_canvas->setProperty("display_stamp_ns", QVariant{});
    if (overlay_model) {overlay_model->replace({});}
    if (overlay_evidence) {overlay_evidence->setText(message);}
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
      queried_id.clear(); validated_name.clear(); details_name.clear(); clear_details();
      mission_list->clear();
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
    if (rviz_view) {rviz_view->stop();}
    if (closing) {editor->stop(); parameter_panel->stop(); import_panel->stop();}
    if (closing && mission_panel) {mission_panel->stop();}
    if (recording_panel) {
      if (closing) {recording_panel->stop();} else {
        recording_panel->finish("observation_disconnected");
      }
    }
    if (source) {source->stop();}
    if (decoder) {decoder->stop();}
    if (overlay_decoder) {overlay_decoder->stop();}
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
    if (overlay_decoder && !overlay_decoder->stats().stopped) {return;}
    if (closing && mission_panel && !mission_panel->stopped()) {return;}
    if (closing && !parameter_panel->stopped()) {return;}
    if (closing && !import_panel->stopped()) {return;}
    if (closing && !editor->stopped()) {return;}
    if (closing && recording_panel && !recording_panel->stopped()) {return;}
    history = current; source.reset(); decoder.reset(); overlay_decoder.reset();
    overlay_submitted_stamp = -1; released = true; disconnecting = false;
    configuration->setEnabled(!closing && (!mission_panel || !mission_panel->unresolved()));
    toggle->setEnabled(!closing); toggle->setText(
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
      live ? (v.sdk.value.connected ? QStringLiteral("已连接") :
      QStringLiteral("未连接")) : unknown);
    state_connection->setToolTip(quality_text(q));
    armed->setText(live ? yes(v.sdk.value.armed) : unknown);
    mode->setText(live ? text(v.sdk.value.flight_mode_name, 256) : unknown);
    position->setText(
      live &&
      v.position_valid ? QStringLiteral("%1 / %2 / %3 m").arg(
        number(v.sdk.value.position_x),
        number(v.sdk.value.position_y),
        number(v.sdk.value.position_z)) : QStringLiteral("—（定位不可用）"));
    const auto flight = flight_instruments(s, now);
    altitude_metric->setText(
      flight.position_valid ? number(v.sdk.value.position_z) : QStringLiteral(
        "—"));
    altitude_metric->setToolTip(
      QStringLiteral("局部原点以上 Z，不是离地/海拔高度；定位不可用时不显示数值。"));
    instruments->snapshot(s, now);
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
    details_view(now);
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
  void clear_details()
  {
    rendered_details = rendered_details_session = 0;
    if (steps_model) {steps_model->replace({});}
    if (mission_content) {mission_content->clear();}
    if (details_summary) {details_summary->setText(QStringLiteral("尚无与所选任务匹配的详情"));}
  }
  void details_view(double now)
  {
    if (details_name.empty()) {return;}
    const auto & observation = current->mission_details;
    if (pending(bridge::Key::MissionDetails, observation.evidence.request_id)) {
      details_summary->setText(QStringLiteral("等待任务详情：") + text(details_name, 128)); return;
    }
    const auto quality = display_quality(observation.evidence, *current, now, 2, false);
    const auto & response = observation.value;
    if (!response || !response->success || !current->mission_plan ||
      response->mission_name != details_name)
    {
      clear_details();
      details_summary->setText(
        QStringLiteral("任务详情不可用：%1 · %2\n%3").arg(
          text(details_name, 128), quality_text(quality),
          response ? text(response->message, 2048) : text(observation.evidence.reason, 2048)));
      return;
    }
    const auto & format = current->mission_format;
    details_summary->setText(
      QStringLiteral(
        "读取文件：%1 · %2 · %3\nSHA-256：%4\n"
        "%5；严格顺序、失败终止；只在读取时静态校验。\n"
        "UI profile 选择：%6（非任务文件字段，也非后端确认）。")
      .arg(
        text(details_name, 128), text(format), quality_text(quality),
        text(response->content_sha256, 64),
        format == "yaml_v1" ? QStringLiteral("schema v1；文件声明 odom / m / rad / s") :
        QStringLiteral("legacy flat；odom / m / rad / s 为格式执行约定，非文件显式声明"),
        text(config.selected_profile, 256)) + "\n" + ages(observation.evidence, *current, now) +
      QStringLiteral(" · 观测 session=%1/request=%2/epoch=%3")
      .arg(current->observation_session_id).arg(observation.evidence.request_id)
      .arg(observation.evidence.time_epoch));
    if (rendered_details == observation.evidence.request_id &&
      rendered_details_session == current->observation_session_id) {return;}
    rendered_details = observation.evidence.request_id;
    rendered_details_session = current->observation_session_id;
    QVector<QStringList> rows;
    for (const auto & step : current->mission_plan->steps) {
      rows.push_back(
        {QString::number(rows.size() + 1), text(step.type, 32),
          mission_step_parameters(step), number(step.timeout, 6)});
    }
    steps_model->replace(std::move(rows));
    mission_content->setPlainText(text(response->content, 65536));
    const auto render_request = rendered_details, render_session = rendered_details_session;
    QTimer::singleShot(
      50, window, [this, render_request, render_session] {
        if (!available() || rendered_details != render_request ||
        rendered_details_session != render_session) {return;}
        auto * parent = document_tabs->parentWidget();
        while (parent) {
          if (auto * scroll = qobject_cast<QScrollArea *>(parent)) {
            const auto bottom = document_tabs->mapTo(
              scroll->widget(), QPoint(0, document_tabs->height())).y();
            scroll->verticalScrollBar()->setValue(
              std::max(
                scroll->verticalScrollBar()->value(),
                bottom - scroll->viewport()->height() + 10));
            break;
          }
          parent = parent->parentWidget();
        }
      });
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
    recording_frame.reset();
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
    overview_camera->frame(decoded->image, caption);
    overview_camera->setProperty(
      "display_stamp_ns",
      QVariant::fromValue<qlonglong>(decoded->stamp_ns));
    recording_frame = decoded;
    if (overlay_mode->currentIndex() == 0) {
      camera_canvas->frame(decoded->image, caption); camera_canvas->overlay({});
      camera_canvas->setProperty(
        "display_stamp_ns",
        QVariant::fromValue<qlonglong>(decoded->stamp_ns));
      overlay_model->replace({});
      overlay_evidence->setText(QStringLiteral("实时原图；同帧感知为独立可选显示，不影响录像"));
      return;
    }
    overlay_view(now, channel, binding);
  }
  void overlay_view(double now, int channel, const ImageBinding & binding)
  {
    const auto & s = *current; const auto & observations = s.camera_perception[channel];
    const bool tracking = overlay_mode->currentIndex() == 2;
    const auto & image = tracking ? observations.track_image : observations.detection_image;
    const auto & evidence =
      tracking ? observations.tracks.evidence : observations.detections.evidence;
    const auto topic =
      tracking ? observations.binding.tracks_topic : observations.binding.detections_topic;
    const auto q = display_quality(evidence, s, now, 1, true);
    const auto image_quality = display_quality(image.evidence, s, now, 1, true);
    const auto fail = [&](const QString & reason) {
        camera_canvas->frame({}, reason); overlay_model->replace({});
        camera_canvas->setProperty("display_stamp_ns", QVariant{});
        overlay_evidence->setText(reason);
      };
    if (observations.binding.image_topic.empty() ||
      observations.binding.image_topic != binding.topic)
    {
      fail(QStringLiteral("该相机没有明确感知绑定；不复用其他相机结果")); return;
    }
    if (!image.value || image_quality != bridge::Quality::ValidAtObservation ||
      q != bridge::Quality::ValidAtObservation)
    {
      if (overlay_decoder) {overlay_decoder->clear();} overlay_submitted_stamp = -1;
      const auto reason = QStringLiteral(" · 未取得新鲜同帧图像，未叠框");
      fail(text(topic, 256) + " · " + quality_text(q) + reason);
      return;
    }
    if (!overlay_decoder) {
      try {
        overlay_decoder = std::make_unique<ImagePipeline>();
      } catch (const std::exception & error) {
        fail(QStringLiteral("感知转换器未启动：") + text(error.what(), 256)); return;
      }
    }
    overlay_decoder->bind(binding);
    const auto stamp = bridge::detail::stamp_ns(image.value->header.stamp);
    const auto converted = overlay_decoder->latest();
    const bool ready = converted && converted->binding == binding &&
      converted->stamp_ns == overlay_submitted_stamp && converted->error.empty() &&
      overlay_observations.has_value();
    const auto overlay = ready ? camera_overlay(
      s, *converted, channel,
      tracking ? CameraOverlayMode::Tracks : CameraOverlayMode::Detections, now,
      &*overlay_observations) : CameraOverlay{};
    // Render only the metadata saved with the completed conversion. The newest
    // observation may advance while this worker is decoding an older image.
    if (stamp != overlay_submitted_stamp) {
      overlay_decoder->submit(image.value, image.evidence.first_source_steady);
      overlay_submitted_stamp = stamp; overlay_observations = observations;
    }
    if (!ready) {
      fail(QStringLiteral("等待同帧感知图像转换（不沿用旧框）")); return;
    }
    if (!overlay.matched) {fail(overlay.status); return;}
    const auto age = static_cast<double>(s.clock.now_ns - converted->stamp_ns) / 1e9;
    camera_canvas->frame(
      converted->image, QStringLiteral("同帧感知 · 非最新原图 · 源年龄 %1s · stamp=%2")
      .arg(number(age)).arg(converted->stamp_ns));
    camera_canvas->overlay(overlay);
    camera_canvas->setProperty(
      "display_stamp_ns",
      QVariant::fromValue<qlonglong>(converted->stamp_ns));
    overlay_evidence->setText(
      overlay.status + "\n" + text(topic, 256) + QStringLiteral(
        " · 所示帧源龄 %1s / 收龄 %2s").arg(number(age), number(now - converted->first_received)));
    QVector<QStringList> rows;
    for (const auto & box : overlay.boxes) {
      rows.push_back(
        {box.id < 0 ? QStringLiteral("—") : QStringLiteral("SORT #%1").arg(box.id),
          box.class_name, number(box.confidence, 3), box.axial_depth ? number(*box.axial_depth) :
          QStringLiteral("—"), box.depth_status});
    }
    overlay_model->replace(std::move(rows));
  }
  void diagnostics_view(double now)
  {
    const auto & s = *current; QVector<QStringList> capabilities; QStringList details;
    details <<
      (mission_options.enabled ?
    QStringLiteral("观测只读；受管仿真操作另行确认，不作为飞行许可。ROS_DOMAIN_ID=%1（非鉴权）") :
    QStringLiteral("只读 UI，不作为飞行许可。ROS_DOMAIN_ID=%1（仅启动环境，非鉴权）")).arg(
      qEnvironmentVariable(
        "ROS_DOMAIN_ID",
        "0"));
    details << QStringLiteral("会话 %1 / epoch %2").arg(s.observation_session_id).arg(s.time_epoch);
    details << QStringLiteral("/uav_state 发布源 %1，其中类型匹配 %2；多个来源不合并")
      .arg(s.state_publishers).arg(s.state_matching_publishers);
    if (qEnvironmentVariableIsSet("EASYFLY_CONNECTION_SESSION")) {
      configuration->setEnabled(false);
      details << QStringLiteral("受连接管理配置约束；修改请返回连接管理并重建工作会话。配置 ID：%1")
        .arg(qEnvironmentVariable("EASYFLY_CONNECTION_ID"));
    }
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
      (mission_options.enabled ?
      QStringLiteral("受管仿真模式 · %1 · 曾观测平台能力端点 %2/4（非当前可飞）") :
      import_options.enabled ? QStringLiteral(
        "飞行只读 / 任务导入启用 · %1 · 能力端点 %2/4（非当前可飞）") :
      parameter_options.enabled ? QStringLiteral(
        "飞行只读 / 元数据编辑启用 · %1 · 能力端点 %2/4（非当前可飞）") :
      QStringLiteral("只读 · %1 · 曾观测平台能力端点 %2/4（非当前可飞）"))
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
        "%6")
      .arg(s.pending_queries).arg(s.counters.timed_out).arg(
        s.counters.dropped_images +
        stats.dropped).arg(stats.pending).arg(s.counters.invalid_samples).arg(
        mission_options.enabled ? QStringLiteral("受管仿真入口已显式启用；状态见任务页") :
        QStringLiteral("不持有飞行操作")));
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
    tools_tick();
    if (source) {current = source->snapshot();}
    if (rviz_view) {
      rviz_view->set_connection(
        source && current && current->running && !current->stopped &&
        !closing && !disconnecting && !released, config.use_sim_time);
    }
    if (mission_panel) {
      const auto name = mission_list->currentItem() ?
        mission_list->currentItem()->text().toStdString() : std::string{};
      mission_panel->observe(
        current, name, !current || details_name != name ||
        pending(bridge::Key::MissionDetails, current->mission_details.evidence.request_id),
        available(), config.use_sim_time);
      if (released && !closing) {configuration->setEnabled(!mission_panel->unresolved());}
    }
    if (disconnecting) {cleanup_tick();}
    if (!current) {return;}
    const double now = bridge::detail::steady_seconds();
    if (source && current->stopped && !disconnecting && !released) {stop_source(false);}
    state_view(now); mission_view(now); geometry_view(bridge::Key::Targets, now);
    geometry_view(bridge::Key::Obstacles, now); image_view(now); diagnostics_view(now);
    events_view();
    if (recording_panel) {
      const int channel = camera->currentIndex();
      recording_panel->observe(
        {recording_frame, current,
          mission_panel ? mission_panel->record_state() : nullptr},
        {current->observation_session_id, current->time_epoch, channel,
          config.image_topics[channel]},
        available() && !closing && !disconnecting, config.use_sim_time, config.selected_profile);
    }
    map->snapshot(current, now); overview_map->snapshot(current, now); update_buttons();
  }
};

MainWindow::MainWindow(
  const bridge::BridgeConfig & config, SourceFactory factory, QWidget * parent,
  const ToolOptions & tools, const MissionOptions & missions, ControlFactory controls,
  const ParameterOptions & parameters, ParameterFactory parameter_controls,
  const ImportOptions & imports, ImportFactory import_controls, const RvizOptions & spatial,
  const RecordingOptions & recording)
: QMainWindow(parent), impl_(std::make_unique<Impl>(
      this, config, std::move(factory), tools, missions, std::move(controls), parameters,
      std::move(parameter_controls), imports, std::move(import_controls), spatial, recording)) {}
MainWindow::~MainWindow() = default;
bool MainWindow::cleanup_complete() const
{
  return impl_->released && !impl_->tools_active() &&
         (!impl_->mission_panel || impl_->mission_panel->stopped()) &&
         (!impl_->closing || (impl_->editor->stopped() && impl_->parameter_panel->stopped() &&
         impl_->import_panel->stopped() &&
         (!impl_->recording_panel || impl_->recording_panel->stopped())));
}
QJsonObject MainWindow::connection_status() const
{
  const auto & p = *impl_;
  const auto s = p.current;
  if (!s) {return {{"summary", QStringLiteral("等待本地观测快照")}, {"unresolved", true}};}
  const auto now = bridge::detail::steady_seconds();
  const auto q = display_quality(s->state.evidence, *s, now, 1.0, true);
  const bool fresh = q == bridge::Quality::ValidAtObservation;
  QString summary = QStringLiteral("等待飞机状态");
  if (!s->running || s->stopped) {
    summary = QStringLiteral("本地观测已停止");
  } else if (s->state_publishers > 1) {
    summary = QStringLiteral("来源冲突：多个 /uav_state 发布源");
  } else if (s->state_graph_observed > 0 && s->state_publishers == 0) {
    summary = QStringLiteral("DDS 运行，尚未发现 /uav_state 发布源");
  } else if (s->state_publishers > 0 && s->state_matching_publishers == 0) {
    summary = QStringLiteral("/uav_state 消息类型不匹配");
  } else if (s->clock.quality != bridge::Quality::ValidAtObservation) {
    summary = QStringLiteral("时间源不可用：") + quality_text(s->clock.quality);
  } else if (fresh) {
    summary = s->state.sdk.value.connected ? QStringLiteral("飞机状态有效 · 飞控已连接") :
      QStringLiteral("飞机状态有效 · 上游报告飞控未连接");
  } else if (s->state.sdk.received) {
    summary = QStringLiteral("飞机状态不可用：") + quality_text(q);
  }
  const bool unresolved = (p.mission_panel && p.mission_panel->unresolved()) ||
    p.import_panel->unresolved() || p.parameter_panel->busy() || p.parameter_panel->fault();
  return {{"summary", summary}, {"unresolved", unresolved}, {"closing", p.closing},
    {"draft_dirty", p.editor->dirty()},
    {"close_prompt", p.exit_prompt || p.draft_exit_prompt || p.mission_exit_prompt ||
      p.parameter_exit_prompt || p.import_exit_prompt},
    {"observer_running", s->running && !s->stopped},
    {"state_quality", quality_text(q)}, {"clock_quality", quality_text(s->clock.quality)},
    {"state_received", s->state.sdk.received}, {"state_fresh", fresh},
    {"flight_connected", fresh && s->state.sdk.value.connected},
    {"state_publishers", static_cast<int>(s->state_publishers)},
    {"matching_publishers", static_cast<int>(s->state_matching_publishers)},
    {"position_valid", fresh && s->state.position_valid},
    {"position_x", fresh && s->state.position_valid ?
      QJsonValue(s->state.sdk.value.position_x) : QJsonValue()},
    {"attitude_valid", fresh && s->state.attitude_valid},
    {"battery_valid", fresh && s->state.battery_valid},
    {"epoch", QString::number(s->time_epoch)},
    {"observation_session", QString::number(s->observation_session_id)},
    {"domain", qEnvironmentVariable("ROS_DOMAIN_ID", "0")},
    {"simulation_time", p.config.use_sim_time}};
}
void MainWindow::closeEvent(QCloseEvent * event)
{
  if (impl_->closing && cleanup_complete()) {event->accept(); return;}
  event->ignore(); impl_->request_close();
}
}  // namespace easyfly_desktop

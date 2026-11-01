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
#include <QDialog>
#include <QFormLayout>
#include <QFontMetrics>
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
#include <QDynamicPropertyChangeEvent>
#include <QMenuBar>
#include <QMenu>
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
#include <QTabBar>
#include <QToolButton>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <utility>
#include "easyfly_desktop/main_window.hpp"
#include "easyfly_desktop/special_observation_panel.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_packages_with_prefixes.hpp>
#include "easyfly_desktop/image_pipeline.hpp"
#include "easyfly_desktop/peripheral_panel.hpp"
#include "easyfly_desktop/mission_editor.hpp"
#include "easyfly_desktop/mission_draft.hpp"
#include "easyfly_desktop/task_step_info.hpp"
#include "easyfly_desktop/overview_task.hpp"
#include "easyfly_desktop/presentation.hpp"
#include "easyfly_ui_bridge/camera_observations.hpp"
#include "widgets.hpp"
#include "easyfly_desktop/geographic_view.hpp"
#include "theme.hpp"
#include "layout_state.hpp"
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
  std::shared_ptr<const ConvertedFrame> completed_overlay_frame;
  std::optional<bridge::CameraObservations> completed_overlay_observations;
  std::optional<bridge::CameraPerceptionBinding> overlay_origin;
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
  std::vector<std::string> storage_protected_paths;
  RecordingPanel * recording_panel{nullptr};
  std::shared_ptr<const ConvertedFrame> recording_frame;
  RvizView * rviz_view{nullptr};
  QWidget * rviz_full_page{nullptr}, *overview_rviz_host{nullptr};
  QVBoxLayout * rviz_full_layout{nullptr}, *overview_rviz_layout{nullptr};
  QTabWidget * overview_map_tabs{nullptr};
  QTabWidget * task_tabs{nullptr}, *space_tabs{nullptr}, *settings_tabs{nullptr};
  QComboBox * task_target{nullptr};
  QLabel * task_target_info{nullptr}, *task_document_source{nullptr};
  QStackedWidget * task_execution_stack{nullptr}, *task_import_stack{nullptr};
  QDialog * task_save_dialog{nullptr};
  QLabel * task_save_hint{nullptr};
  QLabel * task_progress{nullptr};
  QPushButton * review_saved{nullptr};
  QJsonObject simulation_saved, onboard_saved;
  QString saved_catalog_focus;
  bool refresh_saved_catalog{false}, read_saved_catalog{false};
  QVBoxLayout * task_detail_layout{nullptr};
  QJsonObject staged_mission;
  QWidget * execution_page{nullptr};
  QVBoxLayout * execution_layout{nullptr};
  QLabel * effective_configuration{nullptr};
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
  std::optional<std::pair<bridge::Key, uint64_t>> request_note_pending;
  uint64_t request_note_session{0}, request_note_epoch{0};
  std::string queried_id, validated_name;
  std::string details_name;
  uint64_t rendered_details{0}, rendered_details_session{0};
  QPushButton * inspect{nullptr};
  QPushButton * edit_catalog{nullptr};
  QLabel * details_summary{nullptr}, *task_dependencies{nullptr}, *read_summary{nullptr};
  QPlainTextEdit * mission_content{nullptr};
  QTabWidget * document_tabs{nullptr};
  TextTable * steps_model{nullptr};
  uint64_t catalog_revision{0};
  std::map<bridge::Key, uint64_t> geometry_revision;
  std::map<bridge::Key, bool> geometry_visible;
  double event_tail{-1}; std::size_t event_count{0}; uint64_t event_session{0};
  QStackedWidget * pages{nullptr}; QListWidget * navigation{nullptr};
  QWidget * connection_page{nullptr};
  QWidget * onboard_mission_page{nullptr};
  QObject * onboard_session{nullptr};
  int connection_page_index{-1};
  bool connection_lobby{false};
  MissionEditor * editor{nullptr};
  QMessageBox * draft_exit_prompt{nullptr};
  bool draft_exit_confirmed{false};
  QLabel * connection{nullptr}, *clock{nullptr}, *backend_profile{nullptr}, *warning{nullptr};
  QLabel * request_note{nullptr}, *state_connection{nullptr}, *armed{nullptr}, *mode{nullptr};
  QLabel * position{nullptr}, *speed{nullptr}, *battery{nullptr}, *validity{nullptr},
  *state_age{nullptr};
  QLabel * altitude_metric{nullptr};
  QLabel * overview_armed{nullptr}, *overview_vertical_speed{nullptr},
  *overview_total_speed{nullptr};
  QLabel * overview_context{nullptr}, *overview_task{nullptr}, *overview_alert{nullptr};
  QComboBox * overview_map_mode{nullptr}, *overview_camera_selector{nullptr};
  QCheckBox * overview_follow{nullptr};
  QSplitter * overview_split{nullptr};
  QWidget * overview_side{nullptr};
  FlightInstrumentOverlay * instrument_overlay{nullptr};
  QPushButton * overview_map_focus{nullptr}, *overview_camera_focus{nullptr};
  QList<int> overview_split_sizes; int overview_focus{0};
  bool syncing_geography{false}; double overview_gps_good_at{0};
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
  GeographicView * geography{nullptr}, *overview_geography{nullptr};
  TextTable * capability_model{nullptr}, *target_model{nullptr}, *obstacle_model{nullptr};
  TextTable * algorithm_model{nullptr};
  bool peripheral_commands{false};
  PeripheralPanel * peripherals{nullptr};
  SpecialObservationPanel * landing_observations{nullptr}, *prediction_observations{nullptr};
  TextTable * parameter_model{nullptr}, *event_model{nullptr};
  QPlainTextEdit * diagnostics{nullptr};
  Impl(
    MainWindow * owner, const bridge::BridgeConfig & options, SourceFactory source_factory,
    const ToolOptions & tools, const MissionOptions & missions, ControlFactory controls,
    const ParameterOptions & parameters, ParameterFactory parameter_controls,
    const ImportOptions & imports, ImportFactory import_controls, const RvizOptions & spatial,
    const RecordingOptions & recording, bool auto_observe, bool peripheral_enabled)
  : window(owner), config(options), factory(std::move(source_factory)), tool_options(tools),
    mission_options(missions), controls_factory(std::move(controls)), parameter_options(parameters),
    parameter_factory(std::move(parameter_controls)), import_options(imports),
    import_factory(std::move(import_controls)), spatial_options(spatial), recording_options(
      recording), peripheral_commands(peripheral_enabled)
  {
    build();
    if (auto_observe) {connect_source();} else {
      released = true;
      toggle->setEnabled(false);
      connection->setText(QStringLiteral("尚未连接平台"));
      clock->setText(QStringLiteral("请在连接管理页选择场景或连接方式"));
      backend_profile->clear();
    }
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
    layout->setContentsMargins(12, 8, 12, 10); layout->setSpacing(8);
    auto * heading = label(title, "pageHeading", content); heading->setProperty("heading", true);
    auto * heading_row = new QHBoxLayout;
    heading->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    heading_row->addWidget(heading);
    auto * subtitle_label = label(subtitle, "pageSubtitle", content);
    subtitle_label->setVisible(!subtitle.isEmpty()); heading_row->addWidget(subtitle_label, 1);
    layout->addLayout(heading_row);
    scroll->setWidget(content); pages->addWidget(scroll); return layout;
  }
  void build()
  {
    window->setWindowTitle(
      mission_options.enabled ? QStringLiteral("EasyFly · 受管仿真任务") :
      import_options.enabled ? QStringLiteral(
        "EasyFly · 飞行"
        "观测只读 / 任务导入启用") :
      parameter_options.enabled ? QStringLiteral(
        "EasyFly · 飞行观"
        "测只读 / 元数据编辑启用") :
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
    header->setContentsMargins(8, 4, 10, 4);
    auto * brand = new QLabel(top_bar); brand->setObjectName("brand");
    brand->setFixedSize(112, 32); brand->setAlignment(Qt::AlignCenter);
    const auto logo = brand_pixmap();
    brand->setPixmap(logo.scaled(104, 30, Qt::KeepAspectRatio, Qt::SmoothTransformation));
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
    status->addWidget(connection); status->addWidget(clock); backend_profile->hide();
    header->addLayout(status, 1);
    toggle = button(QStringLiteral("断开观测"), "observerToggle", top_bar);
    header->addWidget(toggle);
    QObject::connect(
      toggle, &QPushButton::clicked, window, [this] {
        if (disconnecting || closing) {return;}
        if (connection_lobby) {return;}
        if (connection_page) {
          window->setProperty("connectionReturnRequested", true);
          window->close(); return;
        }
        if (source && current && !current->stopped) {stop_source(false);} else {connect_source();}
      });
    layout->addWidget(top_bar);
    warning = label({}, "warningBanner", root);
    warning->hide(); layout->addWidget(warning);
    auto * workspace = new QHBoxLayout; workspace->setContentsMargins(0, 0, 0, 0);
    workspace->setSpacing(8);
    navigation = new QListWidget(root); navigation->setObjectName("navigation");
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
    navigation->setIconSize(QSize(16, 16));
    navigation->setFixedWidth(148);
    pages = new QStackedWidget(root); workspace->addWidget(navigation);
    workspace->addWidget(pages, 1); layout->addLayout(workspace, 1);
    QObject::connect(
      navigation, &QListWidget::currentRowChanged, pages,
      &QStackedWidget::setCurrentIndex);
    overview_page(); mission_page(); perception_page(); spatial_page(); diagnostics_page();
    tools_page();
    auto * draft_scroll = new QWidget(pages);
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
    auto * draft_page = draft_scroll;
    storage_protected_paths = protected_paths;
    auto * draft_layout = new QVBoxLayout(draft_page);
    editor = new MissionEditor(
      draft_page, std::move(protected_paths), [this] {
        draft_exit_confirmed = false;
        window->setProperty(
          "connectionHandoffBlocked",
          connection_lobby && editor && editor->dirty());
        if (task_target) {update_task_center();}
      });
    auto * import_box = new QGroupBox(QStringLiteral("保存任务到平台"), draft_page);
    auto * import_layout = new QVBoxLayout(import_box);
    import_panel = new ImportPanel(
      import_options, import_factory, import_box,
      [this] {return editor->validated_draft();});
    QObject::connect(
      import_panel, &ImportPanel::saved, window, [this](const QJsonObject & receipt) {
        simulation_saved = receipt;
        simulation_saved["observer_session"] = static_cast<double>(current ?
        current->observation_session_id : 0);
        simulation_saved["observer_epoch"] = static_cast<double>(current ? current->time_epoch : 0);
        refresh_saved_catalog = true;
        if (task_tabs && task_tabs->currentIndex() == 1) {
          saved_catalog_focus = receipt.value("name").toString();
        }
        update_task_center();
      });
    import_layout->addWidget(import_panel);
    draft_layout->setContentsMargins(0, 0, 0, 0); draft_layout->addWidget(editor, 1);
    auto * save_row = new QHBoxLayout;
    task_save_hint = label({}, "taskSaveHint", draft_page); save_row->addWidget(task_save_hint, 1);
    auto * save_platform = button(QStringLiteral("保存到平台…"), "saveTaskToPlatform", draft_page);
    save_row->addWidget(save_platform); draft_layout->addLayout(save_row);
    QObject::connect(
      save_platform, &QPushButton::clicked, window, [this] {
        update_task_center();
        task_save_dialog->setWindowTitle(
          QStringLiteral("保存到平台 · %1").arg(task_target->currentText()));
        task_save_dialog->open();
        if (!onboard_task_target()) {import_panel->prepare_save();}
      });
    pages->addWidget(draft_scroll);
    navigation->setCurrentRow(0);
    request_note = label(
      QStringLiteral(
        "只读模式 · 查询不会启动/"
        "取消任务，也不会切换模式"
        "或写参数。"), "requestNote", root);
    layout->addWidget(request_note); window->setCentralWidget(root);
    if (mission_options.enabled) {
      request_note->setText(
        QStringLiteral(
          "查询仍只读；仿真任务启动"
          "/取消只走独立确认入口。"));
    }
    if (parameter_options.enabled) {
      request_note->setText(
        QStringLiteral(
          "观察查询只读；两项元数据写入仅"
          "走独立确认入口，不开放飞行参数。"));
    }
    if (import_options.enabled) {
      request_note->setText(
        QStringLiteral(
          "文件导入单独确认，不自动执行；"
          "查询只读，任务控制须另行启用。"));
    }
    auto * dock = new QDockWidget(
      QStringLiteral(
        "本地会话事件（有界"
        "，非任务历史数据库）"), window);
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
    organize_pages();
  }
  void organize_pages()
  {
    // Assemble the existing controllers into stable user-facing pages. Reparent widgets,
    // never duplicate a source, editor, controller or rendering surface.
    const auto old = [this](int index) {return pages->widget(index);};
    auto * overview = old(0); auto * catalog = old(1); auto * perception = old(2);
    auto * local = old(3); auto * diagnostic = old(4); auto * tools = old(5);
    auto * draft = old(6);
    auto * task_center = new QWidget; task_center->setObjectName("taskCenterPage");
    auto * task_layout = new QVBoxLayout(task_center);
    task_layout->setContentsMargins(0, 0, 0, 0); task_layout->setSpacing(6);
    auto * target_row = new QHBoxLayout;
    target_row->addWidget(new QLabel(QStringLiteral("任务目标"), task_center));
    task_target = new QComboBox(task_center); task_target->setObjectName("taskTarget");
    task_target->setAccessibleName(QStringLiteral("任务目标平台"));
    task_target->addItem(QStringLiteral("本机仿真 / DDS"), "simulation");
    target_row->addWidget(task_target);
    task_target_info = label({}, "taskTargetIdentity", task_center);
    target_row->addWidget(task_target_info, 1); task_layout->addLayout(target_row);
    task_document_source = label(
      QStringLiteral("任务编辑维护本地副本；导入与执行使用上方所选平台。"),
      "taskDocumentSource", task_center);
    task_layout->addWidget(task_document_source);
    auto * progress_row = new QHBoxLayout;
    task_progress = label({}, "taskDocumentState", task_center);
    progress_row->addWidget(task_progress, 1);
    review_saved = button(QStringLiteral("查看并审阅已保存任务"), "reviewSavedTask", task_center);
    review_saved->hide(); progress_row->addWidget(review_saved);
    task_layout->addLayout(progress_row);
    QObject::connect(review_saved, &QPushButton::clicked, window, [this] {view_saved_task();});
    task_tabs = new QTabWidget(task_center); task_tabs->setObjectName("taskCenterTabs");
    task_layout->addWidget(task_tabs, 1);
    task_execution_stack = new QStackedWidget(task_tabs);
    task_execution_stack->setObjectName("taskExecutionTargets");
    task_execution_stack->addWidget(catalog);
    task_tabs->addTab(task_execution_stack, QStringLiteral("任务与执行"));
    task_tabs->addTab(draft, QStringLiteral("任务编辑"));
    auto * import_box = import_panel->parentWidget();
    auto * import_scroll = new QScrollArea; import_scroll->setWidgetResizable(true);
    import_scroll->setWidget(import_box);
    task_save_dialog = new QDialog(window); task_save_dialog->setObjectName("taskSaveDialog");
    task_save_dialog->setModal(true); task_save_dialog->resize(760, 460);
    auto * save_layout = new QVBoxLayout(task_save_dialog);
    task_import_stack = new QStackedWidget(task_save_dialog);
    task_import_stack->setObjectName("taskImportTargets");
    task_import_stack->addWidget(import_scroll);
    save_layout->addWidget(task_import_stack, 1);
    auto * back_to_editor = button(
      QStringLiteral("返回编辑"), "closeTaskSaveDialog", task_save_dialog);
    save_layout->addWidget(back_to_editor);
    QObject::connect(back_to_editor, &QPushButton::clicked, task_save_dialog, &QDialog::reject);
    auto * execution_scroll = new QScrollArea; execution_scroll->setWidgetResizable(true);
    execution_scroll->setWidget(execution_page);
    execution_scroll->setFrameShape(QFrame::NoFrame);
    execution_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    const int execution_tab = document_tabs->addTab(
      execution_scroll, QStringLiteral("执行与查询"));
    task_detail_layout->insertWidget(0, global_mission);
    if (mission_panel) {task_detail_layout->addWidget(mission_panel->action_bar());}
    auto * execution_toggle = new QToolButton(task_tabs);
    execution_toggle->setObjectName("toggleTaskExecution");
    execution_toggle->setText(QStringLiteral("执行与查询")); execution_toggle->setCheckable(true);
    execution_toggle->setChecked(mission_options.enabled);
    document_tabs->setCurrentIndex(mission_options.enabled ? execution_tab : 0);
    task_tabs->setCornerWidget(execution_toggle);
    QObject::connect(
      execution_toggle, &QToolButton::toggled, window,
      [this, execution_tab](bool checked) {
        document_tabs->setCurrentIndex(checked ? execution_tab : 0);
      });
    QObject::connect(
      document_tabs, &QTabWidget::currentChanged, execution_toggle,
      [execution_toggle, execution_tab](int index) {
        QSignalBlocker block(execution_toggle); execution_toggle->setChecked(
          index == execution_tab);
      });
    QObject::connect(
      task_tabs, &QTabWidget::currentChanged, execution_toggle,
      [this, execution_toggle](int index) {
        execution_toggle->setVisible(index == 0 && !onboard_task_target());
      });
    QObject::connect(
      task_target, QOverload<int>::of(&QComboBox::currentIndexChanged), window,
      [this, execution_toggle] {
        const bool onboard = onboard_task_target();
        task_execution_stack->setCurrentIndex(onboard ? 1 : 0);
        task_import_stack->setCurrentIndex(onboard ? 1 : 0);
        execution_toggle->setVisible(task_tabs->currentIndex() == 0 && !onboard);
        if (mission_panel) {mission_panel->set_target_active(!onboard);}
        if (onboard_session) {
          onboard_session->setProperty("onboardMissionCenterActive", onboard);
          if (onboard_session->metaObject()->indexOfMethod("setMissionCenterActive(bool)") >= 0) {
            QMetaObject::invokeMethod(
              onboard_session, "setMissionCenterActive", Qt::DirectConnection, Q_ARG(
                bool,
                onboard));
          }
        }
        staged_mission = {}; update_task_center(); update_buttons();
      });
    space_tabs = new QTabWidget; space_tabs->setObjectName("spaceViewTabs");
    space_tabs->addTab(local, QStringLiteral("局部二维"));
    if (!rviz_view && RvizView::available()) {
      rviz_full_page = new QWidget; rviz_full_layout = new QVBoxLayout(rviz_full_page);
      rviz_full_layout->setContentsMargins(0, 0, 0, 0);
      rviz_view = new RvizView(rviz_full_page); rviz_view->setObjectName("embeddedRvizPage");
      rviz_full_layout->addWidget(rviz_view);
    }
    if (rviz_view) {
      space_tabs->addTab(rviz_full_page, QStringLiteral("三维 · RViz"));
    } else {
      space_tabs->addTab(
        label(
          QStringLiteral(
            "当前安装未包含内嵌 RViz。独立 R"
            "Viz 入口位于诊断与设置的高级工具。"),
          "rvizBuildUnavailable", nullptr), QStringLiteral("三维 · RViz"));
    }
    geography = new GeographicView(space_tabs); geography->setObjectName("geographicView");
    space_tabs->addTab(geography, QStringLiteral("地理底图 · 离线"));
    const auto synchronize_geo = [this](GeographicView * from, GeographicView * to) {
        if (syncing_geography) {return;} syncing_geography = true;
        if (from->datum()) {to->set_datum(*from->datum());} else {to->clear_datum();}
        if (!from->offline_map_path().isEmpty() &&
          from->offline_map_path() != to->offline_map_path())
        {to->import_map(from->offline_map_path());}
        to->set_map_source(from->map_source()); syncing_geography = false;
      };
    overview_geography->configuration_changed = [this, synchronize_geo] {
        synchronize_geo(overview_geography, geography);
      };
    geography->configuration_changed = [this, synchronize_geo] {
        synchronize_geo(geography, overview_geography);
      };
    QObject::connect(
      camera, qOverload<int>(&QComboBox::currentIndexChanged), window,
      [this](int index) {
        const QSignalBlocker blocker(overview_camera_selector);
        overview_camera_selector->setCurrentIndex(index);
      });
    QWidget * records = recording_panel;
    if (!records && RecordingWorker::available()) {
      auto paths = storage_protected_paths;
      paths.push_back(mission_options.journal_directory);
      paths.push_back(import_options.journal_directory);
      recording_panel = new RecordingPanel(std::move(paths));
      recording_panel->setObjectName("recordingPage"); records = recording_panel;
    }
    auto * records_scroll = new QScrollArea; records_scroll->setWidgetResizable(true);
    records_scroll->setWidget(
      records ? records : label(
        QStringLiteral(
          "当前安装未包含录像与回放组件"
          "。安装媒体构建后可在这里使用。"),
        "recordingBuildUnavailable", nullptr));
    settings_tabs = new QTabWidget; settings_tabs->setObjectName("settingsTabs");
    settings_tabs->addTab(diagnostic, QStringLiteral("状态与诊断"));
    auto * preferences = new QWidget; auto * preference_layout = new QVBoxLayout(preferences);
    effective_configuration = label(
      QStringLiteral("当前使用独立观测配置。"),
      "effectiveConnectionConfiguration", preferences);
    preference_layout->addWidget(effective_configuration);
    preference_layout->addWidget(configuration); preference_layout->addStretch();
    settings_tabs->addTab(preferences, QStringLiteral("连接信息"));
    settings_tabs->addTab(tools, QStringLiteral("高级工具"));
    while (pages->count()) {pages->removeWidget(pages->widget(0));}
    auto * placeholder = new QWidget; auto * placeholder_layout = new QVBoxLayout(placeholder);
    placeholder_layout->addWidget(
      label(
        QStringLiteral(
          "当前为独立观测入口。请通过 EasyFl"
          "y 应用入口管理仿真、DDS 和 QGC。"),
        "connectionUnavailable", placeholder)); placeholder_layout->addStretch();
    const QList<QWidget *> ordered{placeholder, overview, task_center, perception,
      space_tabs, records_scroll, settings_tabs};
    navigation->clear();
    const QStringList names{QStringLiteral("连接管理"), QStringLiteral("运行总览"),
      QStringLiteral("任务中心"), QStringLiteral("相机与感知"), QStringLiteral("空间视图"),
      QStringLiteral("记录与回放"), QStringLiteral("诊断与设置")};
    const QStringList keys{"connection", "overview", "tasks", "perception", "space", "records",
      "settings"};
    const QStringList icons{"link", "overview", "tasks", "camera", "map", "edit", "diagnostics"};
    for (int i = 0; i < ordered.size(); ++i) {
      pages->addWidget(ordered[i]); navigation->addItem(names[i]);
      navigation->item(i)->setData(Qt::UserRole, keys[i]);
      navigation->item(i)->setIcon(dashboard_icon(icons[i]));
    }
    navigation->setCurrentRow(keys.indexOf("overview"));
  }
  void overview_page()
  {
    auto * layout = page(QStringLiteral("运行总览"), {});
    overview_context = label({}, "overviewContext", pages); layout->addWidget(overview_context);
    auto * metrics = new QGridLayout; metrics->setSpacing(10);
    const auto metric =
      [&](const QString & title, const QString & icon, const char * name, int col) {
        auto * card = new QFrame(pages); card->setProperty("dashboardCard", true);
        auto * content = new QVBoxLayout(card); content->setContentsMargins(10, 6, 10, 6);
        auto * heading = new QHBoxLayout;
        auto * glyph = new QLabel(card); glyph->setPixmap(dashboard_icon(icon).pixmap(16, 16));
        glyph->setFixedSize(18, 18); heading->addWidget(glyph);
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
    speed = metric(QStringLiteral("水平速度 · m/s"), "speed", "stateSpeed", 3);
    battery = metric(QStringLiteral("电池"), "battery", "stateBattery", 4);
    overview_armed = label({}, "overviewArmed", mode->parentWidget());
    mode->parentWidget()->layout()->addWidget(overview_armed);
    overview_vertical_speed = label({}, "overviewVerticalSpeed", speed->parentWidget());
    speed->parentWidget()->layout()->addWidget(overview_vertical_speed);
    layout->addLayout(metrics);
    overview_task = label({}, "overviewTaskSummary", pages); layout->addWidget(overview_task);
    auto * alert_row = new QHBoxLayout;
    overview_alert = label({}, "overviewAlert", pages); overview_alert->hide();
    alert_row->addWidget(overview_alert, 1);
    auto * alert_details = button(QStringLiteral("查看详情"), "overviewAlertDetails", pages);
    alert_details->hide(); alert_row->addWidget(alert_details); layout->addLayout(alert_row);
    QObject::connect(
      alert_details, &QPushButton::clicked, window, [this] {
        window->findChild<QWidget *>("overviewEvidencePanel")->show();
        window->findChild<QPushButton *>("overviewEvidenceToggle")->setChecked(true);
      });
    auto * map_tools = new QHBoxLayout;
    overview_map_mode = new QComboBox; overview_map_mode->setObjectName("overviewMapMode");
    overview_map_mode->addItems(
      {QStringLiteral("自动地图"), QStringLiteral("局部地图"),
        QStringLiteral("地理地图")}); map_tools->addWidget(overview_map_mode);
    auto * zoom_in = button("＋", "overviewMapZoomIn", pages);
    auto * zoom_out = button("−", "overviewMapZoomOut", pages);
    zoom_in->setAccessibleName(QStringLiteral("放大局部地图"));
    zoom_out->setAccessibleName(QStringLiteral("缩小局部地图"));
    auto * center = button(QStringLiteral("定位飞机"), "overviewMapCenter", pages);
    auto * fit = button(QStringLiteral("适应任务"), "overviewMapFit", pages);
    overview_follow = new QCheckBox(QStringLiteral("跟随")); overview_follow->setChecked(true);
    overview_follow->setObjectName("overviewMapFollow");
    for (auto * control : {zoom_in, zoom_out, center, fit}) {map_tools->addWidget(control);}
    map_tools->addWidget(overview_follow);
    auto * layers = new QToolButton(pages); layers->setText(QStringLiteral("图层"));
    layers->setObjectName("overviewMapLayers");
    layers->setAccessibleName(QStringLiteral("局部地图图层"));
    layers->setPopupMode(QToolButton::InstantPopup);
    auto * layer_menu = new QMenu(layers);
    auto * grid_layer = layer_menu->addAction(QStringLiteral("障碍栅格"));
    grid_layer->setObjectName("overviewGridLayer");
    grid_layer->setCheckable(true); grid_layer->setChecked(true);
    auto * obstacle_layer = layer_menu->addAction(QStringLiteral("实时障碍框"));
    obstacle_layer->setObjectName("overviewObstacleLayer"); obstacle_layer->setCheckable(true);
    layers->setMenu(layer_menu); map_tools->addWidget(layers); map_tools->addStretch();
    overview_map_focus = button(QStringLiteral("放大地图"), "overviewMapFocus", pages);
    map_tools->addWidget(overview_map_focus); layout->addLayout(map_tools);
    auto * split = new QSplitter(Qt::Horizontal, pages); split->setObjectName("overviewSplitter");
    overview_split = split;
    split->setChildrenCollapsible(false);
    split->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Ignored);
    split->setMinimumHeight(330);
    overview_map_tabs = new QTabWidget(split); overview_map_tabs->setObjectName("overviewMapTabs");
    overview_map_tabs->tabBar()->hide(); overview_map_tabs->setMinimumWidth(320);
    overview_map_tabs->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Ignored);
    overview_map_tabs->setMinimumHeight(290);
    auto * local = new QWidget(overview_map_tabs); auto * local_layout = new QVBoxLayout(local);
    local_layout->setContentsMargins(4, 4, 4, 4);
    overview_map = new MapCanvas(local); overview_map->setObjectName("overviewMapCanvas");
    overview_map->set_grid_enabled(true); overview_map->set_obstacle_overlay(false);
    QObject::connect(
      grid_layer, &QAction::toggled, window,
      [this](bool on) {overview_map->set_grid_enabled(on);});
    QObject::connect(
      obstacle_layer, &QAction::toggled, window,
      [this](bool on) {overview_map->set_obstacle_overlay(on);});
    overview_map->setMinimumHeight(240); local_layout->addWidget(overview_map, 1);
    overview_map_tabs->addTab(local, QStringLiteral("局部地图"));
    overview_geography = new GeographicView(overview_map_tabs);
    overview_geography->setObjectName("overviewGeographicView");
    overview_geography->set_compact(true);
    overview_map_tabs->addTab(overview_geography, QStringLiteral("地理地图"));
    instrument_overlay = new FlightInstrumentOverlay(overview_map);
    instruments = instrument_overlay->canvas();
    const auto attach_instruments = [this](int index) {
        auto * canvas = index == 1 ?
          overview_geography->findChild<QWidget *>("geographicCanvas") : overview_map;
        instrument_overlay->attach(canvas);
      };
    QObject::connect(
      overview_map_tabs, &QTabWidget::currentChanged, window, attach_instruments);
    attach_instruments(overview_map_tabs->currentIndex());
    QObject::connect(zoom_in, &QPushButton::clicked, window, [this] {overview_map->zoom(1.4);});
    QObject::connect(
      zoom_out, &QPushButton::clicked, window,
      [this] {overview_map->zoom(1 / 1.4);});
    QObject::connect(
      center, &QPushButton::clicked, window, [this] {
        overview_map->center_on_vehicle();
      });
    QObject::connect(fit, &QPushButton::clicked, window, [this] {overview_map->fit();});
    QObject::connect(
      overview_follow, &QCheckBox::toggled, window,
      [this](bool on) {overview_map->set_follow(on);});
    QObject::connect(
      overview_map_focus, &QPushButton::clicked, window,
      [this] {set_overview_focus(overview_focus == 1 ? 0 : 1);});
    auto * side = new QWidget(split); side->setMinimumWidth(220); side->setMaximumWidth(380);
    overview_side = side;
    auto * side_layout = new QVBoxLayout(side); side_layout->setContentsMargins(8, 0, 0, 0);
    auto * box = new QGroupBox(QStringLiteral("相机"), side);
    auto * camera_layout = new QVBoxLayout(box);
    auto * camera_tools = new QHBoxLayout;
    overview_camera_selector = new QComboBox;
    overview_camera_selector->setObjectName("overviewCameraSelector");
    overview_camera_selector->addItems({QStringLiteral("前视"), QStringLiteral("下视")});
    camera_tools->addWidget(overview_camera_selector, 1);
    overview_camera_focus = button(QStringLiteral("放大"), "overviewCameraFocus", box);
    camera_tools->addWidget(overview_camera_focus); camera_layout->addLayout(camera_tools);
    QObject::connect(
      overview_camera_selector, qOverload<int>(&QComboBox::currentIndexChanged),
      window, [this](int index) {if (camera) {camera->setCurrentIndex(index);}});
    QObject::connect(
      overview_camera_focus, &QPushButton::clicked, window,
      [this] {set_overview_focus(overview_focus == 2 ? 0 : 2);});
    overview_camera = new CameraCanvas(box); overview_camera->setObjectName("overviewCamera");
    overview_camera->setMinimumHeight(120); camera_layout->addWidget(overview_camera);
    side_layout->addWidget(box, 1);
    side->setMinimumHeight(side_layout->minimumSize().height());
    split->setMinimumHeight(std::max(330, side->minimumHeight()));
    split->setStretchFactor(0, 1); split->setStretchFactor(1, 0); split->setSizes({740, 300});
    remember_splitter(split, "overview");
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
    form->addRow(QStringLiteral("更新时间"), state_age);
    overview_total_speed = label({}, "overviewTotalSpeed", state);
    form->addRow(QStringLiteral("三维合速度"), overview_total_speed);
    evidence_layout->addWidget(state);
    auto * phase = new QGroupBox(
      QStringLiteral(
        "执行阶段原始状态"), evidence_panel);
    auto * p = new QVBoxLayout(phase);
    global_overview = label({}, "globalPhaseOverview", phase); p->addWidget(global_overview);
    evidence_layout->addWidget(phase);
    evidence_layout->addWidget(
      label(
        QStringLiteral(
          "航向是局部 ENU 方位，不是真北或磁北；局部 Z 不是离地高度。"
          "地图、姿态仪与相机分别判断有"
          "效性；缺少观测不填充演示数据。"), "overviewHint",
        evidence_panel));
    auto * actions = new QHBoxLayout;
    auto * expand = button(
      QStringLiteral("详细状态"), "overviewEvidenceToggle", pages);
    expand->setCheckable(true);
    QObject::connect(
      expand, &QPushButton::toggled, window, [evidence_panel, expand](bool shown) {
        evidence_panel->setVisible(shown);
        expand->setText(
          shown ? QStringLiteral("收起详细状态") : QStringLiteral("详细状态"));
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
  void set_overview_focus(int focus)
  {
    if (!overview_split) {return;}
    if (!overview_focus && focus) {overview_split_sizes = overview_split->sizes();}
    overview_focus = focus;
    overview_map_tabs->setVisible(focus != 2); overview_side->setVisible(focus != 1);
    overview_side->layout()->invalidate();
    overview_side->setMinimumHeight(overview_side->layout()->minimumSize().height());
    overview_split->setMinimumHeight(std::max(330, overview_side->minimumHeight()));
    overview_side->setMaximumWidth(focus == 2 ? QWIDGETSIZE_MAX : 380);
    overview_map_focus->setText(
      focus == 1 ? QStringLiteral("恢复布局") : QStringLiteral("放大地图"));
    overview_camera_focus->setText(
      focus == 2 ? QStringLiteral("恢复布局") : QStringLiteral("放大"));
    if (!focus && overview_split_sizes.size() == 2) {
      overview_split->setSizes(overview_split_sizes);
    }
    window->setProperty("overviewFocus", focus);
  }
  void mount_rviz()
  {
    if (rviz_view) {
      rviz_view->set_connection(
        source && current && current->running && !current->stopped &&
        !closing && !disconnecting && !released, config.use_sim_time);
    }
  }
  void mission_page()
  {
    auto * catalog_page = new QWidget(pages); catalog_page->setObjectName("missionCatalogPage");
    auto * outer = new QHBoxLayout(catalog_page); outer->setContentsMargins(0, 0, 0, 0);
    auto * split = new QSplitter(Qt::Horizontal, catalog_page);
    split->setObjectName("taskExecutionSplitter"); split->setChildrenCollapsible(false);
    outer->addWidget(split); pages->addWidget(catalog_page);
    auto * directory = new QWidget(split); directory->setMinimumWidth(200);
    directory->setObjectName("taskCatalogDirectory");
    auto * directory_layout = new QVBoxLayout(directory);
    directory_layout->setContentsMargins(8, 8, 8, 8); directory_layout->setSpacing(6);
    auto * detail = new QWidget(split); detail->setMinimumWidth(0);
    detail->setObjectName("taskCatalogDetails");
    task_detail_layout = new QVBoxLayout(detail);
    task_detail_layout->setContentsMargins(8, 8, 8, 8); task_detail_layout->setSpacing(6);
    auto * scroll = new QScrollArea(detail); scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    auto * body = new QWidget(scroll); auto * layout = new QVBoxLayout(body);
    layout->setContentsMargins(10, 8, 10, 8); layout->setSpacing(6);
    scroll->setWidget(body); task_detail_layout->addWidget(scroll, 1);
    split->addWidget(directory); split->addWidget(detail);
    split->setStretchFactor(0, 0); split->setStretchFactor(1, 1);
    split->setSizes({280, 760}); remember_splitter(split, "mission-catalog-details");
    directory_layout->addWidget(
      label(
        QStringLiteral("任务目录"),
        "missionCatalogHeading", catalog_page));
    execution_page = new QWidget; execution_layout = new QVBoxLayout(execution_page);
    auto * refresh = button(QStringLiteral("刷新目录（只读）"), "listMissions", pages);
    directory_layout->addWidget(refresh);
    QObject::connect(
      refresh, &QPushButton::clicked, window, [this] {
        if (available() && !onboard_task_target()) {
          record(bridge::Key::MissionList, source->list_missions());
        }
      });
    catalog_quality = label({}, "catalogQuality", pages);
    directory_layout->addWidget(catalog_quality);
    mission_list = new QListWidget(pages); mission_list->setObjectName("missionList");
    mission_list->setMinimumHeight(70); directory_layout->addWidget(mission_list, 1);
    validate = button(QStringLiteral("校验所选任务"), "validateMission", pages);
    validate->setToolTip(QStringLiteral("静态检查目录中的任务；检查通过不代表运行条件已满足"));
    validate->setEnabled(false);
    auto * preview_actions = new QVBoxLayout(); preview_actions->addWidget(validate);
    directory_layout->addLayout(preview_actions);
    QObject::connect(
      mission_list, &QListWidget::currentRowChanged, window, [this] {
        const auto name = mission_list->currentItem() ?
        mission_list->currentItem()->text().toStdString() : std::string{};
        if (name != details_name) {details_name.clear(); clear_details();}
        update_buttons();
      });
    QObject::connect(
      validate, &QPushButton::clicked, window, [this] {
        if (available() && !onboard_task_target() && mission_list->currentItem()) {
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
    inspect = button(QStringLiteral("读取任务详情"), "inspectMission", pages);
    inspect->setToolTip(QStringLiteral("读取所选任务的已校验副本，只读，不执行"));
    inspect->setEnabled(false); preview_actions->addWidget(inspect);
    QObject::connect(
      inspect, &QPushButton::clicked, window, [this] {
        if (!available() || onboard_task_target() || !mission_list->currentItem()) {return;}
        const auto name = mission_list->currentItem()->text().toStdString();
        const auto receipt = source->mission_details(name);
        if (receipt.status == bridge::RequestStatus::Queued) {
          details_name = name; clear_details();
        }
        record(bridge::Key::MissionDetails, receipt);
      });
    edit_catalog = button(QStringLiteral("在任务编辑中打开"), "editCatalogMission", pages);
    edit_catalog->setEnabled(false); preview_actions->addWidget(edit_catalog);
    QObject::connect(
      edit_catalog, &QPushButton::clicked, window, [this] {
        if (!available() || onboard_task_target() || !current->mission_details.value ||
        !mission_list->currentItem()) {return;}
        const auto & document = *current->mission_details.value;
        if (!document.success || document.mission_name !=
        mission_list->currentItem()->text().toStdString()) {return;}
        editor->open_document(
          QString::fromUtf8(document.mission_name.data(), document.mission_name.size()),
          QString::fromUtf8(document.content.data(), document.content.size()));
        task_tabs->setCurrentIndex(1);
      });
    details_summary = label(QStringLiteral("尚未读取任务详情"), "missionDetailsSummary", pages);
    read_summary = label(QStringLiteral("尚未读取任务详情"), "missionReadSummary", pages);
    layout->addWidget(read_summary);
    auto * read_details = new QToolButton(pages); read_details->setCheckable(true);
    read_details->setText(QStringLiteral("读取版本详情"));
    read_details->setObjectName("missionReadDetailsToggle"); layout->addWidget(read_details);
    details_summary->hide();
    QObject::connect(read_details, &QToolButton::toggled, details_summary, &QWidget::setVisible);
    layout->addWidget(details_summary);
    task_dependencies = label(
      QStringLiteral("读取任务后检查算法依赖接口。"), "taskAlgorithmDependencies", pages);
    layout->addWidget(task_dependencies);
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
    layout->addWidget(document_tabs, 1);
    auto * mission_limits = label(
      QStringLiteral(
        "详情是读取时的已校验副本，不保证稍后目录内容未变"
        "；SHA-256不是 execution_id、"
        "Action UUID 或飞行许可。任务文件没有"
        " profile 字段，UI 选择不等于后端配置。"
        "受管操作须启动参数显式启用；没"
        "有任意执行接管或完整历史列表。") +
      (mission_options.enabled ? QString{} : QStringLiteral("默认没有启动/取消入口。")),
      "missionLimit", pages);
    auto * mission_help = new QToolButton(pages);
    mission_help->setText(QStringLiteral("展开任务使用说明")); mission_help->setCheckable(true);
    mission_limits->hide(); layout->addWidget(mission_help); layout->addWidget(mission_limits);
    QObject::connect(mission_help, &QToolButton::toggled, mission_limits, &QWidget::setVisible);
    if (mission_options.enabled) {
      mission_panel = new MissionPanel(
        mission_options, controls_factory, pages,
        [this](bridge::Key key, const std::string & name) {return review_read(key, name);});
      mission_panel->setObjectName("managedMissionPanel");
      execution_layout->addWidget(mission_panel);
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
    execution_layout->addLayout(query_row);
    QObject::connect(execution_id, &QLineEdit::textChanged, window, [this] {update_buttons();});
    QObject::connect(
      query, &QPushButton::clicked, window, [this] {
        if (!available() || onboard_task_target()) {return;}
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
    execution_layout->addWidget(query_result);
    global_mission = label({}, "globalPhaseMission", pages); execution_layout->addWidget(
      global_mission);
    execution_layout->addStretch();
  }
  void perception_page()
  {
    auto * page = new QWidget(pages); page->setObjectName("cameraPerceptionPage");
    auto * row = new QVBoxLayout(page); row->setContentsMargins(0, 0, 0, 0);
    auto * toolbar = new QHBoxLayout;
    auto * content = new QSplitter(Qt::Vertical, page);
    content->setObjectName("cameraResultsSplitter"); content->setChildrenCollapsible(false);
    auto * splitter = new QSplitter(Qt::Horizontal, content);
    splitter->setObjectName("cameraPerceptionSplitter"); splitter->setChildrenCollapsible(false);
    auto * scroll = new QScrollArea(splitter); scroll->setObjectName("cameraControlsScroll");
    scroll->setWidgetResizable(true); scroll->setFrameShape(QFrame::NoFrame);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setMinimumWidth(240); scroll->setMaximumWidth(320);
    auto * controls = new QWidget(scroll); controls->setObjectName("cameraControls");
    controls->setAttribute(Qt::WA_StyledBackground, true);
    auto * layout = new QVBoxLayout(controls);
    layout->setContentsMargins(10, 8, 10, 8); layout->setSpacing(8);
    toolbar->addWidget(new QLabel(QStringLiteral("图像源"), page));
    camera = new QComboBox(controls); camera->setObjectName("cameraSelector");
    camera->addItems({QStringLiteral("前视相机"), QStringLiteral("下视相机（可选）")});
    toolbar->addWidget(camera);
    overlay_mode = new QComboBox(controls); overlay_mode->setObjectName("cameraOverlayMode");
    overlay_mode->addItems(
      {QStringLiteral("实时原图（默认）"),
        QStringLiteral("同帧检测框 · 无身份保证"),
        QStringLiteral("同帧跟踪框 · 原始 SORT ID")});
    overlay_mode->addItems(
      {QStringLiteral("同帧椭圆 · 图像几何"),
        QStringLiteral("同帧彩线 · 图像几何")});
    toolbar->addWidget(overlay_mode, 1);
    overlay_evidence = label({}, "cameraOverlayEvidence", controls);
    layout->addWidget(overlay_evidence); overlay_evidence->hide();
    QObject::connect(
      overlay_mode, qOverload<int>(&QComboBox::currentIndexChanged),
      overlay_evidence, [this](int mode) {overlay_evidence->setVisible(mode != 0);});
    overlay_model = new TextTable(
      {"ID", QStringLiteral("类别"), QStringLiteral("分数（算法各自定义）"),
        QStringLiteral("光轴深度 m"), QStringLiteral("深度关联 / 图像几何")}, controls);
    auto * overlay_table = table(overlay_model, "cameraOverlayTable", controls);
    auto * objects = new QTabWidget(content); objects->setObjectName("perceptionObjectTabs");
    objects->setMinimumHeight(150); objects->addTab(overlay_table, QStringLiteral("同帧图像结果"));
    auto * results = new QToolButton(page); results->setText(QStringLiteral("数据表"));
    results->setObjectName("togglePerceptionResults"); results->setCheckable(true);
    toolbar->addWidget(results);
    auto * focus = new QToolButton(page); focus->setText(QStringLiteral("专注观察"));
    focus->setObjectName("cameraFocusMode"); focus->setCheckable(true);
    toolbar->addWidget(focus); row->addLayout(toolbar); objects->hide();
    QObject::connect(results, &QToolButton::toggled, objects, &QWidget::setVisible);
    QObject::connect(
      focus, &QToolButton::toggled, page, [scroll, objects, results, focus](bool on) {
        scroll->setVisible(!on); objects->setVisible(!on && results->isChecked());
        results->setEnabled(!on);
        focus->setText(on ? QStringLiteral("恢复面板") : QStringLiteral("专注观察"));
      });
    QObject::connect(
      overlay_mode, qOverload<int>(&QComboBox::currentIndexChanged), window, [this] {
        clear_overlay_preview();
        camera_canvas->frame({}, QStringLiteral("显示模式已变更，等待同帧观测"));
        camera_canvas->setProperty("display_stamp_ns", QVariant{});
        overlay_evidence->setText(QStringLiteral("显示模式已变更，等待同帧观测"));
        overlay_model->replace({});
      });
    QObject::connect(
      camera, qOverload<int>(&QComboBox::currentIndexChanged), window, [this] {
        submitted_stamp = -1;
        clear_overlay_preview();
        if (decoder) {decoder->clear();} clear_images(
          QStringLiteral(
            "选源已变更，等待该源的新"
            "鲜图像"));
      });
    camera_canvas = new CameraCanvas(splitter); camera_canvas->setObjectName("cameraCanvas");
    camera_canvas->setMinimumWidth(320);
    layout->addWidget(label(QStringLiteral("图像状态"), "cameraImageHeading", controls));
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
    layout->addWidget(label(QStringLiteral("目标观测"), "cameraTargetsHeading", controls));
    layout->addWidget(target_summary);
    objects->addTab(table(target_model, "targetTable", controls), QStringLiteral("目标"));
    layout->addWidget(label(QStringLiteral("障碍观测"), "cameraObstaclesHeading", controls));
    layout->addWidget(obstacle_summary);
    objects->addTab(table(obstacle_model, "obstacleTable", controls), QStringLiteral("障碍"));
    auto * limits = label(
      QStringLiteral(
        "蓝框=检测；琥珀框=上游跟踪，可能包"
        "含预测框；SORT编号不是真实身份，"
        "不与稳定ID或其他源互换。深度为上游融合光轴深度，不是斜距；"
        "需要同帧、唯一ID/类别和像素中心匹配，无效不填零。\n"
        "上游未报告实际深度帧时间/TF回退；这里只核对"
        "事件关联，不证明RGB-D同步或定位精度。\n"
        "标签／降落、模型预测与类别副本请查看专项标签页；"
        "这些只读显示不提供飞行许可。"), "perceptionLimit",
      controls);
    auto * help = new QToolButton(controls); help->setText(QStringLiteral("显示来源与深度说明"));
    help->setCheckable(true); limits->hide();
    QObject::connect(help, &QToolButton::toggled, limits, &QWidget::setVisible);
    layout->addWidget(help); layout->addWidget(limits);
    algorithm_model = new TextTable(
      {QStringLiteral("模块/能力"), QStringLiteral("当前观测"), QStringLiteral(
          "说明")}, window);
    objects->addTab(
      table(algorithm_model, "algorithmStatusTable", objects),
      QStringLiteral("算法与任务接口"));
    landing_observations = new SpecialObservationPanel(true, objects);
    prediction_observations = new SpecialObservationPanel(false, objects);
    objects->addTab(landing_observations, QStringLiteral("标签／降落"));
    objects->addTab(prediction_observations, QStringLiteral("预测／类别"));
    peripherals = new PeripheralPanel(peripheral_commands, objects);
    peripherals->setObjectName("peripheralPanel");
    objects->addTab(peripherals, QStringLiteral("单目标／云台"));
    layout->addStretch(); scroll->setWidget(controls);
    splitter->addWidget(camera_canvas); splitter->addWidget(scroll);
    splitter->setStretchFactor(0, 1); splitter->setStretchFactor(1, 0);
    splitter->setSizes({900, 260}); content->addWidget(splitter); content->addWidget(objects);
    content->setStretchFactor(0, 1); content->setStretchFactor(1, 0);
    content->setSizes({600, 180});
    remember_splitter(splitter, "camera"); remember_splitter(content, "camera-results");
    row->addWidget(content, 1); pages->addWidget(page);
  }
  void spatial_page()
  {
    auto * layout =
      page(
      QStringLiteral("局部二维 · odom/ENU"), QStringLiteral(
        "显示障碍栅格、实际轨迹与当前有效机位／目标；"
        "实时障碍框可在图层中开启。"));
    map = new MapCanvas(pages); map->setObjectName("spatialCanvas"); layout->addWidget(map, 1);
    map->set_grid_enabled(true); map->set_obstacle_overlay(false);
    auto * layers = new QToolButton(pages); layers->setText(QStringLiteral("图层"));
    layers->setObjectName("spatialMapLayers");
    layers->setAccessibleName(QStringLiteral("局部二维图层"));
    layers->setPopupMode(QToolButton::InstantPopup);
    auto * layer_menu = new QMenu(layers);
    auto * grid_layer = layer_menu->addAction(QStringLiteral("障碍栅格"));
    grid_layer->setObjectName("spatialGridLayer");
    grid_layer->setCheckable(true); grid_layer->setChecked(true);
    auto * obstacle_layer = layer_menu->addAction(QStringLiteral("实时障碍框"));
    obstacle_layer->setObjectName("spatialObstacleLayer"); obstacle_layer->setCheckable(true);
    layers->setMenu(layer_menu);
    layout->itemAt(0)->layout()->addWidget(layers);
    QObject::connect(
      grid_layer, &QAction::toggled, window,
      [this](bool on) {map->set_grid_enabled(on);});
    QObject::connect(
      obstacle_layer, &QAction::toggled, window,
      [this](bool on) {map->set_obstacle_overlay(on);});
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
    configuration = new QGroupBox(
      QStringLiteral(
        "下次观测连接配置"
        "（先断开再修改）"), pages);
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
    auto * local_edits = new QGroupBox(
      QStringLiteral(
        "受限平台元数据编"
        "辑（独立显式启用）"), pages);
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
        "可选独立窗口；不会自动启动。进程"
        "运行不是已连接、可飞或控制权确认。"));
    layout->addWidget(
      label(
        QStringLiteral(
          "RViz2：默认基础飞行预设以 map 显示 MA"
          "VROS 位姿坐标轴；导航/感知预设使用 odom，"
          "需对应发布节点和真实 TF。继承本进程 "
          "ROS domain；启动时绑定当前观测"
          "配置，并允许 DDS 访问 /dev/shm"
          "；这是可信 ROS 查看工具，不是强安全沙箱。"
          "所有 RViz 图层是最后收到的内容，不承诺新鲜度；模拟时钟"
          "暂停时标记寿命也可能暂停。预测/语"
          "义副本尚无显示适配，不混入实测图层。"
          "切换 UI 配置后请手动停止并重启 RViz。"
          "\n\nQGC：仅离线隔离预览，独立临时配置，"
          "无主机网络/设备访问；缺少隔离条件就拒"
          "绝启动，无联网回退。QGC 本身不是只读"
          "软件；本入口不管理用户从其他方式启动"
          "的 QGC，也不代表联网控制已验收。"),
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
      peripherals->stopped() &&
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
          "请先关闭参数确认框或等待本次"
          "参数操作收尾；不自动撤销写入。"), 5000);
      return;
    }
    if (editor->busy()) {
      window->statusBar()->showMessage(
        QStringLiteral(
          "请先完成或关闭本地草"
          "稿操作，再退出窗口。"), 5000); return;
    }
    if (task_save_dialog) {task_save_dialog->hide();}
    parameter_panel->exit_question(true);
    import_panel->exit_question(true);
    if (editor->dirty() && !draft_exit_confirmed) {
      draft_exit_prompt = new QMessageBox(
        QMessageBox::Warning, QStringLiteral("本地草稿尚未导出"),
        QStringLiteral(
          "退出会丢弃未导出的本地改动；不改原任"
          "务文件，不取消远端任务。是否仍退出？"),
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
          "文件可能已导入，但本入口不执行任务。关"
          "闭只停止本地通信，不删除/回滚文件。\n"
          "未决记录保留于：%1\n重启只查询原"
          "身份；实例变化仍保锁。是否知悉并退出？")
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
          "仅两项元数据，不影响飞行。关闭只停"
          "止本地通信，不证明远端未写入。\n"
          "参数故障锁不跨重启恢复，旧写入不重"
          "放；请先人工核对后端。是否仍退出？"),
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
          "未决记录保留在：%1\n重启只查询"
          "原身份，不重发、不清锁。是否仍关闭？")
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
        "只停止本次受管进程，不关闭用户原有"
        "工具或平台节点。QGC 是离线预览。"),
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
  bool onboard_task_target() const
  {
    return task_target && task_target->currentData().toString() == "onboard";
  }
  bool same_task_target(const QJsonObject & expected, const QJsonObject & actual) const
  {
    if (expected.isEmpty() || actual.isEmpty()) {return false;}
    for (const auto & key : {"server_id", "boot_id", "client_id"}) {
      if (expected.value(key).toString().isEmpty() || expected.value(key) != actual.value(key)) {
        return false;
      }
    }
    return true;
  }
  QJsonObject saved_task() const
  {
    if (!onboard_task_target()) {
      const bool matches = available() && current &&
        simulation_saved.value("observer_session").toDouble() == current->observation_session_id &&
        simulation_saved.value("observer_epoch").toDouble() == current->time_epoch &&
        import_panel->matches_saved_receipt(simulation_saved);
      return matches ? simulation_saved : QJsonObject{};
    }
    const auto context = onboard_session ?
      onboard_session->property("onboardMissionContext").value<QJsonObject>() : QJsonObject{};
    return same_task_target(
      onboard_saved.value("target").toObject(),
      context.value("target").toObject()) ? onboard_saved : QJsonObject{};
  }
  void view_saved_task()
  {
    const auto receipt = saved_task(); const auto name = receipt.value("name").toString();
    if (name.isEmpty() || closing) {return;}
    task_save_dialog->hide(); window->show_page("tasks"); task_tabs->setCurrentIndex(0);
    if (onboard_task_target()) {
      if (onboard_session &&
        onboard_session->metaObject()->indexOfMethod("focusMission(QString)") >= 0)
      {
        QMetaObject::invokeMethod(
          onboard_session, "focusMission", Qt::DirectConnection, Q_ARG(QString, name));
      }
    } else {
      saved_catalog_focus = name; refresh_saved_catalog = true; read_saved_catalog = true;
      update_buttons();
    }
  }
  QJsonObject simulation_step_support() const
  {
    QJsonObject support;
    const auto now = bridge::detail::steady_seconds();
    const bool fresh_graph = available() && current->backend_graph_observed > 0 &&
      now >= current->backend_graph_observed && now - current->backend_graph_observed <= 2;
    for (const auto & info : task_step_infos()) {
      QString state = "unknown";
      QString reason = QStringLiteral("等待当前平台的后端观测；可继续离线编辑。");
      if (fresh_graph) {
        const auto key = info.key == "waypoint" ? std::string("goto") : info.key.toStdString();
        const auto found = current->task_backends.find(key);
        const bool present = found != current->task_backends.end() && found->second;
        state = present || info.key == "hover" ? "supported" : "not_ready";
        reason = info.key == "hover" ? QStringLiteral("由任务状态机等待；执行任务仍需任务服务。") :
          (present ? QStringLiteral("动作接口已发现；输入有效性及执行条件仍由平台检查。") :
          QStringLiteral("当前连接未发现此动作后端，请检查已启动的运行方案。"));
        if ((info.key == "navigate" || info.key == "track" || info.key == "search" ||
          info.key == "explore") && current->selected_localization_backend == "disabled")
        {
          state = "not_ready"; reason = QStringLiteral("定位已关闭，请配置定位与地图后再检查。");
        }
        if (info.key == "explore" && (current->selected_profile != "navigation_dev" ||
          current->selected_localization_backend != "fast_lio"))
        {
          state = "not_ready";
          reason = QStringLiteral("FUEL 探索需要导航仿真、FAST-LIO、GCOPTER及探索后端。");
        }
        if (info.key == "precision_land" && present) {
          state = "unknown";
          reason = QStringLiteral("接口已发现；需单独确认精降已启用、下视相机及标记观测链。");
          const auto caps = current->capabilities.find(bridge::Key::LandingCapabilities);
          if (caps != current->capabilities.end() && caps->second.value &&
            display_quality(caps->second.evidence, *current, now, 2, false) ==
            bridge::Quality::ValidAtObservation)
          {
            const auto & details = caps->second.value->details;
            if (std::any_of(
                details.begin(), details.end(), [](const auto & value) {
                  return value.find("disabled") != std::string::npos;
                }))
            {
              state = "not_ready";
              reason = QStringLiteral("精降后端尚未启用，请使用完整精降配置。");
            }
          }
        }
      }
      support[info.key] = QJsonObject{{"state", state}, {"reason", reason}};
    }
    return support;
  }
  void update_task_center()
  {
    if (!task_target) {return;}
    const bool onboard = onboard_task_target();
    const auto context = onboard_session ?
      onboard_session->property("onboardMissionContext").value<QJsonObject>() : QJsonObject{};
    const auto target = context.value("target").toObject();
    editor->set_platform_context(
      QJsonObject{{"kind", onboard ? "onboard" : "simulation"},
        {"fresh", onboard ? context.value("fresh").toBool() : available()},
        {"step_support", onboard ? context.value(
            "step_support").toObject() : simulation_step_support()}});
    const auto alias = target.value("alias").toString(QStringLiteral("机载平台"));
    if (task_target->count() > 1) {
      task_target->setItemText(
        1, target.isEmpty() ? QStringLiteral("机载平台（未连接）") : alias);
    }
    task_target_info->setText(
      onboard ?
      (target.isEmpty() ? QStringLiteral("请在连接管理中连接机载平台") :
      QStringLiteral("%1 · %2 · 身份 %3%4")
      .arg(
        alias, target.value("url").toString(), target.value("server_id").toString().left(12),
        context.value("fresh").toBool() ? QString{} : QStringLiteral(" · 状态未更新"))) :
      QStringLiteral("目录来自当前 DDS 连接；任务执行沿用本机仿真权限。"));
    task_target_info->setToolTip(QString::fromUtf8(QJsonDocument(target).toJson()));
    const auto draft = editor->validated_draft();
    task_save_hint->setText(
      onboard ?
      (context.value("fresh").toBool() ?
      QStringLiteral("保存当前校验版本到机载平台；执行前仍需重新读取审阅。") :
      QStringLiteral("连接机载平台后可保存；当前编辑保留在本地。")) :
      (!import_options.enabled ?
      QStringLiteral("当前连接未启用平台保存；本地编辑、校验和导出仍可用。") :
      (!draft ? QStringLiteral("请先校验任务，并应用或重置尚未提交的表单修改。") :
      QStringLiteral("可直接保存当前校验版本；本地导出为独立可选操作。"))));
    import_panel->observe(draft, available() && !onboard);
    const auto saved = saved_task();
    const bool same_version = draft && saved.value("name").toString() ==
      QString::fromStdString(draft->name) && saved.value("sha256").toString() ==
      QString::fromStdString(draft->hash);
    QString progress = editor->property("draftValidationMessage").toString();
    if (same_version) {
      progress = QStringLiteral("当前版本已保存到所选平台；执行前需读取并审阅平台版本。");
    }
    if (progress.isEmpty()) {
      progress = QStringLiteral("本地任务草稿；选择模板或打开任务后开始编辑。");
    }
    task_progress->setText(progress); review_saved->setVisible(!saved.isEmpty());
    review_saved->setToolTip(
      QStringLiteral("查看平台已保存的任务：%1").arg(saved.value("name").toString()));
    if (!onboard_session) {return;}
    QJsonObject candidate{{"revision", static_cast<double>(editor->revision())},
      {"target", target}, {"valid", false}};
    if (onboard && !closing && draft) {
      candidate["valid"] = true;
      candidate["name"] = QString::fromUtf8(draft->name.data(), draft->name.size());
      candidate["content"] = QString::fromUtf8(draft->content.data(), draft->content.size());
      candidate["hash"] = QString::fromStdString(draft->hash);
    }
    if (staged_mission == candidate) {return;}
    staged_mission = candidate;
    onboard_session->setProperty("onboardMissionCandidate", candidate);
    if (onboard_session->metaObject()->indexOfMethod("setMissionCandidate(QJsonObject)") >= 0) {
      QMetaObject::invokeMethod(
        onboard_session, "setMissionCandidate", Qt::DirectConnection,
        Q_ARG(QJsonObject, candidate));
    }
  }
  void update_buttons()
  {
    import_panel->observe(editor->validated_draft(), available() && !onboard_task_target());
    if (!onboard_task_target() && available() && refresh_saved_catalog) {
      const auto receipt = source->list_missions();
      if (receipt.status == bridge::RequestStatus::Queued) {
        refresh_saved_catalog = false; record(bridge::Key::MissionList, receipt);
      }
    }
    parameter_panel->observer_available(available());
    if (mission_panel) {
      const auto name = mission_list->currentItem() ?
        mission_list->currentItem()->text().toStdString() : std::string{};
      mission_panel->observe(
        current, name, !current || details_name != name ||
        pending(bridge::Key::MissionDetails, current->mission_details.evidence.request_id),
        available(), config.use_sim_time);
    }
    const bool simulation = available() && !onboard_task_target();
    if (inspect) {inspect->setEnabled(simulation && mission_list->currentItem());}
    validate->setEnabled(simulation && mission_list->currentItem());
    if (edit_catalog) {
      edit_catalog->setEnabled(
        simulation && current->mission_details.value && current->mission_details.value->success &&
        mission_list->currentItem() &&
        current->mission_details.value->mission_name ==
        mission_list->currentItem()->text().toStdString() &&
        !mission_content->toPlainText().isEmpty());
    }
    query->setEnabled(
      simulation && bridge::detail::execution_id(
        execution_id->text().toStdString()));
  }
  void record(bridge::Key key, bridge::RequestReceipt receipt)
  {
    if (receipt.status == bridge::RequestStatus::Queued) {
      expected[key] = receipt.id;
      request_note_pending = std::make_pair(key, receipt.id);
      request_note_session = current ? current->observation_session_id : 0;
      request_note_epoch = current ? current->time_epoch : 0;
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
    request_note_pending.reset();
    request_note->setText(status + QStringLiteral("；未发送新的请求。"));
  }
  void update_request_note()
  {
    if (!request_note_pending || !current) {return;}
    const auto key = request_note_pending->first;
    const auto id = request_note_pending->second;
    if (current->observation_session_id != request_note_session ||
      current->time_epoch != request_note_epoch)
    {
      request_note->setText(QStringLiteral("观测会话或时间已变化，原查询结果未确认；请重新查询。"));
      request_note_pending.reset(); return;
    }
    const bridge::Evidence * evidence = nullptr;
    auto find = [key, &evidence](const auto & rows) {
        const auto item = rows.find(key);
        if (item != rows.end()) {evidence = &item->second.evidence;}
      };
    find(current->capabilities); find(current->missions);
    find(current->parameters); find(current->perception);
    if (key == bridge::Key::MissionDetails) {evidence = &current->mission_details.evidence;}
    if (evidence && evidence->request_id == id) {
      request_note->setText(
        QStringLiteral("只读查询 #%1 已结束：%2。%3")
        .arg(id).arg(quality_text(evidence->quality), text(evidence->reason)));
      request_note_pending.reset();
    } else if (!available()) {
      request_note->setText(
        QStringLiteral("只读查询 #%1 观测已断开，结果未确认；可重新查询。").arg(id));
      request_note_pending.reset();
    }
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
  void clear_overlay_preview()
  {
    overlay_observations.reset(); overlay_submitted_stamp = -1;
    completed_overlay_frame.reset(); completed_overlay_observations.reset();
    overlay_origin.reset();
    if (overlay_decoder) {overlay_decoder->clear();}
  }
  void clear_images(const QString & message)
  {
    recording_frame.reset();
    if (peripherals) {peripherals->observe(current, {}, false, config.use_sim_time);}
    clear_overlay_preview();
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
    if (closing) {
      editor->stop(); parameter_panel->stop(); import_panel->stop(); peripherals->stop();
    }
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
    if (closing && !peripherals->stopped()) {return;}
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
    const bool flight_state_live = live && v.sdk.value.connected;
    const auto armed_value = v.sdk.value.armed ?
      QStringLiteral("已解锁") : QStringLiteral("未解锁");
    armed->setText(
      flight_state_live ? armed_value :
      (v.sdk.received ? QStringLiteral("未知 · 最后已知：") + armed_value :
      QStringLiteral("状态未知")));
    overview_armed->setText(armed->text());
    mode->setText(
      flight_state_live ? text(v.sdk.value.flight_mode_name, 128) :
      (v.sdk.received ? QStringLiteral("最后已知：") + text(v.sdk.value.flight_mode_name, 128) :
      QStringLiteral("模式未知")));
    mode->setToolTip(mode->text() + " · " + quality_text(q));
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
      QStringLiteral(
        "局部原点以上 Z，不是离地/海"
        "拔高度；定位不可用时不显示数值。"));
    instruments->snapshot(s, now);
    speed->setText(
      live && v.velocity_valid && v.horizontal_speed_mps ?
      number(*v.horizontal_speed_mps) : QStringLiteral("—"));
    overview_vertical_speed->setText(
      live && v.velocity_valid && v.vertical_speed_mps ?
      QStringLiteral("升降 %1%2 m/s").arg(*v.vertical_speed_mps >= 0 ? "+" : "")
      .arg(number(*v.vertical_speed_mps)) : QStringLiteral("升降速度未知"));
    overview_vertical_speed->setToolTip(QStringLiteral("向上为正，向下为负。"));
    overview_total_speed->setText(
      live && v.velocity_valid && v.speed_mps ?
      number(*v.speed_mps) + " m/s" : QStringLiteral("—"));
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
    for (auto * value : {state_connection, mode, altitude_metric, speed, battery}) {
      auto font = value->font(); int pixels = 24; font.setPixelSize(pixels);
      while (pixels > 14 && QFontMetrics(font).horizontalAdvance(value->text()) >
        value->contentsRect().width())
      {
        font.setPixelSize(--pixels);
      }
      if (value->property("metricPixels").toInt() != pixels) {
        value->setProperty("metricPixels", pixels);
        value->setStyleSheet(QString("font-size:%1px;").arg(pixels));
      }
    }
  }
  void overview_view(double now)
  {
    const auto & snapshot = *current;
    const auto & state = snapshot.state;
    overview_geography->snapshot(current, now);
    const bool gps = overview_geography->gps_available();
    const auto quality = display_quality(state.evidence, snapshot, now, 1, true);
    const bool live = quality == bridge::Quality::ValidAtObservation && state.sdk.received &&
      state.sdk.fresh;
    const auto localization = gps ?
      (live && state.position_valid ?
      QStringLiteral("GPS／局部位置可用") : QStringLiteral("GPS可用")) :
      live && state.position_valid ? QStringLiteral("局部位置可用") : QStringLiteral("定位未就绪");
    auto observed_name = window->windowTitle(); observed_name.remove("EasyFly · ");
    overview_context->setText(QStringLiteral("当前观测：%1 · %2").arg(observed_name, localization));
    const auto task = overview_task_overlay(snapshot, now);
    overview_map->set_task_overlay(task); overview_geography->set_task_overlay(task);
    if (geography) {geography->set_task_overlay(task);}
    overview_task->setText(task.bound ? task.summary : task.reason);
    overview_task->setToolTip(task.reason);
    overview_task->setProperty("executionBound", task.bound);
    overview_task->setProperty("currentStep", task.current_step);
    overview_task->setProperty("completedSteps", task.completed_steps);
    if (gps) {overview_gps_good_at = now;}
    const auto mode_index = overview_map_mode->currentIndex();
    const bool geographic = mode_index == 2 || (mode_index == 0 &&
      (gps || (overview_map_tabs->currentIndex() == 1 && overview_gps_good_at > 0 &&
      now - overview_gps_good_at <= 5)));
    overview_map_tabs->setCurrentIndex(geographic ? 1 : 0);
    for (const auto & name : {"overviewMapZoomIn", "overviewMapZoomOut", "overviewMapCenter",
        "overviewMapFit", "overviewMapFollow", "overviewMapLayers"})
    {window->findChild<QWidget *>(name)->setVisible(!geographic);}
    {
      const QSignalBlocker blocker(overview_follow);
      overview_follow->setChecked(overview_map->following());
    }
    QStringList warnings;
    if (state.sdk.received && (!live || !state.sdk.value.connected)) {
      warnings << QStringLiteral("飞控连接或状态失效，解锁和模式仅显示最后已知值");
    } else if (live && !state.position_valid) {
      warnings << QStringLiteral("局部位置暂不可用");
    }
    if (live && state.battery_valid && state.battery_percent && *state.battery_percent < 15) {
      warnings << QStringLiteral("电量偏低：%1%").arg(number(*state.battery_percent, 1));
    }
    if (snapshot.command_uncertain_observed) {
      warnings << QStringLiteral("存在控制结果未确认记录，请查看详细状态");
    }
    overview_alert->setText(
      warnings.isEmpty() ? QString{} : QStringLiteral("⚠ ") +
      warnings.join("；"));
    overview_alert->setVisible(!warnings.isEmpty());
    overview_alert->setStyleSheet("color:#9a5200;");
    window->findChild<QWidget *>("overviewAlertDetails")->setVisible(!warnings.isEmpty());
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
          r.evidence.request_id) ? QStringLiteral(
          "等待目录查询回复"
          "（不推断活动任务）") :
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
        {
          // Rebuilding the same selection is not a user selection change.
          // Its transient empty row must not erase a concurrent detail reply.
          const QSignalBlocker blocker(mission_list);
          mission_list->clear(); catalog_revision = r.evidence.request_id;
          if (r.value && r.value->success) {
            for (const auto & name : r.value->missions) {
              if (bridge::detail::basename(name)) {mission_list->addItem(text(name, 255));}
            }
          }
          const auto matches = mission_list->findItems(selection, Qt::MatchExactly);
          if (!matches.empty()) {mission_list->setCurrentItem(matches[0]);}
          if (!saved_catalog_focus.isEmpty()) {
            const auto saved = mission_list->findItems(saved_catalog_focus, Qt::MatchExactly);
            if (!saved.empty()) {
              mission_list->setCurrentItem(saved[0]); saved_catalog_focus.clear();
            }
          }
        }
        if (!mission_list->currentItem() || mission_list->currentItem()->text() != selection) {
          details_name.clear(); clear_details();
        }
        update_buttons();
        if (read_saved_catalog && mission_list->currentItem() && saved_catalog_focus.isEmpty()) {
          read_saved_catalog = false;
          const auto name = mission_list->currentItem()->text().toStdString();
          const auto receipt = source->mission_details(name);
          if (receipt.status == bridge::RequestStatus::Queued) {
            details_name = name; clear_details(); record(bridge::Key::MissionDetails, receipt);
          }
        }
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
    if (read_summary) {read_summary->setText(QStringLiteral("尚未读取所选平台任务的详情。"));}
    rendered_details = rendered_details_session = 0;
    if (steps_model) {steps_model->replace({});}
    if (mission_content) {mission_content->clear();}
    if (details_summary) {details_summary->setText(QStringLiteral("尚无与所选任务匹配的详情"));}
    if (task_dependencies) {
      task_dependencies->setText(QStringLiteral("读取任务后检查算法依赖接口。"));
    }
  }
  void details_view(double now)
  {
    if (details_name.empty()) {return;}
    const auto & observation = current->mission_details;
    if (pending(bridge::Key::MissionDetails, observation.evidence.request_id)) {
      read_summary->setText(QStringLiteral("正在读取平台任务：") + text(details_name, 128));
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
      read_summary->setText(details_summary->text());
      return;
    }
    const auto dependency = missing_mission_backends(*current, now);
    task_dependencies->setText(
      dependency.isEmpty() ? QStringLiteral(
        "已读取任务所需接口已发现；输入有效性与飞行准入仍由后端检查。") : dependency);
    const auto & format = current->mission_format;
    read_summary->setText(
      QStringLiteral("平台任务：%1 · %2 步 · %3\n已读取平台版本，执行前仍需审阅确认。")
      .arg(text(details_name, 128)).arg(current->mission_plan->steps.size())
      .arg(quality_text(quality)));
    details_summary->setText(
      QStringLiteral(
        "读取文件：%1 · %2 · %3\nSHA-256：%4\n"
        "%5；严格顺序、失败终止；只在读取时静态校验。\n"
        "UI profile 选择：%6（非任务文件字段，也非后端确认）。")
      .arg(
        text(details_name, 128), text(format), quality_text(quality),
        text(response->content_sha256, 64),
        format == "yaml_v1" ? QStringLiteral("schema v1；文件声明 odom / m / rad / s") :
        QStringLiteral(
          "legacy flat；odom / m / "
          "rad / s 为格式执行约定，非文件显式声明"),
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
        {QString::number(rows.size() + 1), task_step_title(step.type),
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
    const bool processed = overlay_mode->currentIndex() != 0;
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
    const auto freeze_preview = [&](const std::shared_ptr<const ConvertedFrame> & decoded) {
        if (!decoder || !available() || !frame.value || !decoded ||
          !(decoded->binding == binding) || !decoded->error.empty() || decoded->image.isNull() ||
          frame.evidence.time_epoch != s.time_epoch ||
          s.clock.quality != bridge::Quality::ValidAtObservation ||
          decoded->stamp_ns <= 0 || decoded->stamp_ns >
          bridge::detail::stamp_ns(frame.value->header.stamp)) {return false;}
        const double receipt_age = now - decoded->first_received;
        const double source_age = static_cast<double>(s.clock.now_ns - decoded->stamp_ns) / 1e9;
        if (receipt_age < 0 || receipt_age > 5 || source_age < -.2) {return false;}
        const auto caption = QStringLiteral("%1 · %2 · 帧源年龄 %3s · stamp=%4")
          .arg(camera->currentText(), text(binding.topic, 256), number(source_age))
          .arg(decoded->stamp_ns);
        overview_camera->frame(decoded->image, caption); overview_camera->set_frozen(true);
        overview_camera->overlay({});
        overview_camera->setProperty("display_stamp_ns", QVariant{});
        if (!processed) {
          camera_canvas->frame(decoded->image, caption); camera_canvas->set_frozen(true);
          camera_canvas->overlay({});
          camera_canvas->setProperty("display_stamp_ns", QVariant{});
          clear_overlay_preview(); overlay_model->replace({});
          overlay_evidence->setText(QStringLiteral("画面暂停，旧帧叠加结果已清除"));
        }
        return true;
      };
    if (q == bridge::Quality::Stale && decoder && freeze_preview(decoder->latest())) {
      if (processed) {overlay_view(now, channel, binding);} return;
    }
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
    if (!decoded || !(decoded->binding == binding)) {
      clear_images(QStringLiteral("等待当前源的新鲜图像")); return;
    }
    const double age = static_cast<double>(s.clock.now_ns - decoded->stamp_ns) / 1e9;
    const bool fresh = decoded->stamp_ns <= ns && age >= -.2 && age <= 1 &&
      now - decoded->first_received <= 1;
    if (!fresh || !decoded->error.empty()) {
      if (!fresh && decoded->error.empty() && freeze_preview(decoded)) {
        if (processed) {overlay_view(now, channel, binding);} return;
      }
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
      overlay_evidence->setText(
        QStringLiteral(
          "实时原图；同帧感知为独"
          "立可选显示，不影响录像"));
      return;
    }
    overlay_view(now, channel, binding);
  }
  void overlay_view(double now, int channel, const ImageBinding & binding)
  {
    const auto & s = *current; const auto & observations = s.camera_perception[channel];
    const bool tracking = overlay_mode->currentIndex() == 2;
    const bool ellipse = overlay_mode->currentIndex() == 3;
    const bool line = overlay_mode->currentIndex() == 4;
    const auto & image =
      ellipse ? observations.ellipse_image : line ? observations.color_line_image :
      tracking ? observations.track_image : observations.detection_image;
    const auto & evidence =
      ellipse ? observations.ellipses.evidence : line ? observations.color_lines.evidence :
      tracking ? observations.tracks.evidence : observations.detections.evidence;
    const auto topic =
      ellipse ? observations.binding.ellipse_topic : line ? observations.binding.color_line_topic :
      tracking ? observations.binding.tracks_topic : observations.binding.detections_topic;
    const auto q = display_quality(evidence, s, now, 1, true);
    const auto image_quality = display_quality(image.evidence, s, now, 1, true);
    const auto empty = [&](const QString & reason) {
        camera_canvas->frame({}, reason); overlay_model->replace({});
        camera_canvas->setProperty("display_stamp_ns", QVariant{});
        overlay_evidence->setText(reason);
      };
    const auto same_origin = [](const auto & a, const auto & b) {
        return a.image_topic == b.image_topic && a.detections_topic == b.detections_topic &&
               a.tracks_topic == b.tracks_topic && a.targets_topic == b.targets_topic &&
               a.target_frame == b.target_frame && a.ellipse_topic == b.ellipse_topic &&
               a.color_line_topic == b.color_line_topic;
      };
    if (overlay_origin && !same_origin(*overlay_origin, observations.binding)) {
      clear_overlay_preview();
    }
    overlay_origin = observations.binding;
    const auto hard_failure = [](bridge::Quality quality) {
        return quality != bridge::Quality::ValidAtObservation &&
               quality != bridge::Quality::Stale && quality != bridge::Quality::Unavailable;
      };
    const bool source_expired = (ellipse || line) && q == bridge::Quality::Invalid &&
      evidence.reason == bridge::kValidatedGeometryExpiry;
    const bool invalid_geometry = !source_expired &&
      ((ellipse && observations.ellipses.value && !observations.ellipses.value->valid) ||
      (line && observations.color_lines.value && !observations.color_lines.value->valid));
    const bool expired_match = (q == bridge::Quality::Stale || source_expired) && !image.value &&
      image_quality == bridge::Quality::NotReceived;
    if (observations.binding.image_topic.empty() ||
      observations.binding.image_topic != binding.topic ||
      s.clock.quality != bridge::Quality::ValidAtObservation || !available() ||
      invalid_geometry ||
      (hard_failure(q) && !source_expired) || (hard_failure(image_quality) && !expired_match))
    {
      clear_overlay_preview();
      empty(
        text(topic, 256) + " · " +
        quality_text(invalid_geometry ? bridge::Quality::Invalid : q) +
        QStringLiteral(" · 同帧观测不可用"));
      return;
    }
    // Harvest the completed conversion with the exact observations submitted
    // alongside it, before accepting a newer candidate. Keep display and work
    // slots separate: metadata/image transport and conversion are asynchronous.
    if (overlay_decoder) {
      const auto converted = overlay_decoder->latest();
      if (converted && converted->binding == binding &&
        converted->stamp_ns == overlay_submitted_stamp && overlay_observations)
      {
        if (!converted->error.empty()) {
          clear_overlay_preview(); empty(text(converted->error)); return;
        }
        const auto matched = camera_overlay(
          s, *converted, channel, static_cast<CameraOverlayMode>(overlay_mode->currentIndex()),
          now, &*overlay_observations);
        if (matched.matched) {
          completed_overlay_frame = converted;
          completed_overlay_observations = *overlay_observations;
        }
      }
    }
    if (image.value && image_quality == bridge::Quality::ValidAtObservation &&
      q == bridge::Quality::ValidAtObservation)
    {
      if (!overlay_decoder) {
        try {
          overlay_decoder = std::make_unique<ImagePipeline>();
        } catch (const std::exception & error) {
          clear_overlay_preview();
          empty(QStringLiteral("感知转换器未启动：") + text(error.what(), 256)); return;
        }
      }
      overlay_decoder->bind(binding);
      const auto stamp = bridge::detail::stamp_ns(image.value->header.stamp);
      if (stamp != overlay_submitted_stamp) {
        overlay_decoder->submit(image.value, image.evidence.first_source_steady);
        overlay_submitted_stamp = stamp; overlay_observations = observations;
      }
    }
    if (!completed_overlay_frame || !completed_overlay_observations ||
      !(completed_overlay_frame->binding == binding))
    {
      empty(QStringLiteral("等待第一组同帧感知图像")); return;
    }
    const auto & converted = completed_overlay_frame;
    const auto overlay = q == bridge::Quality::Stale || source_expired ? CameraOverlay{} :
    camera_overlay(
      s, *converted, channel, static_cast<CameraOverlayMode>(overlay_mode->currentIndex()),
      now, &*completed_overlay_observations);
    const auto age = static_cast<double>(s.clock.now_ns - converted->stamp_ns) / 1e9;
    const auto caption = QStringLiteral("同帧感知 · 非最新原图 · 源年龄 %1s · stamp=%2")
      .arg(number(age)).arg(converted->stamp_ns);
    if (!overlay.matched) {
      const double receipt_age = now - converted->first_received;
      if (age < -.2 || age > 5 || receipt_age < 0 || receipt_age > 5 ||
        converted->image.isNull() || !converted->error.empty())
      {
        completed_overlay_frame.reset(); completed_overlay_observations.reset();
        empty(QStringLiteral("同帧预览已过期，等待新的匹配图像")); return;
      }
      camera_canvas->frame(converted->image, caption); camera_canvas->overlay({});
      camera_canvas->set_frozen(true);
      camera_canvas->setProperty("display_stamp_ns", QVariant{});
      overlay_model->replace({});
      overlay_evidence->setText(QStringLiteral("画面暂停 · 同帧结果已过期，旧叠层已清除"));
      return;
    }
    camera_canvas->frame(converted->image, caption); camera_canvas->overlay(overlay);
    camera_canvas->setProperty(
      "display_stamp_ns", QVariant::fromValue<qlonglong>(converted->stamp_ns));
    overlay_evidence->setText(
      overlay.status + "\n" + text(topic, 256) + QStringLiteral(
        " · 所示帧源龄 %1s / 收龄 %2s").arg(number(age), number(now - converted->first_received)));
    QVector<QStringList> rows;
    for (const auto & box : overlay.boxes) {
      rows.push_back(
        {box.id < 0 ? QStringLiteral("—") : QStringLiteral("SORT #%1").arg(box.id),
          box.class_name, number(box.confidence, 3), box.axial_depth ? number(*box.axial_depth) :
          QStringLiteral(
            "—"),
          (box.shape == CameraOverlayMode::Ellipses || box.shape == CameraOverlayMode::ColorLines) ?
          box.label + " · " + box.depth_status : box.depth_status});
    }
    overlay_model->replace(std::move(rows));
  }
  void diagnostics_view(double now)
  {
    const auto & s = *current; QVector<QStringList> capabilities; QStringList details;
    details <<
      (mission_options.enabled ?
    QStringLiteral(
      "观测只读；受管仿真操作另行确认，不作为飞行许"
      "可。ROS_DOMAIN_ID=%1（非鉴权）") :
    QStringLiteral(
      "只读 UI，不作为飞行许可。ROS_DO"
      "MAIN_ID=%1（仅启动环境，非鉴权）")).arg(
      qEnvironmentVariable(
        "ROS_DOMAIN_ID",
        "0"));
    details << QStringLiteral("会话 %1 / epoch %2").arg(s.observation_session_id).arg(s.time_epoch);
    details << QStringLiteral(
      "/uav_state 发布源 %1"
      "，其中类型匹配 %2；多个来源不合并")
      .arg(s.state_publishers).arg(s.state_matching_publishers);
    if (qEnvironmentVariableIsSet("EASYFLY_CONNECTION_SESSION")) {
      configuration->setEnabled(false);
      details << QStringLiteral(
        "受连接管理配置约束；修改请返回连接"
        "管理并重建工作会话。配置 ID：%1")
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
    QVector<QStringList> algorithms;
    const bool visual_localization = s.selected_localization_backend == "orbslam3" ||
      s.selected_localization_backend == "fast_livo2" ||
      s.selected_localization_backend == "openvins";
    algorithms.push_back(
      {QStringLiteral("启动定位源"), text(s.selected_localization_backend, 64),
        visual_localization ? (s.localization_navigation_enabled ?
        QStringLiteral("导航仿真已启用；定位、地图与飞行准入仍由后端检查") :
        QStringLiteral("观测模式；导航已禁用")) :
        QStringLiteral("启动配置；实际健康状态需单独观测")});
    if (visual_localization && s.localization_navigation_enabled) {
      const auto & nav = s.navigation_localization;
      algorithms.push_back(
        {QStringLiteral("导航定位"), quality_text(display_quality(nav.evidence, s, now, .5, true)),
          nav.value ? QStringLiteral("会话 %1 / 代次 %2 · %3")
          .arg(text(nav.value->session_id, 128)).arg(nav.value->generation)
          .arg(text(nav.value->reason, 2048)) : QStringLiteral("等待对齐后的导航状态")});
    }
    const auto & localization = s.localization;
    const auto localization_quality = display_quality(localization.evidence, s, now, .5, true);
    QString localization_state = QStringLiteral("等待状态");
    QString localization_detail = text(localization.evidence.reason, 2048);
    if (localization.value) {
      const auto & m = *localization.value;
      const QStringList states{QStringLiteral("等待输入"), QStringLiteral("初始化中"),
        QStringLiteral("估计中（仅表示输出新鲜）"), QStringLiteral("数据过期"),
        QStringLiteral("需要重启整个定位会话"), QStringLiteral("失败")};
      localization_state = m.state < states.size() ? states[m.state] : QStringLiteral("状态无效");
      localization_detail = QStringLiteral("会话 %1 / 代次 %2 · %3 · %4")
        .arg(text(m.session_id, 128)).arg(m.generation)
        .arg(text(m.reason, 2048)).arg(text(m.tracked_frame, 128));
    }
    algorithms.push_back(
      {QStringLiteral("定位原生观测"),
        quality_text(localization_quality) + " · " + localization_state, localization_detail});
    const bool graph_fresh = s.running && !s.stopped && s.backend_graph_observed > 0 &&
      now >= s.backend_graph_observed && now - s.backend_graph_observed <= 2;
    for (const auto & entry : s.task_backends) {
      const auto status = entry.second ?
        QStringLiteral("接口已发现") : QStringLiteral("未发现接口");
      algorithms.push_back(
        {text(entry.first, 64), graph_fresh ? status : QStringLiteral("观测已过期"),
          QStringLiteral("接口发现不代表输入数据/飞行准入就绪")});
    }
    if (algorithms.isEmpty()) {
      algorithms.push_back(
        {QStringLiteral("任务接口"), QStringLiteral("等待观测"),
          QStringLiteral("检查连接与运行方案")});
    }
    for (size_t i = 0; i < s.camera_perception.size(); ++i) {
      const auto & observation = s.camera_perception[i];
      if (!observation.binding.detections_topic.empty()) {
        algorithms.push_back(
          {QStringLiteral("目标检测输出"),
            quality_text(display_quality(observation.detections.evidence, s, now, 1, true)),
            text(observation.binding.detections_topic, 128)});
      }
      if (!observation.binding.tracks_topic.empty()) {
        algorithms.push_back(
          {QStringLiteral("SORT 跟踪输出"),
            quality_text(display_quality(observation.tracks.evidence, s, now, 1, true)),
            text(observation.binding.tracks_topic, 128)});
      }
      for (bool ellipse : {true, false}) {
        const auto & evidence =
          ellipse ? observation.ellipses.evidence : observation.color_lines.evidence;
        const auto topic =
          ellipse ? observation.binding.ellipse_topic : observation.binding.color_line_topic;
        if (topic.empty()) {continue;}
        algorithms.push_back(
          {ellipse ? QStringLiteral("椭圆检测") : QStringLiteral("彩线检测"),
            quality_text(display_quality(evidence, s, now, .5, true)),
            QStringLiteral("%1 · 仅图像几何").arg(text(topic, 128))});
      }
    }
    algorithm_model->replace(std::move(algorithms));
    connection->setText(
      QStringLiteral("DDS：%1 · 飞控：%2 · 权限：%3")
      .arg(
        disconnecting ? QStringLiteral("清理中") : s.running &&
        !s.stopped ? QStringLiteral("观测中") : QStringLiteral("已断开"),
        state_connection->text(),
        mission_options.enabled ? QStringLiteral("受管仿真") : QStringLiteral("观测只读")));
    connection->setToolTip(
      QStringLiteral(
        "曾观测能力端点 %1"
        "/4，详细状态见诊断。").arg(observed));
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
      QStringLiteral("%1 · domain %2 · %3").arg(
        s.running && !s.stopped ? QStringLiteral("实时观测") : QStringLiteral("观测已停止"),
        qEnvironmentVariable("ROS_DOMAIN_ID", "0"),
        mission_options.enabled ? QStringLiteral("仿真操作逐项确认") :
        QStringLiteral("飞行观测只读")));
    diagnostics->setToolTip(
      QStringLiteral("请求 %1/8 · 超时 %2 · 图像丢弃 %3 · 转换等待 %4 · 无效 %5")
      .arg(s.pending_queries).arg(s.counters.timed_out).arg(
        s.counters.dropped_images +
        stats.dropped)
      .arg(stats.pending).arg(s.counters.invalid_samples));
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
    update_task_center();
    window->setProperty("connectionHandoffBlocked", connection_lobby && editor->dirty());
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
    if (!current) {peripherals->observe({}, {}, false, config.use_sim_time); return;}
    update_request_note();
    const double now = bridge::detail::steady_seconds();
    if (source && current->stopped && !disconnecting && !released) {stop_source(false);}
    state_view(now); mission_view(now); geometry_view(bridge::Key::Targets, now);
    geometry_view(bridge::Key::Obstacles, now); image_view(now); diagnostics_view(now);
    const auto owned = mission_panel ? mission_panel->record_state() : nullptr;
    landing_observations->observe(*current, now, owned);
    prediction_observations->observe(*current, now, owned);
    peripherals->observe(
      current, recording_frame, available() && !closing && !disconnecting,
      config.use_sim_time);
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
    map->snapshot(current, now); overview_map->snapshot(current, now);
    if (geography) {geography->snapshot(current, now);}
    overview_view(now);
    update_buttons();
  }
};
MainWindow::MainWindow(
  const bridge::BridgeConfig & config, SourceFactory factory, QWidget * parent,
  const ToolOptions & tools, const MissionOptions & missions, ControlFactory controls,
  const ParameterOptions & parameters, ParameterFactory parameter_controls,
  const ImportOptions & imports, ImportFactory import_controls, const RvizOptions & spatial,
  const RecordingOptions & recording, bool auto_observe, bool peripheral_enabled)
: QMainWindow(parent), impl_(std::make_unique<Impl>(
      this, config, std::move(factory), tools, missions, std::move(controls), parameters,
      std::move(parameter_controls), imports, std::move(import_controls), spatial, recording,
      auto_observe, peripheral_enabled)) {}
MainWindow::~MainWindow() = default;
void MainWindow::set_connection_page(QWidget * page, bool lobby)
{
  if (!page || impl_->connection_page) {return;}
  impl_->connection_page = page; impl_->connection_lobby = lobby;
  impl_->onboard_session = page->property("onboardSession").value<QObject *>();
  impl_->onboard_mission_page = qobject_cast<QWidget *>(
    page->property(
      "onboardMissionPage").value<QObject *>());
  if (impl_->onboard_mission_page && impl_->task_tabs) {
    impl_->task_execution_stack->addWidget(impl_->onboard_mission_page);
    auto * transfer = qobject_cast<QWidget *>(
      page->property("onboardMissionTransferPage").value<QObject *>());
    if (!transfer) {
      transfer = label(
        QStringLiteral("当前机载组件未提供统一任务保存入口。"),
        "onboardTransferUnavailable", impl_->task_import_stack);
    }
    impl_->task_import_stack->addWidget(transfer);
    impl_->task_target->addItem(QStringLiteral("机载平台（未连接）"), "onboard");
  }
  if (impl_->onboard_session) {
    impl_->onboard_session->installEventFilter(this);
    impl_->onboard_session->setProperty("onboardMissionCenterActive", false);
    if (impl_->onboard_session->metaObject()->indexOfMethod("setMissionCenterActive(bool)") >= 0) {
      QMetaObject::invokeMethod(
        impl_->onboard_session, "setMissionCenterActive", Qt::DirectConnection, Q_ARG(bool, false));
    }
  }
  impl_->update_task_center();
  impl_->connection_page_index = 0;
  auto * previous = impl_->pages->widget(0); impl_->pages->removeWidget(previous);
  previous->deleteLater(); impl_->pages->insertWidget(0, page);
  setProperty("connectionPageAvailable", true);
  impl_->configuration->hide();
  impl_->qgc_path->setReadOnly(true);
  if (auto * choose = findChild<QPushButton *>("selectQgc")) {choose->hide();}
  impl_->qgc_path->setText(page->property("connectionQgcPath").toString());
  impl_->qgc_path->setPlaceholderText(QStringLiteral("在连接管理的 QGC 页选择程序"));
  impl_->effective_configuration->setText(
    QStringLiteral(
      "连接参数统一在连接管理页"
      "维护；此处仅展示生效信息。"));
  page->installEventFilter(this);
  if (lobby) {
    // Local drafts, record browsing and settings remain available without a ROS source.
    for (const auto & key : {"overview", "perception", "space"}) {
      for (int i = 0; i < impl_->navigation->count(); ++i) {
        if (impl_->navigation->item(i)->data(Qt::UserRole) == key) {
          impl_->navigation->item(i)->setFlags(
            impl_->navigation->item(
              i)->flags() & ~Qt::ItemIsEnabled);
          impl_->navigation->item(i)->setToolTip(QStringLiteral("连接平台后可查看实时数据"));
        }
      }
    }
    setWindowTitle(QStringLiteral("EasyFly")); impl_->toggle->hide();
    impl_->request_note->setText(
      QStringLiteral(
        "选择连接方式进入工作界面；本地"
        "任务编辑与历史回放可离线使用。"));
    show_connection_page();
  } else {
    impl_->toggle->setText(QStringLiteral("结束观测"));
    impl_->toggle->setToolTip(
      QStringLiteral(
        "正常结束本次观测并返回"
        "连接页，保留仿真后台。"));
  }
  if (!page->property("onboardObserverContext").value<QJsonObject>().isEmpty()) {
    QDynamicPropertyChangeEvent target_event("onboardObserverContext");
    QCoreApplication::sendEvent(page, &target_event);
  }
}
bool MainWindow::show_page(const QString & key)
{
  for (int i = 0; i < impl_->navigation->count(); ++i) {
    auto * item = impl_->navigation->item(i);
    if (item->data(Qt::UserRole).toString() == key && (item->flags() & Qt::ItemIsEnabled)) {
      impl_->navigation->setCurrentRow(i); return true;
    }
  }
  return false;
}
bool MainWindow::show_connection_page()
{
  return impl_->connection_page && show_page("connection");
}
bool MainWindow::eventFilter(QObject * object, QEvent * event)
{
  if (object == impl_->onboard_session && event->type() == QEvent::DynamicPropertyChange) {
    const auto name = static_cast<QDynamicPropertyChangeEvent *>(event)->propertyName();
    if (name == "onboardMissionContext") {
      impl_->update_task_center();
    } else if (name == "onboardMissionSaved") {
      const auto saved = object->property("onboardMissionSaved").value<QJsonObject>();
      if (!saved.value("name").toString().isEmpty() &&
        !saved.value("sha256").toString().isEmpty())
      {
        impl_->onboard_saved = saved; impl_->update_task_center();
      }
    } else if (name == "onboardMissionDocument") {
      const auto document = object->property("onboardMissionDocument").value<QJsonObject>();
      const auto context = object->property("onboardMissionContext").value<QJsonObject>();
      const auto expected = document.value("target").toObject();
      const auto actual = context.value("target").toObject();
      bool matches = !actual.isEmpty();
      for (const auto & key : {"server_id", "boot_id", "client_id"}) {
        matches = matches && expected.value(key) == actual.value(key);
      }
      if (matches && impl_->onboard_task_target() && document.value("content").isString()) {
        impl_->editor->open_document(
          document.value("name").toString(), document.value("content").toString(),
          [this, expected] {
            if (impl_->closing || !impl_->onboard_task_target() || !impl_->onboard_session) {
              return false;
            }
            const auto current = impl_->onboard_session->property(
              "onboardMissionContext").value<QJsonObject>().value("target").toObject();
            for (const auto & key : {"server_id", "boot_id", "client_id"}) {
              if (expected.value(key) != current.value(key)) {return false;}
            }
            return !current.isEmpty();
          });
        show_page("tasks"); impl_->task_tabs->setCurrentIndex(1);
      }
    }
  }
  if (object == impl_->connection_page && event->type() == QEvent::DynamicPropertyChange) {
    const auto name = static_cast<QDynamicPropertyChangeEvent *>(event)->propertyName();
    if (name == "onboardMissionRequested") {
      show_page("tasks");
      if (impl_->onboard_mission_page) {
        impl_->task_target->setCurrentIndex(1);
        impl_->task_tabs->setCurrentIndex(0);
      }
    } else if (name == "onboardObserverContext") {
      const auto target = object->property("onboardObserverContext").value<QJsonObject>();
      impl_->request_note->setText(
        target.isEmpty() ?
        QStringLiteral("当前观察配置未关联机载管理目标；任务执行使用各自独立的目标与权限检查。") :
        QStringLiteral(
          "所选机载观察配置关联目标：%1 · domain %2。"
          "此关联不代表 DDS 已认证数据源身份，请核对实际数据源。")
        .arg(target["server_id"].toString().left(12)).arg(target["ros_domain_id"].toInt()));
    } else if (name == "connectionRaiseRequested") {
      show_connection_page(); show(); raise(); activateWindow();
    } else if (name == "connectionExitRequested") {
      close();
    } else if (name == "connectionQgcPath") {
      if (!impl_->qgc_process.active()) {
        impl_->qgc_path->setText(object->property("connectionQgcPath").toString());
      }
    } else if (name == "connectionState") {
      const auto state = object->property("connectionState").value<QJsonObject>();
      const auto active = state["profile"].toObject();
      QString time_label = QStringLiteral("系统时间");
      if (active["simulation_time"].toBool()) {time_label = QStringLiteral("模拟时间");}
      impl_->effective_configuration->setText(
        active.isEmpty() ? QStringLiteral(
          "尚无生效连接。请在连"
          "接管理页选择配置。") :
        QStringLiteral(
          "生效连接：%1\nROS domain：%2 · 时间源：%3\n运行方案"
          " / 观测标签：%4\n前视：%5\n下视：%6\n"
          "修改请前往连接管理。")
        .arg(active["name"].toString()).arg(active["domain"].toInt())
        .arg(
          time_label,
          active["selected_profile"].toString(), active["front_topic"].toString(),
          active["down_topic"].toString()));
      if (impl_->connection_lobby && !impl_->closing) {
        const auto workspace = state["workspace"].toObject();
        if (state["process_running"].toBool() && state["workspace_fresh"].toBool() &&
          workspace["connection_page"].toBool() && !workspace["closing"].toBool() &&
          !impl_->editor->dirty()) {close();} else {
          impl_->connection->setText(
            state["backend_running"].toBool() ?
            QStringLiteral(
              "后台会话保留中，可"
              "连接并进入工作界面") : QStringLiteral("尚未连接平台"));
        }
      }
    }
  }
  return QMainWindow::eventFilter(object, event);
}
bool MainWindow::cleanup_complete() const
{
  return impl_->released && !impl_->tools_active() &&
         (!impl_->mission_panel || impl_->mission_panel->stopped()) &&
         (!impl_->closing || (impl_->editor->stopped() && impl_->parameter_panel->stopped() &&
         impl_->import_panel->stopped() && impl_->peripherals->stopped() &&
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
    p.import_panel->unresolved() || p.peripherals->unresolved() || p.parameter_panel->busy() ||
    p.parameter_panel->fault();
  return {{"summary", summary}, {"unresolved", unresolved}, {"closing", p.closing},
    {"connection_page", p.connection_page != nullptr},
    {"page", p.navigation->currentItem()->data(Qt::UserRole).toString()},
    {"rviz_available", p.rviz_view != nullptr},
    {"rviz_running", p.rviz_view && p.rviz_view->snapshot().running},
    {"rviz_frames", p.rviz_view ? static_cast<double>(p.rviz_view->snapshot().frames) : 0},
    {"recording_available", p.recording_panel != nullptr},
    {"connection_page_visible", p.connection_page &&
      p.navigation->currentRow() == p.connection_page_index},
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
  if (!impl_->closing && impl_->onboard_session) {
    bool discard = false;
    if (!QMetaObject::invokeMethod(
        impl_->onboard_session, "confirmDiscardChanges", Qt::DirectConnection,
        Q_RETURN_ARG(bool, discard)) || !discard) {event->ignore(); return;}
  }
  if (!impl_->closing) {
    setProperty("connectionCloseReturnsToLobby", property("connectionReturnRequested").toBool());
    setProperty("connectionReturnRequested", false);
  }
  if (impl_->closing && cleanup_complete()) {event->accept(); return;}
  event->ignore(); impl_->request_close();
}
}  // namespace easyfly_desktop

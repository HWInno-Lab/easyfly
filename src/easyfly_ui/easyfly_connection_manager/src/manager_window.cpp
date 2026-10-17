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
#include <QButtonGroup>
#include <QFrame>
#include <QGridLayout>
#include <QStyle>
#include <QToolButton>
#include <QPainter>
#include <QResizeEvent>
#include <QTabWidget>
#include <QScrollArea>
#include <QJsonArray>
#include <QStandardItemModel>
#include <QDesktopServices>
#include <QUrl>
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

#include <algorithm>
#include <cmath>

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
  QString poll_request, exit_request, close_request, connection_error, bound_session;
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
  QPushButton * save, * remove, * start, * stop, * stop_backend, * start_qgc,
    * stop_qgc;
  QComboBox * mode, * scene, * transport, * device, * baud;
  QLineEdit * px4, * bind_host, * remote_host, * gcs_host, * qgc_path;
  QSpinBox * gazebo_port, * bind_port, * remote_port, * target_system,
    * gcs_port, * gcs_bind_port;
  QCheckBox * gazebo_gui, * gcs_enabled, * mission_control, * mission_import,
    * parameter_edit;
  QCheckBox * recording, * rviz_embed;
  QLabel * scene_note, * urls_label;
  QFormLayout * fcu_form;
  QPushButton * refresh_serial, * choose_px4, * choose_qgc;
  QProcess catalog_process;
  QByteArray catalog_output;
  QButtonGroup * mode_cards;
  QList<QToolButton *> mode_buttons;
  QLabel * backend_card, * dds_card, * flight_card, * qgc_card;
  QLabel * scene_preview, * preview_caption;
  QString preview_key;
  QPushButton * details_toggle;
  QGridLayout * command_grid{nullptr};
  int command_columns{-1};

  Impl(ManagerWindow * window, const QString & file, const QString & path)
  : owner(window), config_file(file), runtime(private_directory(path))
  {
    const bool embedded = owner->parentWidget() != nullptr;
    owner->setWindowTitle(QStringLiteral("EasyFly 连接管理"));
    auto * host = embedded ? qobject_cast<QMainWindow *>(owner->window()) : owner;
    if (!host) {host = owner;}
    auto * service_menu = host->menuBar()->addMenu(QStringLiteral("管理服务"));
    auto * retry_service = service_menu->addAction(QStringLiteral("重试连接管理服务"));
    QObject::connect(
      retry_service, &QAction::triggered, owner, [this] {
        retries = 0; service_started = false;
        if (!rpc.connected()) {rpc.connect_to(runtime);}
      });
    auto * stop_service =
      service_menu->addAction(QStringLiteral("结束空闲管理服务并退出"));
    QObject::connect(
      stop_service, &QAction::triggered, owner, [this] {
        if (!queried || actual["process_running"].toBool() ||
        actual["backend_running"].toBool() ||
        actual["qgc_running"].toBool())
        {
          notice(
            QStringLiteral(
              "请先正常关闭工作会话；当前不能"
              "结束管理服务。")); return;
        }
        exit_request = rpc.send("exit_service");
      });
    if (!owner->parentWidget()) {owner->resize(1080, 920);}
    auto * central = new QWidget(owner); owner->setCentralWidget(central);
    auto * main = new QVBoxLayout(central);
    auto * title = new QLabel(
      owner->parentWidget() ? QStringLiteral("连接与仿真") :
      QStringLiteral("EasyFly 仿真与连接管理"), central);
    title->setStyleSheet("font-size:22px;font-weight:600;color:#17263b");
    title->setVisible(!embedded);
    main->addWidget(title);
    auto * hint = new QLabel(
      QStringLiteral(
        "选择工作方式、保存配置，再启动"
        "仿真或连接 DDS。飞控和 QGC 配置均"
        "在此完成；关闭 EasyFly 保留后台"
        "。"), central);
    hint->setWordWrap(true); main->addWidget(hint);
    auto * state_row = new QHBoxLayout; state_row->setSpacing(12);
    const auto card = [state_row, central](const char * id, const QString & text) {
        auto * frame = new QFrame(central); frame->setObjectName("connectionStatusCard");
        auto * box = new QVBoxLayout(frame); auto * value = new QLabel(text, frame);
        value->setObjectName(id); value->setTextFormat(Qt::PlainText); value->setWordWrap(true);
        box->addWidget(value); state_row->addWidget(frame, 1); return value;
      };
    backend_card = card("connectionBackendCard", QStringLiteral("仿真 / 通信\n状态未知"));
    dds_card = card("connectionDdsCard", QStringLiteral("DDS 观测\n尚未连接"));
    flight_card = card("connectionFlightCard", QStringLiteral("飞控\n尚未确认"));
    qgc_card = card("connectionQgcCard", QStringLiteral("QGC 转发\n尚未确认"));
    main->addLayout(state_row);
    auto * split = new QSplitter(central); main->addWidget(split, 1);
    auto * left = new QWidget(split);
    left->setMinimumWidth(160); left->setMaximumWidth(260);
    auto * left_layout = new QVBoxLayout(left);
    list = new QListWidget(left); list->setObjectName("connectionList");
    list->setMinimumWidth(120);
    list->setTextElideMode(Qt::ElideRight);
    list->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    left_layout->addWidget(list);
    scene_preview = new QLabel(left); scene_preview->setObjectName("scenePreview");
    scene_preview->setMinimumSize(160, 120); scene_preview->setMaximumHeight(150);
    scene_preview->setAlignment(Qt::AlignCenter); left_layout->addWidget(scene_preview);
    scene_preview->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    preview_caption = new QLabel(QStringLiteral("场景静态几何预览"), left);
    preview_caption->setWordWrap(true); left_layout->addWidget(preview_caption);
    preview_caption->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    auto * actions = new QHBoxLayout; left_layout->addLayout(actions);
    const auto button =
      [this](const QString & text, const char * id, QLayout * layout) {
        auto * b = new QPushButton(text, owner); b->setObjectName(id);
        layout->addWidget(b);
        return b;
      };
    auto * add = button(QStringLiteral("新建"), "newConnection", actions);
    auto * copy = button(QStringLiteral("复制"), "copyConnection", actions);
    remove = button(QStringLiteral("删除"), "deleteConnection", actions);
    auto * transfer = new QHBoxLayout; left_layout->addLayout(transfer);
    auto * import = button(QStringLiteral("导入"), "importConnections", transfer);
    auto * export_button = button(
      QStringLiteral(
        "导出"), "exportConnections", transfer);
    auto * right = new QWidget(split); auto * layout = new QVBoxLayout(right);
    auto * group = new QGroupBox(QStringLiteral("连接配置"), right);
    auto * general = new QFormLayout(group); layout->addWidget(group);
    name = new QLineEdit(group); name->setObjectName("connectionName");
    name->setMaxLength(64);
    mode = new QComboBox(group); mode->setObjectName("connectionMode");
    mode->addItem(QStringLiteral("启动本机仿真（Gazebo + PX4 + EasyFly）"), "sitl");
    mode->addItem(QStringLiteral("连接已运行的平台（仅 DDS）"), "observe");
    mode->addItem(
      QStringLiteral(
        "启动本机通信（UDP / TCP / 串口飞控"
        "）"), "communication");
    general->addRow(QStringLiteral("名称"), name);
    mode->hide();
    auto * choices = new QHBoxLayout; mode_cards = new QButtonGroup(owner);
    const QStringList titles{QStringLiteral("本机仿真\n启动场景与飞机"),
      QStringLiteral("已有平台\n直接连接 DDS"), QStringLiteral("本机飞控\nUDP / TCP / 串口")};
    for (int i = 0; i < titles.size(); ++i) {
      auto * choice = new QToolButton(owner); choice->setObjectName("connectionModeCard");
      choice->setText(titles[i]); choice->setCheckable(true);
      choice->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
      QPixmap symbol(32, 28); symbol.fill(Qt::transparent); QPainter glyph(&symbol);
      glyph.setRenderHint(QPainter::Antialiasing); glyph.setPen(QPen(QColor("#2563eb"), 2));
      if (i == 0) {
        glyph.drawRoundedRect(QRectF(3, 2, 26, 18), 3, 3);
        glyph.drawLine(16, 20, 16, 25); glyph.drawLine(9, 25, 23, 25);
      } else if (i == 1) {
        glyph.drawLine(16, 9, 7, 20); glyph.drawLine(16, 9, 25, 20);
        glyph.drawEllipse(QPointF(16, 6), 4, 4); glyph.drawEllipse(QPointF(6, 22), 4, 4);
        glyph.drawEllipse(QPointF(26, 22), 4, 4);
      } else {
        glyph.drawRoundedRect(QRectF(7, 5, 18, 18), 3, 3); glyph.drawRect(12, 10, 8, 8);
        for (int pin : {10, 16, 22}) {
          glyph.drawLine(pin, 1, pin, 5); glyph.drawLine(pin, 23, pin, 27);
        }
      }
      glyph.end(); choice->setIcon(QIcon(symbol)); choice->setIconSize(QSize(32, 28));
      choice->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
      mode_cards->addButton(choice, i); mode_buttons.push_back(choice); choices->addWidget(choice);
      QObject::connect(choice, &QToolButton::clicked, owner, [this, i] {mode->setCurrentIndex(i);});
    }
    layout->addLayout(choices);
    auto * tabs = new QTabWidget(right); layout->addWidget(tabs, 1);
    tabs->setObjectName("connectionConfigurationTabs");
    if (embedded) {tabs->setMinimumHeight(240);}
    const auto page = [tabs](const QString & title) {
        auto * area = new QScrollArea(tabs); area->setWidgetResizable(true);
        auto * widget = new QWidget(area); area->setWidget(widget);
        tabs->addTab(area, title);
        return new QFormLayout(widget);
      };
    const auto line =
      [this](const char * id, QFormLayout * form, const QString & label) {
        auto * value = new QLineEdit(owner); value->setObjectName(id);
        value->setMaxLength(4096);
        form->addRow(label, value); return value;
      };
    const auto number =
      [this](const char * id, QFormLayout * form, const QString & label,
        int low, int high) {
        auto * value = new QSpinBox(owner); value->setObjectName(id);
        value->setRange(low, high);
        form->addRow(label, value); return value;
      };
    auto * form = page(QStringLiteral("DDS 连接"));
    domain = number(
      "connectionDomain", form, QStringLiteral(
        "ROS domain"), 0, 232);
    domain->setToolTip(
      QStringLiteral(
        "与目标平台保持一致；新建环境建"
        "议选择 0–101。"));
    scope = new QComboBox(owner); scope->setObjectName("connectionScope");
    scope->addItems({QStringLiteral("仅本机"), QStringLiteral("局域网")});
    simulation = new QCheckBox(QStringLiteral("使用模拟时间（需要 /clock）"), owner);
    simulation->setObjectName("connectionSimulationTime");
    profile = new QComboBox(owner); profile->setObjectName("connectionProfile");
    profile->addItem(QStringLiteral("基础仿真（飞控状态）"), "base_sim");
    profile->addItem(QStringLiteral("感知与建图"), "full_perception");
    profile->addItem(QStringLiteral("导航仿真（规划与任务）"), "navigation_dev");
    profile->addItem(QStringLiteral("降落开发准备"), "landing_dev");
    form->addRow(QStringLiteral("通信范围"), scope); form->addRow(simulation);
    form->addRow(QStringLiteral("运行配置"), profile);
    front = line(
      "connectionFrontTopic", form, QStringLiteral(
        "前视图像 topic（空为禁用）"));
    down =
      line("connectionDownTopic", form, QStringLiteral("下视图像 topic（空为禁用）"));
    auto * supported = new QLabel(
      QStringLiteral(
        "DDS 负责 EasyFly 节点间通信，不使"
        "用飞控串口。当前支持 Humble / Fast "
        "DDS 默认发现。"), owner);
    supported->setWordWrap(true); form->addRow(supported);

    auto * sim_form = page(QStringLiteral("仿真场景"));
    scene = new QComboBox(owner); scene->setObjectName("connectionScene");
    sim_form->addRow(QStringLiteral("场景"), scene);
    scene_note = new QLabel(owner); scene_note->setWordWrap(true);
    scene_note->setTextFormat(
      Qt::PlainText);
    sim_form->addRow(scene_note);
    auto * refresh_scenes = new QPushButton(QStringLiteral("刷新场景与依赖检查"), owner);
    refresh_scenes->setObjectName("refreshScenes"); sim_form->addRow(
      refresh_scenes);
    px4 = line("connectionPx4", sim_form, QStringLiteral("兼容 PX4 目录（空为自动）"));
    choose_px4 = new QPushButton(QStringLiteral("选择 PX4 目录…"), owner);
    sim_form->addRow(choose_px4);
    QObject::connect(
      choose_px4, &QPushButton::clicked, owner, [this] {
        const auto path = QFileDialog::getExistingDirectory(
          owner, QStringLiteral(
            "选择已验证的仿真 PX4 目录"));
        if (!path.isEmpty()) {px4->setText(path); load_scenes();}
      });
    gazebo_port = number(
      "connectionGazeboPort", sim_form, QStringLiteral(
        "Gazebo master 端口"), 1024, 65535);
    gazebo_gui = new QCheckBox(
      QStringLiteral(
        "显示 Gazebo 原生窗口（可选；"
        "相机和内嵌三维仍在工作窗口）"), owner);
    gazebo_gui->setObjectName("connectionGazeboGui"); sim_form->addRow(
      gazebo_gui);
    auto * sim_hint = new QLabel(
      QStringLiteral(
        "仅启动已登记、资源和许可校验通"
        "过且支持当前运行配置的场景。启"
        "动不会解锁或执行飞行任务。"), owner);
    sim_hint->setWordWrap(true); sim_form->addRow(sim_hint);
    auto * advanced_sim = new QPushButton(QStringLiteral("高级仿真设置…"), owner);
    advanced_sim->setObjectName("advancedSimulationSettings"); advanced_sim->setCheckable(true);
    sim_form->insertRow(3, advanced_sim);
    const QList<QWidget *> advanced_fields{px4, choose_px4, gazebo_port, gazebo_gui, sim_hint};
    const auto show_advanced = [sim_form, advanced_fields](bool visible) {
        for (auto * field : advanced_fields) {
          field->setVisible(visible);
          if (auto * caption = sim_form->labelForField(field)) {caption->setVisible(visible);}
        }
      };
    show_advanced(false);
    QObject::connect(advanced_sim, &QPushButton::toggled, owner, show_advanced);

    fcu_form = page(QStringLiteral("飞控链路"));
    transport = new QComboBox(owner); transport->setObjectName(
      "connectionTransport");
    transport->addItem(QStringLiteral("UDP 网络 / 仿真"), "udp");
    transport->addItem(QStringLiteral("TCP 网络"), "tcp");
    transport->addItem(QStringLiteral("串口（USB / UART）"), "serial"); fcu_form->addRow(
      QStringLiteral(
        "传输方式"), transport);
    bind_host =
      line("connectionBindHost", fcu_form, QStringLiteral("UDP 本地地址"));
    bind_port = number(
      "connectionBindPort", fcu_form, QStringLiteral(
        "UDP 本地端口"), 1, 65535);
    remote_host =
      line("connectionRemoteHost", fcu_form, QStringLiteral("飞控 IP / 主机"));
    remote_port = number(
      "connectionRemotePort", fcu_form, QStringLiteral(
        "飞控 UDP / TCP 端口"), 1, 65535);
    device = new QComboBox(owner); device->setEditable(true);
    device->setObjectName(
      "connectionSerialDevice");
    fcu_form->addRow(QStringLiteral("串口设备"), device);
    baud = new QComboBox(owner); baud->setEditable(true); baud->setObjectName(
      "connectionBaud");
    baud->addItems(
      {"9600", "57600", "115200", "230400", "460800", "921600",
        "1500000", "3000000"});
    fcu_form->addRow(QStringLiteral("串口波特率"), baud);
    refresh_serial = new QPushButton(QStringLiteral("刷新串口设备"), owner);
    fcu_form->addRow(refresh_serial);
    const auto scan_serial = [this] {
        const auto previous = device->currentText(); device->clear();
        for (const auto & pattern : {"ttyACM*", "ttyUSB*", "ttyS*"}) {
          for (const auto & item :
            QDir("/dev").entryList({pattern}, QDir::System | QDir::Files))
          {
            device->addItem("/dev/" + item);
          }
        }
        for (const auto & item :
          QDir("/dev/serial/by-id").entryList(
            QDir::System | QDir::Files |
            QDir::NoDotAndDotDot))
        {
          device->addItem("/dev/serial/by-id/" + item);
        }
        device->setCurrentText(previous.isEmpty() ? "/dev/ttyACM0" : previous);
      };
    scan_serial(); QObject::connect(
      refresh_serial, &QPushButton::clicked,
      owner, scan_serial);
    target_system = number(
      "connectionTargetSystem", fcu_form, QStringLiteral(
        "飞控 MAVLink system ID"), 1, 255);
    auto * fcu_hint = new QLabel(
      QStringLiteral(
        "本机 SITL 自动使用固定 UDP 链路。"
        "外部飞控选择“启动本机通信”后"
        "可填写串口/波特率或网络地址；"
        "不会自动修改设备权限。"), owner);
    fcu_hint->setWordWrap(true); fcu_form->addRow(fcu_hint);

    auto * gcs_form = page(QStringLiteral("QGC 转发"));
    gcs_enabled = new QCheckBox(
      QStringLiteral(
        "启用 MAVROS → QGC 双向转发（维护"
        "模式）"), owner);
    gcs_enabled->setObjectName("connectionGcsEnabled"); gcs_form->addRow(
      gcs_enabled);
    gcs_host =
      line("connectionGcsHost", gcs_form, QStringLiteral("QGC 所在主机 IP"));
    gcs_port = number(
      "connectionGcsPort", gcs_form, QStringLiteral(
        "QGC 接收端口"), 1, 65535);
    gcs_bind_port = number(
      "connectionGcsBindPort", gcs_form, QStringLiteral(
        "MAVROS 转发本地端口"), 1, 65535);
    qgc_path =
      line(
      "connectionQgcPath", gcs_form, QStringLiteral(
        "QGC AppImage / 程序路径"));
    choose_qgc = new QPushButton(
      QStringLiteral(
        "选择 QGroundControl 程序…"), owner);
    choose_qgc->setObjectName("chooseQgc"); gcs_form->addRow(choose_qgc);
    QObject::connect(
      choose_qgc, &QPushButton::clicked, owner, [this] {
        const auto path =
        QFileDialog::getOpenFileName(owner, QStringLiteral("选择可信的 QGC 程序"));
        if (!path.isEmpty()) {qgc_path->setText(path);}
      });
    auto * gcs_hint = new QLabel(
      QStringLiteral(
        "本机仿真默认接收端口 14560，避开"
        " PX4 的 14550 直连。打开 QGC 时自动"
        "配置独立设置目录和 UDP 端口。"
        "联网 QGC 可以发送控制请求；此模"
        "式关闭 EasyFly 任务/导入/参数修改"
        "。远端 QGC 在目标机器打开。"), owner);
    gcs_hint->setWordWrap(true); gcs_form->addRow(gcs_hint);

    auto * feature_form = page(QStringLiteral("工作界面"));
    const auto feature =
      [this, feature_form](const QString & label, const char * id) {
        auto * check = new QCheckBox(label, owner); check->setObjectName(id);
        feature_form->addRow(
          check); return check;
      };
    mission_control = feature(
      QStringLiteral(
        "启用本机仿真任务执行（每次提交"
        "仍需确认）"), "connectionMissionControl");
    mission_import = feature(
      QStringLiteral(
        "启用任务文件导入"), "connectionMissionImport");
    parameter_edit = feature(
      QStringLiteral(
        "启用平台元数据编辑"), "connectionParameterEdit");
    recording = feature(
      QStringLiteral(
        "显示录像页（需媒体构建，目录在"
        "工作窗口选择）"), "connectionRecording");
    rviz_embed = feature(
      QStringLiteral(
        "显示内嵌三维页（需 RViz 构建）"), "connectionRvizEmbed");
    // Legacy schema2 values remain round-trippable. Page visibility is now a desktop
    // capability, independent of this frozen backend profile; no automatic rendering/recording.
    recording->hide(); rviz_embed->hide();
    auto * features_hint = new QLabel(
      QStringLiteral(
        "三维与记录页面始终可见，按需手动开始。"
        "任务、导入和平台元数据修改须单独启用，逐项确认。"), owner);
    features_hint->setWordWrap(true); feature_form->addRow(features_hint);
    QWidget * environment_tab = tabs->widget(1);
    tabs->removeTab(1); tabs->insertTab(0, environment_tab, QStringLiteral("运行环境"));
    QWidget * flight_tab = tabs->widget(2);
    tabs->removeTab(2); tabs->insertTab(1, flight_tab, QStringLiteral("飞控链路"));
    tabs->setTabText(4, QStringLiteral("操作权限"));
    tabs->setCurrentIndex(0);
    const auto profile_row = form->takeRow(profile);
    if (profile_row.labelItem) {
      delete profile_row.labelItem->widget(); delete profile_row.labelItem;
    }
    delete profile_row.fieldItem;
    sim_form->insertRow(0, QStringLiteral("运行方案 / 观测标签"), profile);
    urls_label = new QLabel(right); urls_label->setTextFormat(Qt::PlainText);
    urls_label->setWordWrap(true);
    layout->addWidget(urls_label);
    command_grid = new QGridLayout; auto * commands = command_grid; layout->addLayout(commands);
    save = button(QStringLiteral("保存配置"), "saveConnection", commands);
    start = button(QStringLiteral("启动仿真并连接 DDS"), "startConnection", commands);
    start->setProperty("primary", true);
    stop = button(QStringLiteral("关闭工作窗口"), "closeConnection", commands);
    if (owner->parentWidget()) {
      stop->setText(QStringLiteral("结束观测 / 管理后台"));
      stop->setToolTip(
        QStringLiteral(
          "正常关闭当前观测并返回本"
          "页，保留仿真和通信后台。"));
    }
    stop_backend = button(QStringLiteral("停止仿真 / 通信"), "stopBackend", commands);
    stop_backend->setProperty("danger", true);
    auto * tools = new QHBoxLayout; layout->addLayout(tools);
    start_qgc = button(QStringLiteral("打开并连接 QGC"), "startQgc", tools);
    stop_qgc = button(QStringLiteral("关闭本次 QGC"), "stopQgc", tools);
    auto * logs =
      button(QStringLiteral("打开运行日志目录"), "openConnectionLogs", tools);
    auto * recovery_tools = new QHBoxLayout; layout->addLayout(recovery_tools);
    auto * recover = button(
      QStringLiteral("重新连接 / 恢复会话"), "recoverConnection", recovery_tools);
    recovery_tools->addStretch();
    recover->setToolTip(
      QStringLiteral(
        "重新查询服务并认证恢复原会话；"
        "不会重发启动、停止或飞行操作。"));
    QObject::connect(
      recover, &QPushButton::clicked, owner, [this] {
        retries = 0; service_started = false; poll_request.clear();
        if (rpc.connected()) {poll_request = rpc.send("status");} else {rpc.connect_to(runtime);}
      });
    QObject::connect(
      logs, &QPushButton::clicked, owner, [this] {
        auto directory = actual["backend"].toObject()["directory"].toString();
        if (directory.isEmpty()) {directory = actual["session_directory"].toString();}
        if (directory.isEmpty()) {directory = actual["runtime_directory"].toString();}
        if (!directory.isEmpty()) {
          QDesktopServices::openUrl(
            QUrl::fromLocalFile(
              directory));
        } else {
          notice(QStringLiteral("当前尚未创建后端日志目录。"));
        }
      });
    QObject::connect(
      stop_backend, &QPushButton::clicked, owner, [this] {
        rpc.send("stop_backend", {{"session", actual["session"]}});
      });
    QObject::connect(
      start_qgc, &QPushButton::clicked, owner, [this] {
        if (QMessageBox::question(
          owner, QStringLiteral("打开联网 QGC"), QStringLiteral(
            "将打开双向联网 QGC，可直接向飞"
            "控发送控制请求。EasyFly 可变任务"
            "入口保持关闭。是否继续？"),
          QMessageBox::Yes | QMessageBox::No,
          QMessageBox::No) == QMessageBox::Yes)
        {
          rpc.send("start_qgc", {{"session", actual["session"]}});
        }
      });
    QObject::connect(
      stop_qgc, &QPushButton::clicked, owner, [this] {
        rpc.send("stop_qgc", {{"session", actual["session"]}});
      });
    QObject::connect(
      refresh_scenes, &QPushButton::clicked, owner, [this] {
        load_scenes();
      });
    catalog_process.setProcessChannelMode(QProcess::SeparateChannels);
    QObject::connect(
      &catalog_process, &QProcess::readyReadStandardOutput, owner, [this] {
        catalog_output += catalog_process.readAllStandardOutput();
        if (catalog_output.size() > 262144) {catalog_process.kill();}
      });
    QObject::connect(
      &catalog_process,
      QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), owner,
      [this](int code, QProcess::ExitStatus) {
        const auto result = QJsonDocument::fromJson(catalog_output).object();
        if (code || !result["scenes"].isArray()) {
          scene_note->setText(
            QStringLiteral("场景检查失败：") + result["error"].toString(
              QString::fromUtf8(catalog_process.readAllStandardError()).right(
                1024))); return;
        }
        const auto selected_scene = scene->currentData().toString();
        const bool previous_loading = loading; loading = true; scene->clear();
        int ready = 0;
        for (const auto & value : result["scenes"].toArray()) {
          const auto row = value.toObject();
          const bool available = row["status"] == "registered";
          scene->addItem(
            row["id"].toString() +
            (available ? QStringLiteral(" · 已登记") : QStringLiteral(" · 暂不可用")),
            row["id"]);
          scene->setItemData(scene->count() - 1, row, Qt::UserRole + 1);
          if (available) {++ready;}
        }
        const int index = scene->findData(selected_scene);
        if (index >= 0) {scene->setCurrentIndex(index);}
        loading = previous_loading;
        scene_note->setText(
          QStringLiteral(
            "已加载 %1 个场景，%2 个已登记；"
            "启动前校验当前资源、许可和运行"
            "配置。")
          .arg(scene->count()).arg(ready)); update();
      });
    summary = new QLabel(right);
    summary->setObjectName("connectionActualState");
    summary->setTextFormat(Qt::PlainText); summary->setWordWrap(true);
    summary->setMaximumHeight(150);
    layout->addWidget(summary);
    effective = new QLabel(right); effective->setTextFormat(Qt::PlainText);
    effective->setObjectName("connectionEffectiveConfig");
    effective->setWordWrap(true);
    layout->insertWidget(0, effective);
    diagnostics = new QPlainTextEdit(right); diagnostics->setObjectName(
      "connectionDiagnostics");
    diagnostics->setReadOnly(true); diagnostics->setMaximumBlockCount(200);
    auto * details = new QPushButton(QStringLiteral("展开连接诊断"), right);
    details_toggle = details;
    details->setCheckable(true); layout->addWidget(details);
    diagnostics->hide();
    summary->hide(); effective->hide();
    QObject::connect(
      details, &QPushButton::toggled, diagnostics,
      &QWidget::setVisible);
    QObject::connect(details, &QPushButton::toggled, owner, [this](bool) {update();});
    layout->addWidget(diagnostics, 1);
    note = new QLabel(central); note->setObjectName("connectionNotice");
    note->setTextFormat(
      Qt::PlainText);
    note->setWordWrap(true); main->addWidget(note);
    split->setSizes({260, 780});
    owner->setStyleSheet(
      "QFrame#connectionStatusCard {background:#ffffff;border:1px solid #dce3eb;"
      "border-top:3px solid #d8e7fc;border-radius:9px;padding:5px;}"
      "QToolButton#connectionModeCard {background:#ffffff;border:1px solid #dce3eb;"
      "border-radius:8px;padding:9px;color:#526176;}"
      "QToolButton#connectionModeCard:checked {backgroun"
      "d:#e5efff;border:2px solid #2563eb;color:#164b9b;}"
      "QToolButton#connectionModeCard:disabled {color:#64748b;}"
      "QPushButton#newConnection,QPushButton#copyConnection,QPushButton#deleteConnection,"
      "QPushButton#importConnections,QPushButton#exportConnections {padding:6px;font-size:12px;}"
      "QLabel[connectionHealth=good] {color:#166534;font-weight:600;}"
      "QLabel[connectionHealth=unknown] {color:#667085;}"
      "QLineEdit,QSpinBox,QComboBox {min-height:24px;}");

    try {
      profiles = read_profiles(config_file);
      for (const auto & p : profiles) {persisted_hashes.insert(p.id, p.hash());}
    } catch (const std::exception & error) {
      healthy = false;
      notice(QString::fromUtf8(error.what()) + QStringLiteral("；保留原文件，不覆盖。"));
    }
    if (healthy && profiles.isEmpty()) {
      auto initial = Profile::create(QStringLiteral("本机仿真"));
      initial.mode = "sitl";
      profiles.push_back(initial);
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
    for (auto * edit :
      {name, front, down, px4, bind_host, remote_host, gcs_host, qgc_path})
    {
      QObject::connect(edit, &QLineEdit::textChanged, owner, changed);
    }
    QObject::connect(
      domain, QOverload<int>::of(
        &QSpinBox::valueChanged), owner, changed);
    QObject::connect(
      scope, QOverload<int>::of(
        &QComboBox::currentIndexChanged), owner, changed);
    QObject::connect(
      profile, QOverload<int>::of(
        &QComboBox::currentIndexChanged), owner, changed);
    QObject::connect(simulation, &QCheckBox::toggled, owner, changed);
    for (auto * item : {mode, scene, transport, device, baud}) {
      QObject::connect(
        item, QOverload<int>::of(
          &QComboBox::currentIndexChanged), owner, changed);
      if (item->isEditable()) {
        QObject::connect(item, &QComboBox::editTextChanged, owner, changed);
      }
    }
    for (auto * item :
      {gazebo_port, bind_port, remote_port, target_system, gcs_port,
        gcs_bind_port})
    {
      QObject::connect(
        item, QOverload<int>::of(
          &QSpinBox::valueChanged), owner, changed);
    }
    for (auto * item :
      {gazebo_gui, gcs_enabled, mission_control, mission_import, parameter_edit,
        recording,
        rviz_embed})
    {
      QObject::connect(item, &QCheckBox::toggled, owner, changed);
    }
    QObject::connect(
      mode, QOverload<int>::of(&QComboBox::currentIndexChanged), owner, [this] {
        if (loading) {return;}
        if (mode->currentData() == "sitl") {
          simulation->setChecked(true); scope->setCurrentIndex(
            0); transport->setCurrentIndex(transport->findData("udp"));
          bind_host->setText("127.0.0.1"); remote_host->setText("127.0.0.1");
          bind_port->setValue(14540); remote_port->setValue(
            14580); target_system->setValue(1);
        }
        if (mode->currentData() == "communication") {
          simulation->setChecked(false);
        }
        if (mode->currentData() == "observe") {gcs_enabled->setChecked(false);}
        if (mode->currentData() != "sitl") {mission_control->setChecked(false);}
        update();
      });
    QObject::connect(
      gcs_enabled, &QCheckBox::toggled, owner, [this](bool checked) {
        if (checked && !loading) {
          mission_control->setChecked(false); mission_import->setChecked(
            false); parameter_edit->setChecked(false);
        }
        update();
      });
    QObject::connect(
      save, &QPushButton::clicked, owner, [this] {
        save_current();
      });
    QObject::connect(
      add, &QPushButton::clicked, owner,
      [this] {create(false);});
    QObject::connect(
      copy, &QPushButton::clicked, owner,
      [this] {create(true);});
    QObject::connect(
      remove, &QPushButton::clicked, owner, [this] {
        if (!editable() || selected < 0) {return;}
        if (QMessageBox::question(
          owner, QStringLiteral("删除连接"),
          QStringLiteral("只删除此配置，保留日志、录像和任务记录。"),
          QMessageBox::Yes | QMessageBox::No,
          QMessageBox::No) != QMessageBox::Yes) {return;}
        auto next = profiles; next.removeAt(selected);
        try {
          write_profiles(
            config_file,
            next); profiles = next; selected = -1; refresh(0);
        } catch (
          const std::exception & error) {notice(error.what());}
      });
    QObject::connect(
      export_button, &QPushButton::clicked, owner, [this] {
        const auto path = QFileDialog::getSaveFileName(
          owner, QStringLiteral("导出到私有目录"),
          QFileInfo(config_file).absolutePath() + "/connections-export.json",
          "JSON (*.json)");
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
            p.id = QUuid::createUuid().toString(
              QUuid::WithoutBraces); next << p;
          }
          write_profiles(config_file, next); profiles = next;
          for (const auto & p : profiles) {
            persisted_hashes.insert(p.id, p.hash());
          }
          refresh(profiles.size() - 1);
          notice(QStringLiteral("已导入为新配置；未启动任何连接。"));
        } catch (const std::exception & error) {notice(error.what());}
      });
    QObject::connect(
      start, &QPushButton::clicked, owner, [this] {
        if (!start->isEnabled()) {return;}
        if (owner->window()->property("connectionHandoffBlocked").toBool()) {
          notice(
            QStringLiteral(
              "请先保存或导出本地草稿，再"
              "连接平台；当前内容已保留。")); return;
        }
        if (actual["backend_running"].toBool() || actual["unresolved"].toBool()) {
          if (dirty()) {
            notice(QStringLiteral("请先保存当前配置编辑，再重开生效工作窗口。")); return;
          }
          start->setEnabled(false);
          rpc.send("start", {{"profile", actual["profile"]}}); return;
        }
        if (selected < 0) {return;}
        if (dirty() || persisted_hashes.value(profiles[selected].id) != profiles[selected].hash()) {
          save_current();
          if (dirty() || persisted_hashes.value(
            profiles[selected].id) != profiles[selected].hash())
          {
            return;
          }
        }
        start->setEnabled(false);
        const auto selected = form_profile();
        if ((selected.gcs_enabled || selected.mission_control ||
        selected.mission_import ||
        selected.parameter_edit) &&
        QMessageBox::question(
          owner, QStringLiteral("启动所选功能"), QStringLiteral(
            "将按保存配置启动仿真/通信及所"
            "选操作入口。联网 QGC 是双向维护"
            "模式；任务提交仍需在工作窗口确"
            "认。继续？"),
          QMessageBox::Yes | QMessageBox::No,
          QMessageBox::No) != QMessageBox::Yes)
        {
          update(); return;
        }
        rpc.send("start", {{"profile", selected.json()}});
      });
    QObject::connect(
      stop, &QPushButton::clicked, owner, [this] {
        if (owner->parentWidget()) {
          owner->window()->setProperty("connectionReturnRequested", true);
        }
        close_request = rpc.send("close", {{"session", actual["session"]}});
        if (close_request.isEmpty()) {
          owner->window()->setProperty("connectionReturnRequested", false);
        }
      });
    QObject::connect(
      &rpc, &RpcClient::ready, owner, [this] {
        retries = 0; queried = false;
        if (note->text() == connection_error) {note->clear();}
        connection_error.clear();
        poll_request = rpc.send(
          "attach_manager", {{"config_file",
            config_file}});
      });
    QObject::connect(
      &rpc, &RpcClient::reply, owner, [this](const QJsonObject & reply) {
        if (reply["event"] == "raise") {
          owner->setProperty(
            "connectionRaiseRequested",
            owner->property("connectionRaiseRequested").toInt() + 1);
          owner->show(); owner->window()->raise(); owner->window()->activateWindow(); return;
        }
        if (reply["id"].toString() == poll_request) {poll_request.clear();}
        if (!close_request.isEmpty() && reply["id"].toString() == close_request) {
          if (!reply["ok"].toBool()) {
            owner->window()->setProperty("connectionReturnRequested", false);
          }
          close_request.clear();
        }
        if (!exit_request.isEmpty() && reply["id"].toString() == exit_request &&
        reply["ok"].toBool())
        {
          if (owner->parentWidget()) {owner->setProperty("connectionExitRequested", true);} else {
            owner->close();
          }
          return;
        }
        if (!reply["ok"].toBool()) {notice(reply["error"].toString());}
        if (reply["status"].isObject()) {
          queried = true; actual = reply["status"].toObject();
          const auto active = actual["profile"].toObject();
          if (bound_session != actual["session"].toString()) {
            bound_session = actual["session"].toString();
            if (!dirty()) {
              for (int row = 0; row < profiles.size(); ++row) {
                if (profiles[row].id == active["id"].toString() &&
                profiles[row].hash() == actual["config_hash"].toString()) {refresh(row); break;}
              }
            }
          }
          owner->setProperty("connectionState", actual);
        }
        response_clock.restart();
        update();
      });
    QObject::connect(
      &rpc, &RpcClient::failure, owner, [this](const QString & error) {
        queried = false; poll_request.clear(); connection_error = error; notice(error); update();
      });
    QObject::connect(
      &rpc, &RpcClient::disconnected, owner, [this] {
        retries = 0; service_started = false;
      });
    timer.setInterval(500);
    QObject::connect(
      &timer, &QTimer::timeout, owner, [this] {
        if (response_clock.isValid() && response_clock.elapsed() > 2500) {
          queried = false; poll_request.clear(); update();
        }
        if (rpc.connected()) {
          if (poll_request.isEmpty()) {
            poll_request = owner->parentWidget() ? rpc.send(
              "attach_manager",
              {{"config_file", config_file}}) : rpc.send("status");
          }
        } else if (++retries <= 10) {
          if (!service_started) {
            service_started = true;
            const auto program = supervisor_program();
            if (!QProcess::startDetached(
              program,
              {"--runtime-directory", runtime}))
            {
              notice(QStringLiteral("无法启动连接管理服务"));
            }
          }
          rpc.connect_to(runtime);
        }
      });
    refresh(0); load_scenes(); timer.start(); rpc.connect_to(runtime);
  }
  void load_scenes()
  {
    if (catalog_process.state() != QProcess::NotRunning) {return;}
    catalog_output.clear();
    scene_note->setText(QStringLiteral("正在检查已安装场景与资源…"));
    catalog_process.start(
      backend_program(), {"--catalog", "--px4-directory", px4->text()});
    QTimer::singleShot(
      15000, owner, [this] {
        if (catalog_process.state() != QProcess::NotRunning) {
          catalog_process.kill(); scene_note->setText(
            QStringLiteral(
              "场景检查超时，请检查安装与资源"
              "目录。"));
        }
      });
  }
  void notice(const QString & text) {note->setText(text.left(2048));}
  ~Impl()
  {
    timer.stop();
    QObject::disconnect(&rpc, nullptr, owner, nullptr);
    QObject::disconnect(&catalog_process, nullptr, owner, nullptr);
    if (catalog_process.state() != QProcess::NotRunning) {
      catalog_process.kill(); catalog_process.waitForFinished(1000);
    }
  }
  Profile form_profile() const
  {
    Profile p = selected >= 0 &&
      selected < profiles.size() ? profiles[selected] : Profile{};
    p.name = name->text(); p.domain = domain->value();
    p.local_only = scope->currentIndex() == 0;
    p.simulation_time = simulation->isChecked();
    p.selected_profile = profile->currentData().toString();
    p.front_topic = front->text(); p.down_topic = down->text();
    p.mode = mode->currentData().toString();
    p.scene = scene->currentData().toString();
    p.px4_directory = px4->text(); p.gazebo_port = gazebo_port->value();
    p.gazebo_gui = gazebo_gui->isChecked();
    p.transport = transport->currentData().toString(); p.bind_host = bind_host->text();
    p.bind_port = bind_port->value();
    p.remote_host = remote_host->text(); p.remote_port = remote_port->value();
    p.serial_device = device->currentText();
    p.baud = baud->currentText().toInt();
    p.target_system = target_system->value();
    p.gcs_enabled = gcs_enabled->isChecked(); p.gcs_host = gcs_host->text();
    p.gcs_port = gcs_port->value();
    p.gcs_bind_port = gcs_bind_port->value(); p.qgc_path = qgc_path->text();
    p.mission_control = mission_control->isChecked();
    p.mission_import = mission_import->isChecked();
    p.parameter_edit = parameter_edit->isChecked();
    p.recording = recording->isChecked();
    p.rviz_embed = rviz_embed->isChecked();
    return p;
  }
  bool dirty() const
  {
    return selected >= 0 && selected < profiles.size() &&
           form_profile().json() != profiles[selected].json();
  }
  void arrange_commands(int width)
  {
    if (!command_grid) {return;}
    const int columns = width < 1000 ? 2 : 4;
    if (command_columns == columns) {return;}
    command_columns = columns;
    const QList<QWidget *> buttons{save, start, stop, stop_backend};
    for (int i = 0; i < buttons.size(); ++i) {
      command_grid->removeWidget(buttons[i]); command_grid->addWidget(
        buttons[i], i / columns,
        i % columns);
    }
  }
  bool editable() const
  {
    return healthy && selected >= 0 && selected < profiles.size() &&
           queried &&
           !((actual["process_running"].toBool() ||
           actual["backend_running"].toBool()) &&
           actual["profile"].toObject()["id"] == profiles[selected].id);
  }
  void load(int row)
  {
    selected = row; loading = true;
    const auto p = profiles.value(row);
    name->setText(p.name); domain->setValue(p.domain); scope->setCurrentIndex(
      p.local_only ? 0 : 1);
    simulation->setChecked(p.simulation_time);
    profile->setCurrentIndex(profile->findData(p.selected_profile)); front->setText(p.front_topic);
    down->setText(
      p.down_topic);
    mode->setCurrentIndex(mode->findData(p.mode));
    if (scene->findData(p.scene) < 0) {scene->addItem(p.scene, p.scene);}
    scene->setCurrentIndex(scene->findData(p.scene)); px4->setText(
      p.px4_directory);
    gazebo_port->setValue(p.gazebo_port); gazebo_gui->setChecked(p.gazebo_gui);
    transport->setCurrentIndex(transport->findData(p.transport)); bind_host->setText(p.bind_host);
    bind_port->setValue(
      p.bind_port);
    remote_host->setText(p.remote_host); remote_port->setValue(p.remote_port);
    device->setCurrentText(p.serial_device); baud->setCurrentText(
      QString::number(
        p.baud));
    target_system->setValue(p.target_system);
    gcs_enabled->setChecked(p.gcs_enabled); gcs_host->setText(p.gcs_host);
    gcs_port->setValue(
      p.gcs_port);
    gcs_bind_port->setValue(p.gcs_bind_port); qgc_path->setText(p.qgc_path);
    mission_control->setChecked(p.mission_control); mission_import->setChecked(
      p.mission_import);
    parameter_edit->setChecked(p.parameter_edit); recording->setChecked(
      p.recording);
    rviz_embed->setChecked(p.rviz_embed);
    loading = false; update();
  }
  void refresh(int row)
  {
    loading = true; list->clear();
    for (const auto & p : profiles) {
      const auto label = p.name + QStringLiteral(" · domain %1").arg(p.domain);
      list->addItem(label); list->item(list->count() - 1)->setToolTip(label);
    }
    loading = false;
    if (!profiles.isEmpty()) {
      list->setCurrentRow(qBound(0, row, profiles.size() - 1));
    } else {
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
      selected >=
      0 ? profiles[selected] : Profile::create(QStringLiteral("新连接"));
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
      for (const auto & item : profiles) {
        persisted_hashes.insert(item.id, item.hash());
      }
      const auto row = selected; selected = -1; refresh(row);
      notice(QStringLiteral("配置已保存；保存不会启动连接。"));
    } catch (const std::exception & error) {notice(error.what());}
  }
  void update()
  {
    arrange_commands(owner->width());
    const bool edit = editable();
    for (int i = 0; i < mode_buttons.size(); ++i) {
      mode_buttons[i]->setChecked(mode->currentIndex() == i); mode_buttons[i]->setEnabled(edit);
    }
    const auto live = actual["workspace"].toObject();
    const auto backend_phase = actual["backend"].toObject()["phase"].toString();
    const bool backend_stopping = backend_phase == "stopping" || backend_phase == "stop_unknown";
    const bool backend_fault = backend_phase == "degraded" || backend_phase == "unknown" ||
      backend_phase == "stop_unknown" || actual["recovery_blocked"].toBool() ||
      !actual["recovery_error"].toString().isEmpty() ||
      (actual["backend_running"].toBool() && !actual["backend_fresh"].toBool());
    backend_card->setText(
      QStringLiteral("仿真 / 通信\n%1").arg(
        actual["backend_running"].toBool() ?
        (backend_fault ? QStringLiteral("运行异常") : QStringLiteral("运行中")) :
        QStringLiteral("未运行")));
    dds_card->setText(
      QStringLiteral("DDS 观测\n%1").arg(
        actual["workspace_fresh"].toBool() && live["observer_running"].toBool() ?
        QStringLiteral("观测中") : QStringLiteral("未连接")));
    flight_card->setText(
      QStringLiteral("飞控\n%1").arg(
        actual["backend_fresh"].toBool() ?
        (actual["backend"].toObject()["connected"].toBool() ? QStringLiteral("已连接") :
        QStringLiteral("未连接")) : actual["workspace_fresh"].toBool() &&
        live["state_fresh"].toBool() ?
        (live["flight_connected"].toBool() ? QStringLiteral("已连接") : QStringLiteral("未连接")) :
        QStringLiteral("状态未知")));
    const auto active_gcs = actual["profile"].toObject().value("gcs").toObject();
    const bool forwarding = active_gcs.value("enabled").toBool();
    qgc_card->setText(
      QStringLiteral("QGC\n%1").arg(
        actual["qgc_running"].toBool() ?
        QStringLiteral("本次进程运行") : forwarding ?
        QStringLiteral("转发已配置") : QStringLiteral("转发关闭")));
    const auto health = [](QLabel * label, bool good) {
        const QString value = good ? "good" : "unknown";
        if (label->property("connectionHealth").toString() != value) {
          label->setProperty("connectionHealth", value);
          label->style()->unpolish(label); label->style()->polish(label);
        }
      };
    health(backend_card, actual["backend_fresh"].toBool() && !backend_fault);
    health(dds_card, actual["workspace_fresh"].toBool() && live["observer_running"].toBool());
    health(
      flight_card, actual["backend_fresh"].toBool() ?
      actual["backend"].toObject()["connected"].toBool() :
      actual["workspace_fresh"].toBool() && live["state_fresh"].toBool() &&
      live["flight_connected"].toBool());
    health(qgc_card, actual["qgc_running"].toBool());
    for (QWidget * widget :
      QList<QWidget *>{name, mode, domain, scope, simulation, profile, front,
        down,
        scene, px4, gazebo_port, gazebo_gui, transport, bind_host, bind_port,
        remote_host,
        remote_port,
        device, baud, target_system, gcs_enabled, gcs_host, gcs_port,
        gcs_bind_port, qgc_path,
        mission_control, mission_import, parameter_edit, recording, rviz_embed})
    {
      widget->setEnabled(edit);
    }
    owner->setProperty("connectionQgcPath", qgc_path->text());
    const auto selected_mode = mode->currentData().toString();
    const bool sim = selected_mode == "sitl",
      comm = selected_mode == "communication";
    for (auto * widget :
      QList<QWidget *>{scene, px4, gazebo_port, gazebo_gui})
    {
      widget->setEnabled(edit && sim);
    }
    for (auto * widget : QList<QWidget *>{transport, target_system}) {
      widget->setEnabled(edit && comm);
    }
    bind_host->setEnabled(edit && comm && transport->currentData().toString() == "udp");
    bind_port->setEnabled(bind_host->isEnabled());
    remote_host->setEnabled(
      edit && comm &&
      transport->currentData().toString() != "serial");
    remote_port->setEnabled(remote_host->isEnabled());
    device->setEnabled(edit && comm && transport->currentData().toString() == "serial");
    baud->setEnabled(device->isEnabled());
    const auto row_visible = [this](QWidget * field, bool visible) {
        field->setVisible(visible);
        if (auto * label = fcu_form->labelForField(field)) {label->setVisible(visible);}
      };
    const bool serial = transport->currentData().toString() == "serial";
    for (auto * field : QList<QWidget *>{bind_host, bind_port}) {
      row_visible(field, !serial && transport->currentData().toString() == "udp");
    }
    for (auto * field : QList<QWidget *>{remote_host, remote_port}) {row_visible(field, !serial);}
    for (auto * field : QList<QWidget *>{device, baud, refresh_serial}) {
      row_visible(field, serial);
    }
    choose_px4->setEnabled(edit && sim);
    choose_qgc->setEnabled(edit);
    qgc_path->setEnabled(edit);
    refresh_serial->setEnabled(edit && comm && serial);
    gcs_enabled->setEnabled(edit && (sim || comm));
    for (auto * widget :
      QList<QWidget *>{gcs_host, gcs_port, gcs_bind_port})
    {
      widget->setEnabled(edit && gcs_enabled->isChecked());
    }
    mission_control->setEnabled(edit && sim && !gcs_enabled->isChecked());
    mission_import->setEnabled(edit && !gcs_enabled->isChecked());
    parameter_edit->setEnabled(
      mission_import->isEnabled());
    scope->setEnabled(edit && !sim); simulation->setEnabled(edit && !sim);
    start->setText(
      sim ? QStringLiteral("启动仿真并连接 DDS") : comm ?
      QStringLiteral("启动通信并连接 DDS") : QStringLiteral("连接 DDS / 打开工作窗口"));
    if (actual["backend_running"].toBool()) {
      start->setText(QStringLiteral("重开生效工作窗口"));
    } else if (actual["unresolved"].toBool()) {
      start->setText(QStringLiteral("重开原窗口 / 核对未决"));
    }
    urls_label->setText(
      selected_mode == "observe" ? QStringLiteral(
        "已有平台：只连接 DDS，不启动 MAVR"
        "OS 或仿真") :
      QStringLiteral("飞控：%1\nQGC 转发：%2").arg(
        form_profile().fcu_url(),
        gcs_enabled->isChecked() ? form_profile().gcs_url() : QStringLiteral(
          "关闭")));
    save->setEnabled(edit); remove->setEnabled(edit);
    bool valid = false;
    try {
      form_profile().validate(); valid = true;
    } catch (const std::exception & error) {start->setToolTip(error.what());}
    const auto scene_info = scene->currentData(Qt::UserRole + 1).toJsonObject();
    scene_preview->setVisible(sim); preview_caption->setVisible(sim);
    const auto shapes = scene_info["preview"].toArray();
    const auto key = scene->currentData().toString() + ":" +
      QString::number(qHash(QJsonDocument(shapes).toJson(QJsonDocument::Compact))) + ":" +
      QString::number(scene_preview->width()) + ":" + QString::number(scene_preview->height());
    if (preview_key != key) {
      preview_key = key;
      if (shapes.isEmpty()) {
        scene_preview->setPixmap({}); scene_preview->setText(QStringLiteral("暂无几何预览"));
      } else {
        QPixmap image(210, 140); image.fill(QColor("#edf3fc")); QPainter painter(&image);
        painter.setRenderHint(QPainter::Antialiasing);
        double extent_x = 1, extent_y = 1;
        for (const auto & item : shapes) {
          const auto shape = item.toObject();
          extent_x = std::max(
            extent_x, std::abs(
              shape["x"].toDouble()) + shape["width"].toDouble() / 2);
          extent_y = std::max(
            extent_y, std::abs(
              shape["y"].toDouble()) + shape["height"].toDouble() / 2);
        }
        const auto scale = std::min(96 / extent_x, 64 / extent_y);
        painter.translate(105, 70); painter.scale(scale, -scale);
        for (const auto & item : shapes) {
          const auto shape = item.toObject(); painter.save();
          painter.translate(shape["x"].toDouble(), shape["y"].toDouble());
          painter.rotate(shape["yaw"].toDouble() * 180 / 3.141592653589793);
          painter.setPen(Qt::NoPen);
          painter.setBrush(QColor(shape["ground"].toBool() ? "#dbe4f1" : "#527bb5"));
          painter.drawRect(
            QRectF(
              -shape["width"].toDouble() / 2, -shape["height"].toDouble() / 2,
              shape["width"].toDouble(), shape["height"].toDouble())); painter.restore();
        }
        painter.end(); scene_preview->setPixmap(
          image.scaled(
            scene_preview->size().boundedTo(QSize(210, 140)), Qt::KeepAspectRatio,
            Qt::SmoothTransformation));
      }
      preview_caption->setText(
        QStringLiteral("%1\n静态基本几何俯视 · 非实时画面；模型与网格省略")
        .arg(scene_info["name"].toString(scene->currentData().toString())));
      preview_caption->setToolTip(preview_caption->text());
    }
    if (sim && !scene_info.isEmpty()) {
      const bool supported = scene_info["status"] == "registered" &&
        scene_info["profiles"].toArray().contains(profile->currentData().toString());
      valid = valid && supported;
      scene_note->setText(
        supported ? QStringLiteral(
          "场景与运行配置匹配；启动前再次"
          "校验当前资源与许可。") :
        QStringLiteral(
          "此场景未登记或不支持当前运行配"
          "置，请选择匹配的已登记场景。"));
    }
    const bool persisted = healthy && !dirty() && selected >= 0 &&
      persisted_hashes.value(profiles[selected].id) ==
      profiles[selected].hash();
    start->setEnabled(
      queried && rpc.connected() && !actual["process_running"].toBool() &&
      !actual["recovery_blocked"].toBool() && !backend_stopping &&
      !actual["close_pending"].toBool() &&
      (actual["backend_running"].toBool() ?
      actual["backend_fresh"].toBool() && !actual["profile"].toObject().isEmpty() :
      actual["unresolved"].toBool() ? !actual["profile"].toObject().isEmpty() :
      valid && healthy && selected >= 0));
    if (!persisted && !actual["backend_running"].toBool()) {
      start->setText(
        sim ? QStringLiteral("保存并启动仿真 / DDS") : comm ?
        QStringLiteral("保存并启动通信 / DDS") : QStringLiteral("保存并连接 DDS"));
    }
    const auto backend = actual["backend"].toObject();
    stop_backend->setEnabled(
      queried && actual["backend_running"].toBool() &&
      !actual["recovery_blocked"].toBool() &&
      actual["backend_fresh"].toBool() && !backend_stopping &&
      !actual["process_running"].toBool() && !actual["qgc_running"].toBool() &&
      (backend["safe_to_stop"].toBool() ||
      backend["can_abort_startup"].toBool()) &&
      !actual["unresolved"].toBool());
    const auto active_config = actual["profile"].toObject();
    start_qgc->setEnabled(
      queried && actual["backend_running"].toBool() &&
      actual["backend_fresh"].toBool() && backend["safe_to_stop"].toBool() && !backend_stopping &&
      active_config["gcs"].toObject()["enabled"].toBool() &&
      !actual["qgc_running"].toBool());
    stop_qgc->setEnabled(queried && actual["qgc_running"].toBool());
    start_qgc->setToolTip(
      QStringLiteral(
        "在 QGC 转发页启用转发并选择程序"
        "；后端确认未解锁、在地面后可打"
        "开。"));
    stop_backend->setToolTip(
      QStringLiteral(
        "先正常关闭本次 QGC 和工作窗口，"
        "再请求停止；已解锁、离地或未知"
        "状态会拒绝。"));
    const auto workspace = actual["workspace"].toObject();
    stop->setEnabled(
      queried && actual["process_running"].toBool() &&
      actual["workspace_fresh"].toBool() && !workspace["unresolved"].toBool(
        true));
    QString state = QStringLiteral("管理服务连接中");
    if (queried) {
      if (!actual["process_running"].toBool()) {
        state = actual["phase"] == "failed" ? QStringLiteral("工作会话失败") :
          QStringLiteral("未连接观测");
      } else if (!actual["workspace_fresh"].toBool()) {
        state = QStringLiteral("工作进程运行，观测状态未知");
      } else {state = workspace["summary"].toString(QStringLiteral("等待工作快照"));}
    }
    if (actual["backend_running"].toBool() || !backend.isEmpty()) {
      state += QStringLiteral("\n仿真/通信：") +
        (backend["error"].toString().isEmpty() ? backend["message"].toString() :
        backend["error"].toString()).left(600);
      if (actual["backend_running"].toBool() &&
        !actual["process_running"].toBool())
      {
        state += QStringLiteral(" · 后台保留，可重开工作窗口或正常停止");
      }
      if (!backend["recovery"].toString().isEmpty()) {
        state += QStringLiteral("\n处理建议：") + backend["recovery"].toString();
      }
    }
    if (!actual["recovery_error"].toString().isEmpty()) {
      state += QStringLiteral("\n恢复诊断：") + actual["recovery_error"].toString();
    }
    if (actual["recovered_backend"].toBool() && actual["backend_fresh"].toBool()) {
      state += QStringLiteral("\n原后台身份已认证，会话与生效配置保持不变。");
    }
    if (active_config["gcs"].toObject()["enabled"].toBool()) {
      const auto endpoint = backend["gcs"].toObject();
      const auto values = endpoint["values"].toObject();
      state += QStringLiteral(
        "\nQGC：%1 · 端点发送 %2 B / 接收 %3 B"
        "（接收不等于界面显示确认）")
        .arg(
        actual["qgc_running"].toBool() ? QStringLiteral(
          "本次进程运行") : QStringLiteral("本次进程未运行"),
        values["Tx total bytes"].toString(
          "未知"), values["Rx total bytes"].toString("未知"));
    }
    if (actual["phase"] == "starting_backend") {
      state = backend["message"].toString(
        QStringLiteral(
          "仿真/通信预检查与启动中，请稍"
          "候…"));
    }
    if (!actual["qgc_message"].toString().isEmpty()) {
      state += "\n" + actual["qgc_message"].toString();
    }
    summary->setText(state); summary->setToolTip(state);
    summary->setVisible(
      details_toggle->isChecked() || backend_fault || actual["phase"] == "failed" ||
      actual["phase"] == "starting_backend" || !backend["error"].toString().isEmpty() ||
      (actual["process_running"].toBool() && !actual["workspace_fresh"].toBool()));
    effective->setVisible(
      details_toggle->isChecked() || actual["backend_running"].toBool() ||
      actual["process_running"].toBool());
    const auto active = actual["profile"].toObject();
    effective->setText(
      active.isEmpty() ? QStringLiteral("当前无生效连接") :
      QStringLiteral("生效连接：%1 · domain %2 · %3 · QGC 转发：%4\n生效版本：%5")
      .arg(active["name"].toString()).arg(active["domain"].toInt())
      .arg(
        active["simulation_time"].toBool() ? QStringLiteral(
          "模拟时间") : QStringLiteral("系统时间"))
      .arg(
        active["gcs"].toObject()["enabled"].toBool() ? QStringLiteral("启用（双向）") :
        QStringLiteral("关闭"))
      .arg(actual["config_hash"].toString().left(12)));
    const auto text = QJsonDocument(actual).toJson(QJsonDocument::Indented);
    if (diagnostics->toPlainText() != QString::fromUtf8(text)) {
      diagnostics->setPlainText(QString::fromUtf8(text));
    }
  }
};
ManagerWindow::ManagerWindow(
  const QString & file, const QString & runtime,
  QWidget * parent)
: QMainWindow(parent)
{
  if (parent) {setWindowFlags(Qt::Widget);}
  impl_ = std::make_unique<Impl>(this, file, runtime);
}
ManagerWindow::~ManagerWindow() = default;
void ManagerWindow::resizeEvent(QResizeEvent * event)
{
  QMainWindow::resizeEvent(event);
  if (impl_) {impl_->arrange_commands(event->size().width());}
}
}  // namespace easyfly_connection_manager

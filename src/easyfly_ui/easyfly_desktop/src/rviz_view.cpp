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
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDebug>
#include <QFile>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHideEvent>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QToolButton>
#include <QGridLayout>
#include <QTimer>
#include <QVBoxLayout>
#include <QVariant>
#include <QXmlStreamReader>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <functional>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include "easyfly_desktop/rviz_view.hpp"
#include "layout_state.hpp"
#include "rviz_layers.hpp"
#ifdef EASYFLY_ENABLE_RVIZ_EMBED
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreMaterialManager.h>
#include <rviz_common/display.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/interaction/selection_manager_iface.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_rendering/render_window.hpp>
#include <rviz_default_plugins/displays/path/path_display.hpp>
#include "easyfly_rviz_plugins/odometry_display.hpp"
#include <rviz_default_plugins/displays/odometry/odometry_display.hpp>
#endif
namespace easyfly_desktop
{
namespace
{
#ifdef EASYFLY_ENABLE_RVIZ_EMBED
void check_versions()
{
  for (const auto * package : {"rviz_common", "rviz_rendering", "rviz_default_plugins"}) {
    QFile file(QString::fromStdString(
        ament_index_cpp::get_package_share_directory(package)) + "/package.xml");
    if (!file.open(QIODevice::ReadOnly) || file.size() > 65536) {
      throw std::runtime_error("Cannot read RViz package version");
    }
    QXmlStreamReader xml(file.readAll()); QString version;
    while (!xml.atEnd()) {
      xml.readNext();
      if (xml.isStartElement() && xml.name() == "version") {version = xml.readElementText();}
    }
    if (xml.hasError() || version != "11.2.26") {
      throw std::runtime_error("RViz embedding requires tested version 11.2.26");
    }
  }
}
// Humble's VisualizationManager creates its executor on the default context.
// Do not reuse/shutdown a foreign context; Observer/control use separate contexts.
struct Runtime
{
  std::shared_ptr<rclcpp::Context> context;
  std::unique_ptr<pluginlib::ClassLoader<rviz_common::Display>> plugins;
  Runtime()
  {
    context = rclcpp::contexts::get_global_default_context();
    if (context->is_valid()) {throw std::runtime_error("Foreign default ROS context is active");}
    context->init(0, nullptr);
    try {
      // Keep the fixed plugin library loaded until node callback groups have
      // been destroyed, not merely until the manager's factories disappear.
      plugins = std::make_unique<pluginlib::ClassLoader<rviz_common::Display>>(
        "rviz_common", "rviz_common::Display");
      for (const auto * display : {"Grid", "Odometry", "Path", "TF"}) {
        plugins->loadLibraryForClass(std::string("rviz_default_plugins/") + display);
      }
    } catch (...) {context->shutdown("RViz plugin startup failed"); throw;}
  }
  ~Runtime()
  {
    plugins.reset(); context->shutdown("EasyFly embedded RViz resources released");
  }
};
std::shared_ptr<Runtime> runtime()
{
  static std::weak_ptr<Runtime> current;
  auto result = current.lock();
  if (result) {throw std::runtime_error("Only one embedded RViz surface is supported");}
  result = std::make_shared<Runtime>(); current = result;
  return result;
}
class ReadOnlyNode : public rviz_common::ros_integration::RosNodeAbstractionIface
{
public:
  explicit ReadOnlyNode(bool simulation)
  {
    static uint64_t sequence = 0;
    rclcpp::NodeOptions options;
    options.use_global_arguments(false).enable_rosout(false).start_parameter_services(false).
    start_parameter_event_publisher(false);
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", simulation)});
    node_ = std::make_shared<rclcpp::Node>(
      "easyfly_ui_rviz_" + std::to_string(++sequence), options);
  }
  std::string get_node_name() const override {return node_->get_name();}
  std::map<std::string, std::vector<std::string>> get_topic_names_and_types() const override
  {return node_->get_topic_names_and_types();}
  rclcpp::Node::SharedPtr get_raw_node() override {return node_;}

private:
  rclcpp::Node::SharedPtr node_;
};
// Version-pinned adapter: deliberately never call base initialize(), which creates
// SetGoal/SetInitialPose publishers. Only change tool initialization, not updates.
class ReadOnlyManager : public rviz_common::VisualizationManager
{
public:
  using rviz_common::VisualizationManager::VisualizationManager;
  void initialize_read_only()
  {
    const auto views = view_manager_->getDeclaredClassIdsFromFactory();
    if (!views.contains("rviz_default_plugins/Orbit") ||
      !views.contains("rviz_default_plugins/TopDownOrtho"))
    {
      throw std::runtime_error("Missing read-only RViz view plugin");
    }
    view_manager_->initialize(); selection_manager_->initialize(); view_picker_->initialize();
    auto * camera = tool_manager_->addTool("rviz_default_plugins/MoveCamera");
    tool_manager_->setDefaultTool(camera); tool_manager_->setCurrentTool(camera);
    last_update_ros_time_ = clock_->now();
    last_update_wall_time_ = std::chrono::system_clock::now();
    fps_property_->setValue(10); background_color_property_->setColor(QColor(255, 255, 255));
  }
  ~ReadOnlyManager() override
  {
    stopUpdate();
    // 11.2.26's base destructor does not own/delete ViewManager. Release its
    // controllers while RenderPanel and scene still exist (before base cleanup).
    delete view_manager_; view_manager_ = nullptr;
  }
};
void set(rviz_common::properties::Property * parent, const QString & name, const QVariant & value)
{
  auto * p = parent->subProp(name);
  if (!p || p->getName() != name) {throw std::runtime_error("Missing RViz display property");}
  p->setValue(value);
}
void status_text(rviz_common::properties::Property * p, QStringList & output, int depth = 0)
{
  if (depth > 5 || output.size() >= 24) {return;}
  if (dynamic_cast<rviz_common::properties::StatusProperty *>(p)) {
    output.push_back(p->getName() + ": " + p->getValue().toString().left(512));
  }
  for (int i = 0; i < p->numChildren() && output.size() < 24; ++i) {
    status_text(p->childAt(i), output, depth + 1);
  }
}
// Observe headers on the SAME message_filters subscriber: no second DDS reader
// or full-message copy. Humble disables MessageFilter's failure callback, so
// its previous Topic: OK alone cannot report an untransformable new input.
class InputDiagnostics
{
public:
  virtual ~InputDiagnostics() = default;
  virtual int actual_depth() const = 0;
};
template<typename Base, typename Message>
class InputFrameDisplay : public Base, public InputDiagnostics
{
public:
  ~InputFrameDisplay() override {unsubscribe();}
  int actual_depth() const override
  {
    if (!this->subscription_ || !this->subscription_->getSubscriber()) {return -1;}
    return static_cast<int>(this->subscription_->getSubscriber()->get_actual_qos().depth());
  }

protected:
  void subscribe() override
  {
    Base::subscribe();
    if (!this->subscription_) {return;}
    input_ = this->subscription_->registerCallback(
      std::function<void(const typename Message::ConstSharedPtr &)>(
        [this](const typename Message::ConstSharedPtr & message) {
          if (!message) {return;}
          const auto & frame = message->header.frame_id;
          std::string error;
          const bool failed = frame.empty() ||
          (frame != this->fixed_frame_.toStdString() &&
          this->context_->getFrameManager()->transformHasProblems(
            frame, rclcpp::Time(
              message->header.stamp,
              this->context_->getClock()->get_clock_type()), error));
          this->setStatus(
            failed ? rviz_common::properties::StatusProperty::Warn :
            rviz_common::properties::StatusProperty::Ok, QStringLiteral("输入坐标系（非新鲜度）"),
            QStringLiteral("%1 → %2 · %3").arg(
              QString::fromStdString(frame).left(256),
              this->fixed_frame_, failed ? QStringLiteral("TF 未就绪 / ") +
              QString::fromStdString(error).left(512) : QStringLiteral("本次输入可转换")));
        }));
  }
  void unsubscribe() override {input_.disconnect(); Base::unsubscribe();}

private:
  message_filters::Connection input_;
};
struct Surface
{
  bool owns_colors{false};
  std::set<Ogre::ResourceHandle> previous_materials;
  std::shared_ptr<Runtime> owner;
  std::shared_ptr<ReadOnlyNode> node;
  rviz_common::RenderPanel * panel{nullptr};
  std::unique_ptr<ReadOnlyManager> manager;
  std::map<QString, rviz_common::Display *> layers;
  Surface(QWidget * parent, bool simulation, int preset)
  {
    check_versions(); owner = runtime(); node = std::make_shared<ReadOnlyNode>(simulation);
    panel = new rviz_common::RenderPanel(parent);
    try {
      // Establish the final native Qt parent chain before Ogre binds the XID.
      // Otherwise showing a lazily added container can recreate its child XID.
      panel->setAttribute(Qt::WA_NativeWindow); (void)panel->winId(); panel->show();
      panel->getRenderWindow()->initialize();
      for (const auto & name : color_names()) {
        if (Ogre::MaterialManager::getSingleton().resourceExists(name, "rviz_rendering")) {
          throw std::runtime_error("Foreign RViz color resources are active");
        }
      }
      auto materials = Ogre::MaterialManager::getSingleton().getResourceIterator();
      while (materials.hasMoreElements()) {
        previous_materials.insert(materials.getNext()->getHandle());
      }
      owns_colors = true;
      manager = std::make_unique<ReadOnlyManager>(
        panel, node, nullptr,
        node->get_raw_node()->get_clock());
      panel->initialize(manager.get()); manager->initialize_read_only();
      manager->setFixedFrame(preset == 0 ? "map" : "odom");
      auto * grid = add("Grid", "网格", true); set(grid, "Color", QColor(190, 199, 212));
      set(grid, "Cell Size", 1.0); set(grid, "Plane Cell Count", 20);
      auto * pose = add("Odometry", "位姿", false);
      configure_topic(pose, preset == 0 ? "/mavros/local_position/odom" : "/odom");
      set(pose, "Keep", 1); set(pose, "Position Tolerance", 0.0);
      set(pose, "Angle Tolerance", 0.0); set(pose, "Shape", "Axes");
      set(pose->subProp("Shape"), "Axes Length", 0.8);
      set(pose->subProp("Shape"), "Axes Radius", 0.04);
      set(pose, "Covariance", false); pose->setEnabled(true);
      auto * aircraft = aircraft_display(preset == 0 ? "/mavros/local_position/odom" : "/odom");
      aircraft->setClassId("easyfly_desktop/ReadOnlyAircraft");
      manager->addDisplay(aircraft, false);
      aircraft->setName(QStringLiteral("完整机模"));
      layers.emplace(QStringLiteral("完整机模"), aircraft);
      if (preset != 0) {
        add_spatial(SpatialKind::Actual, "实际轨迹", "/path", true);
        add_spatial(SpatialKind::Planned, "规划显示线", "/optimal_list", false);
        add_spatial(SpatialKind::Cloud, "SLAM点云", "/cloud_registered", false);
        add_spatial(SpatialKind::Cloud, "过滤点云", "/pointcloud_filtered", false);
        add_spatial(SpatialKind::Map, "占据格中心", "/occupancy_grid", false);
        add_spatial(SpatialKind::Obstacles, "几何障碍框", "/obstacles", false);
        add_spatial(SpatialKind::Landing, "局部降落板法向", "/landing/guidance_target", false);
        add_spatial(SpatialKind::Predictions, "模型预测十字", "/target_predictions", false);
        add_spatial(SpatialKind::Classified, "类别副本几何", "/obstacles_classified", false);
      }
      add("TF", "TF", false);
      manager->getViewManager()->getCurrent()->reset(); manager->startUpdate();
    } catch (const std::exception & e) {
      std::fprintf(stderr, "RViz surface construction failed: %s\n", e.what());
      release(); throw;
    }
  }
  void add_spatial(SpatialKind kind, const QString & name, const QString & topic, bool enabled)
  {
    auto * display = spatial_display(kind, topic);
    display->setClassId("easyfly_desktop/ReadOnlyGeometry");
    manager->addDisplay(display, enabled); display->setName(name); layers.emplace(name, display);
  }
  rviz_common::Display * add(const QString & type, const QString & name, bool enabled)
  {
    rviz_common::Display * display = nullptr;
    if (type == "Path") {
      display = new InputFrameDisplay<rviz_default_plugins::displays::PathDisplay,
          nav_msgs::msg::Path>();
    } else if (type == "Odometry") {
      display = new InputFrameDisplay<easyfly_rviz_plugins::OdometryDisplay,
          nav_msgs::msg::Odometry>();
    }
    const QString class_id = type == "Odometry" ? "easyfly_rviz_plugins/Odometry" :
      "rviz_default_plugins/" + type;
    if (display) {
      display->setClassId(class_id); manager->addDisplay(display, enabled);
      display->setName(name);
    } else {display = manager->createDisplay("rviz_default_plugins/" + type, name, enabled);}
    if (!display || display->getClassId() != class_id) {
      throw std::runtime_error("RViz display plugin unavailable");
    }
    layers.emplace(name, display); return display;
  }
  void configure_topic(rviz_common::Display * display, const QString & topic)
  {
    set(display, "Topic", topic);
    auto * p = display->subProp("Topic"); set(p, "Depth", 1);
    set(p, "Filter size", 1);
    set(p, "Reliability Policy", "Best Effort"); set(p, "Durability Policy", "Volatile");
  }
  void release()
  {
    auto * scene = panel ? rviz_rendering::RenderWindowOgreAdapter::getSceneManager(
      panel->getRenderWindow()) : nullptr;
    if (manager) {manager->stopUpdate();}
    layers.clear(); manager.reset(); delete panel; panel = nullptr; node.reset();
    if (scene) {Ogre::Root::getSingletonPtr()->destroySceneManager(scene);}
    // 11.2.26 also leaves Path's LinesMaterial in the registry. Remove only
    // newly created exact handles in this exclusive surface's resource group,
    // after every display, node and scene has been destroyed. No name globbing.
    if (owns_colors) {
      std::vector<Ogre::ResourceHandle> added;
      auto materials = Ogre::MaterialManager::getSingleton().getResourceIterator();
      while (materials.hasMoreElements()) {
        const auto resource = materials.getNext();
        if (resource->getGroup() == "rviz_rendering" &&
          !previous_materials.count(resource->getHandle()))
        {
          added.push_back(resource->getHandle());
        }
      }
      for (auto handle : added) {Ogre::MaterialManager::getSingleton().remove(handle);}
      owns_colors = false;
    }
  }
  static std::vector<std::string> color_names()
  {
    return {"RVIZ/Red", "RVIZ/Green", "RVIZ/Blue", "RVIZ/Cyan", "RVIZ/ShadedRed",
      "RVIZ/ShadedGreen", "RVIZ/ShadedBlue", "RVIZ/ShadedCyan"};
  }
  ~Surface() {release();}
  RvizSnapshot snapshot() const
  {
    RvizSnapshot result; result.running = true; result.frames = manager->getFrameCount();
    result.node = QString::fromStdString(node->get_node_name());
    result.fixed_frame = manager->getFixedFrame();
    for (const auto & layer : layers) {
      if (const auto * spatial = dynamic_cast<const SpatialDiagnostics *>(layer.second)) {
        result.layers.push_back(spatial->layer_snapshot());
        if (layer.second->isEnabled()) {result.input_depth = result.layers.back().input_depth;}
      }
      if (const auto * input = dynamic_cast<const InputDiagnostics *>(layer.second)) {
        result.input_depth = input->actual_depth();
      }
    }
    for (int i = 0; i < manager->getToolManager()->numTools(); ++i) {
      result.tools.push_back(manager->getToolManager()->getTool(i)->getClassId());
    }
    const auto raw = node->get_raw_node();
    result.ros_time_ns = raw->now().nanoseconds();
    for (const auto & p : raw->get_node_graph_interface()->get_publisher_names_and_types_by_node(
        raw->get_name(), raw->get_namespace()))
    {
      result.publishers.push_back(QString::fromStdString(p.first));
    }
    QStringList status; status_text(manager->getRootDisplayGroup(), status);
    for (const auto & layer : result.layers) {
      status.push_back(
        QStringLiteral(
          "%1 · %2 · %3\n源frame=%4 · 源龄=%5s / 收"
          "龄=%6s · 替换=%7").arg(
          layer.name, layer.topic, layer.status, layer.source_frame).arg(
          layer.source_age, 0, 'f', 2).arg(layer.receipt_age, 0, 'f', 2).arg(layer.replaced));
    }
    result.status = status.join("\n"); return result;
  }
};
#endif
}  // namespace
struct RvizView::Impl
{
  RvizView * widget;
  bool connected{false}, simulation{true};
  QString error;
  uint64_t lifecycle_sequence{0};
  QString lifecycle_reason, last_node;
  QComboBox * preset;
  QPushButton * start;
  QWidget * canvas;
  QVBoxLayout * canvas_layout;
  QLabel * placeholder;
  QPlainTextEdit * status;
  std::map<QString, QCheckBox *> layers;
  QComboBox * view;
  QTimer * timer;
#ifdef EASYFLY_ENABLE_RVIZ_EMBED
  std::unique_ptr<Surface> surface;
#endif
  explicit Impl(RvizView * parent)
  : widget(parent)
  {
    auto * layout = new QVBoxLayout(widget); layout->setContentsMargins(0, 0, 0, 0);
    auto * toolbar = new QHBoxLayout; layout->addLayout(toolbar);
    auto * splitter = new QSplitter(Qt::Horizontal, widget);
    splitter->setObjectName("embeddedRvizSplitter"); splitter->setChildrenCollapsible(false);
    layout->addWidget(splitter, 1);
    auto * scroll = new QScrollArea(splitter);
    scroll->setObjectName("embeddedRvizControlsScroll"); scroll->setWidgetResizable(true);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setMinimumWidth(240); scroll->setMaximumWidth(320);
    auto * controls = new QWidget;
    controls->setObjectName("embeddedRvizControls");
    auto * controls_layout = new QVBoxLayout(controls);
    controls_layout->setContentsMargins(8, 4, 8, 4); controls_layout->setSpacing(6);
    preset = new QComboBox(widget); preset->setObjectName("embeddedRvizPreset");
    preset->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    preset->setMinimumContentsLength(12);
    preset->addItems(
      {QStringLiteral("基础飞行 · map / MAVROS 位姿"),
        QStringLiteral("导航观测 · odom / 机位 /path")}); toolbar->addWidget(preset, 1);
    start = new QPushButton(QStringLiteral("打开只读三维"), widget);
    start->setObjectName("startEmbeddedRviz"); toolbar->addWidget(start);
    auto * stop = new QPushButton(QStringLiteral("释放三维"), widget);
    stop->setObjectName("stopEmbeddedRviz"); toolbar->addWidget(stop);
    auto * focus = new QToolButton(widget); focus->setText(QStringLiteral("专注观察"));
    focus->setObjectName("embeddedRvizFocusMode"); focus->setCheckable(true);
    toolbar->addWidget(focus);
    QObject::connect(
      focus, &QToolButton::toggled, widget, [scroll, focus](bool on) {
        scroll->setVisible(!on);
        focus->setText(on ? QStringLiteral("恢复面板") : QStringLiteral("专注观察"));
      });
    auto * layer_group = new QGroupBox(QStringLiteral("显示图层"), controls);
    auto * layer_layout = new QGridLayout(layer_group); layer_layout->setVerticalSpacing(4);
    int layer_index = 0;
    auto * warning = new QLabel(
      QStringLiteral(
        "实验性只读三维 · 原始话题的最后"
        "收到几何，不代表新鲜状态或可飞许"
        "可。\n"
        "机模使用 base_link 机体位姿和原始时刻传感器 TF；0.5 秒过期清图，旋翼不表示实际转速。\n"
        "网格不是地图。基础预设为 MAVROS 位姿；导航预设为 /odom 机位。\n"
        "导航几何仅按已有 TF 转换；同名 map "
        "不表示同一原点，缺少 TF 不猜测。\n"
        "导航图层：蓝=实际轨迹、橙=规划显"
        "示、绿=点云、紫=占据格中心、红=几"
        "何障碍。\n"
        "导航几何源龄/收龄超过2秒清图；规"
        "划不是已接受轨迹，抽样不代表完整"
        "覆盖。\n"
        "专项图层0.5秒清图：绿线=板中心法"
        "向，青十字=模型预测，紫框=类别副"
        "本几何（类别见感知表）。\n"
        "只操作视角，不发布飞行命令；进程"
        "内插件/显卡故障可能影响整个 UI。"),
      controls);
    warning->setWordWrap(true); warning->setTextFormat(Qt::PlainText);
    for (const auto & name : {QStringLiteral("网格"), QStringLiteral("位姿"),
        QStringLiteral("完整机模"),
        QStringLiteral("实际轨迹"),
        QStringLiteral("规划显示线"),
        QStringLiteral("SLAM点云"),
        QStringLiteral("过滤点云"),
        QStringLiteral("占据格中心"),
        QStringLiteral("几何障碍框"),
        QStringLiteral("局部降落板法向"),
        QStringLiteral("模型预测十字"),
        QStringLiteral("类别副本几何"),
        QStringLiteral("TF")})
    {
      auto * box = new QCheckBox(name, layer_group);
      box->setObjectName("embeddedRvizLayer_" + name);
      box->setChecked(
        name == QStringLiteral("网格") || name == QStringLiteral("位姿") ||
        name == QStringLiteral("实际轨迹"));
      layers.emplace(name, box);
      if (layer_index < 8) {
        layer_layout->addWidget(box, layer_index / 2, layer_index % 2);
      } else {layer_layout->addWidget(box, layer_index - 4, 0, 1, 2);}
      ++layer_index;
      QObject::connect(
        box, &QCheckBox::toggled, widget, [this, name](bool checked) {
#ifdef EASYFLY_ENABLE_RVIZ_EMBED
          if (surface && surface->layers.count(name)) {
            surface->layers.at(name)->setEnabled(checked);
          }
#else
          (void)name; (void)checked;
#endif
        });
    }
    controls_layout->addWidget(layer_group);
    auto * view_group = new QGroupBox(QStringLiteral("视角"), controls);
    auto * view_layout = new QHBoxLayout(view_group);
    view = new QComboBox(view_group); view->setObjectName("embeddedRvizView");
    view->addItems({QStringLiteral("环绕视角"), QStringLiteral("俯视视角")});
    view_layout->addWidget(view);
    auto * reset = new QPushButton(QStringLiteral("重置视角"), view_group);
    reset->setObjectName("resetEmbeddedRvizView"); view_layout->addWidget(reset);
    controls_layout->addWidget(view_group);
    auto * help = new QToolButton(controls);
    help->setText(QStringLiteral("图层与坐标说明"));
    help->setCheckable(true); warning->hide();
    QObject::connect(help, &QToolButton::toggled, warning, &QWidget::setVisible);
    controls_layout->addWidget(help); controls_layout->addWidget(warning);
    status = new QPlainTextEdit(controls); status->setObjectName("embeddedRvizStatus");
    status->setReadOnly(true); status->setFixedHeight(110);
    status->setLineWrapMode(QPlainTextEdit::WidgetWidth);
    controls_layout->addWidget(status);
    controls_layout->addStretch();
    scroll->setWidget(controls);
    canvas = new QWidget(splitter); canvas->setObjectName("embeddedRvizCanvasHost");
    canvas->setMinimumSize(320, 240);
    canvas_layout = new QVBoxLayout(canvas); canvas_layout->setContentsMargins(0, 0, 0, 0);
    placeholder = new QLabel(
      QStringLiteral(
        "三维画布\n选择上方预设，手动打开"
        "只读三维。"),
      canvas);
    placeholder->setObjectName("embeddedRvizPlaceholder");
    placeholder->setAlignment(Qt::AlignCenter);
    placeholder->setWordWrap(true); placeholder->setTextFormat(Qt::PlainText);
    canvas_layout->addWidget(placeholder, 1);
    splitter->addWidget(canvas); splitter->addWidget(scroll);
    splitter->setStretchFactor(0, 1); splitter->setStretchFactor(1, 0);
    splitter->setSizes({900, 260}); remember_splitter(splitter, "rviz");
    QObject::connect(start, &QPushButton::clicked, widget, [this] {open();});
    QObject::connect(stop, &QPushButton::clicked, widget, [this] {close();});
    QObject::connect(
      view, qOverload<int>(&QComboBox::currentIndexChanged), widget, [this] {
        change_view();
      });
    QObject::connect(
      reset, &QPushButton::clicked, widget, [this] {
        reset_view();
      });
    timer = new QTimer(widget); timer->setInterval(500);
    QObject::connect(timer, &QTimer::timeout, widget, [this] {update();});
    timer->start();
    update();
  }
  void change_view()
  {
#ifdef EASYFLY_ENABLE_RVIZ_EMBED
    if (surface) {
      surface->manager->getViewManager()->setCurrentViewControllerType(
        view->currentIndex() == 0 ?
        "rviz_default_plugins/Orbit" : "rviz_default_plugins/TopDownOrtho");
      reset_view();
    }
#endif
  }
  void reset_view()
  {
#ifdef EASYFLY_ENABLE_RVIZ_EMBED
    if (surface) {
      auto * current = surface->manager->getViewManager()->getCurrent(); current->reset();
      if (view->currentIndex() == 0) {
        set(current, "Distance", 12.0); set(current, "Pitch", 0.7); set(current, "Yaw", 0.7);
      } else {set(current, "Scale", 25.0);}
    }
#endif
  }
  void open()
  {
    if (!connected || snapshot().running || !RvizView::available()) {return;}
    error.clear();
#ifdef EASYFLY_ENABLE_RVIZ_EMBED
    try {
      if (QApplication::platformName() != "xcb") {
        throw std::runtime_error("Experimental RViz embedding requires X11/xcb");
      }
      placeholder->hide();
      surface = std::make_unique<Surface>(canvas, simulation, preset->currentIndex());
      surface->panel->setObjectName("embeddedRvizRenderPanel");
      surface->panel->setMinimumSize(320, 240); canvas_layout->addWidget(surface->panel, 1);
      surface->panel->show(); change_view();
      for (const auto & layer : surface->layers) {
        layer.second->setEnabled(layers.at(layer.first)->isChecked());
      }
      last_node = surface->snapshot().node; record_lifecycle("explicit_open");
    } catch (const std::exception & e) {
      surface.reset(); error = QString::fromUtf8(e.what()).left(2048);
      record_lifecycle("open_failed");
    }
#endif
    update();
  }
  void record_lifecycle(const QString & reason)
  {
    lifecycle_reason = reason; ++lifecycle_sequence;
    const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    qInfo().noquote() << "easyfly_rviz_lifecycle sequence=" << lifecycle_sequence <<
      "pid=" << QCoreApplication::applicationPid() << "steady_ns=" << now <<
      "node=" << last_node << "reason=" << reason << "simulation=" << simulation;
  }
  void close(const QString & reason = "explicit_close")
  {
#ifdef EASYFLY_ENABLE_RVIZ_EMBED
    if (surface) {record_lifecycle(reason);}
    surface.reset();
#else
    (void)reason;
#endif
    update();
  }
  RvizSnapshot snapshot() const
  {
    RvizSnapshot result;
#ifdef EASYFLY_ENABLE_RVIZ_EMBED
    if (surface) {result = surface->snapshot();}
#endif
    if (!result.running) {result.status = error;}
    result.lifecycle_sequence = lifecycle_sequence;
    result.lifecycle_reason = lifecycle_reason; result.last_node = last_node;
    return result;
  }
  void update()
  {
    const auto s = snapshot(); preset->setEnabled(!s.running);
    placeholder->setVisible(!s.running);
    start->setEnabled(connected && RvizView::available() && !s.running);
    for (const auto & layer : layers) {
      bool supported = false;
#ifdef EASYFLY_ENABLE_RVIZ_EMBED
      supported = surface && surface->layers.count(layer.first);
#endif
      layer.second->setEnabled(s.running && supported);
    }
    view->setEnabled(s.running);
    if (!s.running) {
      status->setPlainText(
        !error.isEmpty() ? QStringLiteral("内嵌失败，保留独立 RViz / 二维视图：") +
        error : !RvizView::available() ? QStringLiteral(
          "此构建未启用 RViz 内嵌；使用独立 RV"
          "iz。") :
        !connected ? QStringLiteral(
          "观测已断开；三维资源已释放，不自"
          "动重开。") :
        QStringLiteral("尚未初始化渲染/ROS；由你手动打开。"));
    } else {
      status->setPlainText(
        QStringLiteral("%1 · frame=%2 · 更新计数=%3（非实测 FPS）\n%4").arg(
          s.node, s.fixed_frame).arg(s.frames).arg(s.status));
    }
  }
};
RvizView::RvizView(QWidget * parent)
: QWidget(parent), impl_(std::make_unique<Impl>(this)) {}
RvizView::~RvizView() {impl_->close("destroyed");}
bool RvizView::available()
{
#ifdef EASYFLY_ENABLE_RVIZ_EMBED
  return true;
#else
  return false;
#endif
}
void RvizView::set_connection(bool connected, bool use_sim_time)
{
  if (impl_->connected == connected && impl_->simulation == use_sim_time) {return;}
  if (!connected || impl_->simulation != use_sim_time) {
    impl_->close(!connected ? "disconnected" : "clock_mode_changed");
  }
  impl_->connected = connected; impl_->simulation = use_sim_time; impl_->update();
}
void RvizView::stop() {impl_->connected = false; impl_->close("stopped");}
RvizSnapshot RvizView::snapshot() const {return impl_->snapshot();}
void RvizView::hideEvent(QHideEvent * event)
{
  impl_->close("hidden"); QWidget::hideEvent(event);  // hidden pages retain no render loop
}
}  // namespace easyfly_desktop

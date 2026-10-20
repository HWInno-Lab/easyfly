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

#include <gtest/gtest.h>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QFile>
#include <QListWidget>
#include <QLabel>
#include <QPlainTextEdit>
#include <QProcess>
#include <QProcessEnvironment>
#include <QPushButton>
#include <QScreen>
#include <QScrollArea>
#include <QSplitter>
#include <QTest>
#include <QTemporaryDir>
#include <QTabWidget>
#include <QToolButton>
#include <QVBoxLayout>

#include <chrono>
#include <memory>
#include <string>
#include <cstring>
#include <thread>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include "easyfly_desktop/rviz_geometry.hpp"

#include "easyfly_desktop/rviz_view.hpp"
#include "support.hpp"
#ifdef EASYFLY_ENABLE_RVIZ_EMBED
#include <OgreMaterialManager.h>
#include <OgreRoot.h>
#endif
namespace ui = easyfly_desktop;
namespace spatial = easyfly_desktop::spatial;
namespace
{
template<typename Predicate>
bool wait(Predicate predicate, int milliseconds = 4000)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
  do {
    QCoreApplication::processEvents(); if (predicate()) {return true;} QTest::qWait(20);
  } while (std::chrono::steady_clock::now() < deadline);
  return predicate();
}
void click(ui::RvizView & view, const char * name)
{view.findChild<QPushButton *>(name)->click();}
void expect_side_by_side(ui::RvizView & view)
{
  auto * splitter = view.findChild<QSplitter *>("embeddedRvizSplitter");
  auto * scroll = view.findChild<QScrollArea *>("embeddedRvizControlsScroll");
  auto * controls = view.findChild<QWidget *>("embeddedRvizControls");
  auto * canvas = view.findChild<QWidget *>("embeddedRvizCanvasHost");
  ASSERT_NE(splitter, nullptr); ASSERT_NE(scroll, nullptr);
  ASSERT_NE(controls, nullptr); ASSERT_NE(canvas, nullptr);
  EXPECT_EQ(splitter->orientation(), Qt::Horizontal);
  EXPECT_FALSE(splitter->childrenCollapsible());
  const QRect left(canvas->mapToGlobal(QPoint()), canvas->size());
  const QRect right(scroll->mapToGlobal(QPoint()), scroll->size());
  EXPECT_LT(left.right(), right.left()) << "view=" << view.width() <<
    " minimum=" << view.minimumSizeHint().width() << " left=" << left.width() <<
    " canvas=" << right.width(); EXPECT_GE(left.width(), 320);
  EXPECT_GE(left.height(), 240);
  for (const auto * child : view.findChildren<QWidget *>()) {
    if (qobject_cast<const QPushButton *>(child) || qobject_cast<const QComboBox *>(child) ||
      qobject_cast<const QCheckBox *>(child) || qobject_cast<const QPlainTextEdit *>(child))
    {
      EXPECT_FALSE(canvas->isAncestorOf(child));
    }
  }
  if (auto * panel = view.findChild<QWidget *>("embeddedRvizRenderPanel")) {
    EXPECT_EQ(panel->parentWidget(), canvas);
    EXPECT_FALSE(right.intersects(QRect(panel->mapToGlobal(QPoint()), panel->size())));
    EXPECT_EQ(panel->geometry(), canvas->rect());
  }
}
class PrefixOverride
{
public:
  explicit PrefixOverride(const QString & prefix)
  : previous_(qgetenv("AMENT_PREFIX_PATH"))
  {qputenv("AMENT_PREFIX_PATH", prefix.toUtf8() + ":" + previous_);}
  ~PrefixOverride() {qputenv("AMENT_PREFIX_PATH", previous_);}

private:
  QByteArray previous_;
};
bool file(const QString & path, const QByteArray & content)
{
  QFile output(path);
  return output.open(QIODevice::WriteOnly | QIODevice::NewOnly) &&
         output.write(content) == content.size();
}
}  // namespace
TEST(RvizView, LazyDisconnectedAndUnsupportedPlatformFailClosed)
{
  ui::RvizView view; view.show();
  EXPECT_FALSE(view.snapshot().running);
  EXPECT_FALSE(view.findChild<QPushButton *>("startEmbeddedRviz")->isEnabled());
  view.set_connection(true, false); EXPECT_FALSE(view.snapshot().running);
  if (QApplication::platformName() != "xcb") {
    click(view, "startEmbeddedRviz"); EXPECT_FALSE(view.snapshot().running);
    if (ui::RvizView::available()) {EXPECT_TRUE(view.snapshot().status.contains("X11"));}
  }
  view.stop(); EXPECT_FALSE(view.snapshot().running);
}
TEST(RvizView, StableSpacePageIsAvailableWithoutLegacyOptInAndRemainsLazy)
{
  auto state = std::make_shared<desktop_test::FakeState>();
  state->data = desktop_test::sample();
  ui::MainWindow default_window({}, desktop_test::factory(state));
  EXPECT_EQ(default_window.findChild<ui::RvizView *>() != nullptr, ui::RvizView::available());
  EXPECT_EQ(default_window.findChild<QListWidget *>("navigation")->count(), 7);
  default_window.close(); ASSERT_TRUE(wait([&] {return default_window.cleanup_complete();}));
  ui::MainWindow window({}, desktop_test::factory(state), nullptr, ui::default_tool_options(),
    {}, ui::control_factory(), {}, ui::parameter_factory(), {}, ui::import_factory(), {true});
  if (!ui::RvizView::available()) {
    EXPECT_NE(window.findChild<QLabel *>("rvizBuildUnavailable"), nullptr);
    window.close(); EXPECT_TRUE(wait([&] {return window.cleanup_complete();})); return;
  }
  window.show(); auto * view = window.findChild<ui::RvizView *>(); ASSERT_NE(view, nullptr);
  EXPECT_FALSE(view->snapshot().running);
  EXPECT_EQ(window.findChild<QListWidget *>("navigation")->count(), 7);
  auto * tabs = window.findChild<QTabWidget *>("overviewMapTabs"); ASSERT_NE(tabs, nullptr);
  tabs->setCurrentIndex(1);
  EXPECT_FALSE(tabs->isAncestorOf(view)); EXPECT_FALSE(view->snapshot().running);
  EXPECT_EQ(window.findChildren<ui::RvizView *>().size(), 1);
  window.show_page("space"); window.findChild<QTabWidget *>("spaceViewTabs")->setCurrentIndex(1);
  EXPECT_FALSE(tabs->isAncestorOf(view)); EXPECT_FALSE(view->snapshot().running);
  window.close(); EXPECT_TRUE(wait([&] {return window.cleanup_complete();}));
}
TEST(RvizView, CommandLineRequiresSupportedBuildAndNeverAutoStartsRenderer)
{
  QProcess process; auto environment = QProcessEnvironment::systemEnvironment();
  environment.insert("QT_QPA_PLATFORM", "offscreen"); process.setProcessEnvironment(environment);
  process.start(
    EASYFLY_RVIZ_DESKTOP_BINARY,
    {"--enable-rviz-embed", "--system-time", "--front-topic", "", "--down-topic", "",
      "--quit-after-ms", "200"});
  ASSERT_TRUE(process.waitForStarted(3000)); ASSERT_TRUE(process.waitForFinished(5000));
  EXPECT_EQ(process.exitStatus(), QProcess::NormalExit);
  EXPECT_EQ(process.exitCode(), ui::RvizView::available() ? 0 : 2);
  EXPECT_FALSE(
    (process.readAllStandardError() + process.readAllStandardOutput()).contains(
      "Available Renderers"));
}
TEST(RvizView, CanvasAndRightControlsAtSupportedWindowSizes)
{
  if (!ui::RvizView::available()) {GTEST_SKIP() << "RViz is not included in this build";}
  auto state = std::make_shared<desktop_test::FakeState>(); state->data = desktop_test::sample();
  ui::MainWindow window({}, desktop_test::factory(state), nullptr, ui::default_tool_options(),
    {}, ui::control_factory(), {}, ui::parameter_factory(), {}, ui::import_factory(), {true});
  window.show(); window.show_page("space");
  window.findChild<QTabWidget *>("spaceViewTabs")->setCurrentIndex(1);
  auto * view = window.findChild<ui::RvizView *>(); ASSERT_NE(view, nullptr);
  for (const auto & size : {QSize(1024, 720), QSize(1366, 768), QSize(1440, 900)}) {
    window.resize(size); QTest::qWait(60); expect_side_by_side(*view);
    const auto width = view->findChild<QWidget *>("embeddedRvizCanvasHost")->width();
    auto * focus = view->findChild<QToolButton *>("embeddedRvizFocusMode");
    focus->click(); QTest::qWait(20);
    EXPECT_GT(view->findChild<QWidget *>("embeddedRvizCanvasHost")->width(), width);
    focus->click(); QTest::qWait(20); expect_side_by_side(*view);
    EXPECT_FALSE(view->snapshot().running);
    EXPECT_TRUE(view->findChild<QWidget *>("embeddedRvizPlaceholder")->isVisible());
  }
  window.close(); EXPECT_TRUE(wait([&] {return window.cleanup_complete();}));
}
#ifdef EASYFLY_ENABLE_RVIZ_EMBED
class RvizRender : public ::testing::Test
{
protected:
  std::shared_ptr<rclcpp::Context> context;
  std::shared_ptr<rclcpp::Node> node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_odom;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr planned;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map;
  rclcpp::Publisher<easyfly_msgs::msg::ObstacleInfoArray>::SharedPtr boxes;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr transforms;
  sensor_msgs::msg::PointCloud2 cloud_message;
  void spatial_publishers()
  {
    planned = node->create_publisher<visualization_msgs::msg::Marker>("/optimal_list", 1);
    cloud = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 1);
    filtered = node->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud_filtered", 1);
    map = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/occupancy_grid", rclcpp::QoS(1).reliable().transient_local());
    boxes = node->create_publisher<easyfly_msgs::msg::ObstacleInfoArray>(
      "/obstacles", rclcpp::QoS(1).reliable().transient_local());
    transforms = node->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(1).reliable().transient_local());
    cloud_message.height = 1; cloud_message.width = 30000; cloud_message.point_step = 12;
    cloud_message.row_step = cloud_message.width * 12;
    cloud_message.data.resize(cloud_message.row_step);
    for (unsigned i = 0; i < 3; ++i) {
      sensor_msgs::msg::PointField field;
      field.name = std::vector<std::string>{"x", "y", "z"}[i]; field.offset = i * 4;
      field.count = 1; field.datatype = field.FLOAT32; cloud_message.fields.push_back(field);
    }
    for (unsigned i = 0; i < cloud_message.width; ++i) {
      const float xyz[3] = {
        static_cast<float>(i % 200) / 50, static_cast<float>(i / 200) / 50, 0.1F};
      std::memcpy(cloud_message.data.data() + i * 12, xyz, 12);
    }
  }
  void send_spatial(
    int second = 10, const std::string & source = "odom",
    bool invalid_cloud = false)
  {
    send(true, second, source);
    std_msgs::msg::Header header; header.frame_id = source; header.stamp.sec = second;
    visualization_msgs::msg::Marker line; line.header = header;
    line.type = line.LINE_STRIP; line.pose.orientation.w = 1;
    for (int i = 0; i < 4; ++i) {
      geometry_msgs::msg::Point p; p.x = i; p.y = 1; p.z = 1.8; line.points.push_back(p);
    }
    planned->publish(line); cloud_message.header = header;
    if (invalid_cloud) {cloud_message.fields[0].offset = 100;}
    cloud->publish(cloud_message); filtered->publish(cloud_message);
    cloud_message.fields[0].offset = 0;
    nav_msgs::msg::OccupancyGrid grid; grid.header = header;
    grid.info.width = grid.info.height = 2; grid.info.resolution = 0.5;
    grid.info.origin.orientation.w = 1; grid.data = {-1, 0, 50, 100}; map->publish(grid);
    easyfly_msgs::msg::ObstacleInfoArray array; array.header = header;
    easyfly_msgs::msg::ObstacleInfo box; box.header = header;
    box.center_x = 2; box.center_y = 2; box.center_z = 1;
    box.size_x = box.size_y = box.size_z = 1; array.obstacles.push_back(box); boxes->publish(array);
  }
  ui::RvizLayerSnapshot layer(ui::RvizView & view, const QString & name)
  {
    for (const auto & value : view.snapshot().layers) {if (value.name == name) {return value;}}
    ADD_FAILURE() << "Missing layer: " << name.toStdString(); return {};
  }
  void all_layers(ui::RvizView & view)
  {
    for (auto * box : view.findChildren<QCheckBox *>()) {
      if (box->text() != "TF" && box->isEnabled()) {box->setChecked(true);}
    }
  }
  void SetUp() override
  {
    if (QApplication::platformName() != "xcb") {GTEST_SKIP() << "Requires actual X11 rendering";}
    context = std::make_shared<rclcpp::Context>(); context->init(0, nullptr);
    rclcpp::NodeOptions options; options.context(context).use_global_arguments(false);
    node = std::make_shared<rclcpp::Node>("easyfly_rviz_fixture", options);
    rclcpp::ExecutorOptions eo; eo.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);
    executor->add_node(node);
    odom = node->create_publisher<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", rclcpp::SensorDataQoS());
    local_odom = node->create_publisher<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS());
    path = node->create_publisher<nav_msgs::msg::Path>("/path", rclcpp::SensorDataQoS());
    clock = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
  }
  void TearDown() override
  {
    if (executor) {executor->remove_node(node); executor.reset();}
    planned.reset(); cloud.reset(); filtered.reset(); map.reset(); boxes.reset();
    transforms.reset();
    odom.reset(); local_odom.reset(); path.reset(); clock.reset(); node.reset();
    if (context) {context->shutdown("fixture complete"); context.reset();}
  }
  void send(bool navigation = false, int second = 10, const std::string & frame = "")
  {
    rosgraph_msgs::msg::Clock time; time.clock.sec = second; clock->publish(time);
    nav_msgs::msg::Odometry pose; pose.header.frame_id = frame.empty() ? "map" : frame;
    pose.header.stamp.sec = second; pose.pose.pose.orientation.w = 1;
    pose.pose.pose.position.x = 1; pose.pose.pose.position.z = 1.5;
    if (!navigation) {odom->publish(pose);} else {
      if (frame.empty()) {pose.header.frame_id = "odom";}
      local_odom->publish(pose);
      nav_msgs::msg::Path trajectory; trajectory.header = pose.header;
      if (frame.empty()) {trajectory.header.frame_id = "odom";}
      for (int i = 0; i < 5; ++i) {
        geometry_msgs::msg::PoseStamped p; p.header = trajectory.header;
        p.pose.orientation.w = 1; p.pose.position.x = i; p.pose.position.z = 1.5;
        trajectory.poses.push_back(p);
      }
      path->publish(trajectory);
    }
    executor->spin_some();
  }
};
TEST_F(RvizRender, PoseOnlyCameraZeroPublishersAndIndependentContext)
{
  ui::RvizView view; view.resize(1050, 740); view.show(); view.set_connection(true, false);
  click(view, "startEmbeddedRviz"); ASSERT_TRUE(view.snapshot().running) <<
    view.snapshot().status.toStdString();
  ASSERT_TRUE(wait([&] {send(); return view.snapshot().status.contains("messages received");}));
  EXPECT_EQ(view.snapshot().fixed_frame, "map");
  EXPECT_EQ(view.snapshot().tools, QStringList{"rviz_default_plugins/MoveCamera"});
  EXPECT_TRUE(view.snapshot().publishers.empty()); EXPECT_GT(view.snapshot().frames, 0U);
  unsigned matched = 0;
  for (const auto & subscriber :
    node->get_subscriptions_info_by_topic("/mavros/local_position/odom"))
  {
    if (subscriber.node_name() != view.snapshot().node.toStdString()) {continue;}
    ++matched;
    const auto & qos = subscriber.qos_profile().get_rmw_qos_profile();
    EXPECT_EQ(qos.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    EXPECT_EQ(qos.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }
  EXPECT_EQ(matched, 1U);
  // Fast DDS graph discovery reports depth0 (unknown). Inspect the local DDS
  // subscription's actual QoS, not the network-discovery placeholder.
  EXPECT_EQ(view.snapshot().input_depth, 1);
  view.findChild<QComboBox *>("embeddedRvizView")->setCurrentIndex(1);
  click(view, "resetEmbeddedRvizView"); QTest::qWait(200);
  view.findChild<QComboBox *>("embeddedRvizView")->setCurrentIndex(0);
  QTest::qWait(200);
  QDir directory(EASYFLY_RVIZ_SCREENSHOT_DIRECTORY); ASSERT_TRUE(directory.mkpath("."));
  ASSERT_TRUE(
    QApplication::primaryScreen()->grabWindow(view.winId()).save(
      directory.filePath("layout-prototype-pose.png")));
  view.set_connection(false, false); EXPECT_FALSE(view.snapshot().running);
  EXPECT_TRUE(context->is_valid());
  EXPECT_EQ(node->count_subscribers("/mavros/local_position/odom"), 0U);
}
TEST_F(RvizRender, RepeatedOpenCloseAndLayerSwitches)
{
  ui::RvizView view; view.resize(1024, 720); view.show(); view.set_connection(true, true);
  size_t initial_materials = 0;
  for (int i = 0; i < 20; ++i) {
    view.findChild<QComboBox *>("embeddedRvizPreset")->setCurrentIndex(i % 2);
    click(view, "startEmbeddedRviz"); ASSERT_TRUE(view.snapshot().running) <<
      view.snapshot().status.toStdString();
    ASSERT_TRUE(
      wait(
        [&] {
          send(i % 2); return view.snapshot().status.contains("messages received");
        }));
    if (i == 0 || i == 19) {
      auto * focus = view.findChild<QToolButton *>("embeddedRvizFocusMode");
      const auto frames = view.snapshot().frames; focus->click();
      ASSERT_TRUE(wait([&] {send(i % 2); return view.snapshot().frames > frames;}));
      EXPECT_TRUE(view.snapshot().running); focus->click(); QTest::qWait(30);
      expect_side_by_side(view);
    }
    for (auto * layer : view.findChildren<QCheckBox *>()) {
      if (layer->isEnabled()) {layer->setChecked(false); layer->setChecked(true);}
    }
    EXPECT_TRUE(view.snapshot().publishers.empty()); click(view, "stopEmbeddedRviz");
    EXPECT_FALSE(view.snapshot().running); EXPECT_TRUE(context->is_valid());
    size_t count = 0; auto resources = Ogre::MaterialManager::getSingleton().getResourceIterator();
    while (resources.hasMoreElements()) {resources.getNext(); ++count;}
    if (i == 0) {initial_materials = count;} else {EXPECT_EQ(count, initial_materials);}
    EXPECT_TRUE(Ogre::Root::getSingletonPtr()->getSceneManagers().empty());
    EXPECT_TRUE(
      wait(
        [&] {
          return node->count_subscribers("/mavros/local_position/odom") == 0 &&
          node->count_subscribers("/path") == 0 && node->count_subscribers("/odom") == 0;
        }));
  }
}
TEST_F(RvizRender, NavigationPoseUsesCanonicalOdometryWithoutRelabeling)
{
  ui::RvizView view; view.resize(1050, 740); view.show(); view.set_connection(true, true);
  view.findChild<QComboBox *>("embeddedRvizPreset")->setCurrentIndex(1);
  click(view, "startEmbeddedRviz"); ASSERT_TRUE(view.snapshot().running);
  ASSERT_TRUE(wait([&] {send(true); return view.snapshot().status.contains("odom → odom");}));
  EXPECT_EQ(view.snapshot().fixed_frame, "odom");
  EXPECT_TRUE(view.findChild<QCheckBox *>("embeddedRvizLayer_位姿")->isEnabled());
  EXPECT_EQ(node->count_subscribers("/mavros/local_position/odom"), 0U);
  EXPECT_EQ(node->count_subscribers("/odom"), 1U);
  EXPECT_EQ(view.snapshot().input_depth, 1);
  EXPECT_TRUE(view.snapshot().publishers.empty());
  ASSERT_TRUE(
    wait(
      [&] {
        send(true, 11, "missing_frame");
        return view.snapshot().status.contains("missing_frame → odom · TF 未就绪");
      }));
  view.stop(); EXPECT_TRUE(context->is_valid());
  EXPECT_TRUE(wait([&] {return node->count_subscribers("/odom") == 0;}));
}
TEST_F(RvizRender, ForeignContextAndSecondSurfaceRefusedWithoutDisturbingOwner)
{
  const auto foreign = rclcpp::contexts::get_global_default_context(); foreign->init(0, nullptr);
  ui::RvizView view; view.show(); view.set_connection(true, false);
  click(view, "startEmbeddedRviz"); EXPECT_FALSE(view.snapshot().running);
  EXPECT_TRUE(view.snapshot().status.contains("Foreign default")); EXPECT_TRUE(foreign->is_valid());
  foreign->shutdown("foreign fixture complete"); click(view, "startEmbeddedRviz");
  ASSERT_TRUE(view.snapshot().running) << view.snapshot().status.toStdString();
  ui::RvizView second; second.show(); second.set_connection(true, false);
  click(second, "startEmbeddedRviz"); EXPECT_FALSE(second.snapshot().running);
  EXPECT_TRUE(second.snapshot().status.contains("Only one")); EXPECT_TRUE(view.snapshot().running);
  view.hide(); EXPECT_FALSE(view.snapshot().running); EXPECT_TRUE(context->is_valid());
}
TEST_F(RvizRender, MissingPluginFailsClosedAndExplicitRetryAfterRepair)
{
  QTemporaryDir prefix; ASSERT_TRUE(prefix.isValid());
  const auto directory = prefix.path() +
    "/share/ament_index/resource_index/rviz_common__pluginlib__plugin";
  ASSERT_TRUE(QDir().mkpath(directory));
  ASSERT_TRUE(file(directory + "/rviz_default_plugins", "share/missing-plugin.xml\n"));
  ui::RvizView view; view.show(); view.set_connection(true, false);
  {
    PrefixOverride changed(prefix.path()); click(view, "startEmbeddedRviz");
    EXPECT_FALSE(view.snapshot().running); EXPECT_FALSE(view.snapshot().status.isEmpty());
    EXPECT_FALSE(rclcpp::contexts::get_global_default_context()->is_valid());
  }
  EXPECT_FALSE(view.snapshot().running); click(view, "startEmbeddedRviz");
  ASSERT_TRUE(view.snapshot().running) << view.snapshot().status.toStdString();
  EXPECT_TRUE(view.snapshot().publishers.empty()); view.stop(); EXPECT_TRUE(context->is_valid());
}
TEST_F(RvizRender, VersionMismatchFailsBeforeRosAndRenderInitialization)
{
  QTemporaryDir prefix; ASSERT_TRUE(prefix.isValid());
  const auto markers = prefix.path() + "/share/ament_index/resource_index/packages";
  ASSERT_TRUE(QDir().mkpath(markers)); ASSERT_TRUE(file(markers + "/rviz_common", ""));
  ASSERT_TRUE(QDir().mkpath(prefix.path() + "/share/rviz_common"));
  ASSERT_TRUE(
    file(
      prefix.path() + "/share/rviz_common/package.xml",
      "<package><version>0.0.0</version></package>"));
  PrefixOverride changed(prefix.path()); ui::RvizView view; view.show(); view.set_connection(
    true,
    false);
  click(view, "startEmbeddedRviz"); EXPECT_FALSE(view.snapshot().running);
  EXPECT_TRUE(view.snapshot().status.contains("11.2.26"));
  EXPECT_FALSE(rclcpp::contexts::get_global_default_context()->is_valid());
  EXPECT_TRUE(context->is_valid());
}
TEST_F(RvizRender, IntegratedPageSwitchDisconnectReconnectAndClose)
{
  auto state = std::make_shared<desktop_test::FakeState>(); state->data = desktop_test::sample();
  ui::MainWindow window({}, desktop_test::factory(state), nullptr, ui::default_tool_options(),
    {}, ui::control_factory(), {}, ui::parameter_factory(), {}, ui::import_factory(), {true});
  window.resize(1024, 720); window.show();
  window.show_page("space"); window.findChild<QTabWidget *>("spaceViewTabs")->setCurrentIndex(1);
  auto * view = window.findChild<ui::RvizView *>(); ASSERT_NE(
    view,
    nullptr);
  ASSERT_TRUE(wait([&] {return view->findChild<QPushButton *>("startEmbeddedRviz")->isEnabled();}));
  click(*view, "startEmbeddedRviz"); ASSERT_TRUE(view->snapshot().running);
  ASSERT_TRUE(wait([&] {send(); return view->snapshot().status.contains("messages received");}));
  window.show_page("overview"); EXPECT_FALSE(view->snapshot().running);
  auto * overview_tabs = window.findChild<QTabWidget *>("overviewMapTabs");
  overview_tabs->setCurrentIndex(1); EXPECT_FALSE(view->snapshot().running);
  ASSERT_FALSE(overview_tabs->isAncestorOf(view));
  window.show_page("space"); window.findChild<QTabWidget *>("spaceViewTabs")->setCurrentIndex(1);
  ASSERT_TRUE(wait([&] {return view->findChild<QPushButton *>("startEmbeddedRviz")->isEnabled();}));
  click(*view, "startEmbeddedRviz"); ASSERT_TRUE(view->snapshot().running);
  ASSERT_TRUE(wait([&] {send(); return view->snapshot().frames >= 3;}));
  EXPECT_TRUE(view->snapshot().publishers.empty());
  EXPECT_EQ(window.findChildren<ui::RvizView *>().size(), 1);
  window.resize(1440, 900); QTest::qWait(100); expect_side_by_side(*view);
  auto * mark = new QLabel(QStringLiteral("构造观测验收 · 无飞机 / PX4 / 飞行控制"), &window);
  static_cast<QVBoxLayout *>(window.centralWidget()->layout())->insertWidget(0, mark);
  const QDir overview_output(QStringLiteral(EASYFLY_RVIZ_SCREENSHOT_DIRECTORY) +
    "/../../overview-refresh");
  ASSERT_TRUE(overview_output.mkpath(".")); QTest::qWait(50);
  ASSERT_TRUE(
    QApplication::primaryScreen()->grabWindow(window.winId()).save(
      overview_output.filePath("overview-rviz.png")));
  mark->hide(); window.resize(1024, 720);
  window.show_page("overview");
  overview_tabs->setCurrentIndex(0); EXPECT_FALSE(view->snapshot().running);
  window.show_page("space"); window.findChild<QTabWidget *>("spaceViewTabs")->setCurrentIndex(1);
  EXPECT_FALSE(view->snapshot().running);
  click(*view, "startEmbeddedRviz"); ASSERT_TRUE(view->snapshot().running);
  window.findChild<QPushButton *>("observerToggle")->click();
  EXPECT_TRUE(wait([&] {return !view->snapshot().running && state->stop_calls > 0;}));
  ASSERT_TRUE(wait([&] {return window.findChild<QPushButton *>("observerToggle")->isEnabled();}));
  window.findChild<QPushButton *>("observerToggle")->click();
  ASSERT_TRUE(wait([&] {return view->findChild<QPushButton *>("startEmbeddedRviz")->isEnabled();}));
  EXPECT_FALSE(view->snapshot().running); click(*view, "startEmbeddedRviz");
  ASSERT_TRUE(view->snapshot().running);
  ASSERT_TRUE(wait([&] {send(); return view->snapshot().status.contains("messages received");}));
  ASSERT_TRUE(wait([&] {send(); return view->snapshot().frames >= 3;}));
  ASSERT_TRUE(
    wait(
      [&] {
        return !view->findChild<QPlainTextEdit *>("embeddedRvizStatus")->
        toPlainText().contains(QStringLiteral("更新计数=0"));
      }));
  QDir directory(EASYFLY_RVIZ_SCREENSHOT_DIRECTORY); ASSERT_TRUE(directory.mkpath("."));
  ASSERT_TRUE(
    QApplication::primaryScreen()->grabWindow(window.winId()).save(
      directory.filePath("layout-integrated-1024.png")));
  expect_side_by_side(*view);
  EXPECT_FALSE(view->findChild<QWidget *>("embeddedRvizPlaceholder")->isVisible());
  auto * splitter = view->findChild<QSplitter *>("embeddedRvizSplitter");
  const auto before = view->snapshot().frames;
  splitter->setSizes({330, 400}); QTest::qWait(120); expect_side_by_side(*view);
  window.resize(1440, 900); splitter->setSizes({280, 900});
  ASSERT_TRUE(wait([&] {send(); return view->snapshot().frames > before;}));
  QTest::qWait(120); expect_side_by_side(*view);
  EXPECT_TRUE(view->snapshot().publishers.empty());
  ASSERT_TRUE(
    QApplication::primaryScreen()->grabWindow(window.winId()).save(
      directory.filePath("layout-integrated-1440.png")));
  window.close(); EXPECT_TRUE(wait([&] {return window.cleanup_complete();}));
  EXPECT_FALSE(view->snapshot().running); EXPECT_TRUE(context->is_valid());
}
TEST_F(RvizRender, SpatialLayersDefaultOffExplicitSubscriptionsAndBoundedGeometry)
{
  spatial_publishers(); ui::RvizView view; view.resize(1250, 850); view.show();
  view.set_connection(true, true);
  view.findChild<QComboBox *>("embeddedRvizPreset")->setCurrentIndex(1);
  click(view, "startEmbeddedRviz"); ASSERT_TRUE(view.snapshot().running);
  ASSERT_EQ(view.snapshot().layers.size(), 6U);
  for (const auto & s : view.snapshot().layers) {
    EXPECT_EQ(s.enabled, s.name == QStringLiteral("实际轨迹"));
    if (!s.enabled) {EXPECT_EQ(node->count_subscribers(s.topic.toStdString()), 0U);}
  }
  all_layers(view);
  EXPECT_TRUE(layer(view, "规划显示线").status.contains(QStringLiteral("无数据")));
  ASSERT_TRUE(
    wait(
      [&] {
        send_spatial();
        for (const auto & s : view.snapshot().layers) {
          if (!s.visible) {
            return false;
          }
        }
        return true;
      }));
  for (const auto & s : view.snapshot().layers) {
    EXPECT_EQ(node->count_subscribers(s.topic.toStdString()), 1U);
    EXPECT_GE(s.received, 1U); EXPECT_LE(s.vertices, spatial::kCloudPoints);
    EXPECT_EQ(s.input_depth, 1);
    for (const auto & info : node->get_subscriptions_info_by_topic(s.topic.toStdString())) {
      if (info.node_name() == view.snapshot().node.toStdString()) {
        EXPECT_EQ(info.qos_profile().reliability(), rclcpp::ReliabilityPolicy::BestEffort);
        EXPECT_EQ(info.qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile);
      }
    }
  }
  EXPECT_EQ(layer(view, "几何障碍框").vertices, 24U);
  EXPECT_EQ(layer(view, "占据格中心").vertices, 2U);
  EXPECT_TRUE(view.snapshot().publishers.empty()); expect_side_by_side(view);
  ASSERT_TRUE(wait([&] {send_spatial(); return view.snapshot().frames >= 5;}));
  QTest::qWait(120);
  QDir directory(EASYFLY_RVIZ_SCREENSHOT_DIRECTORY); ASSERT_TRUE(directory.mkpath("."));
  ASSERT_TRUE(
    QApplication::primaryScreen()->grabWindow(view.winId()).save(
      directory.filePath("layers-navigation.png")));
  visualization_msgs::msg::Marker deletion; deletion.header.frame_id = "odom";
  deletion.header.stamp.sec = 10; deletion.action = deletion.DELETEALL;
  planned->publish(deletion);
  ASSERT_TRUE(
    wait([&] {return layer(view, "规划显示线").status.contains(QStringLiteral("删除"));}));
  EXPECT_FALSE(layer(view, "规划显示线").visible);
  ASSERT_TRUE(wait([&] {send_spatial(11); return layer(view, "规划显示线").visible;}));
  auto * checkbox = view.findChild<QCheckBox *>("embeddedRvizLayer_过滤点云");
  checkbox->setChecked(false); EXPECT_FALSE(layer(view, "过滤点云").visible);
  EXPECT_TRUE(wait([&] {return node->count_subscribers("/pointcloud_filtered") == 0;}));
  view.stop(); EXPECT_TRUE(context->is_valid());
  for (const auto * topic : {"/path", "/optimal_list", "/cloud_registered",
      "/pointcloud_filtered", "/occupancy_grid", "/obstacles"})
  {
    EXPECT_TRUE(
      wait(
        [&] {
          return node->count_subscribers(topic) == 0;
        }));
  }
}
TEST_F(RvizRender, SpatialSourceFramesNeedRealTfAndRecoverWithoutInventedTransforms)
{
  spatial_publishers(); ui::RvizView view; view.resize(1250, 850); view.show();
  view.set_connection(true, true);
  view.findChild<QComboBox *>("embeddedRvizPreset")->setCurrentIndex(1);
  click(view, "startEmbeddedRviz"); ASSERT_TRUE(view.snapshot().running); all_layers(view);
  ASSERT_TRUE(
    wait(
      [&] {
        send_spatial(10, "camera_init");
        return layer(view, "SLAM点云").status.contains(QStringLiteral("TF 未就绪"));
      }));
  EXPECT_FALSE(layer(view, "SLAM点云").visible);
  EXPECT_FALSE(layer(view, "实际轨迹").visible); EXPECT_TRUE(view.snapshot().publishers.empty());
  tf2_msgs::msg::TFMessage tf;
  geometry_msgs::msg::TransformStamped transform; transform.header.frame_id = "odom";
  transform.child_frame_id = "camera_init"; transform.header.stamp.sec = 10;
  transform.transform.translation.x = 3; transform.transform.rotation.w = 1;
  tf.transforms.push_back(transform); transforms->publish(tf);
  ASSERT_TRUE(wait([&] {send_spatial(11, "camera_init"); return layer(view, "SLAM点云").visible;}));
  EXPECT_EQ(layer(view, "SLAM点云").source_frame, "camera_init");
  ASSERT_TRUE(
    wait(
      [&] {
        send_spatial(12, "world");
        return layer(view, "规划显示线").status.contains(QStringLiteral("TF 未就绪"));
      }));
  EXPECT_FALSE(layer(view, "规划显示线").visible);
  EXPECT_TRUE(view.snapshot().publishers.empty()); view.stop();
}
TEST_F(RvizRender, SpatialDuplicatePauseInvalidFutureAndClockResetClearGeometry)
{
  spatial_publishers(); ui::RvizView view; view.resize(1100, 800); view.show();
  view.set_connection(true, true);
  view.findChild<QComboBox *>("embeddedRvizPreset")->setCurrentIndex(1);
  click(view, "startEmbeddedRviz"); ASSERT_TRUE(view.snapshot().running); all_layers(view);
  ASSERT_TRUE(wait([&] {send_spatial(); return layer(view, "SLAM点云").visible;}));
  ASSERT_TRUE(
    wait(
      [&] {
        send_spatial(); return layer(view, "SLAM点云").status.contains(QStringLiteral("接收超时"));
      }, 3500));
  EXPECT_FALSE(layer(view, "SLAM点云").visible);
  EXPECT_GT(layer(view, "SLAM点云").receipt_age, 2);
  EXPECT_NEAR(layer(view, "SLAM点云").source_age, 0, 0.01);
  ASSERT_TRUE(wait([&] {send_spatial(11); return layer(view, "SLAM点云").visible;}));
  ASSERT_TRUE(
    wait(
      [&] {
        send_spatial(12, "odom", true);
        return layer(view, "SLAM点云").status.contains(QStringLiteral("XYZ"));
      }));
  EXPECT_FALSE(layer(view, "SLAM点云").visible);
  send_spatial(13);
  ASSERT_TRUE(wait([&] {return layer(view, "SLAM点云").visible;}));
  nav_msgs::msg::OccupancyGrid invalid; invalid.header.frame_id = "odom"; map->publish(invalid);
  ASSERT_TRUE(
    wait([&] {return layer(view, "占据格中心").status.contains(QStringLiteral("零时间戳"));}));
  EXPECT_FALSE(layer(view, "占据格中心").visible);
  auto future = cloud_message; future.header.stamp.sec = 100; cloud->publish(future);
  ASSERT_TRUE(wait([&] {return layer(view, "SLAM点云").status.contains(QStringLiteral("未来"));}));
  EXPECT_FALSE(layer(view, "SLAM点云").visible);
  ASSERT_TRUE(wait([&] {send_spatial(14); return layer(view, "SLAM点云").visible;}));
  rosgraph_msgs::msg::Clock clock_reset; clock_reset.clock.sec = 5; clock->publish(clock_reset);
  ASSERT_TRUE(
    wait(
      [&] {
        return view.snapshot().ros_time_ns == 5000000000LL &&
        !layer(view, "SLAM点云").visible;
      }));
  ASSERT_TRUE(wait([&] {send_spatial(5); return layer(view, "SLAM点云").visible;}));
  EXPECT_TRUE(view.snapshot().publishers.empty()); view.stop();
}
TEST_F(RvizRender, SpatialBurstUsesLatestSlotAndDoesNotGrowGeometryHistory)
{
  spatial_publishers(); ui::RvizView view; view.resize(1100, 800); view.show();
  view.set_connection(true, true);
  view.findChild<QComboBox *>("embeddedRvizPreset")->setCurrentIndex(1);
  click(view, "startEmbeddedRviz"); ASSERT_TRUE(view.snapshot().running); all_layers(view);
  ASSERT_TRUE(wait([&] {send_spatial(); return layer(view, "SLAM点云").visible;}));
  // Withhold GUI event processing while native ROS callbacks receive a burst.
  // No per-message queued Qt signal may retain the cloud history.
  const auto before = layer(view, "SLAM点云").received;
  for (int i = 0; i < 200; ++i) {
    cloud->publish(cloud_message);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_GT(layer(view, "SLAM点云").received, before);  // reception outside the GUI thread
  ASSERT_TRUE(wait([&] {return layer(view, "SLAM点云").replaced > 0;}));
  EXPECT_LE(layer(view, "SLAM点云").vertices, spatial::kCloudPoints);
  EXPECT_TRUE(view.snapshot().publishers.empty());
  view.hide(); EXPECT_FALSE(view.snapshot().running); EXPECT_TRUE(context->is_valid());
}

TEST_F(RvizRender, MissingTransformClockPauseAndBackwardReset)
{
  ui::RvizView view; view.resize(1024, 720); view.show(); view.set_connection(true, true);
  view.findChild<QComboBox *>("embeddedRvizPreset")->setCurrentIndex(1);
  click(view, "startEmbeddedRviz"); ASSERT_TRUE(view.snapshot().running) <<
    view.snapshot().status.toStdString();
  ASSERT_TRUE(
    wait(
      [&] {
        send(true, 10); return view.snapshot().status.contains("messages received");
      }));
  auto before = view.snapshot().frames; QTest::qWait(600);
  EXPECT_GT(view.snapshot().frames, before);  // paused /clock does not hang GUI
  EXPECT_EQ(view.snapshot().ros_time_ns, 10000000000LL);
  ASSERT_TRUE(
    wait(
      [&] {
        send(true, 5); return view.snapshot().ros_time_ns == 5000000000LL;
      }));
  ASSERT_TRUE(
    wait(
      [&] {
        send(true, 5, "missing_transform");
        return view.snapshot().status.contains("missing_transform") &&
        view.snapshot().status.contains(QStringLiteral("TF 未就绪"));
      }));
  EXPECT_TRUE(view.snapshot().running); EXPECT_TRUE(view.snapshot().publishers.empty());
  view.stop(); EXPECT_TRUE(context->is_valid());
  EXPECT_TRUE(wait([&] {return node->count_subscribers("/path") == 0;}));
}
#endif

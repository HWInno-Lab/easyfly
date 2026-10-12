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
#include <QCommandLineParser>
#include <QFont>
#include <QTimer>

#include <csignal>

#include "easyfly_desktop/main_window.hpp"
namespace
{
volatile std::sig_atomic_t exit_requested = 0;
void request_exit(int) {exit_requested = 1;}
}  // namespace
int main(int argc, char ** argv)
{
  QApplication app(argc, argv); app.setApplicationName("EasyFly");
  app.setApplicationVersion("0.1.0");
  app.setFont(QFont("Noto Sans CJK SC", 10));
  QCommandLineParser parser;
  parser.setApplicationDescription(
    QStringLiteral(
      "EasyFly 只读桌面：仅观测"
      "已运行的 ROS 图，不启动后"
      "端或控制飞机。"));
  parser.addHelpOption(); parser.addVersionOption();
  parser.addOption({"system-time", QStringLiteral("显式使用系统时间，默认模拟时钟")});
  parser.addOption(
    {"profile", QStringLiteral("UI 选择标签（不启动后端）"), "name", "navigation_dev"});
  parser.addOption(
    {"front-topic", QStringLiteral("前视图像 topic"), "topic",
      "/camera/color/image_raw"});
  parser.addOption(
    {"down-topic", QStringLiteral(
        "下视图像 topic（空字符串禁用）"), "topic", "/camera_down/color/image_raw"});
  parser.addOption(
    {"quit-after-ms", QStringLiteral(
        "隔离 smoke 检查：N ms 后"
        "请求正常异步关闭；默认不"
        "自动退出"), "N"});
  parser.process(app);
  easyfly_ui_bridge::BridgeConfig config;
  config.use_sim_time = !parser.isSet("system-time"); config.selected_profile = parser.value(
    "profile").toStdString();
  config.image_topics =
  {parser.value("front-topic").toStdString(), parser.value("down-topic").toStdString()};
  int duration = 0;
  if (parser.isSet("quit-after-ms")) {
    bool valid = false; duration = parser.value("quit-after-ms").toInt(&valid);
    if (!valid || duration < 100 || duration > 60000) {parser.showHelp(2);}
  }
  easyfly_desktop::MainWindow window(config); window.show();
  std::signal(SIGINT, request_exit); std::signal(SIGTERM, request_exit);
  QTimer signal_timer;
  QObject::connect(
    &signal_timer, &QTimer::timeout, &window, [&window] {
      if (exit_requested) {
        window.close();
      }
    });
  signal_timer.start(100);
  if (duration) {
    QTimer::singleShot(duration, &window, [&window] {window.close();});
  }
  return app.exec();
}

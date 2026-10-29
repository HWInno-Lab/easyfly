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
#include "easyfly_desktop/playback_window.hpp"
namespace
{
volatile std::sig_atomic_t interrupted = 0;
void interrupt(int) {interrupted = 1;}
}  // namespace
int main(int argc, char ** argv)
{
  QApplication app(argc, argv); app.setApplicationName("EasyFly local playback");
  app.setApplicationVersion("0.1.0"); app.setFont(QFont("Noto Sans CJK SC", 10));
  QCommandLineParser parser;
  parser.setApplicationDescription(
    QStringLiteral("独立本地历史查看；不连接ROS或飞机，无控制入口。"));
  parser.addHelpOption(); parser.addVersionOption();
  parser.addOption({"directory", QStringLiteral("专用记录根目录绝对路径，只读"), "path"});
  parser.addOption({"record", QStringLiteral("已收尾记录UUID，不接受文件路径"), "uuid"});
  parser.addOption({"quit-after-ms", QStringLiteral("smoke检查正常异步退出，100–60000ms"), "N"});
  parser.process(app);
  if (parser.isSet("directory") != parser.isSet("record")) {parser.showHelp(2);}
  int duration = 0;
  if (parser.isSet("quit-after-ms")) {
    bool ok = false; duration = parser.value("quit-after-ms").toInt(&ok);
    if (!ok || duration < 100 || duration > 60000) {parser.showHelp(2);}
  }
  easyfly_desktop::PlaybackWindow window;
  if (parser.isSet("directory")) {window.load(parser.value("directory"), parser.value("record"));}
  window.show(); std::signal(SIGINT, interrupt); std::signal(SIGTERM, interrupt);
  QTimer signal_timer;
  QObject::connect(
    &signal_timer, &QTimer::timeout, &window, [&window] {
      if (interrupted) {interrupted = 0; window.close();}
    }); signal_timer.start(100);
  if (duration) {QTimer::singleShot(duration, &window, [&window] {window.close();});}
  return app.exec();
}

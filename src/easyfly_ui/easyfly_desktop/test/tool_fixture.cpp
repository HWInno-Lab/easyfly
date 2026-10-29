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
#include <QCoreApplication>
#include <QTimer>
#include <QTextStream>
#include <QString>
#include <QUdpSocket>
#include <QHostAddress>
#include <unistd.h>
int main(int argc, char ** argv)
{
  QCoreApplication app(argc, argv);
  QTextStream output(stdout);
  output << "fixture started\n" << app.arguments().join("|") << "\n";
  output << "ROS_DOMAIN_ID=" << qEnvironmentVariable("ROS_DOMAIN_ID") << "\n";
  output.flush();
  const auto mode = qEnvironmentVariable("EASYFLY_TOOL_TEST_MODE");
  if (mode == "isolation") {
    QUdpSocket socket;
    socket.writeDatagram(
      "isolated-test", QHostAddress::LocalHost,
      qEnvironmentVariableIntValue("EASYFLY_TOOL_TEST_PORT"));
    char network[256] = {};
    const auto length = readlink("/proc/self/ns/net", network, sizeof(network) - 1);
    if (length > 0) {output << "network=" << network << "\n"; output.flush();}
    QTimer::singleShot(50, &app, [&] {app.exit(0);});
  }
  if (mode == "exit") {QTimer::singleShot(30, &app, [&] {app.exit(0);});}
  if (mode == "fail") {QTimer::singleShot(30, &app, [&] {app.exit(7);});}
  if (mode == "flood") {output << QString(200000, 'x') << "\nEND\n"; output.flush();}
  return app.exec();
}

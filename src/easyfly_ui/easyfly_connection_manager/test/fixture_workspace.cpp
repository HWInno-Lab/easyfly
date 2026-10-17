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
#include <QJsonDocument>
#include <QJsonObject>
#include <QLocalSocket>
#include <QTimer>
#include <QUuid>

int main(int argc, char ** argv)
{
  QCoreApplication app(argc, argv);
  QLocalSocket socket;
  QByteArray input;
  QObject::connect(
    &socket, &QLocalSocket::readyRead, &app, [&] {
      input += socket.readAll();
      while (input.contains('\n')) {
        const auto end = input.indexOf('\n');
        const auto message = QJsonDocument::fromJson(input.left(end)).object();
        input.remove(0, end + 1);
        if (message["event"] == "close") {app.quit();}
      }
    });
  QTimer timer;
  QObject::connect(
    &timer, &QTimer::timeout, &app, [&] {
      if (socket.state() == QLocalSocket::UnconnectedState) {
        socket.connectToServer(qEnvironmentVariable("EASYFLY_CONNECTION_SOCKET"));
      }
      if (socket.state() != QLocalSocket::ConnectedState) {
        socket.flush();
      }
      const QJsonObject message{{"v", 1}, {"op", "workspace"},
        {"id", QUuid::createUuid().toString(QUuid::WithoutBraces)},
        {"session", qEnvironmentVariable("EASYFLY_CONNECTION_SESSION")},
        {"token", qEnvironmentVariable("EASYFLY_CONNECTION_TOKEN")},
        {"status", QJsonObject{{"summary", "fixture-observation"}, {"unresolved", false},
            {"domain", qEnvironmentVariable("ROS_DOMAIN_ID")},
            {"scope", qEnvironmentVariable("ROS_LOCALHOST_ONLY")},
            {"simulation_time", !app.arguments().contains("--system-time")}}}};
      socket.write(QJsonDocument(message).toJson(QJsonDocument::Compact) + '\n');
    });
  timer.start(20);
  QTimer::singleShot(15000, &app, &QCoreApplication::quit);
  return app.exec();
}

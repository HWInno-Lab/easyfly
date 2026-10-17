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

#include <QJsonDocument>
#include <QUuid>

#include "easyfly_connection_manager/rpc.hpp"

namespace easyfly_connection_manager
{
RpcClient::RpcClient(QObject * parent)
: QObject(parent)
{
  connect(&socket_, &QLocalSocket::connected, this, &RpcClient::ready);
  connect(
    &socket_, &QLocalSocket::disconnected, this, [this] {
      input_.clear(); emit failure(QStringLiteral("连接管理服务已断开；操作结果需重新查询"));
    });
  connect(
    &socket_, &QLocalSocket::errorOccurred, this, [this](QLocalSocket::LocalSocketError) {
      emit failure(socket_.errorString());
    });
  connect(
    &socket_, &QLocalSocket::readyRead, this, [this] {
      input_.append(socket_.readAll());
      if (input_.size() > 65536) {socket_.abort(); return;}
      while (input_.contains('\n')) {
        const auto end = input_.indexOf('\n');
        QJsonParseError error;
        const auto doc = QJsonDocument::fromJson(input_.left(end), &error);
        input_.remove(0, end + 1);
        if (error.error != QJsonParseError::NoError || !doc.isObject()) {socket_.abort(); return;}
        emit reply(doc.object());
      }
    });
}
RpcClient::~RpcClient()
{
  QObject::disconnect(&socket_, nullptr, this, nullptr);
  socket_.abort();
}
void RpcClient::connect_to(const QString & directory)
{
  if (socket_.state() != QLocalSocket::UnconnectedState) {return;}
  input_.clear(); socket_.connectToServer(directory + "/control.sock");
}
bool RpcClient::connected() const
{
  return socket_.state() == QLocalSocket::ConnectedState;
}
QString RpcClient::send(const QString & operation, QJsonObject arguments)
{
  if (!connected() || socket_.bytesToWrite() > 65536) {return {};}
  const auto id = QUuid::createUuid().toString(QUuid::WithoutBraces);
  arguments.insert("v", 1); arguments.insert("id", id); arguments.insert("op", operation);
  const auto bytes = QJsonDocument(arguments).toJson(QJsonDocument::Compact) + '\n';
  if (bytes.size() > 16384 || socket_.write(bytes) != bytes.size()) {return {};}
  return id;
}
}  // namespace easyfly_connection_manager

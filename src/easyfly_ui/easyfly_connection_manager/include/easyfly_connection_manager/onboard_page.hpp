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

#pragma once

#include <QJsonObject>
#include <QWidget>
#include <memory>

namespace easyfly_connection_manager
{
/// Persistent native frontend. The adapter never owns or terminates aircraft processes.
class OnboardPage : public QWidget
{
  Q_OBJECT

public:
  explicit OnboardPage(const QString & runtime, QWidget * parent = nullptr);
  ~OnboardPage() override;
  QWidget * missionPage();
  QWidget * missionTransferPage();
  Q_INVOKABLE void setMissionCandidate(const QJsonObject & candidate);
  Q_INVOKABLE void setMissionCenterActive(bool active);
  Q_INVOKABLE void focusMission(const QString & name);
  Q_INVOKABLE bool confirmDiscardChanges();

signals:
  void missionRequested();
  void observeRequested(const QJsonObject & context);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_connection_manager

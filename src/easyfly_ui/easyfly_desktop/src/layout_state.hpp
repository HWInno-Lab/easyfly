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
#ifndef LAYOUT_STATE_HPP_
#define LAYOUT_STATE_HPP_
#include <QSettings>
#include <QSplitter>
namespace easyfly_desktop
{
// Only a deliberate handle drag persists sizes; temporary focus mode never changes them.
inline void remember_splitter(QSplitter * splitter, const QString & key)
{
  const auto setting = QStringLiteral("layout/v1/") + key;
  QSettings preferences("EasyFly", "Desktop");
  const auto previous = preferences.value(setting).toByteArray();
  if (!previous.isEmpty()) {splitter->restoreState(previous);}
  QObject::connect(
    splitter, &QSplitter::splitterMoved, splitter, [splitter, setting] {
      QSettings preferences("EasyFly", "Desktop");
      preferences.setValue(setting, splitter->saveState());
    });
}
}  // namespace easyfly_desktop
#endif  // LAYOUT_STATE_HPP_

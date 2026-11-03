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
#include <QCheckBox>
#include <QComboBox>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QLineEdit>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTabWidget>
#include <QTextEdit>
#include <QTemporaryDir>
#include <QTest>
#include <QTreeWidget>
#include <QVBoxLayout>
#include "easyfly_connection_manager/onboard_page.hpp"

namespace
{
bool until(const std::function<bool()> & condition)
{
  QElapsedTimer timer; timer.start();
  while (!condition() && timer.elapsed() < 10000) {QTest::qWait(20);}
  return condition();
}
}

TEST(OnboardNativePage, SharedTaskPageAndOfflineDraftHaveNoPlatformSideEffect)
{
  QTemporaryDir runtime; ASSERT_TRUE(runtime.isValid());
  QWidget workspace; auto * layout = new QVBoxLayout(&workspace); auto * tabs = new QTabWidget;
  layout->addWidget(tabs);
  auto * page = new easyfly_connection_manager::OnboardPage(runtime.path());
  tabs->addTab(page, "platform"); QWidget * tasks = page->missionPage();
  tabs->addTab(tasks, "tasks");
  workspace.show();
  auto * devices = page->findChild<QTreeWidget *>("onboardDeviceTree");
  auto * algorithms = page->findChild<QTreeWidget *>("onboardAlgorithmTree");
  ASSERT_NE(devices, nullptr); ASSERT_NE(algorithms, nullptr);
  ASSERT_TRUE(
    until(
      [&] {
        return devices->topLevelItemCount() > 0 && algorithms->topLevelItemCount() > 0;
      }));
  EXPECT_EQ(page->missionPage(), tasks);
  // Rendering empty JSON must not insert target keys or start polling a nonexistent platform.
  QTest::qWait(150);
  const auto offline_identity = page->findChild<QTextEdit *>("onboardIdentity")->toPlainText();
  EXPECT_TRUE(offline_identity.contains(QStringLiteral("尚未连接")));
  EXPECT_EQ(page->findChildren<QTabWidget *>().first()->count(), 4);
  auto * management = page->findChild<QTabWidget *>("onboardManagementTabs");
  management->setCurrentIndex(1);
  auto * editor = page->findChild<QWidget *>("onboardDeviceParameterEditor");
  ASSERT_NE(editor, nullptr); EXPECT_FALSE(editor->isVisible());
  ASSERT_NE(devices->currentItem(), nullptr);
  auto * edit = page->findChild<QPushButton *>(
    "onboardEditParameters_" + devices->currentItem()->data(0, Qt::UserRole).toString());
  ASSERT_NE(edit, nullptr); edit->click(); EXPECT_TRUE(editor->isVisible());
  EXPECT_FALSE(page->property("onboardHasUnsavedChanges").toBool());
  auto * close = page->findChild<QPushButton *>("onboardCloseDeviceParameters");
  ASSERT_NE(close, nullptr); close->click(); EXPECT_FALSE(editor->isVisible());
  EXPECT_FALSE(page->property("onboardHasUnsavedChanges").toBool());
  management->setCurrentIndex(0);
  auto * name = page->findChild<QLineEdit *>("onboardRecipeName"); ASSERT_NE(name, nullptr);
  name->setFocus(); QTest::keyClick(name, Qt::Key_A, Qt::ControlModifier); QTest::keyClicks(
    name,
    "offline-recipe");
  EXPECT_TRUE(page->property("onboardHasUnsavedChanges").toBool());
  EXPECT_TRUE(tasks->property("onboardHasUnsavedChanges").toBool());
  tabs->setCurrentIndex(1); tabs->setCurrentIndex(0);
  EXPECT_EQ(name->text(), "offline-recipe");
  auto * start = page->findChild<QPushButton *>("onboardStart"); ASSERT_NE(start, nullptr);
  EXPECT_FALSE(start->isEnabled());  // No authenticated target, even though the draft is editable.
  auto * save = page->findChild<QPushButton *>("onboardSaveDraft"); ASSERT_NE(save, nullptr);
  QTest::mouseClick(save, Qt::LeftButton);
  ASSERT_TRUE(until([&] {return QFileInfo::exists(runtime.path() + "/onboard-ui/draft.json");}));
  ASSERT_TRUE(until([&] {return !page->property("onboardHasUnsavedChanges").toBool();}));
  EXPECT_TRUE(page->confirmDiscardChanges());
  EXPECT_FALSE(QFileInfo::exists(runtime.path() + "/server.json"));
  EXPECT_FALSE(page->property("onboardTaskUnresolved").toBool());
}

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
#include <QComboBox>
#include <QDir>
#include <QFile>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QScrollBar>
#include <QTemporaryDir>
#include <QTabWidget>
#include <QTest>
#include <sys/stat.h>
#include <unistd.h>
#include <limits>
#include <string>
#include <utility>
#include <vector>
#include "easyfly_desktop/mission_draft.hpp"
#include "easyfly_desktop/mission_editor.hpp"
#include "support.hpp"
namespace ui = easyfly_desktop;
namespace mission = easyfly_mission_fsm;
namespace
{
std::string sample()
{
  return ui::draft_yaml(
    {ui::draft_default_step("takeoff"), ui::draft_default_step(
        "hover"), ui::draft_default_step("land")});
}
std::shared_ptr<const ui::DraftResult> run(ui::DraftWorker & worker, ui::DraftJob job)
{
  const auto id = worker.submit(std::move(job)); EXPECT_NE(id, 0u);
  EXPECT_TRUE(
    desktop_test::until(
      [&] {
        const auto r = worker.latest(); return r && r->id == id;
      }, 5000));
  return worker.latest();
}
void write(const QString & path, const std::string & content)
{
  QFile file(path); ASSERT_TRUE(file.open(QIODevice::WriteOnly));
  ASSERT_EQ(file.write(content.data(), content.size()), static_cast<qint64>(content.size()));
  file.close();
}
std::string read(const QString & path)
{
  QFile file(path); EXPECT_TRUE(file.open(QIODevice::ReadOnly));
  return file.readAll().toStdString();
}
ui::DraftJob prepare(const QString & path)
{
  ui::DraftJob j; j.operation = ui::DraftOperation::PrepareDirectory; j.path = path.toStdString();
  return j;
}
ui::DraftJob inspect(const QString & path)
{
  ui::DraftJob j; j.operation = ui::DraftOperation::InspectExport; j.path = path.toStdString();
  j.filename = "mission.yaml"; return j;
}
void accept(ui::MissionEditor & editor)
{
  auto * p = editor.findChild<QMessageBox *>("draftPrompt"); ASSERT_NE(p, nullptr);
  EXPECT_EQ(p->defaultButton(), p->button(QMessageBox::Cancel)); p->done(QMessageBox::Yes);
}
QString status(ui::MissionEditor & editor)
{
  return editor.findChild<QLabel *>("draftStatus")->text();
}
bool checked(ui::MissionEditor & editor)
{
  return status(editor).contains(QStringLiteral("v1 校验通过"));
}
}  // namespace
TEST(DraftModel, AllTypesRoundTripAndSharedLimits)
{
  std::vector<mission::MissionStep> steps;
  for (const auto * type :
    {"takeoff", "hover", "waypoint", "navigate", "track", "search", "land", "precision_land"})
  {
    steps.push_back(ui::draft_default_step(type));
  }
  steps[2].x = 1.123456789012345; steps[2].yaw = .123456789012345;
  const auto text = ui::draft_yaml(steps); const auto parsed = mission::validateMissionText(text);
  ASSERT_TRUE(parsed.valid()) << parsed.error; ASSERT_EQ(parsed.steps.size(), 8u);
  EXPECT_EQ(parsed.steps[2].x, steps[2].x); EXPECT_EQ(parsed.steps[2].yaw, steps[2].yaw);
  EXPECT_EQ(ui::draft_yaml(parsed.steps), text);
  EXPECT_FALSE(ui::draft_set_field(steps[0], "tag_id", 0));
  EXPECT_FALSE(ui::draft_set_field(steps[0], "altitude", std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(ui::draft_set_field(steps[7], "tag_id", 1.1));
  steps[0].altitude = 51; EXPECT_THROW(ui::draft_yaml(steps), std::invalid_argument);
  steps[0].type = "hover\n    duration: 0"; EXPECT_THROW(
    ui::draft_yaml(
      steps), std::invalid_argument);
  EXPECT_THROW(ui::draft_yaml({}), std::invalid_argument);
}
TEST(DraftWorker, BoundedLatestValidationAndShutdown)
{
  ui::DraftWorker worker;
  ui::DraftJob oversized; oversized.content = std::string(65537, 'x');
  EXPECT_EQ(worker.submit(oversized), 0u);
  ui::DraftJob bad_path; bad_path.path = std::string("/tmp/test\0.yaml", 15); EXPECT_EQ(
    worker.submit(
      bad_path), 0u);
  uint64_t last = 0;
  for (uint64_t i = 1; i <= 80; ++i) {
    ui::DraftJob j; j.content = i == 80 ? sample() : "invalid"; j.revision = i;
    last = worker.submit(j);
    EXPECT_NE(last, 0u); EXPECT_LE(worker.state().pending, 1u);
  }
  ASSERT_TRUE(desktop_test::until([&] {auto r = worker.latest(); return r && r->id == last;}));
  EXPECT_TRUE(worker.latest()->success); EXPECT_EQ(worker.latest()->job.revision, 80u);
  ui::DraftJob utf8; utf8.content = std::string(1, static_cast<char>(255));
  EXPECT_FALSE(run(worker, utf8)->success);
  worker.stop(); EXPECT_EQ(worker.submit({}), 0u); EXPECT_TRUE(
    desktop_test::until(
      [&] {
        return worker.state().stopped;
      }));
}
TEST(DraftFiles, DedicatedDirectoryRefusesExistingCatalogAndProtectedTree)
{
  QTemporaryDir temporary; ASSERT_TRUE(temporary.isValid());
  const auto ordinary = temporary.path() + "/catalog"; ASSERT_TRUE(QDir().mkdir(ordinary));
  ui::DraftWorker worker({temporary.path().toStdString() + "/protected"});
  EXPECT_FALSE(run(worker, prepare(ordinary))->success);
  EXPECT_FALSE(QFile::exists(ordinary + "/.easyfly-drafts-v1"));
  ASSERT_TRUE(QDir().mkdir(temporary.path() + "/protected"));
  EXPECT_FALSE(run(worker, prepare(temporary.path() + "/protected/new"))->success);
  const auto root = temporary.path() + "/drafts";
  EXPECT_TRUE(run(worker, prepare(root))->success);
  EXPECT_TRUE(run(worker, prepare(root))->success);
  ASSERT_EQ(chmod(root.toUtf8().constData(), 0755), 0); EXPECT_FALSE(
    run(
      worker, inspect(
        root))->success);
}
TEST(DraftFiles, RelocatedRuntimePackagesAndExecutableStayProtected)
{
  QTemporaryDir temporary; ASSERT_TRUE(temporary.isValid());
  const auto root = temporary.path();
  const auto desktop = root + "/relocated desktop";
  const auto mission_prefix = root + "/relocated mission";
  const auto executable_prefix = root + "/relocated executable";
  const auto configured = root + "/original install";
  for (const auto & prefix : {desktop, mission_prefix, executable_prefix, configured}) {
    ASSERT_TRUE(QDir().mkpath(prefix));
  }
  const auto protected_paths = ui::local_storage_protected_paths(
    {configured.toStdString()},
    {{"easyfly_desktop", desktop.toStdString()},
      {"easyfly_mission_fsm", mission_prefix.toStdString()},
      {"unrelated_package", root.toStdString()}, {"easyfly_empty", ""},
      {"easyfly_relative", "relative/path"}},
    (executable_prefix + "/lib/easyfly_desktop/easyfly_desktop").toStdString());
  ui::DraftWorker worker(protected_paths);
  for (const auto & prefix : {desktop, mission_prefix, executable_prefix, configured}) {
    const auto forbidden = prefix + "/drafts";
    EXPECT_FALSE(run(worker, prepare(forbidden))->success);
    EXPECT_FALSE(QFile::exists(forbidden));
  }
  EXPECT_TRUE(run(worker, prepare(root + "/separate private drafts"))->success);
  const auto without_index = ui::local_storage_protected_paths(
    {}, {}, (executable_prefix + "/lib/easyfly_desktop/easyfly_desktop").toStdString());
  EXPECT_EQ(without_index, std::vector<std::string>{executable_prefix.toStdString()});
  EXPECT_TRUE(ui::local_storage_protected_paths({}, {}, "/tmp/build/easyfly_desktop/test").empty());
}
TEST(DraftFiles, ExportExactBytesAndKeepOverwriteBackup)
{
  QTemporaryDir temporary; const auto root = temporary.path() + "/drafts";
  ui::DraftWorker worker; ASSERT_TRUE(run(worker, prepare(root))->success);
  auto j = inspect(root); auto r = run(worker, j); ASSERT_TRUE(r->success); EXPECT_FALSE(
    r->fingerprint.exists);
  j.operation = ui::DraftOperation::Export; j.expected = r->fingerprint; j.content = sample();
  auto exported = run(worker, j); ASSERT_TRUE(exported->success) << exported->error;
  EXPECT_EQ(read(root + "/mission.yaml"), j.content); EXPECT_TRUE(exported->backup.empty());
  const auto original = j.content; r = run(worker, inspect(root)); ASSERT_TRUE(r->success);
  j.expected = r->fingerprint; j.content += "# local comment\n";
  exported = run(worker, j); ASSERT_TRUE(exported->success) << exported->error;
  EXPECT_FALSE(exported->backup.empty()); EXPECT_EQ(
    read(
      QString::fromStdString(
        exported->backup)), original);
  EXPECT_EQ(read(root + "/mission.yaml"), j.content);
}
TEST(DraftFiles, ChangedTargetAndChangedRootRejectConsent)
{
  QTemporaryDir temporary; const auto root = temporary.path() + "/drafts";
  ui::DraftWorker worker; ASSERT_TRUE(run(worker, prepare(root))->success);
  auto j = inspect(root); auto r = run(worker, j); ASSERT_TRUE(r->success);
  j.operation = ui::DraftOperation::Export; j.content = sample(); j.expected = r->fingerprint;
  write(root + "/mission.yaml", "do not overwrite"); EXPECT_FALSE(run(worker, j)->success);
  EXPECT_EQ(read(root + "/mission.yaml"), "do not overwrite");
  r = run(worker, inspect(root)); j.expected = r->fingerprint;
  write(root + "/mission.yaml", "changed after confirmation");
  EXPECT_FALSE(run(worker, j)->success);
  EXPECT_EQ(read(root + "/mission.yaml"), "changed after confirmation");
  ASSERT_TRUE(QDir().rename(root, root + "-moved"));
  ASSERT_TRUE(run(worker, prepare(root))->success);
  EXPECT_FALSE(run(worker, j)->success); EXPECT_FALSE(QFile::exists(root + "/mission.yaml"));
}
TEST(DraftFiles, RejectTraversalLinksFifoOversizeAndInvalidBody)
{
  QTemporaryDir temporary; const auto root = temporary.path() + "/drafts";
  ui::DraftWorker worker; ASSERT_TRUE(run(worker, prepare(root))->success);
  auto j = inspect(root); j.filename = "../outside.yaml"; EXPECT_FALSE(run(worker, j)->success);
  j.filename = ".hidden.yaml"; EXPECT_FALSE(run(worker, j)->success);
  write(temporary.path() + "/outside.yaml", sample());
  ASSERT_EQ(
    symlink(
      (temporary.path() + "/outside.yaml").toUtf8().constData(),
      (root + "/mission.yaml").toUtf8().constData()), 0);
  EXPECT_FALSE(run(worker, inspect(root))->success);
  ASSERT_TRUE(QFile::remove(root + "/mission.yaml"));
  ASSERT_EQ(
    link(
      (temporary.path() + "/outside.yaml").toUtf8().constData(),
      (root + "/mission.yaml").toUtf8().constData()), 0);
  EXPECT_FALSE(run(worker, inspect(root))->success);
  ASSERT_TRUE(QFile::remove(root + "/mission.yaml"));
  ASSERT_EQ(mkfifo((root + "/mission.yaml").toUtf8().constData(), 0600), 0);
  EXPECT_FALSE(run(worker, inspect(root))->success);
  ASSERT_TRUE(QFile::remove(root + "/mission.yaml"));
  write(root + "/mission.yaml", std::string(65537, 'x')); EXPECT_FALSE(
    run(
      worker, inspect(
        root))->success);
  ASSERT_TRUE(QFile::remove(root + "/mission.yaml"));
  auto r = run(worker, inspect(root)); ASSERT_TRUE(r->success);
  j = inspect(root); j.operation = ui::DraftOperation::Export; j.expected = r->fingerprint;
  j.content = "not valid YAML";
  EXPECT_FALSE(run(worker, j)->success); EXPECT_FALSE(QFile::exists(root + "/mission.yaml"));
}
TEST(DraftEditor, NewPreviewFormRequiresConsentAndInvalidEditStaysLocal)
{
  ui::MissionEditor editor; editor.resize(1120, 800); editor.show();
  auto * body = editor.findChild<QPlainTextEdit *>("draftBody");
  editor.findChild<QPushButton *>("draftNew")->click(); ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return checked(editor);
      }));
  EXPECT_TRUE(editor.dirty()); EXPECT_EQ(editor.findChild<QListWidget *>("draftSteps")->count(), 3);
  editor.findChild<QTabWidget *>("draftTabs")->setCurrentIndex(1);
  auto * field = editor.findChild<QLineEdit *>("draftField_altitude"); ASSERT_NE(field, nullptr);
  field->setFocus(); field->selectAll(); QTest::keyClicks(field, "2.25");
  EXPECT_TRUE(body->isReadOnly()); EXPECT_FALSE(
    editor.findChild<QPushButton *>(
      "draftExport")->isEnabled());
  const auto original = body->toPlainText(); editor.findChild<QPushButton *>("draftApply")->click();
  auto * p = editor.findChild<QMessageBox *>("draftPrompt"); ASSERT_NE(p, nullptr);
  p->done(QMessageBox::Cancel); EXPECT_EQ(body->toPlainText(), original);
  editor.findChild<QPushButton *>("draftApply")->click(); accept(editor);
  ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return checked(editor) && body->toPlainText().contains("2.25");
      }));
  EXPECT_FALSE(body->isReadOnly()); body->setPlainText("schema_version: 9\n");
  ASSERT_TRUE(desktop_test::until([&] {return status(editor).contains(QStringLiteral("未通过"));}));
  EXPECT_EQ(editor.findChild<QListWidget *>("draftSteps")->count(), 0);
  EXPECT_FALSE(editor.findChild<QPushButton *>("draftApply")->isEnabled());
  editor.stop(); EXPECT_TRUE(desktop_test::until([&] {return editor.stopped();}));
}
TEST(DraftEditor, ReadExactCrLfExportAndCancelOverwrite)
{
  QTemporaryDir temporary; const auto source = temporary.path() + "/source.yaml";
  const auto text = QString::fromStdString(sample() + "# 注释\n");
  auto bytes = QString(text).replace("\n", "\r\n").toUtf8().toStdString(); write(source, bytes);
  ui::MissionEditor editor; editor.open_file(source); ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return checked(editor);
      }));
  EXPECT_FALSE(editor.dirty()); const auto root = temporary.path() + "/drafts";
  editor.prepare_directory(root);
  ASSERT_TRUE(desktop_test::until([&] {return checked(editor) && !editor.busy();}));
  editor.export_file("mission.yaml"); ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return status(editor).contains(QStringLiteral("已导出本地草稿"));
      }));
  EXPECT_EQ(read(root + "/mission.yaml"), bytes); EXPECT_EQ(read(source), bytes);
  editor.findChild<QPlainTextEdit *>("draftBody")->appendPlainText("# local edit");
  ASSERT_TRUE(desktop_test::until([&] {return checked(editor);}));
  editor.export_file("mission.yaml");
  ASSERT_TRUE(desktop_test::until([&] {return editor.findChild<QMessageBox *>("draftPrompt");}));
  auto * p = editor.findChild<QMessageBox *>("draftPrompt"); EXPECT_EQ(
    p->defaultButton(), p->button(
      QMessageBox::Cancel));
  p->done(QMessageBox::Cancel); EXPECT_TRUE(editor.dirty()); EXPECT_EQ(
    read(
      root + "/mission.yaml"), bytes);
  editor.export_file("mission.yaml"); ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return editor.findChild<QMessageBox *>("draftPrompt");
      }));
  accept(editor); ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return status(editor).contains(QStringLiteral("已导出本地草稿"));
      }));
  EXPECT_FALSE(editor.dirty()); EXPECT_EQ(read(source), bytes);
  editor.stop(); EXPECT_TRUE(desktop_test::until([&] {return editor.stopped();}));
}
TEST(DraftEditor, AddReorderDeleteAndRejectOutOfRangeForm)
{
  ui::MissionEditor editor; editor.resize(1120, 800); editor.show();
  editor.findChild<QPushButton *>("draftNew")->click();
  ASSERT_TRUE(desktop_test::until([&] {return checked(editor);}));
  auto * list = editor.findChild<QListWidget *>("draftSteps");
  editor.findChild<QComboBox *>("draftAddType")->setCurrentText("search");
  EXPECT_TRUE(editor.findChild<QPushButton *>("draftAdd")->isEnabled());
  editor.findChild<QPushButton *>("draftAdd")->click(); accept(editor);
  ASSERT_TRUE(desktop_test::until([&] {return checked(editor) && list->count() == 4;}));
  EXPECT_TRUE(list->item(3)->text().contains("search")); list->setCurrentRow(3);
  editor.findChild<QPushButton *>("draftUp")->click(); accept(editor);
  ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return checked(editor) && list->item(2)->text().contains("search");
      }));
  list->setCurrentRow(2); editor.findChild<QPushButton *>("draftDelete")->click(); accept(editor);
  ASSERT_TRUE(desktop_test::until([&] {return checked(editor) && list->count() == 3;}));
  editor.findChild<QTabWidget *>("draftTabs")->setCurrentIndex(1);
  auto * field = editor.findChild<QLineEdit *>("draftField_altitude"); ASSERT_NE(field, nullptr);
  field->setFocus(); field->selectAll(); QTest::keyClicks(field, "51");
  auto * body = editor.findChild<QPlainTextEdit *>("draftBody");
  const auto before = body->toPlainText();
  editor.findChild<QPushButton *>("draftApply")->click(); accept(editor);
  ASSERT_TRUE(desktop_test::until([&] {return status(editor).contains(QStringLiteral("未通过"));}));
  EXPECT_EQ(body->toPlainText(), before); EXPECT_TRUE(editor.dirty()); EXPECT_TRUE(
    body->isReadOnly());
  editor.findChild<QPushButton *>("draftResetForm")->click(); EXPECT_FALSE(body->isReadOnly());
  editor.stop(); EXPECT_TRUE(desktop_test::until([&] {return editor.stopped();}));
}
TEST(DraftEditor, ReadFailurePreservesDocumentAndLegacyCannotExport)
{
  QTemporaryDir temporary; const auto source = temporary.path() + "/invalid.yaml";
  write(source, "schema_version: 9\n"); ui::MissionEditor editor;
  editor.open_file(source); ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return status(editor).contains(QStringLiteral("未通过"));
      }));
  auto * body = editor.findChild<QPlainTextEdit *>("draftBody");
  const auto before = body->toPlainText();
  editor.open_file(temporary.path() + "/absent"); ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return !editor.busy();
      }));
  EXPECT_EQ(body->toPlainText(), before);
  write(source, "type=takeoff\naltitude=1.5\n---\ntype=land\n"); editor.open_file(source);
  ASSERT_TRUE(desktop_test::until([&] {return status(editor).contains("legacy_flat");}));
  EXPECT_FALSE(editor.findChild<QPushButton *>("draftExport")->isEnabled());
  EXPECT_FALSE(editor.findChild<QPushButton *>("draftAdd")->isEnabled()); EXPECT_FALSE(
    editor.dirty());
  editor.stop(); EXPECT_TRUE(desktop_test::until([&] {return editor.stopped();}));
}
TEST(DraftEditor, MainWindowOfflineDirtyExitRequiresDefaultStayAndNoRpc)
{
  auto state = std::make_shared<desktop_test::FakeState>();
  state->data = std::make_shared<easyfly_ui_bridge::UiSnapshot>(); state->data->running = true;
  ui::MainWindow window(easyfly_ui_bridge::BridgeConfig{}, desktop_test::factory(state));
  window.show();
  auto * navigation = window.findChild<QListWidget *>("navigation"); ASSERT_EQ(
    navigation->count(), 7);
  navigation->setCurrentRow(6); auto * editor = window.findChild<ui::MissionEditor *>(); ASSERT_NE(
    editor, nullptr);
  window.findChild<QPushButton *>("draftNew")->click(); ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return checked(*editor);
      }));
  QDir().mkpath(EASYFLY_DRAFT_SCREENSHOT_DIRECTORY);
  window.setWindowTitle(QStringLiteral("D6-5 · 离线 UI 测试，非飞行场景"));
  window.findChild<QLabel *>("requestNote")->setText(
    QStringLiteral("D6-5 离线构造测试 · 非真实飞行 · 编辑器未发送任务请求"));
  QTest::qWait(100);
  EXPECT_TRUE(
    window.grab().save(
      QString(
        EASYFLY_DRAFT_SCREENSHOT_DIRECTORY) + "/editor-offline.png"));
  window.findChild<QTabWidget *>("draftTabs")->setCurrentIndex(1); QTest::qWait(100);
  EXPECT_TRUE(
    window.grab().save(
      QString(EASYFLY_DRAFT_SCREENSHOT_DIRECTORY) + "/editor-parameters-offline.png"));
  auto * ancestor = editor->parentWidget();
  while (ancestor && !qobject_cast<QScrollArea *>(ancestor)) {ancestor = ancestor->parentWidget();}
  auto * scroll = qobject_cast<QScrollArea *>(ancestor);
  ASSERT_NE(scroll, nullptr); scroll->verticalScrollBar()->setValue(
    scroll->verticalScrollBar()->maximum());
  QTest::qWait(100);
  EXPECT_TRUE(
    window.grab().save(
      QString(EASYFLY_DRAFT_SCREENSHOT_DIRECTORY) + "/editor-export-offline.png"));
  window.close(); auto * prompt = window.findChild<QMessageBox *>("draftExitPrompt"); ASSERT_NE(
    prompt, nullptr);
  EXPECT_EQ(prompt->defaultButton(), prompt->button(QMessageBox::Cancel)); prompt->done(
    QMessageBox::Cancel);
  EXPECT_TRUE(window.isVisible()); EXPECT_TRUE(editor->dirty()); EXPECT_EQ(state->stop_calls, 0);
  EXPECT_EQ(
    state->list_calls + state->validate_calls + state->query_calls + state->details_calls,
    0);
  window.close(); prompt = window.findChild<QMessageBox *>("draftExitPrompt"); ASSERT_NE(
    prompt,
    nullptr);
  prompt->done(QMessageBox::Yes); EXPECT_TRUE(
    desktop_test::until(
      [&] {
        return window.cleanup_complete();
      }));
  EXPECT_TRUE(editor->stopped());
}
TEST(DraftEditor, DirtyExitGuardStillAppliesAfterObserverDisconnected)
{
  auto state = std::make_shared<desktop_test::FakeState>();
  ui::MainWindow window(easyfly_ui_bridge::BridgeConfig{}, desktop_test::factory(state));
  window.show();
  window.findChild<QPushButton *>("observerToggle")->click();
  ASSERT_TRUE(desktop_test::until([&] {return window.cleanup_complete();}));
  auto * editor = window.findChild<ui::MissionEditor *>(); ASSERT_NE(editor, nullptr);
  window.findChild<QPushButton *>("draftNew")->click();
  ASSERT_TRUE(desktop_test::until([&] {return checked(*editor);}));
  EXPECT_FALSE(editor->stopped()); window.close();
  auto * prompt = window.findChild<QMessageBox *>("draftExitPrompt"); ASSERT_NE(prompt, nullptr);
  prompt->done(QMessageBox::Cancel); EXPECT_TRUE(window.isVisible());
  window.close(); prompt = window.findChild<QMessageBox *>("draftExitPrompt"); ASSERT_NE(
    prompt,
    nullptr);
  prompt->done(QMessageBox::Yes);
  EXPECT_TRUE(desktop_test::until([&] {return window.cleanup_complete() && editor->stopped();}));
}

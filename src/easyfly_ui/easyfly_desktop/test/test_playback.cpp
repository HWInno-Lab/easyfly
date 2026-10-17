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
#include <sys/stat.h>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QSlider>
#include <QTableView>
#include <QTemporaryDir>
#include <QTest>
#include <QCryptographicHash>
#include <filesystem>
#include <functional>
#include "easyfly_desktop/playback_window.hpp"
#include "easyfly_desktop/recording.hpp"
namespace
{
using easyfly_desktop::PlaybackWorker;
using easyfly_desktop::PlaybackWindow;
using easyfly_desktop::RecordingWorker;
using easyfly_desktop::RecordingPhase;
bool until(const std::function<bool()> & done)
{
  for (int i = 0; i < 500; ++i) {
    if (done()) {return true;} QTest::qWait(10);
  }
  return done();
}
QByteArray read(const QString & path)
{
  QFile file(path); if (!file.open(QIODevice::ReadOnly)) {return {};} return file.readAll();
}
void write(const QString & path, const QByteArray & value)
{
  QFile file(path); ASSERT_TRUE(file.open(QIODevice::WriteOnly | QIODevice::Truncate));
  ASSERT_EQ(file.write(value), value.size()); file.close();
}
struct Fixture
{
  QTemporaryDir tmp;
  RecordingWorker recorder;
  QString root, id, session;
  explicit Fixture(int count = 6)
  {
    easyfly_desktop::RecordingConfig config;
    root = tmp.path() + "/records"; config.directory = root.toStdString();
    config.binding = {42, 3, 0, "/test/playback"};
    config.segment_frames = 2; config.reserve_bytes = 0; config.profile = "unconfirmed-fixture";
    for (int i = 0; i < count; ++i) {
      auto frame = std::make_shared<easyfly_desktop::ConvertedFrame>();
      frame->binding = config.binding; frame->sequence = i + 1;
      frame->stamp_ns = 9007199254740993LL + i * 1000000000LL;
      frame->first_received = easyfly_ui_bridge::detail::steady_seconds();
      frame->image = QImage(i >= 4 ? 128 : 96, 64, QImage::Format_RGB888);
      frame->image.fill(QColor(180, 40 + i * 20, 15));
      auto observation = std::make_shared<easyfly_ui_bridge::UiSnapshot>();
      observation->running = true; observation->produced_steady = frame->first_received;
      observation->observation_session_id = 42; observation->time_epoch = 3;
      observation->clock.now_ns = frame->stamp_ns;
      observation->clock.quality = easyfly_ui_bridge::Quality::ValidAtObservation;
      observation->events.push_back({frame->first_received, 3, "unattributed-fixture", 42});
      auto control = std::make_shared<easyfly_ui_control::ControlSnapshot>();
      control->intent = easyfly_ui_control::Intent{};
      control->intent->execution = "exact-execution";
      control->intent->request = "exact-request"; control->intent->goal = "exact-goal";
      easyfly_desktop::RecordingPacket packet{frame, observation, control};
      if (i == 0) {EXPECT_TRUE(recorder.start(config, packet));} else {recorder.offer(packet);}
      EXPECT_TRUE(until([&] {return recorder.state().frames == static_cast<unsigned>(i + 1);}));
      if (i + 1 < count) {QTest::qWait(110);}
    }
    recorder.finish("operator_stop");
    EXPECT_TRUE(
      until(
        [&] {
          return recorder.state().phase == RecordingPhase::Completed ||
          recorder.state().phase == RecordingPhase::Failed;
        }));
    EXPECT_EQ(recorder.state().phase, RecordingPhase::Completed) << recorder.state().error;
    id = QString::fromStdString(recorder.state().id); session = root + "/" + id;
    recorder.stop(); EXPECT_TRUE(until([&] {return recorder.state().stopped;}));
  }
  void rehash(const QString & name)
  {
    const auto manifest_path = session + "/manifest.json";
    auto value = QJsonDocument::fromJson(read(manifest_path)).object();
    value[name + "_sha256"] = QString::fromLatin1(
      QCryptographicHash::hash(
        read(session + "/" + name + (name == "config" ? ".json" : ".jsonl")),
        QCryptographicHash::Sha256).toHex());
    write(manifest_path, QJsonDocument(value).toJson(QJsonDocument::Compact));
  }
};
bool loaded(PlaybackWorker & worker)
{
  return until([&] {return !worker.state().loading;}) && worker.state().record != nullptr;
}
TEST(Playback, SegmentsReadOnlyExactTimingAndExplicitAttribution)
{
  Fixture f; PlaybackWorker worker;
  QMap<QString, QByteArray> originals;
  for (const auto & name : QDir(f.session).entryList(QDir::Files)) {
    originals[name] = read(f.session + "/" + name);
  }
  ASSERT_TRUE(worker.open(f.root, f.id));
  ASSERT_TRUE(loaded(worker)) << worker.state().error.toStdString();
  auto state = worker.state(); ASSERT_EQ(state.record->frames.size(), 6u);
  EXPECT_EQ(state.record->frames[0]["source_ns"].toString(), "9007199254740993");
  EXPECT_FALSE(state.image.isNull()); EXPECT_EQ(state.frame, 0);
  for (int frame = 0; frame < 6; ++frame) {
    ASSERT_TRUE(worker.seek(frame));
    ASSERT_TRUE(until([&] {return !worker.state().pending && worker.state().frame == frame;}));
    state = worker.state();
    EXPECT_EQ(state.image.width(), frame >= 4 ? 128 : 96); EXPECT_EQ(state.image.height(), 64);
    EXPECT_NEAR(state.image.pixelColor(0, 0).red(), 180, 8);
    EXPECT_NEAR(state.image.pixelColor(0, 0).green(), 40 + frame * 20, 8);
  }
  bool exact = false, global = false;
  for (const auto & event : state.record->events) {
    if (event["execution_id"].toString() == "exact-execution") {exact = true;}
    if (event["attribution"].toString() == "global_unattributed") {
      global = true; EXPECT_FALSE(event.contains("execution_id"));
    }
  }
  EXPECT_TRUE(exact); EXPECT_TRUE(global);
  worker.stop(); ASSERT_TRUE(until([&] {return worker.state().stopped;}));
  for (auto it = originals.begin(); it != originals.end(); ++it) {
    EXPECT_EQ(read(f.session + "/" + it.key()), it.value());
  }
  EXPECT_EQ(QDir(f.session).entryList(QDir::Files).size(), originals.size());
}
TEST(Playback, SelectionBoundedAndLatestRequestWins)
{
  Fixture f; PlaybackWorker worker; worker.open(f.root, f.id);
  ASSERT_TRUE(loaded(worker)) << worker.state().error.toStdString();
  EXPECT_FALSE(worker.seek(-1)); EXPECT_FALSE(worker.seek(6));
  for (int i = 0; i < 300; ++i) {
    EXPECT_TRUE(worker.seek(i % 6));
  }
  ASSERT_TRUE(until([&] {return !worker.state().pending;})); EXPECT_EQ(worker.state().frame, 5);
}
TEST(Playback, IncompleteOrMissingFileClearsPreviousImage)
{
  Fixture f(1); PlaybackWorker worker; worker.open(f.root, f.id); ASSERT_TRUE(loaded(worker));
  ASSERT_TRUE(QFile::remove(f.session + "/manifest.json"));
  worker.open(f.root, f.id); ASSERT_TRUE(until([&] {return !worker.state().loading;}));
  EXPECT_FALSE(worker.state().record); EXPECT_TRUE(worker.state().image.isNull());
  EXPECT_FALSE(worker.state().error.isEmpty());
}
TEST(Playback, ChangedMetadataAndNonmonotonicTimesRejected)
{
  Fixture f(2); PlaybackWorker worker;
  auto lines = read(f.session + "/frames.jsonl").split('\n');
  auto second = QJsonDocument::fromJson(lines[1]).object();
  second["source_ns"] = QJsonDocument::fromJson(lines[0]).object()["source_ns"];
  lines[1] = QJsonDocument(second).toJson(QJsonDocument::Compact);
  write(f.session + "/frames.jsonl", lines.join('\n'));
  worker.open(f.root, f.id); ASSERT_TRUE(until([&] {return !worker.state().loading;}));
  EXPECT_TRUE(worker.state().error.contains("integrity"));
  f.rehash("frames"); worker.open(f.root, f.id);
  ASSERT_TRUE(until([&] {return !worker.state().loading;}));
  EXPECT_TRUE(worker.state().error.contains("timing")); EXPECT_TRUE(worker.state().image.isNull());
}
TEST(Playback, WrongFrameMappingRejectedEvenWithUpdatedHash)
{
  Fixture f(1); PlaybackWorker worker;
  auto frame = QJsonDocument::fromJson(read(f.session + "/frames.jsonl")).object();
  frame["segment"] = "../outside.avi";
  write(f.session + "/frames.jsonl", QJsonDocument(frame).toJson(QJsonDocument::Compact) + '\n');
  f.rehash("frames"); worker.open(f.root, f.id);
  ASSERT_TRUE(until([&] {return !worker.state().loading;}));
  EXPECT_TRUE(worker.state().error.contains("mapping"));
}
TEST(Playback, BoundedAviDimensionsRejectedEvenWithUpdatedHash)
{
  Fixture f(1); PlaybackWorker worker;
  auto bytes = read(f.session + "/segment-0.avi");
  bytes[66] = 0x10;  // Header would advertise an image far above 1280x720.
  write(f.session + "/segment-0.avi", bytes);
  auto manifest = QJsonDocument::fromJson(read(f.session + "/manifest.json")).object();
  auto segments = manifest["segments"].toArray(); auto segment = segments[0].toObject();
  segment["sha256"] = QString::fromLatin1(
    QCryptographicHash::hash(
      bytes, QCryptographicHash::Sha256).toHex()); segments[0] = segment;
  manifest["segments"] = segments;
  write(f.session + "/manifest.json", QJsonDocument(manifest).toJson(QJsonDocument::Compact));
  worker.open(f.root, f.id); ASSERT_TRUE(until([&] {return !worker.state().loading;}));
  EXPECT_TRUE(worker.state().error.contains("dimensions")); EXPECT_TRUE(
    worker.state().image.isNull());
}
TEST(Playback, MissingTimingAndRecordBudgetRejected)
{
  Fixture f(1); PlaybackWorker worker;
  auto frame = QJsonDocument::fromJson(read(f.session + "/frames.jsonl")).object();
  frame.remove("source_ns");
  write(f.session + "/frames.jsonl", QJsonDocument(frame).toJson(QJsonDocument::Compact) + '\n');
  f.rehash("frames"); worker.open(f.root, f.id);
  ASSERT_TRUE(until([&] {return !worker.state().loading;}));
  EXPECT_TRUE(worker.state().error.contains("integer"));
  auto manifest = QJsonDocument::fromJson(read(f.session + "/manifest.json")).object();
  manifest["frames"] = "36001";
  write(f.session + "/manifest.json", QJsonDocument(manifest).toJson(QJsonDocument::Compact));
  worker.open(f.root, f.id); ASSERT_TRUE(until([&] {return !worker.state().loading;}));
  EXPECT_TRUE(worker.state().error.contains("counts"));
}
TEST(Playback, UnsafePathsSymlinkHardlinkFifoAndPermissions)
{
  Fixture f(1); PlaybackWorker worker;
  auto rejected = [&](const QString & root, const QString & id) {
      worker.open(root, id); EXPECT_TRUE(until([&] {return !worker.state().loading;}));
      EXPECT_FALSE(worker.state().record); EXPECT_FALSE(worker.state().error.isEmpty());
    };
  rejected(f.root, "../escape");
  const auto alias = f.tmp.path() + "/alias";
  std::filesystem::create_directory_symlink(f.root.toStdString(), alias.toStdString());
  rejected(alias, f.id);
  const auto events = f.session + "/events.jsonl"; const auto content = read(events);
  ASSERT_TRUE(QFile::remove(events)); ASSERT_EQ(mkfifo(events.toUtf8().constData(), 0600), 0);
  rejected(f.root, f.id); ASSERT_TRUE(QFile::remove(events));
  write(events, content); ASSERT_EQ(chmod(events.toUtf8().constData(), 0600), 0);
  std::filesystem::create_hard_link(
    events.toStdString(),
    (f.tmp.path() + "/hardlink").toStdString());
  rejected(f.root, f.id); ASSERT_TRUE(QFile::remove(f.tmp.path() + "/hardlink"));
  ASSERT_EQ(chmod(events.toUtf8().constData(), 0644), 0); rejected(f.root, f.id);
}
TEST(Playback, VideoModifiedAfterLoadFailsWithoutStaleDisplay)
{
  Fixture f(2); PlaybackWorker worker;
  worker.open(f.root, f.id); ASSERT_TRUE(loaded(worker));
  const auto path = f.session + "/segment-0.avi";
  auto bytes = read(path); ASSERT_GT(bytes.size(), 240);
  bytes[240] = bytes[240] ^ 1; write(path, bytes);
  ASSERT_TRUE(worker.seek(0)); ASSERT_TRUE(until([&] {return !worker.state().pending;}));
  EXPECT_TRUE(worker.state().error.contains("changed")); EXPECT_TRUE(worker.state().image.isNull());
}
TEST(Playback, OpenSupersedesAndShutdownNonblocking)
{
  Fixture f; PlaybackWorker worker;
  for (int i = 0; i < 30; ++i) {
    worker.open(f.root, i % 2 ? f.id : "invalid");
  }
  ASSERT_TRUE(loaded(worker)) << worker.state().error.toStdString();
  worker.open(f.root, f.id);
  const auto before = easyfly_ui_bridge::detail::steady_seconds(); worker.stop();
  EXPECT_LT(easyfly_ui_bridge::detail::steady_seconds() - before, .1);
  ASSERT_TRUE(until([&] {return worker.state().stopped;}));
  EXPECT_FALSE(worker.open(f.root, f.id)); EXPECT_FALSE(worker.seek(0));
}
TEST(Playback, HistoricalWindowPausedSeekEventsAndNormalClose)
{
  Fixture f; PlaybackWindow window;
  window.load(f.root, f.id); window.show();
  ASSERT_TRUE(until([&] {return window.state().record != nullptr;}));
  auto * play = window.findChild<QPushButton *>("playbackPlay"); ASSERT_NE(play, nullptr);
  QTest::qWait(250); EXPECT_EQ(window.state().frame, 0);  // Never auto-play.
  EXPECT_TRUE(window.findChild<QLabel *>("warningBanner")->text().contains("不连接"));
  auto * events = window.findChild<QTableView *>("playbackEvents"); ASSERT_NE(events, nullptr);
  ASSERT_GT(events->model()->rowCount(), 0); events->selectRow(0);
  EXPECT_TRUE(
    until(
      [&] {
        return !window.findChild<QPlainTextEdit *>(
          "playbackEventDetails")->toPlainText().isEmpty();
      }));
  window.findChild<QSlider *>("playbackSeek")->setValue(3);
  ASSERT_TRUE(until([&] {return !window.state().pending && window.state().frame == 3;}));
  QTest::mouseClick(play, Qt::LeftButton);
  ASSERT_TRUE(until([&] {return window.state().frame == 5;})); QTest::qWait(150);
  EXPECT_EQ(play->text(), QString::fromUtf8("播放 10fps"));
  if (qEnvironmentVariable("QT_QPA_PLATFORM") != "offscreen") {
    const auto directory = qEnvironmentVariable("EASYFLY_PLAYBACK_SCREENSHOT_DIRECTORY");
    if (!directory.isEmpty()) {
      QDir().mkpath(directory);
      EXPECT_TRUE(window.grab().save(directory + "/playback-native.png"));
      window.resize(1024, 720); QTest::qWait(150);
      EXPECT_TRUE(window.grab().save(directory + "/playback-compact.png"));
    }
  }
  const auto before = easyfly_ui_bridge::detail::steady_seconds(); window.close();
  EXPECT_LT(easyfly_ui_bridge::detail::steady_seconds() - before, .1);
  ASSERT_TRUE(until([&] {return window.state().stopped && !window.isVisible();}));
}
TEST(Playback, StandaloneBinaryNoRosLinkageOrControlOptions)
{
  QProcess dependencies; dependencies.start("ldd", {EASYFLY_PLAYBACK_BINARY});
  ASSERT_TRUE(dependencies.waitForFinished(10000));
  EXPECT_EQ(dependencies.exitCode(), 0);
  const auto linkage = dependencies.readAllStandardOutput();
  for (const auto & library : {"librcl", "librmw", "libfastrtps", "libfastdds",
      "libeasyfly_ui", "libmavros"})
  {
    EXPECT_FALSE(linkage.contains(library)) << library;
  }
  QProcess smoke; smoke.start(EASYFLY_PLAYBACK_BINARY, {"--quit-after-ms", "300"});
  ASSERT_TRUE(smoke.waitForFinished(10000)); EXPECT_EQ(smoke.exitCode(), 0);
  QProcess control; control.start(EASYFLY_PLAYBACK_BINARY, {"--enable-mission-control"});
  ASSERT_TRUE(control.waitForFinished(10000)); EXPECT_NE(control.exitCode(), 0);
}
}  // namespace

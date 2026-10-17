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

#include <sys/stat.h>
#include <sys/resource.h>
#include <sys/vfs.h>
#include <linux/magic.h>
#include <fcntl.h>
#include <unistd.h>
#include <gtest/gtest.h>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QTemporaryDir>
#include <QTest>
#include <QLabel>
#include <QListWidget>
#include <QDir>
#include <QTimer>
#include <QTableWidget>
#include <QProcess>
#include <QProcessEnvironment>
#include <QDebug>
#include <filesystem>
#include <functional>
#include <limits>
#include <csignal>
#include <array>
#include <cerrno>
#include "easyfly_desktop/recording_panel.hpp"
#include "easyfly_desktop/mission_draft.hpp"
#include "support.hpp"
#ifdef EASYFLY_ENABLE_MEDIA
#include <opencv2/videoio.hpp>
#endif
namespace
{
namespace bridge = easyfly_ui_bridge;
using easyfly_desktop::ConvertedFrame;
using easyfly_desktop::RecordingConfig;
using easyfly_desktop::RecordingOptions;
using easyfly_desktop::RecordingPacket;
using easyfly_desktop::RecordingPanel;
using easyfly_desktop::RecordingPhase;
using easyfly_desktop::RecordingWorker;
bool until(const std::function<bool()> & done)
{
  for (int i = 0; i < 400; ++i) {
    if (done()) {
      return true;
    }
    QTest::qWait(10);
  }
  return done();
}
RecordingPacket packet(uint64_t sequence = 1)
{
  const auto now = bridge::detail::steady_seconds();
  auto f = std::make_shared<ConvertedFrame>(); f->binding = {42, 3, 0, "/test/camera"};
  f->sequence = sequence; f->stamp_ns = sequence * 1000000000LL; f->first_received = now;
  f->image = QImage(96, 64, QImage::Format_RGB888); f->image.fill(QColor(150, 50, 10));
  for (int y = 56; y < 64; ++y) {
    for (int x = 0; x < 96; ++x) {
      f->image.setPixelColor(x, y, QColor(10, 200, 30));
    }
  }
  auto s = std::make_shared<bridge::UiSnapshot>(); s->running = true; s->produced_steady = now;
  s->observation_session_id = 42; s->time_epoch = 3;
  s->clock.quality = bridge::Quality::ValidAtObservation; s->clock.now_ns = f->stamp_ns;
  s->events.push_back({now, 3, "fixture observer event", 42});
  auto global = std::make_shared<easyfly_msgs::msg::MissionStatus>();
  global->header.stamp.sec = sequence; global->phase_name = "fixture-global-phase";
  s->last_mission_event.value = global;
  s->last_mission_event.evidence.quality = bridge::Quality::ValidAtObservation;
  auto query = std::make_shared<easyfly_msgs::srv::ManageMission::Response>();
  query->success = true;
  query->execution_id = "query-known-exec"; query->status = "running";
  s->missions[bridge::Key::MissionQuery].value = query;
  s->missions[bridge::Key::MissionQuery].evidence.request_id = 17;
  auto c = std::make_shared<easyfly_ui_control::ControlSnapshot>();
  c->phase = easyfly_ui_control::Phase::Running;
  c->intent = easyfly_ui_control::Intent{}; c->intent->execution = "exec-known";
  c->intent->goal = "uuid-known"; c->intent->request = "request-known";
  return {f, s, c};
}
RecordingConfig settings(const QTemporaryDir & tmp)
{
  RecordingConfig config; config.directory = tmp.path().toStdString() + "/records";
  config.binding = {42, 3, 0, "/test/camera"}; config.profile = "test-unconfirmed";
  config.segment_frames = 2; config.reserve_bytes = 0; return config;
}
QByteArray read_file(const std::string & path)
{
  QFile file(QString::fromStdString(path)); if (!file.open(QIODevice::ReadOnly)) {return {};}
  return file.readAll();
}
TEST(Recording, DefaultDisabledAndBackendAvailability)
{
  EXPECT_FALSE(RecordingOptions{}.enabled);
  RecordingWorker worker;
  if (!RecordingWorker::available()) {
    QTemporaryDir tmp; EXPECT_FALSE(worker.start(settings(tmp), packet()));
    EXPECT_FALSE(std::filesystem::exists(tmp.path().toStdString() + "/records"));
  }
  worker.stop(); ASSERT_TRUE(until([&] {return worker.state().stopped;}));
}
#ifdef EASYFLY_ENABLE_MEDIA
TEST(Recording, RelocatedRuntimeInstallPrefixCannotBecomeRecordDirectory)
{
  QTemporaryDir tmp; ASSERT_TRUE(tmp.isValid());
  const auto prefix = tmp.path().toStdString() + "/relocated install";
  ASSERT_TRUE(std::filesystem::create_directory(prefix));
  const auto paths = easyfly_desktop::local_storage_protected_paths(
    {}, {{"easyfly_desktop", prefix}}, prefix + "/lib/easyfly_desktop/easyfly_desktop");
  RecordingWorker protected_worker(paths);
  auto config = settings(tmp); config.directory = prefix + "/records";
  ASSERT_TRUE(protected_worker.start(config, packet()));
  ASSERT_TRUE(until([&] {return protected_worker.state().phase == RecordingPhase::Failed;}));
  EXPECT_NE(
    protected_worker.state().error.find("protected project/journal directory"),
    std::string::npos);
  EXPECT_FALSE(std::filesystem::exists(config.directory));
  protected_worker.stop(); ASSERT_TRUE(until([&] {return protected_worker.state().stopped;}));
  RecordingWorker separate_worker(paths); config = settings(tmp);
  ASSERT_TRUE(separate_worker.start(config, packet()));
  ASSERT_TRUE(until([&] {return separate_worker.state().frames == 1;}));
  separate_worker.finish("runtime_prefix_regression");
  ASSERT_TRUE(until([&] {return separate_worker.state().phase == RecordingPhase::Completed;}));
  separate_worker.stop(); ASSERT_TRUE(until([&] {return separate_worker.state().stopped;}));
}
TEST(Recording, SegmentedReadbackMetadataIdentityAndIndex)
{
  QTemporaryDir tmp; ASSERT_TRUE(tmp.isValid()); RecordingWorker worker;
  auto config = settings(tmp); ASSERT_TRUE(worker.start(config, packet()));
  ASSERT_TRUE(
    until(
      [&] {
        return worker.state().frames == 1 || worker.state().phase == RecordingPhase::Failed;
      }));
  ASSERT_EQ(worker.state().frames, 1u) << worker.state().error;
  for (uint64_t i = 2; i <= 5; ++i) {
    QTest::qWait(110); worker.offer(packet(i));
    ASSERT_TRUE(
      until(
        [&] {
          return worker.state().frames == i || worker.state().phase == RecordingPhase::Failed;
        }));
    ASSERT_EQ(worker.state().frames, i) << worker.state().error;
  }
  worker.finish("operator_stop");
  ASSERT_TRUE(
    until(
      [&] {
        return worker.state().phase == RecordingPhase::Completed ||
        worker.state().phase == RecordingPhase::Failed;
      }));
  auto state = worker.state(); ASSERT_EQ(state.phase, RecordingPhase::Completed) << state.error;
  EXPECT_EQ(state.segments, 3u); EXPECT_EQ(state.pending, 0u);
  unsigned total = 0;
  for (unsigned i = 0; i < 3; ++i) {
    const auto path = state.path + "/segment-" + std::to_string(i) + ".avi";
    cv::VideoCapture capture(path, cv::CAP_OPENCV_MJPEG); ASSERT_TRUE(capture.isOpened());
    cv::Mat frame;
    while (capture.read(frame)) {
      EXPECT_EQ(frame.cols, 96); EXPECT_EQ(frame.rows, 64); ++total;
      const auto color = frame.at<cv::Vec3b>(0, 0); EXPECT_NEAR(color[2], 150, 5);
      EXPECT_NEAR(frame.at<cv::Vec3b>(63, 0)[1], 200, 15);
    }
    struct stat info {}; ASSERT_EQ(stat(path.c_str(), &info), 0); EXPECT_EQ(
      info.st_mode & 0777,
      0600u);
  }
  EXPECT_EQ(total, 5u);
  const auto events = read_file(state.path + "/events.jsonl");
  EXPECT_TRUE(events.contains("exec-known")); EXPECT_TRUE(events.contains("uuid-known"));
  EXPECT_TRUE(events.contains("global_unattributed"));
  EXPECT_TRUE(events.contains("query-known-exec"));
  bool saw_global = false;
  for (const auto & line : events.split('\n')) {
    const auto value = QJsonDocument::fromJson(line).object();
    if (value["kind"].toString() == "global_mission_status") {
      saw_global = true; EXPECT_EQ(value["attribution"].toString(), "global_unattributed");
      EXPECT_FALSE(value.contains("execution_id")); EXPECT_FALSE(value.contains("action_uuid"));
    }
  }
  EXPECT_TRUE(saw_global);
  const auto frames = read_file(state.path + "/frames.jsonl");
  EXPECT_EQ(frames.count('\n'), 5); EXPECT_TRUE(frames.contains("source_ns"));
  EXPECT_TRUE(frames.contains("first_received_steady"));
  EXPECT_TRUE(frames.contains("written_utc"));
  EXPECT_TRUE(read_file(state.path + "/config.json").contains("profile_label_unconfirmed"));
  ASSERT_TRUE(worker.scan(config.directory));
  ASSERT_TRUE(until([&] {return worker.state().index.size() == 1;}));
  EXPECT_EQ(worker.state().index[0].status, "verified");
  worker.stop(); ASSERT_TRUE(until([&] {return worker.state().stopped;}));
}
TEST(Recording, MetadataSnapshotBudgetProducesVisibleUnattributedGapWithoutReplay)
{
  QTemporaryDir tmp; RecordingWorker worker; const auto config = settings(tmp);
  ASSERT_TRUE(worker.start(config, packet()));
  ASSERT_TRUE(until([&] {return worker.state().frames == 1;})); QTest::qWait(120);
  auto next = packet(2); auto observation = std::make_shared<bridge::UiSnapshot>(*next.observation);
  observation->events.clear(); const auto now = bridge::detail::steady_seconds();
  for (int i = 0; i < 80; ++i) {
    observation->events.push_back(
      {now - .08 + i * .0005, 3,
        "C06 MOCK metadata event " + std::to_string(i), 42});
  }
  next.observation = observation; worker.offer(next);
  ASSERT_TRUE(until([&] {return worker.state().frames == 2;}));
  worker.offer(next); QTest::qWait(50); worker.finish("operator_stop");
  ASSERT_TRUE(until([&] {return worker.state().phase == RecordingPhase::Completed;}));
  const auto events = read_file(worker.state().path + "/events.jsonl");
  EXPECT_EQ(events.count("\"kind\":\"event_gap\""), 1);
  EXPECT_EQ(events.count("\"kind\":\"observer_event\""), 65);  // Initial event + retained 64.
  bool gap = false;
  for (const auto & line : events.split('\n')) {
    const auto value = QJsonDocument::fromJson(line).object();
    if (value["kind"] == "event_gap") {
      gap = true; EXPECT_EQ(value["reason"], "snapshot event budget");
      EXPECT_FALSE(value.contains("execution_id")); EXPECT_FALSE(value.contains("action_uuid"));
      EXPECT_FALSE(value.contains("request_id"));
    }
  }
  EXPECT_TRUE(gap);
  const auto root = qEnvironmentVariable("EASYFLY_C06_GAP_EVIDENCE_ROOT");
  if (!root.isEmpty()) {
    tmp.setAutoRemove(false); QFile report(root + "/record.json");
    ASSERT_TRUE(report.open(QIODevice::WriteOnly | QIODevice::NewOnly));
    report.write(
      QJsonDocument(
        QJsonObject{{"artifact_directory", tmp.path()},
          {"record_path", QString::fromStdString(worker.state().path)},
          {"kind", "C06 MOCK metadata snapshot budget; initial + 80 offered events"},
          {"gap_count", events.count("\"kind\":\"event_gap\"")},
          {"assertions_passed", !::testing::Test::HasFailure()}}).toJson());
  }
  worker.stop(); ASSERT_TRUE(until([&] {return worker.state().stopped;}));
}
TEST(Recording, InvalidDirectorySymlinkProtectedAndDiskReserve)
{
  QTemporaryDir tmp; auto config = settings(tmp);
  std::filesystem::create_directory(config.directory);  // Ordinary directory cannot be adopted.
  RecordingWorker worker; ASSERT_TRUE(worker.start(config, packet()));
  ASSERT_TRUE(until([&] {return worker.state().phase == RecordingPhase::Failed;}));
  EXPECT_FALSE(std::filesystem::exists(config.directory + "/.easyfly-recordings-v1"));
  auto blocked = config; blocked.directory = tmp.path().toStdString() + "/protected/new";
  std::filesystem::create_directory(tmp.path().toStdString() + "/protected");
  RecordingWorker protected_worker({tmp.path().toStdString() + "/protected"});
  ASSERT_TRUE(protected_worker.start(blocked, packet()));
  ASSERT_TRUE(until([&] {return protected_worker.state().phase == RecordingPhase::Failed;}));
  EXPECT_FALSE(std::filesystem::exists(blocked.directory));
  auto link = config; link.directory = tmp.path().toStdString() + "/link";
  std::filesystem::create_directory_symlink(config.directory, link.directory);
  ASSERT_TRUE(worker.start(link, packet()));
  ASSERT_TRUE(until([&] {return worker.state().phase == RecordingPhase::Failed;}));
  auto full = config; full.directory = tmp.path().toStdString() + "/full";
  full.reserve_bytes = std::numeric_limits<uint64_t>::max();
  ASSERT_TRUE(worker.start(full, packet()));
  ASSERT_TRUE(until([&] {return worker.state().phase == RecordingPhase::Failed;}));
  EXPECT_NE(worker.state().error.find("reserve"), std::string::npos);
}
TEST(Recording, RealWriteFailureRetainsFailedRecord)
{
  if (!qEnvironmentVariableIsSet("EASYFLY_MEDIA_FSIZE_CHILD")) {
    QProcess child; auto environment = QProcessEnvironment::systemEnvironment();
    environment.insert("EASYFLY_MEDIA_FSIZE_CHILD", "1"); child.setProcessEnvironment(environment);
    child.start(
      QCoreApplication::applicationFilePath(),
      {"--gtest_filter=Recording.RealWriteFailureRetainsFailedRecord"});
    ASSERT_TRUE(child.waitForFinished(10000)); EXPECT_EQ(child.exitStatus(), QProcess::NormalExit);
    EXPECT_EQ(child.exitCode(), 0) << child.readAllStandardOutput().toStdString() <<
      child.readAllStandardError().toStdString(); return;
  }
  struct RestoreLimit
  {
    rlimit original{};
    decltype(SIG_DFL) handler;
    RestoreLimit()
    : handler(std::signal(SIGXFSZ, SIG_IGN)) {getrlimit(RLIMIT_FSIZE, &original);}
    ~RestoreLimit() {setrlimit(RLIMIT_FSIZE, &original); std::signal(SIGXFSZ, handler);}
  } restore;
  auto limited = restore.original; limited.rlim_cur = 2048;
  ASSERT_EQ(setrlimit(RLIMIT_FSIZE, &limited), 0);
  QTemporaryDir tmp; RecordingWorker worker; auto p = packet();
  auto noise = std::make_shared<ConvertedFrame>(*p.frame); uint32_t value = 123;
  for (int y = 0; y < noise->image.height(); ++y) {
    for (int x = 0; x < noise->image.width() * 3; ++x) {
      value = value * 1664525 + 1013904223; noise->image.scanLine(y)[x] = value >> 24;
    }
  }
  p.frame = noise; ASSERT_TRUE(worker.start(settings(tmp), p));
  ASSERT_TRUE(until([&] {return worker.state().phase == RecordingPhase::Failed;}));
  EXPECT_NE(worker.state().error.find("write failed"), std::string::npos);
  EXPECT_TRUE(read_file(worker.state().path + "/manifest.json").contains("failed"));
  worker.stop(); ASSERT_TRUE(until([&] {return worker.state().stopped;}));
}
TEST(Recording, PrivateSmallTmpfsRealEnospcNeverResumesOrVerifiesPartialRecord)
{
  const auto root = qEnvironmentVariable("EASYFLY_C07_ENOSPC_ROOT");
  if (root.isEmpty()) {GTEST_SKIP() << "Requires explicit private small tmpfs and mount namespace";}
  ASSERT_TRUE(root.startsWith("/tmp/easyfly-c07-enospc."));
  const auto host_namespace = qEnvironmentVariable("EASYFLY_C07_HOST_MOUNT_NS");
  ASSERT_FALSE(host_namespace.isEmpty()); char target[128];
  const auto length = readlink("/proc/self/ns/mnt", target, sizeof(target)); ASSERT_GT(length, 0);
  ASSERT_NE(QString::fromLocal8Bit(target, length), host_namespace);
  struct statfs fs {}; ASSERT_EQ(statfs(root.toLocal8Bit().constData(), &fs), 0);
  ASSERT_EQ(fs.f_type, TMPFS_MAGIC);
  ASSERT_GT(fs.f_blocks, 0u); ASSERT_LE(
    static_cast<uint64_t>(fs.f_blocks) * fs.f_bsize,
    16ULL * 1024 * 1024);
  QTemporaryDir tmp(root + "/case-XXXXXX"); ASSERT_TRUE(tmp.isValid()); tmp.setAutoRemove(false);
  RecordingWorker worker; const auto config = settings(tmp);
  ASSERT_TRUE(worker.start(config, packet()));
  ASSERT_TRUE(until([&] {return worker.state().frames == 1;}));
  struct Filler
  {
    int fd{-1};
    ~Filler() {if (fd >= 0) {::close(fd);}}
  } filler;
  filler.fd = ::open(
    (tmp.path() + "/private-generated-filler").toLocal8Bit().constData(),
    O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600); ASSERT_GE(filler.fd, 0);
  std::array<char, 65536> block{}; uint64_t written = 0; int write_error = 0;
  while (written <= 16ULL * 1024 * 1024) {
    const auto count = ::write(filler.fd, block.data(), block.size());
    if (count < 0) {write_error = errno; break;} ASSERT_GT(count, 0); written += count;
  }
  ASSERT_EQ(write_error, ENOSPC); ASSERT_GT(written, 8192u);
  // Leave one block: the reserve precheck passes, but a larger frame encounters real ENOSPC.
  ASSERT_EQ(ftruncate(filler.fd, written - 4096), 0); QTest::qWait(120);
  auto next = packet(2); auto noise = std::make_shared<ConvertedFrame>(*next.frame);
  noise->image = QImage(512, 512, QImage::Format_RGB888); uint32_t random = 123;
  for (int y = 0; y < 512; ++y) {
    for (int x = 0; x < 512 * 3; ++x) {
      random = random * 1664525 + 1013904223; noise->image.scanLine(y)[x] = random >> 24;
    }
  }
  next.frame = noise; worker.offer(next);
  ASSERT_TRUE(until([&] {return worker.state().phase == RecordingPhase::Failed;}));
  const auto failed = worker.state(); EXPECT_EQ(failed.frames, 1u);
  EXPECT_NE(failed.error.find("write failed"), std::string::npos);
  EXPECT_TRUE(QFileInfo::exists(QString::fromStdString(failed.path) + "/inprogress.json"));
  ASSERT_EQ(ftruncate(filler.fd, 0), 0);  // Reclaim only this newly generated filler, no records.
  worker.offer(packet(3)); QTest::qWait(150);
  EXPECT_EQ(worker.state().phase, RecordingPhase::Failed); EXPECT_EQ(worker.state().id, failed.id);
  EXPECT_EQ(worker.state().frames, 1u); EXPECT_EQ(worker.state().pending, 0u);
  ASSERT_TRUE(worker.scan(config.directory));
  ASSERT_TRUE(until([&] {return !worker.state().index_pending;}));
  ASSERT_EQ(worker.state().index.size(), 1u); EXPECT_NE(worker.state().index[0].status, "verified");
  const auto output = qEnvironmentVariable("EASYFLY_C07_ENOSPC_EVIDENCE_ROOT");
  if (!output.isEmpty()) {
    QFile report(output + "/result.json");
    ASSERT_TRUE(report.open(QIODevice::WriteOnly | QIODevice::NewOnly));
    report.write(
      QJsonDocument(
        QJsonObject{{"kind", "private <=16MiB tmpfs real ENOSPC"},
          {"record_path", QString::fromStdString(failed.path)}, {"write_errno", write_error},
          {"filler_bytes_before_failure", QString::number(written)},
          {"record_error", QString::fromStdString(failed.error)},
          {"index_status", QString::fromStdString(worker.state().index[0].status)},
          {"index_reason", QString::fromStdString(worker.state().index[0].reason)},
          {"assertions_passed", !::testing::Test::HasFailure()}}).toJson());
  }
  worker.stop(); ASSERT_TRUE(until([&] {return worker.state().stopped;}));
}
TEST(Recording, BoundedQueueRateLimitAndByteBudgetFailure)
{
  QTemporaryDir tmp; auto config = settings(tmp); RecordingWorker worker;
  ASSERT_TRUE(worker.start(config, packet()));
  ASSERT_TRUE(until([&] {return worker.state().frames == 1;}));
  for (uint64_t i = 2; i < 1000; ++i) {
    worker.offer(packet(i)); EXPECT_LE(worker.state().pending, 1u);
  }
  QTest::qWait(30); EXPECT_GT(worker.state().replaced + worker.state().rate_dropped, 0u);
  worker.stop(); ASSERT_TRUE(until([&] {return worker.state().stopped;}));
  EXPECT_EQ(worker.state().phase, RecordingPhase::Completed);
  config.directory = tmp.path().toStdString() + "/budget"; config.byte_limit = 65536;
  RecordingWorker small; ASSERT_TRUE(small.start(config, packet()));
  ASSERT_TRUE(until([&] {return small.state().phase == RecordingPhase::Failed;}));
  EXPECT_NE(small.state().error.find("byte limit"), std::string::npos);
  EXPECT_TRUE(read_file(small.state().path + "/manifest.json").contains("failed"));
}
TEST(Recording, CorruptionAndIncompleteRecordsNeverMarkedVerified)
{
  QTemporaryDir tmp; auto config = settings(tmp); RecordingWorker worker;
  ASSERT_TRUE(worker.start(config, packet())); ASSERT_TRUE(
    until(
      [&] {
        return worker.state().frames == 1;
      }));
  worker.finish("test"); ASSERT_TRUE(
    until(
      [&] {
        return worker.state().phase == RecordingPhase::Completed;
      }));
  const auto path = worker.state().path;
  QFile video(QString::fromStdString(path + "/segment-0.avi"));
  ASSERT_TRUE(video.open(QIODevice::ReadWrite));
  ASSERT_TRUE(video.seek(300)); ASSERT_EQ(video.write("BAD!", 4), 4); video.close();
  ASSERT_TRUE(worker.scan(config.directory)); ASSERT_TRUE(
    until(
      [&] {
        return !worker.state().index.empty();
      }));
  EXPECT_EQ(worker.state().index[0].status, "incomplete/corrupt");
  EXPECT_NE(worker.state().index[0].reason.find("integrity"), std::string::npos);
}
TEST(RecordingPanel, ExplicitConfirmationAndSourceChangeFinalizes)
{
  QTemporaryDir tmp; RecordingPanel panel({}); panel.show();
  auto first = packet(); const auto config = settings(tmp);
  auto * directory = panel.findChild<QLineEdit *>("recordingDirectory"); ASSERT_NE(
    directory,
    nullptr);
  directory->setText(QString::fromStdString(config.directory));
  panel.observe(first, config.binding, true, true, "test");
  EXPECT_FALSE(std::filesystem::exists(config.directory));
  auto * button = panel.findChild<QPushButton *>("recordingStart"); ASSERT_NE(button, nullptr);
  ASSERT_TRUE(until([&] {return button->isEnabled();})); QTest::mouseClick(button, Qt::LeftButton);
  auto * prompt = panel.findChild<QMessageBox *>("recordingConfirmation");
  ASSERT_NE(prompt, nullptr);
  EXPECT_FALSE(std::filesystem::exists(config.directory)); prompt->done(QMessageBox::Ok);
  ASSERT_TRUE(
    until(
      [&] {
        return std::filesystem::exists(config.directory + "/.easyfly-recordings-v1");
      }));
  auto changed = config.binding; changed.camera = 1;
  panel.observe({}, changed, true, true, "test");
  ASSERT_TRUE(
    until(
      [&] {
        for (const auto & e : std::filesystem::directory_iterator(config.directory)) {
          if (e.is_directory() && std::filesystem::exists(e.path() / "manifest.json")) {
            return true;
          }
        }
        return false;
      }));
  panel.stop(); ASSERT_TRUE(until([&] {return panel.stopped();}));
}
TEST(Recording, NewSessionNeverOverwritesAndStaleFramesAreDropped)
{
  QTemporaryDir tmp; auto config = settings(tmp); RecordingWorker worker;
  ASSERT_TRUE(worker.start(config, packet()));
  ASSERT_TRUE(until([&] {return worker.state().frames == 1;}));
  auto stale_packet = packet(2);
  auto stale = std::make_shared<ConvertedFrame>(*stale_packet.frame);
  stale->first_received -= 2; stale_packet.frame = stale; worker.offer(stale_packet);
  ASSERT_TRUE(until([&] {return worker.state().stale_dropped == 1;}));
  worker.finish("first");
  ASSERT_TRUE(until([&] {return worker.state().phase == RecordingPhase::Completed;}));
  const auto first_path = worker.state().path; const auto old = read_file(
    first_path + "/manifest.json");
  ASSERT_TRUE(worker.start(config, packet(3)));
  ASSERT_TRUE(until([&] {return worker.state().frames == 1;}));
  worker.stop(); ASSERT_TRUE(until([&] {return worker.state().stopped;}));
  EXPECT_NE(worker.state().path, first_path); EXPECT_EQ(
    read_file(
      first_path + "/manifest.json"), old);
  EXPECT_TRUE(read_file(worker.state().path + "/manifest.json").contains("window_closed"));
}
TEST(Recording, IncompleteAndMetadataCorruptionAreVisible)
{
  QTemporaryDir tmp; auto config = settings(tmp); RecordingWorker worker;
  ASSERT_TRUE(worker.start(config, packet())); ASSERT_TRUE(
    until(
      [&] {
        return worker.state().frames == 1;
      }));
  worker.finish("test"); ASSERT_TRUE(
    until(
      [&] {
        return worker.state().phase == RecordingPhase::Completed;
      }));
  QFile events(QString::fromStdString(worker.state().path + "/events.jsonl"));
  ASSERT_TRUE(events.open(QIODevice::Append)); ASSERT_GT(events.write("corruption\n"), 0);
  events.close();
  const auto orphan = config.directory + "/10000000-0000-0000-0000-000000000001";
  ASSERT_EQ(mkdir(orphan.c_str(), 0700), 0);
  ASSERT_TRUE(worker.scan(config.directory));
  ASSERT_TRUE(
    until(
      [&] {
        return worker.state().index.size() == 2 && !worker.state().index_pending;
      }));
  for (const auto & entry : worker.state().index) {
    EXPECT_EQ(entry.status, "incomplete/corrupt");
  }
}
TEST(Recording, InvalidClockLatchesFinishBeforeRecoveryAndAllowsExplicitNewRecord)
{
  for (int kind = 0; kind < 7; ++kind) {
    SCOPED_TRACE(kind);
    QTemporaryDir tmp; const auto config = settings(tmp); RecordingWorker worker;
    ASSERT_TRUE(worker.start(config, packet()));
    ASSERT_TRUE(until([&] {return worker.state().frames == 1;}));
    auto invalid = packet(2);
    auto s = std::make_shared<bridge::UiSnapshot>(*invalid.observation);
    std::string reason;
    switch (kind) {
      case 0: s->clock.quality = bridge::Quality::ClockPaused; reason = "clock_paused"; break;
      case 1: s->clock.quality = bridge::Quality::ClockNotReady; reason = "clock_unavailable";
        break;
      case 2: s->clock.quality = bridge::Quality::ClockReset; reason = "clock_unavailable"; break;
      case 3: ++s->time_epoch; reason = "camera_session_clock_changed"; break;
      case 4: ++s->observation_session_id; reason = "camera_session_clock_changed"; break;
      case 5: s->produced_steady -= 2; reason = "clock_evidence_stale"; break;
      default: s.reset(); reason = "observation_disconnected"; break;
    }
    invalid.observation = s; invalid.frame.reset(); worker.offer(invalid);
    worker.offer(packet(3));  // No event-loop wait: recovery must not overwrite the stop evidence.
    ASSERT_TRUE(until([&] {return worker.state().phase == RecordingPhase::Completed;}));
    EXPECT_EQ(worker.state().reason, reason); EXPECT_EQ(worker.state().frames, 1u);
    const auto original_path = worker.state().path;
    const auto manifest = read_file(original_path + "/manifest.json");
    EXPECT_TRUE(manifest.contains("verified")); EXPECT_TRUE(manifest.contains(reason.c_str()));
    worker.offer(packet(4)); QTest::qWait(120);
    EXPECT_EQ(worker.state().frames, 1u);
    EXPECT_EQ(read_file(original_path + "/manifest.json"), manifest);
    ASSERT_TRUE(worker.start(config, packet(5)));
    ASSERT_TRUE(until([&] {return worker.state().frames == 1;}));
    EXPECT_NE(worker.state().path, original_path);
    worker.stop(); ASSERT_TRUE(until([&] {return worker.state().stopped;}));
    EXPECT_EQ(read_file(original_path + "/manifest.json"), manifest);
  }
}
TEST(Recording, InvalidInitialClockCreatesNoFiles)
{
  QTemporaryDir tmp; const auto config = settings(tmp); RecordingWorker worker;
  auto invalid = packet(); invalid.observation.reset();
  EXPECT_FALSE(worker.start(config, invalid));
  auto s = std::make_shared<bridge::UiSnapshot>(*packet().observation);
  s->produced_steady = std::numeric_limits<double>::quiet_NaN(); invalid.observation = s;
  EXPECT_FALSE(worker.start(config, invalid));
  EXPECT_FALSE(std::filesystem::exists(config.directory));
}
TEST(Recording, ClockStopDuringStartingCannotBeReplacedByRecovery)
{
  QTemporaryDir tmp; const auto config = settings(tmp); RecordingWorker worker;
  ASSERT_TRUE(worker.start(config, packet()));
  auto invalid = packet(2); auto s = std::make_shared<bridge::UiSnapshot>(*invalid.observation);
  s->clock.quality = bridge::Quality::ClockPaused; invalid.observation = s;
  worker.offer(invalid); worker.offer(packet(3));
  ASSERT_TRUE(until([&] {return worker.state().phase == RecordingPhase::Completed;}));
  EXPECT_EQ(worker.state().reason, "clock_paused"); EXPECT_LE(worker.state().frames, 1u);
  const auto manifest = read_file(worker.state().path + "/manifest.json");
  worker.offer(packet(4)); QTest::qWait(120);
  EXPECT_EQ(worker.state().phase, RecordingPhase::Completed);
  EXPECT_EQ(read_file(worker.state().path + "/manifest.json"), manifest);
}
TEST(RecordingPanel, PausedClockAndFrozenEvidenceFinishWithoutAutomaticResume)
{
  for (const bool pause : {true, false}) {
    SCOPED_TRACE(pause);
    QTemporaryDir tmp; const auto config = settings(tmp); RecordingPanel panel({}); panel.show();
    panel.findChild<QLineEdit *>("recordingDirectory")->setText(
      QString::fromStdString(config.directory));
    panel.observe(packet(), config.binding, true, true, "test");
    auto * button = panel.findChild<QPushButton *>("recordingStart");
    ASSERT_TRUE(until([&] {return button->isEnabled();})); button->click();
    panel.findChild<QMessageBox *>("recordingConfirmation")->done(QMessageBox::Ok);
    auto * label = panel.findChild<QLabel *>("recordingStatus");
    ASSERT_TRUE(until([&] {return label->text().contains(QStringLiteral("已写 1 帧"));}));
    if (pause) {
      auto invalid = packet(2); auto s = std::make_shared<bridge::UiSnapshot>(*invalid.observation);
      s->clock.quality = bridge::Quality::ClockPaused; invalid.observation = s;
      invalid.frame.reset(); panel.observe(invalid, config.binding, true, true, "test");
    }  // Otherwise leave the snapshot frozen: the panel timer must expire existing evidence.
    ASSERT_TRUE(until([&] {return label->text().contains(QStringLiteral("已收尾"));}));
    EXPECT_TRUE(label->text().contains(pause ? "clock_paused" : "clock_evidence_stale"));
    panel.observe(packet(3), config.binding, true, true, "test"); QTest::qWait(150);
    EXPECT_TRUE(label->text().contains(QStringLiteral("已收尾")));
    EXPECT_TRUE(label->text().contains(QStringLiteral("已写 1 帧")));
    ASSERT_TRUE(button->isEnabled()); button->click();
    ASSERT_NE(panel.findChild<QMessageBox *>("recordingConfirmation"), nullptr);
    panel.findChild<QMessageBox *>("recordingConfirmation")->done(QMessageBox::Cancel);
    panel.stop(); ASSERT_TRUE(until([&] {return panel.stopped();}));
  }
}
TEST(RecordingPanel, CanceledAndExpiredConfirmationDoNotWrite)
{
  QTemporaryDir tmp; RecordingPanel panel({}); panel.show(); const auto config = settings(tmp);
  panel.findChild<QLineEdit *>("recordingDirectory")->setText(
    QString::fromStdString(
      config.
      directory));
  auto p = packet(); panel.observe(p, config.binding, true, true, "test");
  auto * button = panel.findChild<QPushButton *>("recordingStart");
  ASSERT_TRUE(until([&] {return button->isEnabled();})); QTest::mouseClick(button, Qt::LeftButton);
  auto * prompt = panel.findChild<QMessageBox *>("recordingConfirmation");
  ASSERT_NE(prompt, nullptr);
  prompt->done(QMessageBox::Cancel); EXPECT_FALSE(std::filesystem::exists(config.directory));
  QTest::qWait(20);
  QTest::mouseClick(button, Qt::LeftButton); prompt = panel.findChild<QMessageBox *>(
    "recordingConfirmation");
  ASSERT_NE(prompt, nullptr);
  ASSERT_TRUE(until([&] {return !panel.findChild<QMessageBox *>("recordingConfirmation");}));
  EXPECT_FALSE(std::filesystem::exists(config.directory)); panel.stop();
  ASSERT_TRUE(until([&] {return panel.stopped();}));
}
TEST(RecordingPanel, DisconnectAndClockSourceChangeFinishWithoutResume)
{
  for (const bool disconnect : {true, false}) {
    QTemporaryDir tmp; const auto config = settings(tmp); RecordingPanel panel({}); panel.show();
    panel.findChild<QLineEdit *>("recordingDirectory")->setText(
      QString::fromStdString(
        config.
        directory));
    auto p = packet(); panel.observe(p, config.binding, true, true, "test");
    auto * button = panel.findChild<QPushButton *>("recordingStart");
    ASSERT_TRUE(until([&] {return button->isEnabled();}));
    QTest::mouseClick(button, Qt::LeftButton);
    panel.findChild<QMessageBox *>("recordingConfirmation")->done(QMessageBox::Ok);
    auto * label = panel.findChild<QLabel *>("recordingStatus");
    ASSERT_TRUE(until([&] {return label->text().contains(QStringLiteral("已写 1 帧"));}));
    panel.observe({}, config.binding, !disconnect, false, "test");
    ASSERT_TRUE(until([&] {return label->text().contains(QStringLiteral("已收尾"));}));
    panel.observe(packet(2), config.binding, true, true, "test"); QTest::qWait(150);
    EXPECT_TRUE(label->text().contains(QStringLiteral("已收尾"))); panel.stop();
    ASSERT_TRUE(until([&] {return panel.stopped();}));
  }
}
TEST(RecordingPanel, SourceChangeOrCloseDismissesOpenPromptWithoutWriting)
{
  for (const bool close : {true, false}) {
    QTemporaryDir tmp; auto config = settings(tmp); RecordingPanel panel({}); panel.show();
    panel.findChild<QLineEdit *>("recordingDirectory")->setText(
      QString::fromStdString(
        config.
        directory));
    panel.observe(packet(), config.binding, true, true, "test");
    auto * button = panel.findChild<QPushButton *>("recordingStart");
    ASSERT_TRUE(until([&] {return button->isEnabled();}));
    QTest::mouseClick(button, Qt::LeftButton);
    ASSERT_NE(panel.findChild<QMessageBox *>("recordingConfirmation"), nullptr);
    if (close) {panel.stop();} else {
      auto changed = config.binding; ++changed.epoch;
      panel.observe({}, changed, true, true, "test");
      panel.stop();
    }
    ASSERT_TRUE(until([&] {return panel.stopped();}));
    EXPECT_FALSE(std::filesystem::exists(config.directory));
    EXPECT_EQ(panel.findChild<QMessageBox *>("recordingConfirmation"), nullptr);
  }
}
TEST(RecordingWindow, AsyncCloseFinalizesWithNoTaskControl)
{
  QTemporaryDir tmp; auto fake = std::make_shared<desktop_test::FakeState>();
  auto image = std::make_shared<sensor_msgs::msg::Image>(); image->header.stamp.sec = 10;
  image->header.frame_id = "camera"; image->width = 96; image->height = 64;
  image->encoding = "rgb8";
  image->step = 288; image->data.assign(96 * 64 * 3, 120);
  fake->data->images[0] = {image, desktop_test::evidence()};
  easyfly_desktop::MainWindow window({}, desktop_test::factory(fake), nullptr,
    easyfly_desktop::default_tool_options(), {}, easyfly_desktop::control_factory(), {},
    easyfly_desktop::parameter_factory(), {}, easyfly_desktop::import_factory(), {}, {true});
  window.show();
  auto * panel = window.findChild<RecordingPanel *>("recordingPage"); ASSERT_NE(panel, nullptr);
  const auto config = settings(tmp);
  panel->findChild<QLineEdit *>("recordingDirectory")->setText(
    QString::fromStdString(
      config.
      directory));
  auto * nav = window.findChild<QListWidget *>("navigation");
  if (!nav) {
    const auto lists = window.findChildren<QListWidget *>();
    for (auto * list : lists) {
      if (list->count() >= 7) {
        nav = list; break;
      }
    }
  }
  ASSERT_NE(nav, nullptr); nav->setCurrentRow(nav->count() - 1);
  auto * start = panel->findChild<QPushButton *>("recordingStart");
  ASSERT_TRUE(until([&] {return start->isEnabled();})); QTest::mouseClick(start, Qt::LeftButton);
  panel->findChild<QMessageBox *>("recordingConfirmation")->done(QMessageBox::Ok);
  auto * label = panel->findChild<QLabel *>("recordingStatus");
  ASSERT_TRUE(until([&] {return label->text().contains(QStringLiteral("已写 1 帧"));}));
  const auto screenshot = qEnvironmentVariable("EASYFLY_RECORDING_SCREENSHOT");
  if (!screenshot.isEmpty()) {
    EXPECT_TRUE(window.grab().save(screenshot));
    window.resize(1024, 720); QTest::qWait(100);
    EXPECT_TRUE(window.grab().save(screenshot + ".compact.png"));
    tmp.setAutoRemove(false); qInfo().noquote() << "RETAINED_MEDIA_ARTIFACT" << tmp.path();
  }
  const auto before = bridge::detail::steady_seconds(); window.close();
  EXPECT_LT(bridge::detail::steady_seconds() - before, .1);
  ASSERT_TRUE(until([&] {return window.cleanup_complete();})); EXPECT_FALSE(window.isVisible());
  EXPECT_TRUE(panel->stopped()); EXPECT_EQ(fake->stop_calls, 1);
  EXPECT_EQ(fake->list_calls + fake->query_calls + fake->validate_calls, 0);
  bool finalized = false;
  for (const auto & entry : std::filesystem::directory_iterator(config.directory)) {
    if (entry.is_directory() && std::filesystem::exists(entry.path() / "manifest.json")) {
      finalized = read_file((entry.path() / "manifest.json").string()).contains("verified");
    }
  }
  EXPECT_TRUE(finalized);
}
TEST(Recording, HistoricalEntryRequiresVerifiedIndexAndMatchingDirectory)
{
  QTemporaryDir tmp; RecordingWorker worker; const auto config = settings(tmp);
  ASSERT_TRUE(worker.start(config, packet()));
  ASSERT_TRUE(until([&] {return worker.state().frames == 1;}));
  worker.finish("operator_stop");
  ASSERT_TRUE(until([&] {return worker.state().phase == RecordingPhase::Completed;}));
  RecordingPanel panel({}); panel.show();
  auto * directory = panel.findChild<QLineEdit *>("recordingDirectory");
  auto * index = panel.findChild<QTableWidget *>("recordingIndex");
  auto * replay = panel.findChild<QPushButton *>("recordingReplay");
  ASSERT_NE(replay, nullptr); EXPECT_FALSE(replay->isEnabled());
  directory->setText(QString::fromStdString(config.directory));
  QTest::qWait(150);
  QTest::mouseClick(panel.findChild<QPushButton *>("recordingScan"), Qt::LeftButton);
  ASSERT_TRUE(until([&] {return index->rowCount() == 1;}));
  EXPECT_FALSE(replay->isEnabled()); index->setCurrentCell(0, 0);
  ASSERT_TRUE(until([&] {return replay->isEnabled();}));
  directory->setText(tmp.path() + "/unscanned");
  ASSERT_TRUE(until([&] {return !replay->isEnabled();}));
  QTest::mouseClick(panel.findChild<QPushButton *>("recordingScan"), Qt::LeftButton);
  ASSERT_TRUE(until([&] {return index->rowCount() == 0;}));
  EXPECT_FALSE(replay->isEnabled()); panel.stop(); worker.stop();
  ASSERT_TRUE(until([&] {return panel.stopped() && worker.state().stopped;}));
}
#endif
}  // namespace

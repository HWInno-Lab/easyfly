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
#include <unistd.h>
#include <filesystem>
#include <fstream>
#include <memory>
#include <iomanip>
#include <sstream>
#include <string>
#include "journal.hpp"
#include "confirmation_ledger.hpp"
using namespace easyfly_ui_control;  // NOLINT(build/namespaces)
namespace fs = std::filesystem;
TEST(ConfirmationLedgerTest, BoundedNoEvictionAndMalformedIdsDoNotConsumeCapacity)
{
  ConfirmationLedger ledger;
  EXPECT_FALSE(ledger.consume("bad\nid")); EXPECT_FALSE(ledger.consume(""));
  for (int i = 0; i < 256; ++i) {
    std::ostringstream id; id << std::hex << std::setw(32) << std::setfill('0') << i;
    EXPECT_TRUE(ledger.consume(id.str())); EXPECT_FALSE(ledger.consume(id.str()));
  }
  EXPECT_FALSE(ledger.consume(std::string(32, 'f')));
  EXPECT_FALSE(ledger.consume(std::string(32, '0')));  // Old IDs were not evicted.
  EXPECT_FALSE(ledger.consume(""));  // No alternate path bypasses the confirmation ledger.
}
class JournalTest : public ::testing::Test
{
protected:
  fs::path root;
  void SetUp() override
  {
    char path[] = "/tmp/easyfly-control-journal-XXXXXX";
    const auto directory = mkdtemp(path); ASSERT_NE(directory, nullptr); root = directory;
  }
  void TearDown() override {if (!root.empty()) {fs::remove_all(root);}}
  Intent intent()
  {
    return {"domain=113;localhost=1", std::string(32, 'a'), std::string(32, 'b'),
      "demo.mission", std::string(64, 'c'), "", "", false, false};
  }
  void write(const std::string & text)
  {
    std::ofstream(root / "intent.json", std::ios::binary) << text;
    chmod((root / "intent.json").c_str(), 0600);
  }
};
TEST_F(JournalTest, EmptyRoundTripAndDurableReplacement)
{
  Journal j(root.string()); EXPECT_FALSE(j.load());
  auto v = intent(); j.save(v);
  ASSERT_TRUE(j.load()); EXPECT_EQ(j.load()->request, v.request); EXPECT_FALSE(j.load()->resolved);
  v.execution = std::string(32, 'd'); v.goal = std::string(32, 'e');
  v.cancel_dispatched = true; j.save(v);
  EXPECT_TRUE(j.load()->cancel_dispatched);
  v.resolved = true; j.save(v); EXPECT_TRUE(j.load()->resolved);
  struct stat st {}; ASSERT_EQ(stat((root / "intent.json").c_str(), &st), 0);
  EXPECT_EQ(st.st_mode & 0777, 0600);
  EXPECT_EQ(std::distance(fs::directory_iterator(root), fs::directory_iterator{}), 2);
}
TEST_F(JournalTest, ExclusiveOwnershipAndReopen)
{
  {
    Journal j(root.string());
    EXPECT_THROW(Journal another(root.string()), std::runtime_error);
    j.save(intent());
  }
  Journal j(root.string()); ASSERT_TRUE(j.load()); EXPECT_FALSE(j.load()->resolved);
}
TEST_F(JournalTest, CreatesDedicatedPrivateChildDirectory)
{
  const auto path = root / "private"; Journal j(path.string()); j.save(intent());
  struct stat st {}; ASSERT_EQ(stat(path.c_str(), &st), 0);
  EXPECT_EQ(st.st_mode & 0777, 0700); EXPECT_TRUE(j.load());
}
TEST_F(JournalTest, CorruptDuplicateUnknownAndOversizedNeverEmpty)
{
  Journal j(root.string());
  for (const auto & bad : {"", "{}", "{\"schema_version\":1,\"schema_version\":1}", "null",
      "{\"schema_version\":2}", "{\"schema_version\":1} trailing"})
  {
    write(bad); EXPECT_THROW(j.load(), std::runtime_error) << bad;
  }
  write(std::string(4097, 'x')); EXPECT_THROW(j.load(), std::runtime_error);
  j.save(intent());
  std::ifstream stream(root / "intent.json");
  std::string text((std::istreambuf_iterator<char>(stream)), {});
  text.insert(1, "\"unknown\":0,"); write(text); EXPECT_THROW(j.load(), std::runtime_error);
}
TEST_F(JournalTest, SymlinkHardlinkAndFifoRejected)
{
  Journal j(root.string());
  const auto source = root / "source"; std::ofstream(source) << "{}";
  fs::create_symlink(source, root / "intent.json");
  EXPECT_THROW(j.load(), std::runtime_error); fs::remove(root / "intent.json");
  fs::create_hard_link(source, root / "intent.json");
  EXPECT_THROW(j.load(), std::runtime_error); fs::remove(root / "intent.json");
  ASSERT_EQ(mkfifo((root / "intent.json").c_str(), 0600), 0);
  EXPECT_THROW(j.load(), std::runtime_error);
}
TEST_F(JournalTest, UnsafeDirectoryAndLockRejected)
{
  chmod(root.c_str(), 0755); EXPECT_THROW(Journal j(root.string()), std::runtime_error);
  chmod(root.c_str(), 0700);
  fs::create_symlink(root / "missing", root / "lock");
  EXPECT_THROW(Journal j(root.string()), std::runtime_error);
  fs::create_directory_symlink(root, root / "linked");
  EXPECT_THROW(Journal j((root / "linked").string()), std::runtime_error);
  EXPECT_THROW(Journal j("relative-path"), std::invalid_argument);
  EXPECT_THROW(Journal j(root.string() + "/"), std::invalid_argument);
  EXPECT_THROW(Journal j(root.string() + "/."), std::invalid_argument);
}
TEST_F(JournalTest, InvalidIdentityCannotOverwriteUnresolvedRecord)
{
  Journal j(root.string()); const auto good = intent(); j.save(good);
  auto bad = good; bad.hash = "not-a-hash";
  EXPECT_THROW(j.save(bad), std::runtime_error); EXPECT_EQ(j.load()->hash, good.hash);
  bad = good; bad.cancel_dispatched = true;
  EXPECT_THROW(j.save(bad), std::runtime_error);
  bad = good; bad.scope = "domain=999;localhost=1";
  EXPECT_THROW(j.save(bad), std::runtime_error);
}
TEST_F(JournalTest, FailedCommitKeepsOldRecord)
{
  Journal j(root.string()); j.save(intent());
  fs::rename(root / "intent.json", root / "original.json");
  fs::create_directory(root / "intent.json");
  auto v = intent(); v.resolved = true;
  EXPECT_THROW(j.save(v), std::runtime_error);
  EXPECT_TRUE(fs::exists(root / "original.json"));
  fs::remove(root / "intent.json"); fs::rename(root / "original.json", root / "intent.json");
  EXPECT_FALSE(j.load()->resolved);
}
TEST_F(JournalTest, ValidJsonBitCorruptionFailsIntegrityCheck)
{
  Journal j(root.string()); j.save(intent());
  std::ifstream stream(root / "intent.json");
  std::string text((std::istreambuf_iterator<char>(stream)), {});
  const auto at = text.find("\"resolved\":false"); ASSERT_NE(at, std::string::npos);
  text.replace(at, std::string("\"resolved\":false").size(), "\"resolved\":true");
  write(text); EXPECT_THROW(j.load(), std::runtime_error);
}

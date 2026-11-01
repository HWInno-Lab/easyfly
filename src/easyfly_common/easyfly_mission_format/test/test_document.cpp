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
#include <unistd.h>
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <string>
#include "easyfly_mission_format/mission_format.hpp"
namespace
{
namespace format = easyfly_mission_fsm;
class MissionDocumentTest : public ::testing::Test
{
protected:
  std::filesystem::path directory;
  void SetUp() override
  {
    char name[] = "/tmp/easyfly-mission-document-XXXXXX";
    const auto * result = mkdtemp(name); ASSERT_NE(result, nullptr); directory = result;
  }
  void TearDown() override {std::filesystem::remove_all(directory);}
  void write(const std::string & content)
  {
    std::ofstream stream(directory / "mission.yaml", std::ios::binary); stream << content;
    ASSERT_TRUE(stream.good());
  }
};
TEST_F(MissionDocumentTest, PreservesValidatedCrLfBytesAndContentHash)
{
  const std::string content = "schema_version: 1\r\nframe_id: odom\r\n"
    "units: {position: m, angle: rad, time: s}\r\n"
    "steps: [{type: hover, duration: 0.2}]\r\n";
  write(content);
  const auto result = format::loadMissionDocument((directory / "mission.yaml").string());
  ASSERT_TRUE(result.validation.valid()) << result.validation.error;
  EXPECT_EQ(result.content, content);
  EXPECT_EQ(
    format::missionContentSha256(result.content),
    "ed6a0de793da13212216a9bd0de38a6ddf96461cbe2545466455293af45836f9");
  ASSERT_EQ(result.validation.steps.size(), 1u);
  EXPECT_EQ(result.validation.steps[0].type, "hover");
  // A subsequent path replacement cannot change the already validated document.
  write("not a mission");
  EXPECT_EQ(result.content, content);
  EXPECT_FALSE(format::validateMissionFile((directory / "mission.yaml").string()).valid());
}
TEST_F(MissionDocumentTest, RejectsDirectoriesFifosAndOversizedFilesBeforeParsing)
{
  const auto folder = format::loadMissionDocument(directory.string());
  EXPECT_FALSE(folder.validation.valid()); EXPECT_TRUE(folder.content.empty());
  const auto fifo = directory / "fifo"; ASSERT_EQ(mkfifo(fifo.c_str(), 0600), 0);
  const auto pipe = format::loadMissionDocument(fifo.string());
  EXPECT_FALSE(pipe.validation.valid()); EXPECT_TRUE(pipe.content.empty());
  write(std::string(65537, 'x'));
  const auto large = format::loadMissionDocument((directory / "mission.yaml").string());
  EXPECT_FALSE(large.validation.valid()); EXPECT_TRUE(large.content.empty());
  EXPECT_FALSE(format::validateMissionFile((directory / "mission.yaml").string()).valid());
}
}  // namespace

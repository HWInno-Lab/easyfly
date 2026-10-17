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

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <string>

#include "easyfly_mission_format/mission_format.hpp"

namespace mission = easyfly_mission_fsm;
TEST(MissionMetadata, ExactByteDigestAndUtf8)
{
  EXPECT_EQ(
    mission::missionContentSha256("abc"),
    "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_NE(mission::missionContentSha256("abc"), mission::missionContentSha256("abc\n"));
  EXPECT_TRUE(mission::missionTextUtf8("任务说明"));
  EXPECT_TRUE(mission::missionTextUtf8(""));
  for (const auto & invalid : {std::string("a\0b", 3), std::string("\xc0\x80", 2),
      std::string("\xed\xa0\x80", 3), std::string("\xf4\x90\x80\x80", 4),
      std::string("\xe4\xb8", 2)})
  {
    EXPECT_FALSE(mission::missionTextUtf8(invalid));
  }
}
TEST(MissionMetadata, ClassificationDoesNotGrantValidation)
{
  EXPECT_EQ(
    mission::missionFormatName("# comment\n---\naltitude: 1\ntype: takeoff\n"),
    "legacy_flat");
  EXPECT_EQ(mission::missionFormatName("schema_version: 1\n"), "yaml_v1");
  EXPECT_FALSE(mission::validateMissionText("schema_version: 1\n").valid());
}
TEST(MissionMetadata, FileReadingRemainsBoundedAndRejectsSpecialFiles)
{
  char directory[] = "/tmp/easyfly-format-test.XXXXXX";
  ASSERT_NE(mkdtemp(directory), nullptr);
  const std::string root(directory);
  struct Cleanup
  {
    std::string root;
    ~Cleanup()
    {
      unlink((root + "/valid").c_str()); unlink((root + "/large").c_str());
      unlink((root + "/fifo").c_str()); rmdir(root.c_str());
    }
  } cleanup{root};
  {std::ofstream out(root + "/valid"); out << "type=land\n";}
  EXPECT_TRUE(mission::validateMissionFile(root + "/valid").valid());
  {std::ofstream out(root + "/large"); out << std::string(65537, 'x');}
  EXPECT_FALSE(mission::validateMissionFile(root + "/large").valid());
  ASSERT_EQ(mkfifo((root + "/fifo").c_str(), 0600), 0);
  EXPECT_FALSE(mission::validateMissionFile(root + "/fifo").valid());
  EXPECT_FALSE(mission::validateMissionFile(root).valid());
  EXPECT_FALSE(mission::validateMissionFile(root + "/missing").valid());
}

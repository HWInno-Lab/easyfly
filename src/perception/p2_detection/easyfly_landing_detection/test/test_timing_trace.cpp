// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_landing_detection/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project

#include <gtest/gtest.h>
#include <fstream>
#include <limits>
#include "timing_trace.hpp"

TEST(TimingTrace, DisabledHasNoFile)
{
  ef_timing::Trace trace("");
  EXPECT_FALSE(trace.enabled());
  trace.record({}, "not_recorded");
}

TEST(TimingTrace, BoundedAndDoesNotOverwrite)
{
  char directory[] = "/tmp/easyfly-ingress-XXXXXX";
  ASSERT_NE(mkdtemp(directory), nullptr);
  const std::string path = std::string(directory) + "/trace.tsv";
  {
    ef_timing::Trace trace(path);
    EXPECT_TRUE(trace.enabled());
    EXPECT_THROW(ef_timing::Trace existing(path), std::runtime_error);
    for (int i = 0; i < 8200; ++i) {
      ef_timing::Event event;
      event.source_ns = i;
      trace.record(event, "test");
    }
    std::ifstream live(path);
    EXPECT_EQ(live.peek(), std::ifstream::traits_type::eof());
  }
  std::ifstream file(path);
  std::string header, columns, first;
  std::getline(file, header);
  std::getline(file, columns);
  std::getline(file, first);
  EXPECT_EQ(header, "# total=8200 retained=8192 dropped=8");
  EXPECT_EQ(first.substr(0, 6), "8\t0\t8\t");
  file.close();
  EXPECT_EQ(unlink(path.c_str()), 0);
  EXPECT_EQ(rmdir(directory), 0);
}

TEST(TimingTrace, Signed64BitTimestampFormatting)
{
  char directory[] = "/tmp/easyfly-ingress-format-XXXXXX";
  ASSERT_NE(mkdtemp(directory), nullptr);
  const std::string path = std::string(directory) + "/trace.tsv";
  ef_timing::Event event;
  event.source_ns = std::numeric_limits<int64_t>::min();
  event.writer_wall_ns = std::numeric_limits<int64_t>::max();
  event.input = true;
  event.valid = true;
  {
    ef_timing::Trace trace(path);
    trace.record(event, "format");
  }
  std::ifstream file(path);
  std::string line;
  std::getline(file, line);
  std::getline(file, line);
  std::getline(file, line);
  EXPECT_EQ(
    line, "0\t1\t" + std::to_string(event.source_ns) + "\t" +
    std::to_string(event.writer_wall_ns) + "\t0\t0\t0\t0\t0\t1\tformat");
  file.close();
  EXPECT_EQ(unlink(path.c_str()), 0);
  EXPECT_EQ(rmdir(directory), 0);
}

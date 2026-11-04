// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_utils/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
#ifndef EASYFLY_UTILS__TIME_HPP_
#define EASYFLY_UTILS__TIME_HPP_
#include <chrono>
#include <string>
namespace easyfly_utils
{
inline double now_seconds()
{
  auto now = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(now.time_since_epoch()).count();
}
inline uint64_t now_nanos()
{
  auto now = std::chrono::steady_clock::now();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    now.time_since_epoch()).count();
}
inline std::string timestamp()
{
  auto now = std::chrono::system_clock::now();
  auto t = std::chrono::system_clock::to_time_t(now);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
  return buf;
}
}  // namespace easyfly_utils
#endif  // EASYFLY_UTILS__TIME_HPP_

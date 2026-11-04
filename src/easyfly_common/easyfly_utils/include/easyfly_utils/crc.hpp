// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_utils/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
#ifndef EASYFLY_UTILS__CRC_HPP_
#define EASYFLY_UTILS__CRC_HPP_
#include <cstdint>
#include <cstddef>
namespace easyfly_utils
{
inline uint8_t crc8(const uint8_t * data, size_t len)
{
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
    }
  }
  return crc;
}
}  // namespace easyfly_utils
#endif  // EASYFLY_UTILS__CRC_HPP_

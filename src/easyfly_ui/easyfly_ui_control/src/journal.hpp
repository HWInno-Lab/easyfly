// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_ui_control/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
#ifndef JOURNAL_HPP_
#define JOURNAL_HPP_
#include <optional>
#include <string>
#include "easyfly_ui_control/controller.hpp"
namespace easyfly_ui_control
{
bool hex_id(const std::string & value, size_t size);
bool catalog_name(const std::string & value);
std::string new_id();
class Journal
{
public:
  explicit Journal(const std::string & directory);
  ~Journal();
  Journal(const Journal &) = delete;
  Journal & operator=(const Journal &) = delete;
  std::optional<Intent> load() const;
  void save(const Intent & intent) const;

private:
  int directory_{-1}, lock_{-1};
};
}  // namespace easyfly_ui_control
#endif  // JOURNAL_HPP_

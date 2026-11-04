// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_ui_control/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
#ifndef IMPORT_JOURNAL_HPP_
#define IMPORT_JOURNAL_HPP_
#include <optional>
#include <string>
#include "easyfly_ui_control/importer.hpp"
#include "journal.hpp"
namespace easyfly_ui_control
{
// Reuses the existing private-directory/exclusive-lock acquisition only.
// Distinct strict schema/file; never loads or saves flight mission intents.
class ImportJournal
{
public:
  explicit ImportJournal(const std::string & directory);
  ~ImportJournal();
  std::optional<ImportIntent> load() const;
  void save(const ImportIntent & intent) const;

private:
  Journal lock_;
  int directory_{-1};
};
}  // namespace easyfly_ui_control
#endif  // IMPORT_JOURNAL_HPP_

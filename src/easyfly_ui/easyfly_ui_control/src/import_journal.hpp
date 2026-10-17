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

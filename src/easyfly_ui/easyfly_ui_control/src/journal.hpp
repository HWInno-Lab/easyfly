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

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

#ifndef CONFIRMATION_LEDGER_HPP_
#define CONFIRMATION_LEDGER_HPP_
#include <set>
#include <string>
#include "journal.hpp"
namespace easyfly_ui_control
{
// Worker enqueue mutex owns this process-local, bounded, non-evicting set.
class ConfirmationLedger
{
public:
  bool consume(const std::string & id)
  {
    if (id.empty()) {return true;}  // Explicit non-GUI API: logged as unattributed.
    if (!hex_id(id, 32) || consumed_.size() >= 256 || consumed_.count(id)) {return false;}
    consumed_.insert(id); return true;
  }

private:
  std::set<std::string> consumed_;
};
}  // namespace easyfly_ui_control
#endif  // CONFIRMATION_LEDGER_HPP_

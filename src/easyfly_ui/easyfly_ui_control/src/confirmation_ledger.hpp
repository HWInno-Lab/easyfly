// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_ui_control/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
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
    if (!hex_id(id, 32) || consumed_.size() >= 256 || consumed_.count(id)) {return false;}
    consumed_.insert(id); return true;
  }

private:
  std::set<std::string> consumed_;
};
}  // namespace easyfly_ui_control
#endif  // CONFIRMATION_LEDGER_HPP_

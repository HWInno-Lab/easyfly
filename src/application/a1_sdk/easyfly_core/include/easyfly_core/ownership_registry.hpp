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
#pragma once
#include <cstdint>
#include <deque>
#include <mutex>
#include <set>
#include <string>
namespace easyfly_core
{
template<class UUID> std::string goalIdentity(const UUID & uuid)
{
  const char * digits = "0123456789abcdef";
  std::string result;
  for (const auto byte : uuid) {
    result += digits[(byte >> 4) & 15]; result += digits[byte & 15];
  }
  return result;
}
// Active IDs never expire. Evicted released IDs become UNKNOWN, never a false
// proof of release. Call release only after every owned child is truly drained.
class OwnershipRegistry
{
public:
  void own(const std::string & id) {std::lock_guard<std::mutex> lock(mutex_); owned_.insert(id);}
  void release(const std::string & id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!owned_.erase(id)) {return;}
    released_.insert(id); history_.push_back(id);
    if (history_.size() > 128) {released_.erase(history_.front()); history_.pop_front();}
  }
  uint8_t state(const std::string & id) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return owned_.count(id) ? 1 : released_.count(id) ? 2 : 0;
  }

private:
  mutable std::mutex mutex_;
  std::set<std::string> owned_, released_;
  std::deque<std::string> history_;
};
}  // namespace easyfly_core

// Copyright 2026 EasyFly Project
//
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
#include <memory>
#include <string>
#include "easyfly_sort/sort.hpp"
#include "easyfly_tracking_api/tracker.hpp"

namespace easyfly_multi_object_tracking
{
// Platform adapter factory; algorithm implementations live in independent libraries.
// Only 'sort' is implemented. Unknown names throw, never silently select a fallback.
std::unique_ptr<easyfly::tracking::Tracker> make_tracker(
  const std::string & backend, const easyfly::sort::SortParams & params);
}  // namespace easyfly_multi_object_tracking

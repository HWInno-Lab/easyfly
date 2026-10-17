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

#ifndef IMPORT_WORKER_HPP_
#define IMPORT_WORKER_HPP_
#include <functional>
#include <memory>
#include "import_catalog.hpp"
// One active + eight queued metadata jobs; never runs filesystem work on ROS executor.
class ImportWorker
{
public:
  using Api = ImportCatalog::Api;
  using Handler = std::function<Api::Response(const Api::Request &)>;
  explicit ImportWorker(Handler handler);
  ~ImportWorker();
  bool submit(Api::Request::SharedPtr request, std::function<void(Api::Response)> reply);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
#endif  // IMPORT_WORKER_HPP_

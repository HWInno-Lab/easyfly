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

#ifndef IMPORT_CATALOG_HPP_
#define IMPORT_CATALOG_HPP_
#include <memory>
#include <string>
#include <vector>
#include <easyfly_msgs/srv/import_mission.hpp>
// Fixed private catalog, no actions/flight clients. Records never evicted.
class ImportCatalog
{
public:
  using Api = easyfly_msgs::srv::ImportMission;
  ImportCatalog(
    bool enabled, const std::string & directory, const std::string & instance,
    int capacity, const std::vector<std::string> & protected_roots);
  ~ImportCatalog();
  ImportCatalog(const ImportCatalog &) = delete;
  ImportCatalog & operator=(const ImportCatalog &) = delete;
  Api::Response handle(const Api::Request & request);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
#endif  // IMPORT_CATALOG_HPP_

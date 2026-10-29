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
#ifndef EASYFLY_UTILS__PARAM_LOADER_HPP_
#define EASYFLY_UTILS__PARAM_LOADER_HPP_
#include <string>
#include <map>
#include <fstream>
#include <iostream>
namespace easyfly_utils
{
class ParamLoader
{
public:
  bool load(const std::string & path)
  {
    // 简化实现: 读取 key=value 格式文件
    std::ifstream file(path);
    if (!file.is_open()) {return false;}
    std::string line;
    while (std::getline(file, line)) {
      auto pos = line.find('=');
      if (pos != std::string::npos) {
        params_[line.substr(0, pos)] = line.substr(pos + 1);
      }
    }
    return true;
  }
  std::string get(const std::string & key, const std::string & def = "") const
  {
    auto it = params_.find(key);
    return (it != params_.end()) ? it->second : def;
  }

private:
  std::map<std::string, std::string> params_;
};
}  // namespace easyfly_utils
#endif  // EASYFLY_UTILS__PARAM_LOADER_HPP_

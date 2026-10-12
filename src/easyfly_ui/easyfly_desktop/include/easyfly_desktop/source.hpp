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

#ifndef EASYFLY_DESKTOP__SOURCE_HPP_
#define EASYFLY_DESKTOP__SOURCE_HPP_
#include <functional>
#include <memory>
#include <string>

#include <easyfly_ui_bridge/observer.hpp>

namespace easyfly_desktop
{
namespace bridge = easyfly_ui_bridge;
// Narrow observation port shared by the real bridge and isolated test sources.
// It intentionally cannot express control, arbitrary services or parameter writes.
class Source
{
public:
  virtual ~Source() = default;
  virtual std::shared_ptr<const bridge::UiSnapshot> snapshot() const = 0;
  virtual bridge::RequestReceipt capabilities(bridge::CapabilitySource source) = 0;
  virtual bridge::RequestReceipt list_missions() = 0;
  virtual bridge::RequestReceipt validate_mission(const std::string & name) = 0;
  virtual bridge::RequestReceipt query_mission(const std::string & id) = 0;
  virtual bridge::RequestReceipt read_parameter(bridge::Parameter parameter) = 0;
  virtual bridge::RequestReceipt perception(bridge::PerceptionKind kind) = 0;
  virtual void stop() = 0;
};
using SourceFactory = std::function<std::unique_ptr<Source>(
      const bridge::BridgeConfig &, std::shared_ptr<const bridge::UiSnapshot>)>;
SourceFactory bridge_factory();
}  // namespace easyfly_desktop
#endif  // EASYFLY_DESKTOP__SOURCE_HPP_

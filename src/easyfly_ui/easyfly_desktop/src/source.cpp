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
#include <utility>
#include "easyfly_desktop/source.hpp"
namespace easyfly_desktop
{
namespace
{
class BridgeSource final : public Source
{
public:
  BridgeSource(const bridge::BridgeConfig & config, std::shared_ptr<const bridge::UiSnapshot> old)
  : observer_(config, std::move(old)) {}
  std::shared_ptr<const bridge::UiSnapshot> snapshot() const override {return observer_.snapshot();}
  bridge::RequestReceipt capabilities(bridge::CapabilitySource s) override
  {
    return observer_.capabilities(s);
  }
  bridge::RequestReceipt list_missions() override {return observer_.list_missions();}
  bridge::RequestReceipt validate_mission(const std::string & n) override
  {
    return observer_.validate_mission(n);
  }
  bridge::RequestReceipt query_mission(const std::string & id) override
  {
    return observer_.query_mission(id);
  }
  bridge::RequestReceipt mission_details(const std::string & name) override
  {
    return observer_.mission_details(name);
  }
  bridge::RequestReceipt read_parameter(bridge::Parameter p) override
  {
    return observer_.read_parameter(p);
  }
  bridge::RequestReceipt perception(bridge::PerceptionKind k) override
  {
    return observer_.perception(k);
  }
  void stop() override {observer_.stop();}

private:
  bridge::Observer observer_;
};
}  // namespace
SourceFactory bridge_factory()
{
  return [](const bridge::BridgeConfig & config, std::shared_ptr<const bridge::UiSnapshot> old) {
           return std::make_unique<BridgeSource>(config, std::move(old));
         };
}
}  // namespace easyfly_desktop

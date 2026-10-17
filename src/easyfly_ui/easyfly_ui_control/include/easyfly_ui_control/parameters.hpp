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

#ifndef EASYFLY_UI_CONTROL__PARAMETERS_HPP_
#define EASYFLY_UI_CONTROL__PARAMETERS_HPP_
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <variant>
namespace easyfly_ui_control
{
enum class LocalParameter {OperatorNote, ExportPretty};
using ParameterValue = std::variant<std::string, bool>;
const char * local_parameter_name(LocalParameter parameter);
std::string parameter_json(LocalParameter parameter, const ParameterValue & value);
std::optional<ParameterValue> parameter_value(LocalParameter parameter, const std::string & json);
double parameter_steady_seconds();
struct ParameterReview
{
  uint64_t id{0};
  LocalParameter parameter{LocalParameter::OperatorNote};
  std::string json;
  double received{0};
};
enum class ParameterPhase {Idle, Reading, Reviewed, Precheck, Writing, Checking, Verified, Rejected,
  Uncertain};
struct ParameterSnapshot
{
  bool ready{false}, stopped{false}, pending{false}, fault{false};
  bool dispatched{false}, acknowledged{false};
  ParameterPhase phase{ParameterPhase::Idle};
  std::optional<ParameterReview> review;
  std::string message, last_write;
  size_t confirmations{0};
};
struct ParameterConfig {double response_timeout{2};};
// Independent, opt-in metadata writer. Fixed service/two keys, no flight API.
// One submission, read/compare before write, independent readback after reply.
// Not CAS, server identity, persistence, authentication or exactly-once transport.
class ParameterControl
{
public:
  explicit ParameterControl(const ParameterConfig & config = {});
  ~ParameterControl();
  ParameterControl(const ParameterControl &) = delete;
  ParameterControl & operator=(const ParameterControl &) = delete;
  std::shared_ptr<const ParameterSnapshot> snapshot() const;
  bool read(LocalParameter parameter);
  bool write_confirmed(
    const ParameterReview & review, const ParameterValue & desired,
    const std::string & confirmation);
  void inhibit(bool active);
  void stop();  // Nonblocking; no retry/rollback or fault-lock persistence across restart.

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_ui_control
#endif  // EASYFLY_UI_CONTROL__PARAMETERS_HPP_

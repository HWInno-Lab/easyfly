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
#ifndef EASYFLY_DESKTOP__RECORDING_PANEL_HPP_
#define EASYFLY_DESKTOP__RECORDING_PANEL_HPP_
#include <QWidget>
#include <memory>
#include <string>
#include <vector>
#include "easyfly_desktop/recording.hpp"
namespace easyfly_desktop
{
class RecordingPanel : public QWidget
{
public:
  explicit RecordingPanel(std::vector<std::string> protected_paths, QWidget * parent = nullptr);
  ~RecordingPanel() override;
  void observe(
    const RecordingPacket & packet, const ImageBinding & binding, bool connected,
    bool simulation_clock, const std::string & profile);
  void finish(const std::string & reason);
  void stop();
  bool stopped() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_desktop
#endif  // EASYFLY_DESKTOP__RECORDING_PANEL_HPP_

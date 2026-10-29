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
#ifndef EASYFLY_DESKTOP__RVIZ_VIEW_HPP_
#define EASYFLY_DESKTOP__RVIZ_VIEW_HPP_
#include <QString>
#include <QStringList>
#include <QWidget>
#include <cstdint>
#include <memory>
#include <vector>
namespace easyfly_desktop
{
struct RvizOptions {bool enabled{false};};
struct RvizLayerSnapshot
{
  QString name, topic, source_frame, status;
  bool enabled{false}, visible{false};
  uint64_t received{0}, replaced{0};
  // Diagnostic identity/history survives recovery; it does not renew freshness.
  uint64_t epoch{0}, invalidations{0}, source_changes{0};
  QString source_gid, last_invalidation;
  int64_t received_stamp_ns{0}, invalidated_at_ns{0};
  size_t vertices{0};
  int input_depth{-1};
  double source_age{-1}, receipt_age{-1};
};
struct RvizSnapshot
{
  bool running{false};
  uint64_t frames{0};
  uint64_t lifecycle_sequence{0};
  QString lifecycle_reason, last_node;
  int64_t ros_time_ns{-1};
  int input_depth{-1};
  QString node, fixed_frame, status;
  QStringList tools, publishers;
  std::vector<RvizLayerSnapshot> layers;
};
// Only fixed read-only displays and MoveCamera. No .rviz config or arbitrary plugins.
class RvizView : public QWidget
{
  Q_OBJECT

public:
  explicit RvizView(QWidget * parent = nullptr);
  ~RvizView() override;
  static bool available();
  void set_connection(bool connected, bool use_sim_time);
  void stop();
  RvizSnapshot snapshot() const;

protected:
  void hideEvent(QHideEvent * event) override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_desktop
#endif  // EASYFLY_DESKTOP__RVIZ_VIEW_HPP_

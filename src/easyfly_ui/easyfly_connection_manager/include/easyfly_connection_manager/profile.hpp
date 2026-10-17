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
#include <QJsonObject>
#include <QProcessEnvironment>
#include <QString>
#include <QVector>

namespace easyfly_connection_manager
{
struct Profile
{
  QString id, name;
  int domain{0};
  bool local_only{true}, simulation_time{true};
  QString selected_profile{"navigation_dev"};
  QString front_topic{"/camera/color/image_raw"}, down_topic{"/camera_down/color/image_raw"};
  QString mode{"observe"}, scene{"primitives_sparse_seed42_v1"}, px4_directory;
  int gazebo_port{11345};
  bool gazebo_gui{false};
  QString transport{"udp"}, bind_host{"127.0.0.1"}, remote_host{"127.0.0.1"};
  int bind_port{14540}, remote_port{14580}, baud{921600}, target_system{1};
  QString serial_device{"/dev/ttyACM0"};
  bool gcs_enabled{false};
  QString gcs_host{"127.0.0.1"}, qgc_path;
  int gcs_bind_port{14558}, gcs_port{14560};
  bool mission_control{false}, mission_import{false}, parameter_edit{false};
  bool recording{false}, rviz_embed{false};
  static Profile create(const QString & name);
  static Profile from_json(const QJsonObject & object);
  QJsonObject json() const;
  QString hash() const;
  void validate() const;
  QString fcu_url() const;
  QString gcs_url() const;
};
QString private_directory(const QString & path, bool create = true);
QString default_runtime_directory();
QString default_config_file();
QString desktop_program();
QString backend_program();
QString supervisor_program();
QProcessEnvironment session_environment(const Profile & profile, const QString & session);
QVector<Profile> read_profiles(const QString & path);
void write_profiles(const QString & path, const QVector<Profile> & profiles);
}  // namespace easyfly_connection_manager

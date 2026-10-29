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

#ifndef QGC_SETTINGS_HPP_
#define QGC_SETTINGS_HPP_

#include <QSettings>

namespace easyfly_connection_manager
{
inline void configure_qgc_udp(QSettings & settings, int listen_port, int target_port)
{
  // QGC 4.4 uses LinkManager. QGC 5 migrates that group into AutoConnect;
  // identical values keep the selected endpoint across either startup path.
  for (const auto & group : {"LinkManager", "AutoConnect"}) {
    settings.beginGroup(group);
    for (const auto & key : {"autoConnectPixhawk", "autoConnectSiKRadio", "autoConnectRTKGPS",
        "autoConnectLibrePilot", "autoConnectZeroConf", "autoConnectPX4Flow"})
    {
      settings.setValue(key, false);
    }
    settings.setValue("autoConnectUDP", true);
    settings.setValue("udpListenPort", listen_port);
    settings.setValue("udpTargetHostIP", "127.0.0.1");
    settings.setValue("udpTargetHostPort", target_port);
    settings.endGroup();
  }
}
}  // namespace easyfly_connection_manager

#endif  // QGC_SETTINGS_HPP_

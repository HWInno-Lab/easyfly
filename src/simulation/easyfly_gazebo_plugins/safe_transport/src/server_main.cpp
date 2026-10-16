// Copyright 2026 EasyFly Project
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *     http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
// Derived from Gazebo Classic gazebo11_11.10.2 gazebo/server_main.cc.
#include <gazebo/rendering/ogre_gazebo.h>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <gazebo/Server.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/util/LogRecord.hh>
#include "easyfly_transport/compat.hpp"

int main(int argc, char ** argv)
{
  if (!easyfly_transport::ValidateLibrary()) {
    std::cerr << "EasyFly transport: unsupported Gazebo transport fingerprint; refusing start\n";
    return 2;
  }
  ::setenv("RMT_PORT", "1500", true);
  std::unique_ptr<gazebo::Server> server;
  int result = 0;
  try {
    gzLogInit("server-", "gzserver.log");
    gazebo::util::LogRecord::Instance()->Init("gzserver");
    server = std::make_unique<gazebo::Server>();
    if (!server->ParseArgs(argc, argv)) {
      result = -1;
    } else {
      server->Run();
      server->Fini();
    }
  } catch (gazebo::common::Exception & error) {
    error.Print();
    if (server) {server->Fini();}
    result = -1;
  } catch (Ogre::Exception & error) {
    gzerr << "Ogre Error:" << error.getFullDescription() << "\n";
    if (server) {server->Fini();}
    result = -1;
  } catch (const std::exception & error) {
    std::cerr << "EasyFly gzserver failed: " << error.what() << "\n";
    if (server) {server->Fini();}
    result = -1;
  }
  // ServerPrivate contains transport owners: release it before closing cleanup.
  server.reset();
  easyfly_transport::ShutdownConnections();
  if (!easyfly_transport::Finish()) {
    const auto stats = easyfly_transport::GetStatistics();
    std::cerr << "EasyFly transport: shutdown retained live IO managers: live=" <<
      stats.live << " pending=" << stats.pending << " deferred=" << stats.deferred <<
      " completed=" << stats.completed << easyfly_transport::DescribeConnections() <<
      easyfly_transport::DescribeManagers() << "\n";
    return 2;
  }
  auto stats = easyfly_transport::GetStatistics();
  std::cerr << "EasyFly transport drained: live=" << stats.live <<
    " pending=" << stats.pending << " deferred=" << stats.deferred <<
    " completed=" << stats.completed << "\n";
  return result;
}

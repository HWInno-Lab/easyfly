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
#include <algorithm>
#include <string>
#include <vector>
#include <gazebo/Server.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/common/Events.hh>
#include <dlfcn.h>
#include <atomic>
#include <gazebo/util/LogRecord.hh>
#include "easyfly_transport/compat.hpp"
#include "easyfly_transport/signal_relay.hpp"

extern "C" bool EasyflyCameraClockCompatibility();

namespace
{
using Handler = void (*)(int);
std::atomic<Handler> native_sigint{nullptr};
}

// The installed Gazebo library binds Server::SigInt directly, so overriding
// that C++ symbol is insufficient. Redirect only its exact handler registration
// in this standalone executable; all other sigaction calls retain libc behavior.
extern "C" int sigaction(int signal, const struct sigaction * action,
  struct sigaction * previous) noexcept
{
  using Function = int (*)(int, const struct sigaction *, struct sigaction *);
  static auto real = reinterpret_cast<Function>(dlsym(RTLD_NEXT, "sigaction"));
  if (!real) {errno = ENOSYS; return -1;}
  if (signal == SIGINT && action && action->sa_handler == native_sigint.load() &&
    native_sigint.load() != nullptr)
  {
    auto safe = *action;
    safe.sa_handler = easyfly_transport::SignalRelay::notify;
    const int result = real(signal, &safe, previous);
    if (result == 0) {std::cerr << "EasyFly Gazebo: async-safe SIGINT handoff installed\n";}
    return result;
  }
  return real(signal, action, previous);
}

int main(int argc, char ** argv)
{
  // Upstream Server::Run dispatches locked C++ callbacks from its handler.
  // Keep those callbacks in the joined ordinary thread instead.
  easyfly_transport::SignalRelay signals;
  if (!EasyflyCameraClockCompatibility()) {return 2;}
  std::cerr << "EasyFly Gazebo: camera scene-time compatibility active\n";
  if (!easyfly_transport::ValidateLibrary()) {
    std::cerr << "EasyFly transport: unsupported Gazebo transport fingerprint; refusing start\n";
    return 2;
  }
  void * library = dlopen("libgazebo.so.11", RTLD_NOW | RTLD_NOLOAD);
  auto handler = library ? reinterpret_cast<Handler>(
    dlsym(library, "_ZN6gazebo6Server6SigIntEi")) : nullptr;
  if (!handler) {std::cerr << "Gazebo native SIGINT entry not available\n"; return 2;}
  native_sigint.store(handler);
  ::setenv("RMT_PORT", "1500", true);
  std::unique_ptr<gazebo::Server> server;
  int result = 0;
  try {
    gzLogInit("server-", "gzserver.log");
    gazebo::util::LogRecord::Instance()->Init("gzserver");
    server = std::make_unique<gazebo::Server>();
    std::vector<std::string> arguments(argv, argv+argc);
    const char * lockstep = std::getenv("EASYFLY_GAZEBO_LOCKSTEP");
    if (lockstep && std::string(lockstep) == "1" && !std::getenv("LP_NUM_THREADS")) {
      // Keep the software rasterizer inside the VIO simulation CPU budget.
      // An explicit caller setting takes precedence; hardware drivers ignore this.
      ::setenv("LP_NUM_THREADS", "2", false);
      std::cerr << "EasyFly Gazebo: VIO render budget LP_NUM_THREADS=2\n";
    }
    if (lockstep && std::string(lockstep) == "1" &&
        std::find(arguments.begin(), arguments.end(), "--lockstep") == arguments.end()) {
      arguments.insert(arguments.begin()+1, "--lockstep");
      std::cerr << "EasyFly Gazebo: OpenVINS synchronous rendering enabled\n";
    }
    std::vector<char *> pointers;
    for (auto & argument : arguments) pointers.push_back(argument.data());
    pointers.push_back(nullptr);
    if (!server->ParseArgs(static_cast<int>(arguments.size()), pointers.data())) {
      result = -1;
    } else {
      signals.start([&server] {
        // Same stop/event semantics as upstream Server::SigInt, from a
        // normal thread which cannot interrupt a plugin while it owns a lock.
        server->Stop();
        gazebo::event::Events::sigInt();
      });
      server->Run();
      signals.finish();
      server->Fini();
    }
  } catch (gazebo::common::Exception & error) {
    signals.finish();
    error.Print();
    if (server) {server->Fini();}
    result = -1;
  } catch (Ogre::Exception & error) {
    signals.finish();
    gzerr << "Ogre Error:" << error.getFullDescription() << "\n";
    if (server) {server->Fini();}
    result = -1;
  } catch (const std::exception & error) {
    signals.finish();
    std::cerr << "EasyFly gzserver failed: " << error.what() << "\n";
    if (server) {server->Fini();}
    result = -1;
  }
  // ServerPrivate contains transport owners: release it before closing cleanup.
  signals.finish();
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

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
#include <gazebo/common/Events.hh>
#include <easyfly_transport/signal_relay.hpp>
#include <sys/wait.h>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>

// The same event dispatch performed by upstream Server::SigInt. A plugin
// callback needs a mutex held by the interrupted simulator thread.
static void upstream_handler(int)
{
  gazebo::event::Events::stop();
  gazebo::event::Events::sigInt();
}
static int child(bool relay, int signum)
{
  alarm(2);
  std::unique_ptr<easyfly_transport::SignalRelay> signals;
  if (relay) {signals = std::make_unique<easyfly_transport::SignalRelay>();}
  std::mutex plugin;
  std::atomic<int> calls{0};
  auto connection = gazebo::event::Events::ConnectSigInt([&] {
    std::lock_guard<std::mutex> lock(plugin); ++calls;
  });
  if (relay) {signals->start([] {upstream_handler(SIGINT);});}
  else {std::signal(signum, upstream_handler);}
  {
    std::lock_guard<std::mutex> lock(plugin);
    // Native unsafe handler never returns; the relay lets the interrupted
    // thread finish normally before invoking the identical locked callback.
    if (pthread_kill(pthread_self(), signum) != 0) {return 2;}
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
  const auto deadline = std::chrono::steady_clock::now()+std::chrono::seconds(1);
  while (calls.load() != 1 && std::chrono::steady_clock::now()<deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (signals) {signals->finish();}
  alarm(0);
  return calls.load() == 1 ? 0 : 3;
}
int main()
{
  // Force delivery to the mutex-owning thread in both controls. The self-pipe
  // must remain safe even when a library thread has unblocked these signals.
  for (int signum : {SIGINT, SIGTERM}) {
    pid_t pid = fork(); if (pid == 0) {_exit(child(false, signum));}
    int status=0; waitpid(pid,&status,0);
    if (!WIFSIGNALED(status) || WTERMSIG(status)!=SIGALRM) {return 4;}
    std::cerr << "unsafe event handler alarm: " << signum << '\n';
    pid = fork(); if (pid == 0) {_exit(child(true, signum));}
    waitpid(pid,&status,0);
    if (!WIFEXITED(status) || WEXITSTATUS(status)!=0) {return 5;}
  }
  return 0;
}

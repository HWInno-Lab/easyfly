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
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <easyfly_core/client.hpp>
static double number(const char * text)
{
  std::size_t end;
  const std::string value(text);
  const double result = std::stod(value, &end);
  if (end != value.size() || !std::isfinite(result)) {
    throw std::invalid_argument("invalid coordinate");
  }
  return result;
}
int main(int argc, char ** argv)
{
  easyfly_core::ClientOptions options;
  int arg = 1;
  if (arg < argc && std::string(argv[arg]) == "--sim-time") {options.use_sim_time = true; ++arg;}
  easyfly_core::Client client(options);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  const auto state = client.state();
  std::cout << "received=" << state.received << " fresh=" << state.fresh
            << " navigate_server=" << client.server_ready("navigate") << '\n';
  std::shared_ptr<easyfly_core::Operation> operation;
  try {
    if (arg == argc) {
      return client.close(2.0) ? 0 : 1;                  // Default query only.
    }
    if (std::string(argv[arg]) == "--navigate" && argc - arg == 4) {
      easyfly_msgs::action::NavigateTo::Goal goal;
      goal.target_x = number(argv[arg + 1]); goal.target_y = number(argv[arg + 2]);
      goal.target_z = number(argv[arg + 3]);
      operation = client.navigate(goal);
    } else if (std::string(argv[arg]) == "--mission" && argc - arg == 2) {
      easyfly_msgs::action::ExecuteMission::Goal goal; goal.mission_file = argv[arg + 1];
      operation = client.execute_mission(goal);
    } else {throw std::invalid_argument("usage: [--sim-time] [--navigate x y z | --mission path]");}
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n'; return 2;
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (!operation->wait(.2) && std::chrono::steady_clock::now() < deadline) {
    const auto snapshot = operation->snapshot();
    std::cout << "admission=" << static_cast<int>(snapshot.admission) << " pending=" <<
      snapshot.remote_execution_pending;
    for (const auto & field : snapshot.feedback.numbers) {
      std::cout << ' ' << field.first << '=' << field.second;
    }
    std::cout << '\n';
  }
  if (operation->snapshot().remote_execution_pending) {
    operation->cancel();  // Explicit example policy, not wait's implicit behavior.
    operation->wait(5.0);
  }
  const auto result = operation->snapshot();
  std::cout << "success=" << result.success << " message=" << result.message
            << " remote_pending=" << result.remote_execution_pending << '\n';
  const bool drained = client.close(2.0);
  if (!drained) {
    std::cerr <<
      "WARNING: peer execution is still unresolved; process exit cannot guarantee stop\n";
  }
  return result.success && drained ? 0 : 1;
}

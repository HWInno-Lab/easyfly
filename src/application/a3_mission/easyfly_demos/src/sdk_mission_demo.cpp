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
#include <filesystem>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <easyfly_core/client.hpp>
#include <easyfly_mission_fsm/mission_format.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
int main(int argc, char ** argv)
{
  try {
    std::string path =
      (std::filesystem::path(
        ament_index_cpp::get_package_share_directory(
          "easyfly_mission_scripts")) /
      "missions/takeoff_hold_land_v1.yaml").string();
    bool execute = false, sim = false; double cancel_after = 0;
    for (int i = 1; i < argc; ++i) {
      const std::string flag = argv[i];
      if (flag == "--execute") {execute = true;} else if (flag == "--sim-time") {
        sim = true;
      } else if (flag == "--mission" && i + 1 < argc) {
        path = argv[++i];
      } else if (flag == "--cancel-after" && i + 1 < argc) {
        const std::string text = argv[++i]; size_t consumed = 0;
        cancel_after = std::stod(text, &consumed);
        if (consumed != text.size() || !std::isfinite(cancel_after) || cancel_after <= 0 ||
          cancel_after > 60)
        {
          throw std::invalid_argument("cancel-after must be in (0,60] seconds");
        }
      } else {
        throw std::invalid_argument(
                "usage: sdk_mission_demo [--mission PATH] [--execute] "
                "[--sim-time] [--cancel-after SECONDS]");
      }
    }
    auto validation = easyfly_mission_fsm::validateMissionFile(path);
    if (!validation.valid()) {throw std::invalid_argument(validation.error);}
    std::cout << "valid " << validation.steps.size() << " steps; " <<
      (execute ? "explicit execution requested" : "validation only, no motion") << '\n';
    if (!execute) {return 0;}
    easyfly_core::ClientOptions options; options.use_sim_time = sim;
    options.require_managed_ownership = true;
    easyfly_core::Client client(options);
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!client.server_ready("execute_mission") && std::chrono::steady_clock::now() < until) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!client.server_ready("execute_mission")) {
      throw std::runtime_error("mission/ownership endpoint unavailable");
    }
    easyfly_msgs::action::ExecuteMission::Goal goal; goal.mission_file = path;
    auto operation = client.execute_mission(goal);
    const double budget = cancel_after > 0 ? cancel_after : 180;
    if (!operation->wait(budget)) {
      operation->cancel();
      if (!operation->wait(10)) {
        std::cerr << "cancel/drain unconfirmed; external execution may remain\n"; client.close(1);
        return 1;
      }
    }
    const auto result = operation->snapshot(); std::cout << result.message << '\n';
    const bool drained = client.close(2);
    return drained &&
           (result.success ||
           (cancel_after > 0 && result.status == easyfly_core::Status::Canceled)) ? 0 : 1;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';return 1;
  }
}

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
#include <chrono>
#include <csignal>
#include <exception>
#include <iostream>
#include <string>
#include <thread>
// Debugger regression fixture only, never linked into the real ROS node.
void fixture_terminate()
{
  std::terminate();
}
int main(int argc, char ** argv)
{
  if (argc != 2) {
    return 2;
  }
  if (std::string(argv[1]) == "terminate") {
    fixture_terminate();
  }
  if (std::string(argv[1]) == "ignore-shutdown") {
    std::signal(SIGINT, SIG_IGN);
    std::signal(SIGTERM, SIG_IGN);
    std::cout << "fixture shutdown handlers ready" << std::endl;
    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
  return std::stoi(argv[1]);
}

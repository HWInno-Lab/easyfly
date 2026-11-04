// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_semantic_map/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
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

// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Read-only model queries using the same owned transport as easyfly_gzserver.
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <iomanip>
#include <iostream>
#include <sstream>
#include "easyfly_transport/compat.hpp"

int main(int argc, char ** argv)
{
  if (argc != 3 || (std::string(argv[2]) != "--pose" && std::string(argv[2]) != "--info")) {
    std::cerr << "usage: easyfly_model_query MODEL --pose|--info\n";
    return 2;
  }
  if (!easyfly_transport::ValidateLibrary()) return 2;
  int result = 1;
  std::ostringstream output;
  gazebo::transport::NodePtr node;
  try {
    if (!gazebo::transport::init("", 0, 5)) throw std::runtime_error("native master unavailable");
    gazebo::transport::run();
    node.reset(new gazebo::transport::Node());
    if (!node->TryInit(gazebo::common::Time(5, 0)))
      throw std::runtime_error("native world unavailable");
    auto response = gazebo::transport::request(
      node->GetTopicNamespace(), "entity_info", argv[1], gazebo::common::Time(5, 0));
    gazebo::msgs::Model model;
    if (!response || response->response() != "success" ||
        response->type() != model.GetTypeName() ||
        !model.ParseFromString(response->serialized_data()) ||
        model.name() != argv[1] || !model.has_pose())
      throw std::runtime_error("native model identity/pose unavailable");
    if (std::string(argv[2]) == "--info") {
      output << model.DebugString();
    } else {
      const auto pose = gazebo::msgs::ConvertIgn(model.pose());
      if (!pose.IsFinite()) throw std::runtime_error("nonfinite native pose");
      output << std::setprecision(17) << pose.Pos().X() << ' ' << pose.Pos().Y() << ' '
             << pose.Pos().Z() << ' ' << pose.Rot().Roll() << ' '
             << pose.Rot().Pitch() << ' ' << pose.Rot().Yaw() << '\n';
    }
    result = 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
  }
  if (node) node->Fini();
  node.reset();
  gazebo::transport::fini();
  easyfly_transport::ShutdownConnections();
  if (!easyfly_transport::Finish()) {
    std::cerr << "native model query retained transport owners\n";
    return 3;
  }
  // A valid response alone is insufficient: publish it only after clean retirement.
  if (result == 0) std::cout << output.str();
  return result;
}

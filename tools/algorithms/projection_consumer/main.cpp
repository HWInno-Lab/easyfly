// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <stdexcept>
#include "easyfly_projection/projection.hpp"
int main()
{
  std::vector<float> values(9,2);
  auto depth=easyfly::projection::medianDepth({.5f,.5f,1,1},
    {reinterpret_cast<const std::uint8_t *>(values.data()),36,3,3,12});
  const auto xyz=easyfly::projection::pinhole(330,220,depth,100,200,320,240);
  if(depth!=2||xyz[0]!=.2||xyz[1]!=-.2||xyz[2]!=2)throw std::runtime_error("installed geometry mismatch");
}

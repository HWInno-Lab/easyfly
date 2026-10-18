// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>
struct SampleBox {float bbox_center_x, bbox_center_y, bbox_width, bbox_height;};
struct SampleImage {std::string encoding{"32FC1"}; unsigned width, height, step; std::vector<std::uint8_t> data;};
struct SampleOptions {double min_depth{.3},max_depth{9.5},min_valid_ratio{.1};};
#ifdef EASYFLY_LEGACY_PROJECTION
#include "legacy_projection.hpp"
#else
#include "easyfly_projection/projection.hpp"
#endif
int main()
{
  std::mt19937 rng(761);
  std::cout << std::setprecision(17);
  for(int sample=0;sample<240;++sample){
    SampleImage image;image.width=61;image.height=47;image.step=(image.width+3)*4;
    std::vector<float> values((image.width+3)*image.height);
    for(auto & v:values){
      v=static_cast<float>(rng()%2500)/100;
      if(rng()%11==0)v=NAN;
      if(rng()%23==0)v=INFINITY;
    }
    image.data.resize(values.size()*4);std::memcpy(image.data.data(),values.data(),image.data.size());
    SampleBox box{static_cast<float>(rng()%120)/100-.1f,static_cast<float>(rng()%120)/100-.1f,
      static_cast<float>(rng()%90)/100,static_cast<float>(rng()%90)/100};
    SampleOptions options{.3,19.5,static_cast<double>(sample%4)/4};
    float depth;
#ifdef EASYFLY_LEGACY_PROJECTION
    LegacySampler legacy;legacy.params_=options;depth=legacy.medianDepth(box,image);
#else
    depth=easyfly::projection::medianDepth(
      {box.bbox_center_x,box.bbox_center_y,box.bbox_width,box.bbox_height},
      {image.data.data(),image.data.size(),static_cast<int>(image.width),static_cast<int>(image.height),image.step},
      {options.min_depth,options.max_depth,options.min_valid_ratio});
#endif
    int u=static_cast<int>(box.bbox_center_x*image.width),v=static_cast<int>(box.bbox_center_y*image.height);
    std::array<double,3> xyz;
#ifdef EASYFLY_LEGACY_PROJECTION
    xyz={(u-30.)*depth/110.,(v-23.)*depth/120.,depth};
#else
    xyz=easyfly::projection::pinhole(u,v,depth,110.,120.,30.,23.);
#endif
    std::cout<<sample<<' '<<depth<<' '<<xyz[0]<<' '<<xyz[1]<<' '<<xyz[2]<<'\n';
  }
}

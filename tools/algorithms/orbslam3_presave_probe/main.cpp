// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Synthetic cleanup contract on a deserialized in-memory copy. Never writes
// an atlas, runs tracking, injects observations into a live estimator or proves
// localization/loop accuracy. The source atlas remains byte-for-byte unchanged.
#include <Atlas.h>
#include <KeyFrameDatabase.h>
#include <KeyFrame.h>
#include <ORBVocabulary.h>
#include <boost/archive/binary_iarchive.hpp>
#include <fstream>
#include <iostream>
#include <stdexcept>

int main(int argc, char ** argv)
{
  if (argc != 3) {return 2;}
  ORB_SLAM3::ORBVocabulary vocabulary;
  if (!vocabulary.loadFromTextFile(argv[2])) {throw std::runtime_error("vocabulary load failed");}
  ORB_SLAM3::KeyFrameDatabase database(vocabulary);
  ORB_SLAM3::Atlas * atlas = nullptr;
  std::string name, checksum;
  {
    std::ifstream input(argv[1], std::ios::binary);
    boost::archive::binary_iarchive archive(input);
    archive >> name >> checksum >> atlas;
  }
  atlas->SetKeyFrameDababase(&database);atlas->SetORBVocabulary(&vocabulary);atlas->PostLoad();
  auto maps = atlas->GetAllMaps();
  if (maps.size() != 1) {throw std::runtime_error("single saved map fixture required");}
  auto * map = maps.front();
  const auto before = map->MapPointsInMap();
  if (before < 100) {throw std::runtime_error("real nonempty fixture required");}
  // Exercise precisely the foreign-map observation branch that PreSave cleans.
  // Change only this private deserialized copy; the atlas file is never opened
  // for writing. This makes cleanup remove observations and cull their points.
  ORB_SLAM3::Map foreign(0);
  for (auto * keyframe : map->GetAllKeyFrames()) {keyframe->UpdateMap(&foreign);}
  const auto cameras = atlas->GetAllCameras();
  std::set<ORB_SLAM3::GeometricCamera *> camera_set(cameras.begin(), cameras.end());
  std::cout << "presave_cleanup_contract_start points=" << before << std::endl;
  map->PreSave(camera_set);
  const auto after = map->MapPointsInMap();
  std::cout << "{\"passed\":" << (after == 0 ? "true" : "false")
            << ",\"original_points\":" << before << ",\"remaining_points\":" << after
            << ",\"scope\":\"synthetic in-memory cleanup contract only\"}" << std::endl;
  return after == 0 ? 0 : 1;
}

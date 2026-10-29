// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Deserialize actual saved data for diagnosis. No tracking, map optimization,
// injected observations, truth initialization or algorithm threshold changes.
#include <Atlas.h>
#include <KeyFrameDatabase.h>
#include <KeyFrame.h>
#include <ORBVocabulary.h>
#include <boost/archive/binary_iarchive.hpp>
#include <algorithm>
#include <fstream>
#include <stdexcept>
#include <vector>

int main(int argc, char ** argv)
{
  if (argc != 4) {return 2;}
  ORB_SLAM3::ORBVocabulary vocabulary;
  if (!vocabulary.loadFromTextFile(argv[2])) {throw std::runtime_error("vocabulary load failed");}
  ORB_SLAM3::KeyFrameDatabase database(vocabulary);
  ORB_SLAM3::Atlas * atlas = nullptr;
  std::string original_vocabulary, original_checksum;
  {
    std::ifstream input(argv[1], std::ios::binary);
    boost::archive::binary_iarchive archive(input);
    archive >> original_vocabulary >> original_checksum >> atlas;
  }
  atlas->SetKeyFrameDababase(&database);
  atlas->SetORBVocabulary(&vocabulary);
  atlas->PostLoad();
  std::ofstream output(argv[3]);
  for (auto * map : atlas->GetAllMaps()) {
    auto keyframes = map->GetAllKeyFrames();
    std::sort(keyframes.begin(), keyframes.end(), [](auto * a, auto * b) {return a->mnId < b->mnId;});
    for (auto * current : keyframes) {
      const auto connections = current->GetConnectedKeyFrames();
      const auto center = current->GetCameraCenter();
      output << "{\"map\":" << map->GetId() << ",\"keyframe\":" << current->mnId
             << ",\"source_s\":" << current->mTimeStamp << ",\"camera_center\":["
             << center.x() << ',' << center.y() << ',' << center.z() << "],\"loop_edges\":"
             << current->GetLoopEdges().size() << ",\"connected\":[";
      bool first = true;
      for (auto * connected : connections) {
        if (!first) {output << ',';} first = false;
        output << connected->mnId;
      }
      output << "],\"loop_neighbors\":[";
      first = true;
      for (auto * other : current->GetLoopEdges()) {
        if (!first) {output << ',';} first = false;
        output << "{\"keyframe\":" << other->mnId << ",\"source_s\":" << other->mTimeStamp << '}';
      }
      output << "],\"top_bow\":[";
      std::vector<std::pair<double, ORB_SLAM3::KeyFrame *>> scores;
      for (auto * other : keyframes) {
        if (other != current) {scores.push_back({vocabulary.score(current->mBowVec, other->mBowVec), other});}
      }
      std::sort(scores.begin(), scores.end(), [](auto a, auto b) {return a.first > b.first;});
      first = true;
      for (size_t i = 0; i < std::min<size_t>(8, scores.size()); ++i) {
        if (!first) {output << ',';} first = false;
        output << "{\"keyframe\":" << scores[i].second->mnId << ",\"score\":" << scores[i].first
               << ",\"already_connected\":" << (connections.count(scores[i].second) ? "true" : "false") << '}';
      }
      output << "]}\n";
    }
  }
  return output.good() ? 0 : 1;
}

// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Native serialization regression using a real, immutable recorded atlas.
// Transferring objects below is a structural fixture, not a SLAM merge result.
#include <Atlas.h>
#include <KeyFrameDatabase.h>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <sstream>

using namespace ORB_SLAM3;
std::string vocabularyName, vocabularyChecksum;
Atlas* read(const std::string& path, ORBVocabulary& vocabulary, KeyFrameDatabase& db) {
    std::ifstream stream(path, std::ios::binary);
    if(!stream) throw std::runtime_error("missing fixture");
    boost::archive::binary_iarchive archive(stream);
    Atlas* atlas=nullptr;
    archive >> vocabularyName >> vocabularyChecksum >> atlas;
    atlas->SetORBVocabulary(&vocabulary);
    atlas->SetKeyFrameDababase(&db);
    atlas->PostLoad();
    return atlas;
}
std::string bytes(Atlas* atlas) {
    atlas->PreSave();
    std::ostringstream stream(std::ios::binary);
    { boost::archive::binary_oarchive archive(stream);
      archive << vocabularyName << vocabularyChecksum << atlas; }
    return stream.str();
}
void save(const std::filesystem::path& file, const std::string& data) {
    if(std::filesystem::exists(file)) throw std::runtime_error("refuse existing fixture output");
    std::ofstream stream(file, std::ios::binary); stream.write(data.data(),data.size());
    if(!stream) throw std::runtime_error("fixture write failed");
}
int main(int argc, char** argv) {
  if(argc!=5) return 2;
  try {
    const std::filesystem::path output=argv[4];
    if(!std::filesystem::create_directory(output)) throw std::runtime_error("output must be new");
    ORBVocabulary vocabulary;
    if(!vocabulary.loadFromTextFile(argv[1])) throw std::runtime_error("vocabulary load failed");
    KeyFrameDatabase firstDb(vocabulary);
    Atlas* original=read(argv[2],vocabulary,firstDb);
    const auto maps=original->GetAllMaps();
    if(maps.size()!=1) throw std::runtime_error("fixture requires one accepted map");
    Map* old=maps.front();
    const auto keyframes=old->GetAllKeyFrames(); const auto points=old->GetAllMapPoints();
    if(keyframes.size()<10 || points.size()<100) throw std::runtime_error("empty fixture");
    const auto first=bytes(original), second=bytes(original);
    if(first!=second) throw std::runtime_error("repeated PreSave changed the archive");
    save(output/"first.osa",first);
    KeyFrameDatabase secondDb(vocabulary);
    Atlas* loaded=read((output/"first.osa").string(),vocabulary,secondDb);
    old=loaded->GetAllMaps().at(0);
    // Populate the old snapshot, then empty its live sets exactly as the
    // transfer phase of native MergeLocal2 does. Pose geometry is untouched.
    loaded->PreSave();
    loaded->CreateNewMap(); Map* next=loaded->GetCurrentMap();
    for(KeyFrame* keyframe:old->GetAllKeyFrames()) {
      keyframe->UpdateMap(next); next->AddKeyFrame(keyframe); old->EraseKeyFrame(keyframe);
    }
    for(MapPoint* point:old->GetAllMapPoints()) {
      point->UpdateMap(next); next->AddMapPoint(point); old->EraseMapPoint(point);
    }
    next->mvpKeyFrameOrigins=old->mvpKeyFrameOrigins;
    const auto transferred=bytes(loaded);
    if(transferred!=bytes(loaded)) throw std::runtime_error("repeated transferred snapshot changed");
    save(output/"transferred.osa",transferred);
    KeyFrameDatabase thirdDb(vocabulary);
    Atlas* result=read((output/"transferred.osa").string(),vocabulary,thirdDb);
    if(result->GetAllMaps().size()!=1 ||
       result->GetAllMaps().front()->GetAllKeyFrames().size()!=keyframes.size() ||
       result->GetAllMaps().front()->GetAllMapPoints().size()!=points.size())
      throw std::runtime_error("serialization lost or duplicated live geometry");
    bool rejected=false; std::string rejection;
    try { KeyFrameDatabase badDb(vocabulary); read(argv[3],vocabulary,badDb); }
    catch(const std::exception& error) { rejected=true; rejection=error.what(); }
    if(!rejected) throw std::runtime_error("malformed historical archive was silently accepted");
    std::ofstream report(output/"result.json");
    report << "{\"passed\":true,\"kind\":\"native structural serialization regression\","
           << "\"repeated_save_identical\":true,\"live_maps\":1,\"keyframes\":" << keyframes.size()
           << ",\"map_points\":" << points.size() << ",\"malformed_rejected\":true,\"reason\":\""
           << rejection << "\"}\n";
    return 0;
  } catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}

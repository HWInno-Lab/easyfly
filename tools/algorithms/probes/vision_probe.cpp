// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <iomanip>
#include <iostream>
#ifdef EASYFLY_LEGACY_VISION
#include "tag_core.hpp"
#include "track_core.hpp"
namespace tag = ef_tag;
namespace sot = ef_single;
#else
#include "easyfly_tag_pose/tag_core.hpp"
#include "easyfly_csrt/track_core.hpp"
namespace tag = easyfly::tag_pose;
namespace sot = easyfly::csrt;
#endif
int main()
{
  cv::setNumThreads(2);
  std::cout << std::setprecision(12);
  cv::Mat K = (cv::Mat_<double>(3,3) << 600,0,320,0,600,240,0,0,1);
  std::vector<cv::Point3f> object{{-.1,.1,0},{.1,.1,0},{.1,-.1,0},{-.1,-.1,0}};
  for (int i=0; i<8; ++i) {
    std::vector<cv::Point2f> corners;
    cv::projectPoints(object, cv::Vec3d(2.7+i*.04,.2,.1), cv::Vec3d(.1,.05,1+i*.1), K, cv::Mat(), corners);
    if (i==6) corners[0].x+=5;
    auto p=tag::estimate(corners,K,cv::Mat(),i==7?0:.2,i==6?.001:2,i==5?100:.2);
    std::cout << "pose " << i << ' ' << p.valid << ' ' << p.reason << ' ' << p.error;
    if(p.valid) std::cout << ' ' << p.t << ' ' << p.q;
    std::cout << '\n';
  }
  cv::Mat marker; cv::aruco::drawMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11),7,160,marker);
  cv::Mat gray(240,240,CV_8U,cv::Scalar(255));marker.copyTo(gray(cv::Rect(40,40,160,160)));
#ifdef EASYFLY_LEGACY_VISION
  std::vector<int> ids;std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(gray,cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11),corners,ids,tag::detector_parameters());
#else
  auto detections=tag::detect(gray);auto ids=detections.ids;auto corners=detections.corners;
#endif
  for(size_t i=0;i<ids.size();++i)std::cout<<"marker "<<ids[i]<<' '<<corners[i]<<'\n';
  cv::Mat patch(40,40,CV_8UC3);cv::RNG rng(42);rng.fill(patch,cv::RNG::UNIFORM,0,255);
  cv::Rect box(60,50,40,40);
#ifdef EASYFLY_LEGACY_VISION
  auto tracker=cv::TrackerCSRT::create();
#else
  auto tracker=sot::make_tracker("csrt");
#endif
  for(int i=0;i<12;++i){
    cv::Mat frame(160,200,CV_8UC3,cv::Scalar(30,30,30));
    if(i!=10)patch.copyTo(frame(cv::Rect(60+i,50,40,40)));
    bool ok=true;
    if(i==0||i==11){
      box={60+i,50,40,40};
#ifdef EASYFLY_LEGACY_VISION
      tracker=cv::TrackerCSRT::create();tracker->init(frame,box);
#else
      tracker->reset();ok=tracker->initialize(frame,box);
#endif
    }else{
#ifdef EASYFLY_LEGACY_VISION
      ok=tracker->update(frame,box);
#else
      auto result=tracker->update(frame);ok=result.ok;box=result.box;
      if(result.calibrated_confidence.has_value())return 3;
#endif
    }
    std::cout<<"track "<<i<<' '<<ok<<' '<<box<<'\n';
  }
  std::cout<<"roi "<<sot::roi(.1,.2,.2,.2)<<' '<<sot::pixels(.1,.2,.2,.2,200,160)<<'\n';
}

// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "easyfly_object_detection/backend_factory.hpp"
namespace d = easyfly::detection;
namespace b = easyfly::backends;
namespace p = easyfly_object_detection;
class FixtureDetector final : public d::DetectorBase {
public:
  static d::DetectorParams fixture_params() {d::DetectorParams p; p.backend = "fixture"; return p;}
  explicit FixtureDetector(double score) : DetectorBase(fixture_params()), score_(score) {}
  std::string activeProvider() const override {return "fixture";}
protected:
  void detectImpl(const cv::Mat &, std::vector<d::Detection> & out, d::DetectionTiming *) override {
    out = {{1, 2, 3, 4, static_cast<float>(score_), 0, "fixture"}};
  }
  void unletterboxImpl(d::Detection &) override {}
private: double score_;
};
TEST(DetectorFactory, PublicSelectionDoesNotRequireAnOnnxImplementationHeader) {
  p::DetectorFactory factory;
  factory.add("fixture", [](const b::Parameters & params) {
    return std::make_unique<FixtureDetector>(b::parameter<double>(params, "score"));
  });
  auto detector = p::createBackend(b::BackendConfig{"fixture", {{"score", .75}}}, factory);
  detector->warmUp(); std::vector<d::Detection> out;
  detector->detect(cv::Mat(8, 8, CV_8UC3), out);
  ASSERT_EQ(out.size(), 1u); EXPECT_FLOAT_EQ(out[0].confidence, .75f);
  EXPECT_EQ(detector->activeProvider(), "fixture");
  EXPECT_THROW(p::createBackend(b::BackendConfig{"fixture", {}}), std::invalid_argument);
}
TEST(DetectorFactory, RejectBeforeModelOrWorkerCreation) {
  EXPECT_THROW(p::createBackend(b::BackendConfig{"tensorrt", {}}), std::invalid_argument);
  EXPECT_THROW(p::createBackend(b::BackendConfig{"onnx", {{"imgsz", true}}}), std::invalid_argument);
  EXPECT_THROW(p::createBackend(b::BackendConfig{"onnx", {{"max_detections", 101}}}), std::invalid_argument);
  EXPECT_THROW(p::createBackend(b::BackendConfig{"onnx", {{"vendor", 1}}}), std::invalid_argument);
  EXPECT_THROW(p::createBackend(b::BackendConfig{"onnx", {{"model_path", std::string("/missing/model.onnx")}, {"use_gpu", false}}}), std::exception);
}

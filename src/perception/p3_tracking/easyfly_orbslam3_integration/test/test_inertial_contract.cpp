// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <limits>
#include "easyfly_orbslam3_integration/inertial_covariance.hpp"
#include "easyfly_orbslam3_integration/atlas_manifest.hpp"

using easyfly_orbslam3::Information15;
using easyfly_orbslam3::Covariance6;
using easyfly_orbslam3::conditionalBaseCovariance;

TEST(InertialCovariance, MarginalizesVelocityAndBiasRatherThanConditioningThemAway)
{
  Information15 h = Information15::Identity() * 10.;
  h(3, 6) = h(6, 3) = 6.;
  Covariance6 c;
  ASSERT_TRUE(
    conditionalBaseCovariance(
      h, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
      c));
  EXPECT_NEAR(c(0, 0), 1. / (10. - 36. / 10.), 1e-12);
  EXPECT_GT(c(0, 0), .1);
  EXPECT_NEAR(c(3, 3), .1, 1e-12);
}

TEST(InertialCovariance, LeverArmJacobianMatchesIndependentCentralPosePerturbations)
{
  const Eigen::Matrix3d r =
    Eigen::AngleAxisd(.8, Eigen::Vector3d(1., 2., 3.).normalized()).toRotationMatrix();
  const Eigen::Vector3d lever(.011, .02329, -.12412);
  Covariance6 jacobian;
  constexpr double epsilon = 1e-6;
  for (int column = 0; column < 6; ++column) {
    Eigen::Vector3d w = Eigen::Vector3d::Zero(), t = Eigen::Vector3d::Zero();
    if (column < 3) {w[column] = epsilon;} else {t[column - 3] = epsilon;}
    auto pose = [&](double sign) {
        const auto delta = column <
          3 ? Eigen::AngleAxisd(
          sign * epsilon,
          w.normalized()).toRotationMatrix() : Eigen::Matrix3d::Identity().eval();
        Eigen::Matrix<double, 6, 1> result;
        result.head<3>() = r * (sign * t + delta * lever);
        Eigen::AngleAxisd orientation(r * delta * r.transpose());
        result.tail<3>() = orientation.angle() * orientation.axis();
        return result;
      };
    jacobian.col(column) = (pose(1.) - pose(-1.)) / (2 * epsilon);
  }
  Information15 h = Information15::Identity();
  for (int i = 0; i < 15; ++i) {
    h(i, i) = 1. + i;
  }
  Covariance6 c;
  ASSERT_TRUE(conditionalBaseCovariance(h, r, lever, c));
  const Covariance6 oracle = jacobian * h.inverse().topLeftCorner<6, 6>() * jacobian.transpose();
  EXPECT_LT((oracle - c).norm(), 1e-9);
}

TEST(InertialCovariance, InvalidOrUnobservableInformationCannotBecomeFakeCovariance)
{
  Covariance6 c;
  auto h = Information15::Identity().eval();h(0, 0) = 0.;
  EXPECT_FALSE(
    conditionalBaseCovariance(
      h, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
      c));
  h(0, 0) = -1.;
  EXPECT_FALSE(
    conditionalBaseCovariance(
      h, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
      c));
  h.setIdentity();h(0, 1) = .1;
  EXPECT_FALSE(
    conditionalBaseCovariance(
      h, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
      c));
  h.setIdentity();h(2, 2) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
    conditionalBaseCovariance(
      h, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
      c));
  h.setIdentity();
  EXPECT_FALSE(
    conditionalBaseCovariance(
      h, 2. * Eigen::Matrix3d::Identity(),
      Eigen::Vector3d::Zero(), c));
}

TEST(InertialAtlas, ManifestBindsBytesCalibrationModeAndCompletedInitialization)
{
  char path[] = "/tmp/easyfly-orb-manifest-XXXXXX";
  ASSERT_NE(::mkdtemp(path), nullptr);
  const std::filesystem::path directory(path);
  const std::string stem = (directory / "atlas").string();
  std::ofstream(stem + ".osa") << "component fixture, not a native atlas";
  const std::string settings(64, 'a'), vocabulary(64, 'b');
  EXPECT_THROW(
    easyfly_orbslam3::verifyInertialAtlas(stem, settings, vocabulary),
    std::runtime_error);
  easyfly_orbslam3::saveInertialManifest(stem, settings, vocabulary, true, 15);
  EXPECT_NO_THROW(easyfly_orbslam3::verifyInertialAtlas(stem, settings, vocabulary));
  EXPECT_THROW(
    easyfly_orbslam3::verifyInertialAtlas(
      stem, vocabulary,
      vocabulary), std::runtime_error);
  EXPECT_THROW(easyfly_orbslam3::verifyInertialAtlas(stem, settings, settings), std::runtime_error);
  EXPECT_THROW(
    easyfly_orbslam3::saveInertialManifest(
      stem, settings, vocabulary, true,
      15), std::runtime_error);
  std::ofstream(stem + ".osa", std::ios::app) << "changed";
  EXPECT_THROW(
    easyfly_orbslam3::verifyInertialAtlas(stem, settings, vocabulary),
    std::runtime_error);
  std::filesystem::remove(stem + ".easyfly.yaml");
  easyfly_orbslam3::saveInertialManifest(stem, settings, vocabulary, false, 15);
  EXPECT_THROW(
    easyfly_orbslam3::verifyInertialAtlas(stem, settings, vocabulary),
    std::runtime_error);
  std::filesystem::remove_all(directory);
}

TEST(InertialAtlas, FinalFileCannotOverwriteAnOutputCreatedDuringTheSession)
{
  char path[] = "/tmp/easyfly-orb-exclusive-XXXXXX";
  ASSERT_NE(::mkdtemp(path), nullptr);
  const std::filesystem::path root(path), native = root / "native.osa", target = root / "final.osa";
  std::ofstream(native) << "new native output";
  std::ofstream(target) << "other session output";
  const auto before = easyfly_orbslam3::fileSha256(target, 1024);
  EXPECT_THROW(easyfly_orbslam3::publishNewInertialAtlas(native, target), std::exception);
  EXPECT_EQ(easyfly_orbslam3::fileSha256(target, 1024), before);
  const auto fresh = root / "fresh.osa";
  EXPECT_NO_THROW(easyfly_orbslam3::publishNewInertialAtlas(native, fresh));
  EXPECT_EQ(easyfly_orbslam3::fileSha256(fresh, 1024), easyfly_orbslam3::fileSha256(native, 1024));
  std::filesystem::remove_all(root);
}

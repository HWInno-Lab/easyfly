// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "G2oTypes.h"

double stationary_residual(float measured_acceleration) {
    using namespace ORB_SLAM3;
    IMU::Bias bias(0, 0, 0, 0, 0, 0);
    IMU::Calib calibration(Sophus::SE3f(), 1e-5f, .0012f, 1e-6f, 1e-4f);
    IMU::Preintegrated integrated(bias, calibration);
    for (int i = 0; i < 2000; ++i) {
        integrated.IntegrateNewMeasurement(Eigen::Vector3f(0, 0, measured_acceleration),
            Eigen::Vector3f::Zero(), .005f);
    }
    ImuCamPose pose;
    pose.Rwb.setIdentity();
    pose.twb.setZero();
    VertexPose first, second;
    first.setEstimate(pose); second.setEstimate(pose);
    VertexVelocity v1, v2;
    v1.setEstimate(Eigen::Vector3d::Zero()); v2.setEstimate(Eigen::Vector3d::Zero());
    VertexGyroBias gyro;
    VertexAccBias accelerometer;
    gyro.setEstimate(Eigen::Vector3d::Zero());
    accelerometer.setEstimate(Eigen::Vector3d::Zero());
    EdgeInertial edge(&integrated);
    edge.setVertex(0, &first); edge.setVertex(1, &v1);
    edge.setVertex(2, &gyro); edge.setVertex(3, &accelerometer);
    edge.setVertex(4, &second); edge.setVertex(5, &v2);
    edge.computeError();
    return edge.error().norm();
}

TEST(GazeboGravity, StationaryTenSecondNativeInertialFactor) {
    // The upstream preintegrator accumulates 2000 steps in float. Allow its
    // centimetre-scale roundoff; a 0.01 m/s^2 calibration error is over 0.5 m.
    EXPECT_LT(stationary_residual(9.8f), .025);
}

TEST(GazeboGravity, WrongGravityIsObservableInTheFactor) {
    EXPECT_GT(stationary_residual(9.81f), .4);
}

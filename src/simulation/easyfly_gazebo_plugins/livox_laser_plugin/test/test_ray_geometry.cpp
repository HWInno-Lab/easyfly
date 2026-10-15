// Copyright 2026 EasyFly Project
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <gazebo/ode/collision.h>
#include <ignition/math/Quaternion.hh>

#include "livox_laser_simulation/ray_geometry.hpp"

namespace
{

class RayGeometry : public testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_NE(dInitODE2(0), 0);
    ray = dCreateRay(nullptr, 0);
    box = dCreateBox(nullptr, 0.1, 0.2, 0.2);
  }

  void TearDown() override
  {
    dGeomDestroy(ray);
    dGeomDestroy(box);
    dCloseODE();
  }

  void setRay(double minimum, double maximum)
  {
    const auto start = easyfly_livox::rayStart({}, {1, 0, 0}, minimum);
    dGeomRaySet(ray, start.X(), start.Y(), start.Z(), 1, 0, 0);
    dGeomRaySetLength(ray, maximum - minimum);
  }

  dGeomID ray = nullptr;
  dGeomID box = nullptr;
};

TEST_F(RayGeometry, ExternalSurfaceHasUnbiasedNativeOdeRange)
{
  setRay(0.2, 25.0);
  dGeomSetPosition(box, 2.05, 0, 0);  // Nearest surface is exactly 2m away.
  dContactGeom contact{};
  ASSERT_EQ(dCollide(ray, box, 1, &contact, sizeof(contact)), 1);
  EXPECT_NEAR(contact.depth, 1.8, 1e-9);
  EXPECT_NEAR(easyfly_livox::sensorRange(contact.depth, 0.2), 2.0, 1e-9);
}

TEST_F(RayGeometry, ReturnInsidePreprocessingBlindZoneStaysInside)
{
  setRay(0.2, 25.0);
  dGeomSetPosition(box, 0.3, 0, 0);  // Surface at 0.25m, not an apparent 0.45m.
  dContactGeom contact{};
  ASSERT_EQ(dCollide(ray, box, 1, &contact, sizeof(contact)), 1);
  EXPECT_NEAR(easyfly_livox::sensorRange(contact.depth, 0.2), 0.25, 1e-9);
  EXPECT_LT(easyfly_livox::sensorRange(contact.depth, 0.2), 0.3);
}

TEST_F(RayGeometry, GeometryEntirelyBeforeMinimumIsNotMeasured)
{
  setRay(0.2, 25.0);
  dGeomSetPosition(box, 0.1, 0, 0);  // Far surface 0.15m < configured min.
  dContactGeom contact{};
  EXPECT_EQ(dCollide(ray, box, 1, &contact, sizeof(contact)), 0);
}

TEST_F(RayGeometry, ExternalGeometryBeyondMaximumIsNotMeasured)
{
  setRay(0.2, 25.0);
  dGeomSetPosition(box, 25.2, 0, 0);
  dContactGeom contact{};
  EXPECT_EQ(dCollide(ray, box, 1, &contact, sizeof(contact)), 0);
  EXPECT_DOUBLE_EQ(easyfly_livox::sensorRange(dGeomRayGetLength(ray), 0.2), 25.0);
}

TEST_F(RayGeometry, GroundContactRetainsSensorOriginClearance)
{
  const double sensor_height = 0.35;
  const double minimum = 0.2;
  const auto start = easyfly_livox::rayStart(
    {0, 0, sensor_height}, {0, 0, -1}, minimum);
  dGeomRaySet(ray, start.X(), start.Y(), start.Z(), 0, 0, -1);
  dGeomRaySetLength(ray, 25.0 - minimum);
  dGeomSetPosition(box, 0, 0, -0.1);  // Top surface is z=0.
  dContactGeom contact{};
  ASSERT_EQ(dCollide(ray, box, 1, &contact, sizeof(contact)), 1);
  EXPECT_NEAR(contact.depth, sensor_height - minimum, 1e-9);
  const double distance = easyfly_livox::sensorRange(contact.depth, minimum);
  EXPECT_NEAR(sensor_height - distance, 0.0, 1e-9);
  // Publishing contact.depth alone would move the floor up by 0.2m.
  EXPECT_NEAR(sensor_height - contact.depth, minimum, 1e-9);
}

TEST(RayStart, RotationAndTranslationRemainInCollisionFrame)
{
  const ignition::math::Vector3d origin(1, -2, 0.08);
  const ignition::math::Quaterniond rotation(0, 0, 1.5707963267948966);
  const auto axis = rotation * ignition::math::Vector3d::UnitX;
  const auto start = easyfly_livox::rayStart(origin, axis, 0.2);
  EXPECT_NEAR(start.X(), 1.0, 1e-12);
  EXPECT_NEAR(start.Y(), -1.8, 1e-12);
  EXPECT_DOUBLE_EQ(start.Z(), 0.08);
  EXPECT_NEAR((start - origin).Length(), 0.2, 1e-12);
}

TEST(RayStart, ZeroMinimumRetainsSensorOrigin)
{
  const ignition::math::Vector3d origin(1, 2, 3);
  EXPECT_EQ(easyfly_livox::rayStart(origin, {0, 0, 1}, 0), origin);
}

}  // namespace

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "plan_env/trusted_cloud_persistence.h"

TEST(TrustedCloudPersistence, SparseTrustedFramesRetainThenExpireVoxel)
{
  std::vector<char> occupied(3, 0);
  std::vector<std::uint16_t> age(3, 0);
  plan_env::markTrustedCloudVoxel(occupied, age, 1);

  ASSERT_TRUE(plan_env::ageTrustedCloudVoxels(occupied, age, 2));
  EXPECT_EQ(occupied[1], 1);
  EXPECT_EQ(age[1], 1);
  ASSERT_TRUE(plan_env::ageTrustedCloudVoxels(occupied, age, 2));
  EXPECT_EQ(occupied[1], 0);
  EXPECT_EQ(age[1], 0);
}

TEST(TrustedCloudPersistence, ReobservedVoxelResetsAge)
{
  std::vector<char> occupied(1, 0);
  std::vector<std::uint16_t> age(1, 0);
  plan_env::markTrustedCloudVoxel(occupied, age, 0);
  ASSERT_TRUE(plan_env::ageTrustedCloudVoxels(occupied, age, 2));
  EXPECT_EQ(age[0], 1);
  plan_env::markTrustedCloudVoxel(occupied, age, 0);
  EXPECT_EQ(age[0], 0);
  ASSERT_TRUE(plan_env::ageTrustedCloudVoxels(occupied, age, 2));
  EXPECT_EQ(occupied[0], 1);
  EXPECT_EQ(age[0], 1);
}

TEST(TrustedCloudPersistence, ValidSparseNongroundKeepsEvidenceThenInvalidClears)
{
  std::vector<char> occupied(1, 0);
  std::vector<std::uint16_t> age(1, 0);

  // A valid non-empty nonground cloud with no accepted obstacle cluster still
  // reaches GridMap as a trusted frame, so an older voxel is retained.
  plan_env::markTrustedCloudVoxel(occupied, age, 0);
  ASSERT_TRUE(plan_env::ageTrustedCloudVoxels(occupied, age, 3));
  EXPECT_EQ(occupied[0], 1);

  // Invalid/empty input is a reset protocol, not a missing observation.
  plan_env::clearTrustedCloudStorage(occupied, age);
  EXPECT_EQ(occupied[0], 0);
  EXPECT_EQ(age[0], 0);
}

TEST(TrustedCloudPersistence, InvalidOdomClearsAndValidRecoveryStartsFresh)
{
  std::vector<char> occupied(2, 0);
  std::vector<std::uint16_t> age(2, 0);
  plan_env::markTrustedCloudVoxel(occupied, age, 1);

  // Invalid odometry must clear every trusted voxel, independent of camera
  // position; a later valid odometry/cloud pair can then create new evidence.
  plan_env::clearTrustedCloudStorage(occupied, age);
  EXPECT_EQ(occupied[1], 0);
  EXPECT_EQ(age[1], 0);
  plan_env::markTrustedCloudVoxel(occupied, age, 0);
  EXPECT_EQ(occupied[0], 1);
  EXPECT_EQ(occupied[1], 0);
}

TEST(TrustedCloudPersistence, InvalidStorageOrLimitFailsClosed)
{
  std::vector<char> occupied(1, 1);
  std::vector<std::uint16_t> age;
  EXPECT_FALSE(plan_env::ageTrustedCloudVoxels(occupied, age, 2));
  age.resize(1);
  EXPECT_FALSE(plan_env::ageTrustedCloudVoxels(occupied, age, 0));
  EXPECT_FALSE(plan_env::ageTrustedCloudVoxels(
    occupied, age, static_cast<std::size_t>(std::numeric_limits<std::uint16_t>::max()) + 1));
}

TEST(TrustedCloudPersistence, OutOfRangeMarksDoNotMutateStorage)
{
  std::vector<char> occupied(1, 0);
  std::vector<std::uint16_t> age(1, 0);
  plan_env::markTrustedCloudVoxel(occupied, age, 9);
  EXPECT_EQ(occupied[0], 0);
  EXPECT_EQ(age[0], 0);
  plan_env::clearTrustedCloudVoxel(occupied, age, 9);
  EXPECT_EQ(occupied[0], 0);
  EXPECT_EQ(age[0], 0);
}

TEST(TrustedCloudPersistence, RetentionBoundaryClearsOnConfiguredMissingFrame)
{
  std::vector<char> occupied(1, 0);
  std::vector<std::uint16_t> age(1, 0);
  plan_env::markTrustedCloudVoxel(occupied, age, 0);

  for (std::size_t missing = 1; missing < 20; ++missing) {
    ASSERT_TRUE(plan_env::ageTrustedCloudVoxels(occupied, age, 20));
    ASSERT_EQ(occupied[0], 1);
  }
  ASSERT_TRUE(plan_env::ageTrustedCloudVoxels(occupied, age, 20));
  EXPECT_EQ(occupied[0], 0);
  EXPECT_EQ(age[0], 0);
}
